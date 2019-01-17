#include "system.h"
#include "easylogging++.h"
#include "matrixutils.h"
#include "utils.h"
#include <boost/filesystem.hpp>
#include <boost/date_time.hpp>
#include <sstream>
#include <chrono>

System::System(cv::FileStorage& fSettings, const std::string& vocabularyFile, const gtsam::Pose3& imuTcam)
        : stereoCamera_(fSettings),
        imageLeftPrev_(stereoCamera_.height(), stereoCamera_.width(), CV_8UC1),
        imageRightPrev_(stereoCamera_.height(), stereoCamera_.width(), CV_8UC1),
        imageLeftMatch_(stereoCamera_.height(), stereoCamera_.width(), CV_8UC1),
        imageRightMatch_(stereoCamera_.height(), stereoCamera_.width(), CV_8UC1),
        frameIdCurr_(0), frameIdPrev_(0), frameIdMatch_(-1),
        vosOdom_(stereoCamera_), vosLoop_(stereoCamera_),
        mapDrawer_(frames_, gtPoses_, mutexPoses_),
        optimizer_(stereoCamera_, imuTcam),
        numLoops_(0),
        state_(State::WaitingForFirstImage)
{
    if(closeLoops_ && !optimize_)
    {
        LOG(WARNING) << "Optimization must be activated to close loops. Disabling loop closure.";
        closeLoops_ = false;
    }

    if(closeLoops_)
    {
        loopDetector_ = new LoopDetector(vocabularyFile, fSettings);
    }

    if(useMapViewer_)
    {
        mapDrawerThread_ = std::thread(&MapDrawer::run, &mapDrawer_);
    }
}

System::System(cv::FileStorage& fSettings, const std::string& vocabularyFile, const gtsam::Pose3& imuTcam, const std::vector<gtsam::Pose3>& gtPoses)
: System(fSettings, vocabularyFile, imuTcam)
{
    gtPoses_ = gtPoses;
}

System::~System()
{
    if(useMapViewer_)
    {
        mapDrawer_.requestFinish();
        mapDrawerThread_.join();
    }

    if(closeLoops_)
    {
        delete loopDetector_;
    }
}

void System::process(const cv::Mat& imageLeftCurr, const cv::Mat& imageRightCurr, const oxts& navData, const double& timestamp)
{

    if(closeLoops_ && frameIdMatch_ >= 0)
    {
        float averageFlow;
        if(vosLoop_.process(deltaTMatch_, averageFlow, imageLeftMatch_, imageRightMatch_, imageLeftPrev_, imageRightPrev_))
        {
            optimizer_.addRelativePoseConstraint(deltaTMatch_, frameIdPrev_, frameIdMatch_, true);
        }
    }

    std::thread tLoopDetection;
    if(closeLoops_)
    {
        tLoopDetection = std::thread(&LoopDetector::process, loopDetector_, imageLeftCurr, std::ref(loopResult_));
    }

    if(state_ == State::Initialized)
    {
        pointsLeftPrev_.clear(); pointsRightPrev_.clear(); pointsLeftCurr_.clear(); pointsRightCurr_.clear();

        float averageFlow;
        auto tic = std::chrono::high_resolution_clock::now();
        bool odometryOk = vosOdom_.process(deltaTOdom_, averageFlow,
                                           imageLeftCurr, imageRightCurr,
                                           imageLeftPrev_, imageRightPrev_,
                                           pointsLeftPrev_,
                                           pointsRightPrev_,
                                           pointsLeftCurr_,
                                           pointsRightCurr_);
        auto toc = std::chrono::high_resolution_clock::now();
        std::chrono::duration<float> diff = toc-tic;
        voTimes_.push_back(diff.count());

        if (odometryOk) {

            framePose_ = framePose_ * deltaTOdom_;
            frames_.emplace_back(frameIdCurr_, timestamp, framePose_, navData);
            if(optimize_)
            {
                optimizer_.addPose(framePose_, frameIdCurr_, timestamp);
                optimizer_.addRelativePoseConstraint(deltaTOdom_, frameIdPrev_, frameIdCurr_, false);
            }
        }
        // ELSE STATE=TRACK_LOST
    }
    else
    {
        framePose_ = gtsam::Pose3();
        frames_.emplace_back(frameIdCurr_, timestamp, framePose_, navData);
        if(optimize_)
        {
            optimizer_.addPose(framePose_, frameIdCurr_, timestamp);
        }
        state_ = State::Initialized;
    }

    if(closeLoops_)
    {
        tLoopDetection.join();
        if(loopResult_.detection())
        {
            frameIdMatch_ = loopResult_.match;
            imageLeftLoaderThread_ = std::thread(loadImageLeft, std::ref(imageLeftMatch_), frameIdMatch_, "/home/fbjerkas/datasets/kitti-gray/sequences/00/");
            imageRightLoaderThread_ = std::thread(loadImageRight, std::ref(imageRightMatch_), frameIdMatch_, "/home/fbjerkas/datasets/kitti-gray/sequences/00/");
            numLoops_++;
        }
        else
        {
            frameIdMatch_ = -1;
        }
        LOG(DEBUG) << "Loops detected so far: " << numLoops_;
    }

    timestamps_.push_back(timestamp);

    if(optimize_)
    {
        optimizer_.optimize();
        {
            std::unique_lock<std::mutex> lock(mutexPoses_);
            for(unsigned int frameId = 0; frameId < frames_.size(); frameId++)
            {
                frames_[frameId].updatePose(optimizer_.getCurrentPoseEstimate(frameId));
            }
        }
        framePose_ = frames_.back().getPose();
    }

    // --------------------
    // Update visualization
    // --------------------
    if(useFrameViewer_)
    {
        displayFrame(imageLeftCurr, pointsLeftPrev_, pointsLeftCurr_);
    }

    // -----------------------------------------
    // Prepare image for next frame
    // -----------------------------------------
    imageLeftPrev_ = imageLeftCurr;
    imageRightPrev_ = imageRightCurr;
    frameIdPrev_ = frameIdCurr_;
    frameIdCurr_++;

    if(closeLoops_ && frameIdMatch_ >= 0)
    {
        imageLeftLoaderThread_.join();
        imageRightLoaderThread_.join();
    }
}























void System::saveSettings(const std::string& settingsFile)
{
    std::fstream f;
    f.open(settingsFile, std::ios_base::app);
    if(f.is_open())
    {
        f << "////////////////// System Settings //////////////////" << std::endl;
        f << "optimize: " << (optimize_ ? "true" : "false") << std::endl;
        f << "closeLoops: " << (closeLoops_ ? "true" : "false") << std::endl;
        f << "useMapViewer: " << (useMapViewer_ ? "true" : "false") << std::endl;
        f << "useFrameViewer: " << (useFrameViewer_ ? "true" : "false") << std::endl;
        f.close();
    }
    else
    {
        LOG(ERROR) << "System could not open" << settingsFile;
    }
}

void System::saveVoTimes(const std::string& outFile)
{
    std::fstream f;
    f.open(outFile, std::ios_base::out);
    if(f.is_open())
    {
        for(unsigned int i = 0; i < voTimes_.size(); i++)
        {
            f << i << " " << voTimes_[i] << std::endl;
        }
    }
    else
    {
        LOG(ERROR) << "System could not open " << outFile;
    }
}

void System::saveOverallTimes(const std::string& outFile)
{
    std::fstream f;
    f.open(outFile, std::ios_base::out);
    if(f.is_open())
    {
        for(unsigned int i = 0; i < overallTimes_.size(); i++)
        {
            f << i << " " << overallTimes_[i] << std::endl;
        }
    }
    else
    {
        LOG(ERROR) << "System could not open " << outFile;
    }
}


void System::saveLoopTimes(const std::string& outFile)
{
    std::fstream f;
    f.open(outFile, std::ios_base::out);
    if(f.is_open())
    {
        for(unsigned int i = 0; i < loopTimes_.size(); i++)
        {
            f << i << " " << loopTimes_[i] << std::endl;
        }
    }
    else
    {
        LOG(ERROR) << "System could not open " << outFile;
    }
}

void System::saveOptimizationTimes(const std::string& outFile)
{
    std::fstream f;
    f.open(outFile, std::ios_base::out);
    if(f.is_open())
    {
        for(unsigned int i = 0; i < optimizationTimes_.size(); i++)
        {
            f << i << " " << optimizationTimes_[i] << std::endl;
        }
    }
    else
    {
        LOG(ERROR) << "System could not open " << outFile;
    }
}

void System::save()
{
    typedef boost::filesystem::path PATH;
    boost::posix_time::ptime timeLocal =
            boost::posix_time::second_clock::local_time();
    std::stringstream ss;
    ss << timeLocal.date().year();
    ss << "-";
    ss << std::setw(2) << std::setfill('0') << timeLocal.date().month().as_number();
    ss << "-";
    ss << std::setw(2) << std::setfill('0') << timeLocal.date().day().as_number();
    ss << "-";
    ss << std::setw(2) << std::setfill('0') << timeLocal.time_of_day().hours();
    ss << "-";
    ss << std::setw(2) << std::setfill('0') << timeLocal.time_of_day().minutes();

    PATH outputDirectoryPath ( "../output/" + ss.str() );
    boost::system::error_code returnedError;
    boost::filesystem::create_directories( outputDirectoryPath, returnedError );

    if ( returnedError )
        LOG(ERROR) << "Could not create output directory";
    else
    {
        //directories successfully created
        PATH trajectoryEstimatePath = outputDirectoryPath / "stamped_traj_estimate.txt";
        saveTrajectoryRpg(trajectoryEstimatePath.string(), poses_, timestamps_);

        PATH groundTruthPath = outputDirectoryPath / "stamped_groundtruth.txt";
        saveTrajectoryRpg(groundTruthPath.string(), gtPoses_, timestamps_);

        PATH gpsTrackPath = outputDirectoryPath / "traj_lat_lon.txt";
        optimizer_.saveTrajectoryLatLon(gpsTrackPath.string());

        PATH bowDatabasePath = outputDirectoryPath / "bow_database.txt";
        if(closeLoops_)
        {
            loopDetector_->saveDatabase(bowDatabasePath.string());
        }

        PATH graphAndValuesPath = outputDirectoryPath / "graph_values.txt";
        optimizer_.saveGraphAndValues(graphAndValuesPath.string());

        PATH settingsPath = outputDirectoryPath / "settings.txt";
        this->saveSettings(settingsPath.string());
        vosOdom_.saveSettings(settingsPath.string());
        optimizer_.saveSettings(settingsPath.string());
        if(closeLoops_)
        {
            loopDetector_->saveSettings(settingsPath.string());
        }
        stereoCamera_.saveSettings(settingsPath.string());

        PATH voTimesPath = outputDirectoryPath / "voTimes.txt";
        saveVoTimes(voTimesPath.string());

        PATH loopTimesPath = outputDirectoryPath / "loopTimes.txt";
        saveLoopTimes(loopTimesPath.string());

        PATH overallTimesPath = outputDirectoryPath / "overallTimes.txt";
        saveOverallTimes(overallTimesPath.string());

        PATH optimizationTimesPath = outputDirectoryPath / "optimizationTimes.txt";
        saveOptimizationTimes(optimizationTimesPath.string());

    }
}
