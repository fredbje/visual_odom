#include "system.h"
#include "easylogging++.h"
#include "matrixutils.h"
#include "utils.h"
#include <boost/filesystem.hpp>
#include <boost/date_time.hpp>
#include <sstream>


System::System(cv::FileStorage& fSettings, const std::string& vocabularyFile, const gtsam::Pose3& imuTcam)
        : stereoCamera_(fSettings),
        imageLeftPrev_(stereoCamera_.height(), stereoCamera_.width(), CV_8UC1),
        imageRightPrev_(stereoCamera_.height(), stereoCamera_.width(), CV_8UC1),
        imageLeftMatch_(stereoCamera_.height(), stereoCamera_.width(), CV_8UC1),
        imageRightMatch_(stereoCamera_.height(), stereoCamera_.width(), CV_8UC1),
        frameIdCurr_(0), frameIdPrev_(0), frameIdMatch_(-1),
        vosOdom_(stereoCamera_), vosLoop_(stereoCamera_),
        optimizer(stereoCamera_, imuTcam),
        loopDetector(vocabularyFile, fSettings), numLoops_(0),
        state_(State::WaitingForFirstImage)
{
    if(useMapViewer_)
    {
        mapDrawerThread_ = std::thread(&MapDrawer::run, &mapDrawer_);
    }
}

System::System(cv::FileStorage& fSettings, const std::string& vocabularyFile, const gtsam::Pose3& imuTcam, const std::vector<gtsam::Pose3>& gtPoses)
: System(fSettings, vocabularyFile, imuTcam)
{
    gtPoses_ = gtPoses;
    if(useMapViewer_)
    {
        mapDrawer_.setGtPoses(gtPoses);
    }
}

System::~System()
{
    if(useMapViewer_)
    {
        mapDrawer_.requestFinish();
        mapDrawerThread_.join();
    }
}

void System::process(const cv::Mat& imageLeftCurr, const cv::Mat& imageRightCurr, const oxts& navData, const double& timestamp)
{

    if(closeLoops_ && frameIdMatch_ >= 0)
    {
        float averageFlow;
        if(vosLoop_.process(deltaTMatch_, averageFlow, imageLeftMatch_, imageRightMatch_, imageLeftPrev_, imageRightPrev_))
        {
            optimizer.addRelativePoseConstraint(deltaTMatch_, frameIdPrev_, frameIdMatch_, true);
        }
    }

    std::thread tLoopDetection;
    if(closeLoops_)
    {
        tLoopDetection = std::thread(&LoopDetector::process, &loopDetector, imageLeftCurr, imageRightCurr, std::ref(loopResult_));
    }

    if(state_ == State::Initialized)
    {
        pointsLeftPrev_.clear(); pointsRightPrev_.clear(); pointsLeftCurr_.clear(); pointsRightCurr_.clear();

        float averageFlow;
        if (vosOdom_.process(deltaTOdom_, averageFlow,
                         imageLeftCurr, imageRightCurr,
                         imageLeftPrev_, imageRightPrev_,
                         pointsLeftPrev_,
                         pointsRightPrev_,
                         pointsLeftCurr_,
                         pointsRightCurr_)) {

            framePose_ = framePose_ * deltaTOdom_;
            optimizer.addPose(framePose_, frameIdCurr_, timestamp);
            optimizer.addRelativePoseConstraint(deltaTOdom_, frameIdPrev_, frameIdCurr_, false);
        }
    }
    else
    {
        framePose_ = gtsam::Pose3();
        optimizer.addPose(framePose_, frameIdCurr_, timestamp);
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

    optimizer.optimize();
    poses_ = optimizer.getCurrentEstimate();
    framePose_ = poses_.back();

    // --------------------
    // Update visualization
    // --------------------
    if(useMapViewer_)
    {
        mapDrawer_.updateAllPoses(poses_);
    }

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
        optimizer.saveTrajectoryLatLon(gpsTrackPath.string());

        PATH bowDatabasePath = outputDirectoryPath / "bow_database.txt";
        loopDetector.saveDatabase(bowDatabasePath.string());

        PATH graphAndValuesPath = outputDirectoryPath / "graph_values.txt";
        optimizer.saveGraphAndValues(graphAndValuesPath.string());
    }
}