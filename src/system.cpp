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
        frameId_(0), matchId_(-1),
        vosOdom_(stereoCamera_), vosLoop_(stereoCamera_),
        optimizer(stereoCamera_, imuTcam),
        loopDetector(vocabularyFile, fSettings), numLoops_(0),
        state_(State::WaitingForFirstImage)
{
    mapDrawerThread_ = std::thread(&MapDrawer::run, &mapDrawer_);
}

System::System(cv::FileStorage& fSettings, const std::string& vocabularyFile, const gtsam::Pose3& imuTcam, const std::vector<gtsam::Pose3>& gtPoses)
: System(fSettings, vocabularyFile, imuTcam)
{
    gtPoses_ = gtPoses;
    mapDrawer_.setGtPoses(gtPoses);
}

System::~System()
{
    mapDrawer_.requestFinish();
    mapDrawerThread_.join();
}

void System::process(const cv::Mat& imageLeftCurr, const cv::Mat& imageRightCurr, const oxts& navData, const double& timestamp)
{

    if(closeLoops_ && matchId_ >= 0)
    {
        //loadImageLeft(imageLeftMatch, matchId_, "/home/fbjerkas/datasets/kitti-gray/sequences/00/");
        //loadImageRight(imageRightMatch, matchId_, "/home/fbjerkas/datasets/kitti-gray/sequences/00/");
        if(vosLoop_.process(deltaTMatch_, imageLeftMatch_, imageRightMatch_, imageLeftPrev_, imageRightPrev_))
        {
            unsigned int lastFrameId = frameId_ - 1;
            optimizer.addRelativePoseConstraint(deltaTMatch_, lastFrameId, matchId_);
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

        if (vosOdom_.process(deltaTOdom_,
                         imageLeftCurr, imageRightCurr,
                         imageLeftPrev_, imageRightPrev_,
                         pointsLeftPrev_,
                         pointsRightPrev_,
                         pointsLeftCurr_,
                         pointsRightCurr_)) {

            framePose_ = framePose_ * deltaTOdom_;
            optimizer.addPose(framePose_, frameId_, timestamp);
            int lastFrameId = frameId_ - 1;
            optimizer.addRelativePoseConstraint(deltaTOdom_, lastFrameId, frameId_);
        }
    }
    else
    {
        framePose_ = gtsam::Pose3();
        optimizer.addPose(framePose_, frameId_, timestamp);
        state_ = State::Initialized;
    }

    if(closeLoops_)
    {
        tLoopDetection.join();
        if(loopResult_.detection())
        {
            matchId_ = loopResult_.match;
            imageLeftLoaderThread_ = std::thread(loadImageLeft, std::ref(imageLeftMatch_), matchId_, "/home/fbjerkas/datasets/kitti-gray/sequences/00/");
            imageRightLoaderThread_ = std::thread(loadImageRight, std::ref(imageRightMatch_), matchId_, "/home/fbjerkas/datasets/kitti-gray/sequences/00/");
            numLoops_++;
        }
        else
        {
            matchId_ = -1;
        }
        LOG(DEBUG) << "Loops detected so far: " << numLoops_;
    }

    timestamps_.push_back(timestamp);

    optimizer.optimize();
    poses_ = optimizer.getCurrentEstimate();
    framePose_ = poses_.back();
    mapDrawer_.updateAllPoses(poses_);

    // -----------------------------------------
    // Prepare image for next frame
    // -----------------------------------------
    displayFrame(imageLeftCurr, pointsLeftPrev_, pointsLeftCurr_);
    imageLeftPrev_ = imageLeftCurr;
    imageRightPrev_ = imageRightCurr;
    frameId_++;

    if(closeLoops_ && matchId_ >= 0)
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