#include <opencv2/core/persistence.hpp>
#include "mapDrawer.h"


MapDrawer::MapDrawer() = default;

MapDrawer::MapDrawer(const std::vector<gtsam::Pose3>& gtPoses)
{
    gtPoses_ = gtPoses;
}

MapDrawer::~MapDrawer() {
    std::cout << "MapDrawer destructor called." << std::endl;
}

void MapDrawer::setGtPoses(const std::vector<gtsam::Pose3> &gtPoses) {
    gtPoses_ = gtPoses;
}

void MapDrawer::updateAllPoses(const std::vector<gtsam::Pose3>& poses) {
    std::unique_lock<std::mutex> lock(mutexPoses_);
    poses_ = poses;
}

void MapDrawer::updateNewPoses(const std::vector<gtsam::Pose3>& poses) {
    std::unique_lock<std::mutex> lock(mutexPoses_);
    poses_.insert(poses_.end(), poses.begin() + poses_.size(), poses.end());
}

void MapDrawer::updateLastPose(const gtsam::Pose3& pose) {
    std::unique_lock<std::mutex> lock(mutexPoses_);
    poses_.push_back(pose);
}

void MapDrawer::drawCamera(pangolin::OpenGlMatrix &Twc, Color color) {
    float CameraSize = 0.5;
    float mCameraLineWidth = 1;
    const float &w = CameraSize;
    const auto h = (float)(w*0.75);
    const auto z = (float)(w*0.6);

    glPushMatrix();
    glMultMatrixd(Twc.m);

    glLineWidth(mCameraLineWidth);
    if(color == red) {
        glColor3f(1.0f, 0.0f, 0.0f);
    } else if(color == green) {
        glColor3f(0.0f, 1.0f, 0.0f);
    }else if (color == blue) {
        glColor3f(0.0f, 0.0f, 1.0f);
    }

    glBegin(GL_LINES);
    glVertex3f(0,0,0);
    glVertex3f(w,h,z);
    glVertex3f(0,0,0);
    glVertex3f(w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,h,z);
    glVertex3f(w,h,z);
    glVertex3f(w,-h,z);
    glVertex3f(-w,h,z);
    glVertex3f(-w,-h,z);
    glVertex3f(-w,h,z);
    glVertex3f(w,h,z);
    glVertex3f(-w,-h,z);
    glVertex3f(w,-h,z);
    glEnd();

    glPopMatrix();
}

pangolin::OpenGlMatrix MapDrawer::getOpenGlMatrix(const gtsam::Pose3& gtsamPose) {
    pangolin::OpenGlMatrix Twc;
    Eigen::Matrix4d pose = gtsamPose.matrix();
    Twc.m[ 0] = pose(0, 0);
    Twc.m[ 1] = pose(1, 0);
    Twc.m[ 2] = pose(2, 0);
    Twc.m[ 3] = pose(3, 0);
    Twc.m[ 4] = pose(0, 1);
    Twc.m[ 5] = pose(1, 1);
    Twc.m[ 6] = pose(2, 1);
    Twc.m[ 7] = pose(3, 1);
    Twc.m[ 8] = pose(0, 2);
    Twc.m[ 9] = pose(1, 2);
    Twc.m[10] = pose(2, 2);
    Twc.m[11] = pose(3, 2);
    Twc.m[12] = pose(0, 3);
    Twc.m[13] = pose(1, 3);
    Twc.m[14] = pose(2, 3);
    Twc.m[15] = pose(3, 3);
    return Twc;
}

void MapDrawer::requestFinish() {
    std::unique_lock<std::mutex> lock(mutexFinish_);
    finishRequested_ = true;
}

bool MapDrawer::checkFinish() {
    std::unique_lock<std::mutex> lock(mutexFinish_);
    return finishRequested_;
}

// TODO Make it possible to option out viewing different poses and graphs.
void MapDrawer::run() {
    pangolin::CreateWindowAndBind("Pose Viewer", 1024, 768);
    glEnable(GL_DEPTH_TEST);

    // Issue specific OpenGl we might need
    glEnable (GL_BLEND);
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::CreatePanel("menu").SetBounds(0.0,1.0,0.0,pangolin::Attach::Pix(175));
    pangolin::Var<bool> menuFollowCamera("menu.Follow Camera",true,true);

    // Define Camera Render Object (for view / scene browsing)
    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(1024,768,viewpointF_,viewpointF_,512,389,0.1,1000),
            pangolin::ModelViewLookAt(viewpointX_,viewpointY_,viewpointZ_, 0,0,0,0.0,-1.0, 0.0)
    );

    // Add named OpenGL viewport to window and provide 3D Handler
    auto *handler3D = new pangolin::Handler3D(s_cam);
    pangolin::View& d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f/768.0f)
            .SetHandler(handler3D);

    // Matrix that changes where the camera is
    pangolin::OpenGlMatrix Tcw;
    Tcw.SetIdentity();

    bool bFollow = true;

    // TODO Fix so that viewer stops when pressing ESC.
    while(true /*!pangolin::ShouldQuit()*/) {
        // Clear screen and activate view to render into
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // Skip if there are no poses
        if(poses_.empty()) {
            continue;
        }

        Tcw = getOpenGlMatrix(poses_.back());
        if(menuFollowCamera && bFollow)
        {
            s_cam.Follow(Tcw);
        }
        else if(menuFollowCamera && !bFollow)
        {
            s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(viewpointX_,viewpointY_,viewpointZ_, 0,0,0,0.0,-1.0, 0.0));
            s_cam.Follow(Tcw);
            bFollow = true;
        }
        else if(!menuFollowCamera && bFollow)
        {
            bFollow = false;
        }

        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

        // The two poses we are drawing lines between
        pangolin::OpenGlMatrix T1Gtsam;
        pangolin::OpenGlMatrix T2Gtsam;
        pangolin::OpenGlMatrix T1Gt;
        pangolin::OpenGlMatrix T2Gt;

        // Draw current frame
        size_t i = 1;
        while(true) {
            std::unique_lock<std::mutex> lockGtsam(mutexPoses_);
            if(i >= poses_.size()) {
                break;
            }

            T1Gtsam    = getOpenGlMatrix(poses_.at(i - 1));
            T2Gtsam    = getOpenGlMatrix(poses_.at(i));
            lockGtsam.unlock();
            if(i == 1) {
                drawCamera(T1Gtsam, green);
            }
            drawCamera(T2Gtsam, green);
            drawLines(T1Gtsam, T2Gtsam, green);

            if(!gtPoses_.empty()) {
                T1Gt = getOpenGlMatrix(gtPoses_.at(i - 1));
                T2Gt = getOpenGlMatrix(gtPoses_.at(i));
                if (i == 1) {
                    drawCamera(T1Gt, red);
                }
                drawCamera(T2Gt, red);
                drawLines(T1Gt, T2Gt, red);
            }
            i++;
        }

        // Swap frames and Process Events
        pangolin::FinishFrame();

        if(checkFinish()) {
            break;
        }
    }
    pangolin::DestroyWindow("Pose Viewer");
    delete handler3D;
    std::cout << "Exiting MapDrawer thread." << std::endl;
}

void MapDrawer::drawLines(pangolin::OpenGlMatrix T1, pangolin::OpenGlMatrix T2, Color color) {
    float mLineSize = 3;
    glLineWidth(mLineSize);
    if(color == red) {
        glColor3f(1.0f, 0.0f, 0.0f);
    } else if(color == green) {
        glColor3f(0.0f, 1.0f, 0.0f);
    }else if (color == blue) {
        glColor3f(0.0f, 0.0f, 1.0f);
    }
    glBegin(GL_LINES);
    glVertex3f((float) T1.m[12], (float) T1.m[13], (float) T1.m[14]);
    glVertex3f((float) T2.m[12], (float) T2.m[13], (float) T2.m[14]);
    glEnd();
}




