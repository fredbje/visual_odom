#include <opencv2/core/persistence.hpp>
#include "mapDrawer.h"
#include "easylogging++.h"

MapDrawer::MapDrawer(const std::vector<Frame>& frames, const std::vector<gtsam::Pose3>& gtPoses, std::mutex& mutexPoses)
: frames_(frames), gtPoses_(gtPoses), mutexPoses_(mutexPoses)
{
}

MapDrawer::~MapDrawer()
{
    LOG(INFO) << "MapDrawer destructor called.";
}

void MapDrawer::drawCamera(pangolin::OpenGlMatrix &Twc, Color color)
{
    float CameraSize = 0.5;
    float mCameraLineWidth = 1;
    const float &w = CameraSize;
    const auto h = w*0.75f;
    const auto z = w*0.6f;

    glPushMatrix();
    glMultMatrixd(Twc.m);

    glLineWidth(mCameraLineWidth);
    if(color == red)
    {
        glColor3f(1.0f, 0.0f, 0.0f);
    }
    else if(color == green)
    {
        glColor3f(0.0f, 1.0f, 0.0f);
    }
    else if (color == blue)
    {
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

void MapDrawer::requestFinish()
{
    std::unique_lock<std::mutex> lock(mutexFinish_);
    finishRequested_ = true;
}

bool MapDrawer::checkFinish()
{
    std::unique_lock<std::mutex> lock(mutexFinish_);
    return finishRequested_;
}

// TODO Make it possible to option out viewing different poses and graphs.
void MapDrawer::run()
{
    pangolin::CreateWindowAndBind("Pose Viewer", 1024, 768);
    glEnable(GL_DEPTH_TEST);

    // Issue specific OpenGl we might need
    glEnable (GL_BLEND);
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::CreatePanel("menu").SetBounds(0.0,1.0,0.0,pangolin::Attach::Pix(175));
    pangolin::Var<bool> menuFollowCamera("menu.Follow Camera",true,true);
    pangolin::Var<bool>* menuShowGt;
    if(!gtPoses_.empty())
        menuShowGt = new pangolin::Var<bool>("menu.Show Ground Truth", true, true);

    // Define Camera Render Object (for view / scene browsing)
    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(1024,768,viewpointF_,viewpointF_,512,389,0.1,1000),
            pangolin::ModelViewLookAt(viewpointX_,viewpointY_,viewpointZ_, 0,0,0,0.0,-1.0, 0.0)
    );

    // Add named OpenGL viewport to window and provide 3D Handler
    auto *handler3D = new pangolin::Handler3D(s_cam);
    pangolin::View& d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0/768.0)
            .SetHandler(handler3D);

    // Matrix that changes where the camera is
    pangolin::OpenGlMatrix Tcw;
    Tcw.SetIdentity();

    bool bFollow = true;

    // TODO Fix so that viewer stops when pressing ESC.
    while(true /*!pangolin::ShouldQuit()*/)
    {
        // Clear screen and activate view to render into
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // Skip if there are no poses
        if(frames_.empty())
        {
            continue;
        }

        Tcw = pangolin::OpenGlMatrix(frames_.back().getPose().matrix());
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
        while(true)
        {
            std::unique_lock<std::mutex> lockGtsam(mutexPoses_);
            if(i >= frames_.size())
            {
                break;
            }

            bool isT1Ref = frames_.at(i - 1).isRef();
            bool isT2Ref = frames_.at(i - 1).isRef();

            T1Gtsam    = pangolin::OpenGlMatrix(frames_.at(i - 1).getPose().matrix());
            T2Gtsam    = pangolin::OpenGlMatrix(frames_.at(i).getPose().matrix());
            lockGtsam.unlock();
            if(i == 1)
            {
                drawCamera(T1Gtsam, blue);
            }

            if(isT2Ref)
            {
                drawCamera(T2Gtsam, blue);
            }
            else
            {
                drawCamera(T2Gtsam, green);
            }

            if(isT1Ref)
            {
                drawLines(T1Gtsam, T2Gtsam, blue);
            }
            else
            {
                drawLines(T1Gtsam, T2Gtsam, green);
            }

            if(!gtPoses_.empty() && *menuShowGt)
            {
                T1Gt = pangolin::OpenGlMatrix(gtPoses_.at(i - 1).matrix());
                T2Gt = pangolin::OpenGlMatrix(gtPoses_.at(i).matrix());
                if (i == 1)
                {
                    drawCamera(T1Gt, red);
                }
                drawCamera(T2Gt, red);
                drawLines(T1Gt, T2Gt, red);
            }
            i++;
        }

        // Swap frames and Process Events
        pangolin::FinishFrame();

        if(checkFinish())
        {
            break;
        }
    }
    pangolin::DestroyWindow("Pose Viewer");
    delete handler3D;
    if(menuShowGt != nullptr)
        delete menuShowGt;
    LOG(INFO) << "Exiting MapDrawer thread.";
}

void MapDrawer::drawLines(pangolin::OpenGlMatrix T1, pangolin::OpenGlMatrix T2, Color color)
{
    float mLineSize = 3;
    glLineWidth(mLineSize);

    if(color == red)
    {
        glColor3f(1.0f, 0.0f, 0.0f);
    }
    else if(color == green)
    {
        glColor3f(0.0f, 1.0f, 0.0f);
    }
    else if (color == blue)
    {
        glColor3f(0.0f, 0.0f, 1.0f);
    }

    glBegin(GL_LINES);
    glVertex3f(static_cast<float>(T1.m[12]), static_cast<float>(T1.m[13]), static_cast<float>(T1.m[14]));
    glVertex3f(static_cast<float>(T2.m[12]), static_cast<float>(T2.m[13]), static_cast<float>(T2.m[14]));
    glEnd();
}




