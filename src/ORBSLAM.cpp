#include "ORBSLAM.h"

#include <iostream>
#include <fstream>

#include <boost/thread.hpp>

#include <opencv2/core/core.hpp>
#include <GSLAM/core/Timer.h>
#include <GSLAM/core/HashMap.h>
#include <GSLAM/core/Event.h>

using namespace pi;
namespace GSLAM{

class ORBFrame: public MapFrame{
public:
    ORBFrame(ORB_SLAM::KeyFrame* fr)
        : MapFrame(fr->mnId,fr->mTimeStamp){
        update(fr);
    }

    void update(ORB_SLAM::KeyFrame* fr){
        cv::Mat pose_cv=fr->GetPose();
        pi::SE3f pose;
        pose.fromMatrix((float*)pose_cv.data);
        setPose(pose.inverse());
        depth=fr->ComputeSceneMedianDepth();
    }

    virtual double getMedianDepth(){return depth;}

    float depth;
};

class ORBMapPoint: public MapPoint{
public:
    ORBMapPoint(ORB_SLAM::MapPoint* pt): MapPoint(pt->mnId,mat2p3d(pt->GetWorldPos())){

    }

    void update(ORB_SLAM::MapPoint* pt){
        setPose(mat2p3d(pt->GetWorldPos()));
    }

    static Point3d mat2p3d(cv::Mat mat){
        return Point3d(mat.at<float>(0),mat.at<float>(1),mat.at<float>(2));
    }

};

void ScommandCallback(void* ptr, std::string sCommand, std::string sParams)
{
    ORBSLAM* slam=(ORBSLAM*)ptr;
    slam->flushMap();
}

void ORBSLAM::flushMap()
{
    if(!handle) return;

    MapPtr _map(new HashMap());
    vector<ORB_SLAM::KeyFrame*> keyframes=World->GetAllKeyFrames();
    vector<ORB_SLAM::MapPoint*> mappoints=World->GetAllMapPoints();

    GSLAM::FramePtr curFrame;
    for(ORB_SLAM::KeyFrame* fr:keyframes){
        SPtr<ORBFrame> frame=std::static_pointer_cast<ORBFrame>(_map->getFrame(fr->mnId));
        if(frame) frame->update(fr);
        else frame=SPtr<ORBFrame>(new ORBFrame(fr));
        _map->insertMapFrame(frame);
        if(!curFrame||frame->id()>curFrame->id()) curFrame=frame;
    }

    for(ORB_SLAM::MapPoint* kp:mappoints){
        PointPtr pt=_map->getPoint(kp->mnId);
        if(pt) pt->setPose(ORBMapPoint::mat2p3d(kp->GetWorldPos()));
        else  _map->insertMapPoint(PointPtr(new ORBMapPoint(kp)));
    }

    setMap(_map);
    handle->handle(_map);

    if(curFrame){
        handle->handle(curFrame);
        handle->handle(new CurrentFrameEvent(curFrame));
    }
}

ORBSLAM::ORBSLAM()
{
    using namespace ORB_SLAM;
    scommand.RegisterCommand("MapFlush",ScommandCallback,this);
    //Load ORB Vocabulary
    string strVocFile = svar.GetString("ORBVocabularyFile", "./Data/GSLAM/ORBvoc.yml");
    cout << endl << "Loading ORB Vocabulary: " << strVocFile << " ...";
    Vocabulary = SPtr<ORBVocabulary>(new ORBVocabulary());
    Vocabulary->loadFast(strVocFile);
    cout << endl;

    //Create KeyFrame Database
    Database = SPtr<KeyFrameDatabase>(new KeyFrameDatabase(*Vocabulary));

    //Create the map
    World = SPtr<ORB_SLAM::Map>(new ORB_SLAM::Map);

    //Initialize the Tracking Thread and launch
    Tracker = SPtr<Tracking>(new Tracking(Vocabulary.get(), World.get()));
    Tracker->SetKeyFrameDatabase(Database.get());

    //Initialize the Local Mapping Thread and launch
    LocalMapper = SPtr<LocalMapping>(new LocalMapping(World.get()));

    //Initialize the Loop Closing Thread and launch
    LoopCloser = SPtr<LoopClosing>(new LoopClosing(World.get(), Database.get(), Vocabulary.get()));


    //Set pointers between threads
    Tracker->SetLocalMapper(LocalMapper.get());
    Tracker->SetLoopClosing(LoopCloser.get());

    LocalMapper->SetTracker(Tracker.get());
    LocalMapper->SetLoopCloser(LoopCloser.get());

    LoopCloser->SetTracker(Tracker.get());
    LoopCloser->SetLocalMapper(LocalMapper.get());

    setMap(MapPtr(new HashMap()));
    run();
}

ORBSLAM::~ORBSLAM()
{
    LocalMapper->RequestStop();
    LoopCloser->stopRequested=true;

    while(!LocalMapper->isStopped()||!LoopCloser->isStoped){
        GSLAM::Rate::sleep(0.01);
    }
    LOG(ERROR)<<"Stoped.";
}

void ORBSLAM::run()
{

    boost::thread localMappingThread(&ORB_SLAM::LocalMapping::Run, LocalMapper.get());

    boost::thread loopClosingThread(&ORB_SLAM::LoopClosing::Run, LoopCloser.get());

    // running map cleanup thread
//    boost::thread mapClearnThread(&ORB_SLAM::Map::Run, World);

}

bool ORBSLAM::track(FramePtr &frame)
{
    if(!frame) return false;
    bool ret= Tracker->track(frame);
    return ret;
}

bool ORBSLAM::valid()const
{
    return Vocabulary.get()&&Database.get()&&LocalMapper.get()&&LoopCloser.get()&&World.get()&&Tracker.get();
}

void ORBSLAM::call(const std::string& command,void* arg)
{
    if(command=="Start")
    {
        run();
    }
    else if(command=="Stop")
    {
    }
    else if(command=="IsRunning"&&arg)
    {
    }

}

void ORBSLAM::draw()
{
    if(!valid()) return;
    Tracker->Draw_Something();
    World->Draw_Something();
}

}

USE_GSLAM_PLUGIN(GSLAM::ORBSLAM);
