#include "ORBSLAM.h"

#include <iostream>
#include <fstream>

#include <boost/thread.hpp>

#include <opencv2/core/core.hpp>
#include <GSLAM/core/Timer.h>

using namespace pi;
using namespace ORB_SLAM;
namespace GSLAM{


ORBSLAM::ORBSLAM()
{
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
    run();
}

void ORBSLAM::run()
{

    boost::thread localMappingThread(&ORB_SLAM::LocalMapping::Run, LocalMapper.get());

    boost::thread loopClosingThread(&ORB_SLAM::LoopClosing::Run, LoopCloser.get());

    // running map cleanup thread
    boost::thread mapClearnThread(&ORB_SLAM::Map::Run, World);

}

bool ORBSLAM::track(FramePtr &frame)
{
    return Tracker->track(frame);
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
