#ifndef ORBSLAM_H
#define ORBSLAM_H
#include <GSLAM/core/GSLAM.h>
#include "Map.h"
#include "Tracking.h"
#include "Map.h"
#include "LocalMapping.h"
#include "LoopClosing.h"
#include "KeyFrameDatabase.h"
#include "ORBVocabulary.h"
#include "Converter.h"

namespace GSLAM{

class ORBSLAM
{
public:
    ORBSLAM();
    virtual ~ORBSLAM();
    virtual std::string type()const{return "ORBSLAM";}
    virtual bool valid()const;

    virtual bool track(FramePtr& frame);

    virtual void draw();

    virtual void call(const std::string& command,void* arg=NULL);

    virtual void run();

    virtual bool isDrawable()const{return false;}

    void flushMap();

private:
    std::shared_ptr<ORB_SLAM::ORBVocabulary>           Vocabulary;
    std::shared_ptr<ORB_SLAM::KeyFrameDatabase>        Database;
    std::shared_ptr<ORB_SLAM::Map>                     World;
    std::shared_ptr<ORB_SLAM::Tracking>                Tracker;
    std::shared_ptr<ORB_SLAM::LocalMapping>            LocalMapper;
    std::shared_ptr<ORB_SLAM::LoopClosing>             LoopCloser;
    Subscriber                                         subFlushMap;
    Publisher                                          pubMap,pubCurFrame;
};

}
#endif // ORBSLAM_H
