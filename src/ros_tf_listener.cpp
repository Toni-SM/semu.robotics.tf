#include <iostream>
#include <math.h>

#include "ros/ros.h"
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

using namespace std;


typedef struct{
    std::string name;
    std::string parentName;
    struct{
        double x;
        double y;
        double z;
    } translation;
    struct{
        double x;
        double y;
        double z;
        double w;
    } rotation;
} Frame;


class RosTfListener{
    private:
        tf2_ros::Buffer *m_tfBuffer = NULL;
        tf2_ros::TransformListener *m_tfTransformListener = NULL;
        std::string m_rootFrameName = "world";
        std::vector<Frame> m_frames;
        std::vector<std::string> m_frameNames;

        Frame getFrame(const std::string&);

    public:
        RosTfListener(int, char**);
        ~RosTfListener();

        void reset();
        void showFrame(const Frame&);
        void computeTransforms();
        
        std::vector<Frame> getFrames();
        std::vector<std::string> getFrameNames(const bool&);

        void setRootFrameName(const std::string&);
        
};

RosTfListener::RosTfListener(int argc=0, char **argv=NULL){
    ros::init(argc, argv, "SemuRoboticsRosTfListener");
    ros::NodeHandle node;

    m_tfBuffer = new tf2_ros::Buffer();
    m_tfTransformListener = new tf2_ros::TransformListener(*m_tfBuffer);
    // ros::Duration(1.0).sleep();
}

RosTfListener::~RosTfListener(){
    // if(m_tfBuffer)
    //     delete m_tfBuffer;
    // m_tfBuffer = NULL;
    // if(m_tfTransformListener)
    //     delete m_tfTransformListener;
    // m_tfTransformListener = NULL;

    // ros::shutdown();
}

Frame RosTfListener::getFrame(const std::string& name){
    Frame frame;
    // name
    frame.name = name;
    // parent name
    bool hasParent = m_tfBuffer->_getParent(frame.name, ros::Time(), frame.parentName);
    if(!hasParent)
        frame.parentName.clear();
    // transform
    try{
        geometry_msgs::TransformStamped transform = m_tfBuffer->lookupTransform(m_rootFrameName, frame.name, ros::Time(0));
        frame.translation.x = transform.transform.translation.x;
        frame.translation.y = transform.transform.translation.y;
        frame.translation.z = transform.transform.translation.z;
        frame.rotation.x = transform.transform.rotation.x;
        frame.rotation.y = transform.transform.rotation.y;
        frame.rotation.z = transform.transform.rotation.z;
        frame.rotation.w = transform.transform.rotation.w;
    } catch(...){
        frame.translation = {NAN, NAN, NAN};
        frame.rotation = {NAN, NAN, NAN, NAN};
    }
    return frame;
}

void RosTfListener::reset(){
    m_tfBuffer->clear();
}

void RosTfListener::computeTransforms(){
    std::vector<Frame> frames;
    for(const auto& name : getFrameNames(true)){
        frames.push_back(getFrame(name));
    }
    m_frames = frames;
}

void RosTfListener::showFrame(const Frame& frame){
    cout<<"Frame: "<<frame.name<<endl;
    cout<<"  |-- parent: "<<frame.parentName<<endl;
    cout<<"  |-- translation: "<<frame.translation.x<<", "<<frame.translation.y<<", "<<frame.translation.z<<endl;
    cout<<"  |-- rotation: "<<frame.rotation.x<<", "<<frame.rotation.y<<", "<<frame.rotation.z<<", "<<frame.rotation.w<<endl;
}

std::vector<Frame> RosTfListener::getFrames(){
    return m_frames;
}

std::vector<std::string> RosTfListener::getFrameNames(const bool& refresh=true){
    if(refresh){
        m_frameNames.clear();
        m_tfBuffer->_getFrameStrings(m_frameNames);
    }
    return m_frameNames;
}

void RosTfListener::setRootFrameName(const std::string& frameName){
    m_rootFrameName = frameName;
}


#ifdef EXECUTABLE
int main(int argc, char **argv){
    RosTfListener listener = RosTfListener(argc, argv);

    listener.computeTransforms();

    for(const auto& frame : listener.getFrames()){
        listener.showFrame(frame);
    }
    return 0;
}
#endif

#ifdef CTYPES
extern "C"
{
    RosTfListener * rosTfListener(){ 
		return new RosTfListener(0, NULL); 
	}
}
#endif
