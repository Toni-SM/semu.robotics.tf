#include <iostream>
#include <math.h>
#include <thread>

#include <ros/ros.h>
#include <ros/console.h>
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
        std::vector<std::string> m_frameNames;
        std::vector<Frame> m_frames;

        std::thread m_thread;
        bool m_stop = true;
        void _run();

        Frame getFrame(const std::string&);
        void computeTransforms();

    public:
        RosTfListener(int, char**);
        ~RosTfListener();

        void start();
        void stop();
        void resetBuffer();
        void showFrame(const Frame&);
        
        std::vector<Frame> getFrames();
        std::vector<std::string> getFrameNames(const bool&);
        Frame getTransform(const std::string&, const std::string&);

        void setRootFrameName(const std::string&);
        
};

RosTfListener::RosTfListener(int argc=0, char **argv=NULL){

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

// private methods

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

void RosTfListener::computeTransforms(){
    std::vector<Frame> frames;
    for(const auto& name : getFrameNames(true)){
        frames.push_back(getFrame(name));
    }
    m_frames = frames;
}

void RosTfListener::_run(){
    ROS_DEBUG("[RosTfListener._run] Initializing ROS node");
    int argc = 0;
    ros::init(argc, NULL, "SemuRoboticsRosTfListener");
    ros::NodeHandle node;

    ROS_DEBUG("[RosTfListener._run] Create tf2 buffer and listener");
    m_tfBuffer = new tf2_ros::Buffer();
    m_tfTransformListener = new tf2_ros::TransformListener(*m_tfBuffer);
    ros::Duration duration(0.01);

    ROS_DEBUG("[RosTfListener._run] Start computation loop");
    while(!m_stop && ros::ok()){
        computeTransforms();
        duration.sleep();
    }
    ROS_DEBUG("[RosTfListener._run] Leave thread");
}

// public methods

void RosTfListener::start(){
    ROS_DEBUG("[RosTfListener.start] Starting thread..");
    m_stop = false;
    m_thread = std::thread(&RosTfListener::_run, this);
}

void RosTfListener::stop(){
    ROS_DEBUG("[RosTfListener.stop] Stopping thread..");
    m_stop = true;
    // wait for the thread to finish and join
    if(m_thread.joinable()){
        ROS_DEBUG("[RosTfListener.stop] Joining thread..");
        m_thread.join();
        ROS_DEBUG("[RosTfListener.stop] Thread joined");
    }
    ROS_DEBUG("[RosTfListener.stop] Stopped thread");
}

void RosTfListener::resetBuffer(){
    m_tfBuffer->clear();
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

Frame RosTfListener::getTransform(const std::string& targetFrame, const std::string& sourceFrame){
    Frame frame;
    // transform
    try{
        geometry_msgs::TransformStamped transform = m_tfBuffer->lookupTransform(targetFrame, sourceFrame, ros::Time(0));
        frame.translation.x = transform.transform.translation.x;
        frame.translation.y = transform.transform.translation.y;
        frame.translation.z = transform.transform.translation.z;
        frame.rotation.x = transform.transform.rotation.x;
        frame.rotation.y = transform.transform.rotation.y;
        frame.rotation.z = transform.transform.rotation.z;
        frame.rotation.w = transform.transform.rotation.w;
    } catch(const std::exception& e){
        frame.name = e.what();
        frame.parentName = e.what();
        frame.translation = {NAN, NAN, NAN};
        frame.rotation = {NAN, NAN, NAN, NAN};
    }
    return frame;
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
