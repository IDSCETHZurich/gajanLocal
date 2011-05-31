#include <rtt/RTT.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Property.hpp>
#include <rtt/PropertyBag.hpp>
#include <kdl/frames.hpp>
#include <ocl/Component.hpp>
#include <tf/transform_listener.h>
#include <re_srvs/getNextPose.h>
#include <friComm.h>


#include <ros/ros.h>
#include <sensor_msgs/JointState.h>


namespace re_articulationOROCOS_coupling
{
 
  class Connector
    : public RTT::TaskContext
  {
    
  public:
    Connector(std::string name);
    virtual ~Connector();
    void cartDesCallback(const geometry_msgs::PoseConstPtr&  cartDes);
    
    
  private:
    virtual void updateHook();  
    virtual bool startHook();
    virtual bool configureHook();
    virtual void stopHook();

    int i;
    ros::NodeHandle* n;
    ros::Publisher chatter_pub;
    ros::Publisher joint_pos_pub;

    ros::ServiceClient client;
    re_srvs::getNextPose srv;

    geometry_msgs::Pose msg;
    sensor_msgs::JointState jntStateMsg;
    geometry_msgs::Pose cartPosMsr,cartPosCmd;
    double qx,qy,qz,qw;
    std::vector<double> jntPosArray;
    tf::TransformListener* listener;
    tf::StampedTransform transform;

    tFriRobotState robotState;
    tFriIntfState friState;

    RTT::OperationCaller<bool(void)> isRunningFRIServer;
    RTT::OperationCaller<bool(void)> isRunningtrajectoryGeneratorJntPos;

    /*********
    DATAPORTS
    *********/
        
    RTT::InputPort<tFriRobotState> RobotStatePort;
    RTT::InputPort<tFriIntfState> FRIStatePort;
    RTT::InputPort<geometry_msgs::Pose> cartPosPort;
    RTT::InputPort<std::vector<double> > jntPosPort;
    RTT::OutputPort<geometry_msgs::Pose> cartPosCmdPort;

    

    bool invokeMoveTo;
    bool doRecord; 
    bool doPlay;
    int  playDirection; // 1 open ... -1 close 

    std::vector<int32_t> KRLintTmp;
    //Attributes
    RTT::Attribute<std::vector<int32_t> > toKRLintAttr;

    
  };
};//namespace

