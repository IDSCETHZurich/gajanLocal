#include "Coupling.hpp"

ORO_CREATE_COMPONENT(re_articulationOROCOS_coupling::Connector);

#define PI 3.141592653

namespace re_articulationOROCOS_coupling
{
  using namespace RTT;
  using namespace KDL;
  using namespace std;

  Connector::Connector(string name)
  : TaskContext(name),
    msg(),
    cartPosMsr(),
    cartPosCmd(),
    vecKDLFrames(),
    qx(0.0),
    qy(0.0),
    qz(0.0),
    qw(0.0),
    trajectorySize(0),trajectoryPointer(0),
    RobotStatePort("RobotState"),
    FriStatePort("FriState"),
    cartPosMsrPort("LWR_Pos_Msr"),
    jntPosPort("msrJntPos"),
    jntPosCmdPort("LWR_Pos_Cmd"),
    invokeMoveTo(false), doRecord(true), doPlay(false),
    playDirection(-1),
    KRLintTmp(vector<int32_t>(FRI_USER_SIZE)),
    toKRLintAttr("toKRLint",KRLintTmp)
    {
      int argc = 0;
      char* argv[0];
      log(Info)<<"Creating ros node"<<endlog();
      ros::init(argc, argv, "Connector(Orocos-LWR)");
      n = new ros::NodeHandle();
      listener = new tf::TransformListener();
      client = n->serviceClient<re_srvs::getNextPose>("get_next_pose");
      this->addPort("cartPosMsrPort", cartPosMsrPort);
      this->addPort("RobotStatePort", RobotStatePort);
      this->addPort("FriStatePort", FriStatePort);
      this->addPort("jntPosPort", jntPosPort);

      this->addPort("jntPosCmdPort", jntPosCmdPort);
    }

    Connector::~Connector()
    {
      delete n;
      delete listener;
    }

  bool Connector::configureHook(){
    chatter_pub   = n->advertise<geometry_msgs::PoseStamped>("tool_pose", 1);
    joint_pos_pub = n->advertise<sensor_msgs::JointState>("joint_positions", 1);

    // Initialize joint names
    jntStateMsg.header.frame_id = "arm_0_link";
    jntStateMsg.name.push_back("arm_1_joint");
    jntStateMsg.name.push_back("arm_2_joint");
    jntStateMsg.name.push_back("arm_3_joint");
    jntStateMsg.name.push_back("arm_4_joint");
    jntStateMsg.name.push_back("arm_5_joint");
    jntStateMsg.name.push_back("arm_6_joint");
    jntStateMsg.name.push_back("arm_7_joint");
    
    return true;
  } 
  bool Connector::startHook()
    {   
	toKRLintAttr = this->getPeer("FRIServer")->attributes()->getAttribute("toKRLint");

	//wait while FRI Server + trajectoryGeneratorJntPos is running
	std::cout << "wait while FRI Server + trajectoryGeneratorJntPos is running" << endl;
	if(!this->hasPeer("trajectoryGeneratorJntPos") || !this->hasPeer("FRIServer"))
		{std::cout << "Did not find my two peers"; return false;}
	isRunningFRIServer = this->getPeer("FRIServer")->getOperation("isRunning");
	isRunningtrajectoryGeneratorJntPos = this->getPeer("trajectoryGeneratorJntPos")->getOperation("isRunning");
	do{
		sleep(1);	
	}while(!isRunningFRIServer() || !isRunningtrajectoryGeneratorJntPos());
	//Now FRIServer + trajectoryGeneratorJntPos is Running
 	std::cout << "Now FRIServer + trajectoryGeneratorJntPos is Running " << endl;

	//wait till FRI is in monitor mode
	std::cout << "wait till FRI is in monitor mode and the quality is perfect" << endl;
	FriStatePort.read(friState);

	do{
		sleep(1);
		FriStatePort.read(friState);
	}while(friState.state!=1 || friState.quality!=3);
	std::cout << "FRI is in monitor mode." << endl;

    // Wait for the robot to power up
	std::cout << "wait for the robot to power up" << endl;

	std::cout << "Resetting the FRI_FRM_INT[1] to 0 " << endl;
	//Resetting the FRI_FRM_INT[1] to 0  
	KRLintTmp[1] = 0; 	
	KRLintTmp[0] = 0; 
        toKRLintAttr.set(KRLintTmp);
        
	std::cout << "Send [GOTO Command mode] to KRL" << endl;
	//Send [GOTO Command mode] to KRL
	
    KRLintTmp[0] = 1;
	toKRLintAttr.set(KRLintTmp);

	std::cout << "Wait till command mode " << endl;
	//Wait till command mode 
	FriStatePort.read(friState);
	while(friState.state == 2){
		sleep(1);
		FriStatePort.read(friState);
	}

	//All configuration OK
	std::cout << "All configuration OK !!" << endl;

	return true;
    }   
  void Connector::updateHook(){  

	// Publish joint angles
	jntPosPort.read(jntPosArray);
	jntStateMsg.position.clear();
	for(i=0;i<LBR_MNJ;i++){
		jntStateMsg.position.push_back(jntPosArray[i]);
	}
	joint_pos_pub.publish(jntStateMsg);
      
   //get moveTo parameters from ROS and send it to FRI, if the command is executed publish the result as a ros topic.
   //Publish from LWR to ROS
	FriStatePort.read(friState);
	RobotStatePort.read(robotState);
	invokeMoveTo = (friState.state == 2)&&(robotState.power==127);

    cartPosMsrPort.read(cartPosMsr);

	cartPosMsr.M.GetQuaternion(qx,qy,qz,qw);

	msg.header.stamp = ros::Time::now();
	msg.header.frame_id = "arm_0_link";
	msg.pose.position.x = cartPosMsr.p[0];
	msg.pose.position.y = cartPosMsr.p[1];
	msg.pose.position.z = cartPosMsr.p[2];
	msg.pose.orientation.x = qx;
	msg.pose.orientation.y = qy;
	msg.pose.orientation.z = qz;
	msg.pose.orientation.w = qw;
	
	chatter_pub.publish(msg);
    //if(moveToCommand.done() && invokeMoveTo){
	  //resetting the command
	  //moveToCommand = this->getPeer("trajectoryGeneratorJntPos")->commands()->getCommand<bool(KDL::Frame,double)>("moveTo");

	//} //end of if executed
	

	//if(moveToCommand.ready() && invokeMoveTo)
	{
	  cartPosMsrPort.read(cartPosMsr);
	  cartPosMsr.M.GetQuaternion(qx,qy,qz,qw);

          msg.header.stamp = ros::Time::now();
          msg.header.frame_id = "arm_0_link";
          msg.pose.position.x = cartPosMsr.p[0];
          msg.pose.position.y = cartPosMsr.p[1];
          msg.pose.position.z = cartPosMsr.p[2];
	
	  msg.pose.orientation.x = qx; 
          msg.pose.orientation.y = qy;
          msg.pose.orientation.z = qz;
          msg.pose.orientation.w = qw;
	  srv.request.currentPose = msg.pose;

	}//end of if ready  
  }

  void Connector::stopHook()
    { 
	//this will stop & close FRI
	KRLintTmp[1] = 1; 
	toKRLintAttr.set(KRLintTmp); 
	sleep(5);
	
    }
 
}//end of name-space re_articulationOROCOS_coupling



