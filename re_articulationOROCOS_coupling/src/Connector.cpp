#include "Connector.hpp"

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
    qx(0.0),
    qy(0.0),
    qz(0.0),
    qw(0.0),
    invokeMoveTo(false), doRecord(true), doPlay(false),
    KRLintTmp(vector<int32_t>(FRI_USER_SIZE))
    //toKRLintAttr("toKRLint",KRLintTmp)
    {
      int argc = 0;
      char* argv[0];
      log(Info)<<"Creating ros node"<<endlog();
      ros::init(argc, argv, "Connector");
      n = new ros::NodeHandle();
      listener = new tf::TransformListener();
      client = n->serviceClient<re_srvs::getNextPose>("get_next_pose");
      this->addPort("msrCartPosPort", cartPosPort);
      this->addPort("msrJntPosPort", jntPosPort);
      this->addPort("RobotStatePort", RobotStatePort);
      this->addPort("FRIStatePort", FRIStatePort);

      this->addPort("cartPosCmdPort", cartPosCmdPort);
    }

    Connector::~Connector()
    {
      delete n;
      delete listener;
    }

  bool Connector::configureHook(){
    chatter_pub   = n->advertise<geometry_msgs::PoseStamped>("tool_pose", 1);
    joint_pos_pub = n->advertise<sensor_msgs::JointState>("joint_positions_connector", 1);

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
/*	toKRLintAttr = this->getPeer("FRIServer")->attributes()->getAttribute("toKRLint");

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
	FRIStatePort.read(friState);

	do{
		sleep(1);
		FRIStatePort.read(friState);
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
	FRIStatePort.read(friState);
	while(friState.state == 2){
		sleep(1);
		FRIStatePort.read(friState);
	}

	//All configuration OK
	std::cout << "All configuration OK !!" << endl;
*/
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
	FRIStatePort.read(friState);
	RobotStatePort.read(robotState);
	invokeMoveTo = (friState.state == 2)&&(robotState.power==127);

    cartPosPort.read(cartPosMsr);
	
	chatter_pub.publish(cartPosMsr);
	srv.request.currentPose = cartPosMsr;
	
	if (client.call(srv)){
		cartPosCmdPort.write(srv.response.commandPose);
	}else{
		//std::cout << "getNextPose service call failed " << std::endl;
	}

  }

  void Connector::stopHook()
    { 
	//this will stop & close FRI
	KRLintTmp[1] = 1; 
	toKRLintAttr.set(KRLintTmp); 
	sleep(5);
	
    }
 
}//end of name-space re_articulationOROCOS_coupling



