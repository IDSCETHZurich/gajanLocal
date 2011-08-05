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
    invokeMoveTo(false), doLearning(true), doPlayBack(false)
    {
      int argc = 0;
      char* argv[0];
      log(Info)<<"Creating ROS node"<<endlog();
      ros::init(argc, argv, "Connector");
      n = new ros::NodeHandle();
      listener = new tf::TransformListener();
      client = n->serviceClient<re_srvs::getNextPose>("get_next_pose");
      this->addPort("msrCartPosPort", cartPosPort);
      this->addPort("msrJntPosPort", jntPosPort);
      this->addPort("RobotStatePort", RobotStatePort);
      this->addPort("FRIStatePort", FRIStatePort);
      this->addPort("cartPosCmdPort", cartPosCmdPort);
      this->addPort("jntPosCmdPort", jntPosCmdPort);

      this->addProperty("learningSpeedFactor",learningSpeedFactor).doc("Speed Factor for learning");
      this->addProperty("newPeriod4PlayBack",newPeriod4PlayBack).doc("New Period for play back");
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

	//wait while FRI Server + cartesianGenerator is running
	std::cout << "wait while FRI Server + cartesianGenerator is running" << endl;
	if(!this->hasPeer("cartesianGenerator") || !this->hasPeer("FRIServer"))
		{std::cout << "Did not find my two peers" << std::endl; return false;}


	isRunningFRIServer = this->getPeer("FRIServer")->getOperation("isRunning");
	isRunningcartesianGenerator = this->getPeer("cartesianGenerator")->getOperation("isRunning");
	do{
		if(isRunningFRIServer()) std::cout << "FRI Server running" << std::endl;
		else std::cout << "FRI Server NOT running" << std::endl;


		sleep(1);
	}while(!isRunningFRIServer() );//|| !isRunningcartesianGenerator());

	do{
		if(isRunningcartesianGenerator()){
			std::cout << "cartesianGenerator running" << std::endl;
		}
		else std::cout << "cartesianGenerator NOT running" << std::endl;

		sleep(1);
	}while(!isRunningcartesianGenerator());


	//Now FRIServer + cartesianGenerator is Running
 	std::cout << "Now FRIServer + cartesianGenerator is Running " << endl;

	//wait till FRI is in monitor mode
	std::cout << "wait till FRI is in monitor mode and the quality is perfect" << endl;
	FRIStatePort.read(friState);

	do{
		sleep(1);
		FRIStatePort.read(friState);
	}while(friState.state!=1 || friState.quality!=3);
	std::cout << "FRI is in monitor mode." << endl;

	RTT::OperationCaller<void(void)> resetPosition
						= this->getPeer("cartesianGenerator")->getOperation("resetPosition");
	resetPosition();

    // TODO:Wait for the robot to power up

	toKRLintAttr = this->getPeer("FRIServer")->getAttribute("toKRL");

	std::cout << "Resetting the FRI_FRM_INT[1] to 0 " << endl;
	//Resetting the FRI_FRM_INT[1] to 0  
	KRLintTmp.intData[1]=0;
	KRLintTmp.intData[0]=0;
	toKRLintAttr.set(KRLintTmp);

	sleep(1);
        
	std::cout << "Send [GOTO Command mode] to KRL" << endl;
	//Send [GOTO Command mode] to KRL
	
	KRLintTmp.intData[0]=1;
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

	//slow down velocity for learning
	maxVel_property = this->getPeer("cartesianGenerator")->getProperty("max_vel");
	maxVel = maxVel_property.get();
	if(learningSpeedFactor < 1){
		std::cout << "learningSpeedFactor should be greater or equal to 1" << std::endl;
		return false;
	}

	for(int i=0;i<(int)maxVel.size();i++)
		maxVel[i]/= learningSpeedFactor;
	maxVel_property.set(maxVel);

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
	
	if(doLearning){
		cartPosPort.read(cartPosMsr);

		chatter_pub.publish(cartPosMsr);
		srv.request.currentPose = cartPosMsr;

		if (client.call(srv)){
			cartPosCmdPort.write(srv.response.commandPose);
			//record measured position
			jntPosPort.read(measuredJntPos);
			measuredJntState.position.clear();
			for(int i=0; i < 7; i++){
				measuredJntState.position.push_back(measuredJntPos[i]);
			}
			recordedTrajectory.push_back(measuredJntState);

			if(srv.response.learningFinished){
				//Increase the velocity
				std::cout << "recorded poses: " << recordedTrajectory.size() << std::endl;
				trajectoryPointer = recordedTrajectory.size()-1-1; //0-offset + command to the last pose
				playDirection=-1;
				doLearning = false; doPlayBack = true;

				//Increase Velocity for PlayBack
				std::cout << "Maximum Velocities" << std::endl;
				for(int i=0;i<(int)maxVel.size();i++){
					std::cout << "OldJoint" << i << ": " << maxVel[i];
					maxVel[i]*= learningSpeedFactor;
					std::cout << " ==> NewJoint" << i << ": " << maxVel[i] << std::endl;
				}
				maxVel_property.set(maxVel);
				std::cout << "MaxVel Property Set" << std::endl;

				//Update @ faster rates
				//this->stop();
				std::cout << "Period for learning: " << this->getPeriod() << std::endl;
				if (this->setPeriod(newPeriod4PlayBack))
					std::cout << "Period for playback: " << this->getPeriod() << std::endl;
				else
					std::cout << "The period could not be changed" << std::endl;

			}
		}else{
			std::cout << "getNextPose service call failed " << std::endl;
		}
	}//end of doLearning

	if(doPlayBack){
		jntPosCmdPort.write(recordedTrajectory[trajectoryPointer]);

		if(trajectoryPointer==0){
			playDirection=+1;
			std::cout << "-----" << "Start Ripping" << std::endl;
		}

		if(trajectoryPointer==(int)recordedTrajectory.size()-1){
			playDirection=-1;
		}

		std::cout << "trajectoryPointer" << trajectoryPointer << std::endl;

		trajectoryPointer+=playDirection;

	}//end of doPlayBack

  }

  void Connector::stopHook()
  {
	//Changing the update rate of the movement for the playback
/*	std::cout << "Period for learning: " << this->getPeriod() << std::endl;
	if (this->setPeriod(newPeriod4PlayBack))
		std::cout << "Period for playback: " << this->getPeriod() << std::endl;
	else
		std::cout << "The period could not be changed" << std::endl;
*/  }


  void Connector::cleanupHook()
  {
	//this will stop & close FRI
	KRLintTmp.intData[1] = 1;
	toKRLintAttr.set(KRLintTmp); 
	sleep(5);
  }

}//end of name-space re_articulationOROCOS_coupling
