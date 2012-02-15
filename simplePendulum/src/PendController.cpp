#include "PendController.hpp"
#include <ocl/Component.hpp>
ORO_CREATE_COMPONENT_TYPE();
ORO_LIST_COMPONENT_TYPE(simplePendulum::PendController);
//ORO_CREATE_COMPONENT(simplePendulum::PendController);

using namespace std;

namespace simplePendulum
{

    using namespace RTT;
    using namespace KDL;
    using namespace std;

    const double PendController::gainLQR [] =
    	{ -10.7359,  -34.3973,   -2.6273,   -1.2995,  -10.7359,  -34.3973,   -2.6273,   -1.2995};


    PendController::PendController(string name)
        : TaskContext(name,PreOperational)
    {
    	//Hardcoding values. (Herman Alert!)
    	originX = 0.250;
    	originY = 0.0;
    	originZ = 0.830;

    	Xr = 0.30;
    	Yr = 0.30;

    	s_t = std::vector<double>(8, 0.0);
    	s_tm1 = std::vector<double>(8, 0.0); //not used now

    	u_t = std::vector<double>(2, 0.0);
    	u_tm1 = std::vector<double>(2, 0.0);//not used now

    	xr_old = 0.0;
    	yr_old = 0.0;

        //Adding Ports
    	this->addPort("CartesianPoseOutput", m_position_desi);
    	this->addPort("pendPos_inputPort", pendPos_inputPort);
    	this->addPort("pendVel_inputPort", pendVel_inputPort);
    	this->addPort("robotPos_inputPort", robotPos_inputPort);
    	this->addOperation("moveToInitialPose",&PendController::moveToInitialPose,this,OwnThread);

    	//Butterworth - Lowpass derivatives
    	xrdot_est = std::vector<double>(NPOLES+1, 0.0);
    	xrdot = std::vector<double>(NZEROS+1, 0.0);
    	yrdot_est = std::vector<double>(NPOLES+1, 0.0);
    	yrdot = std::vector<double>(NZEROS+1, 0.0);

    	stateLogger.open("stateLog.txt");

    	//debugging
    	t = 0.0;
    	commandedState = std::vector<double>(13, 0.0);
    }

    PendController::~PendController()
    {

    }

    bool PendController::configureHook()
    {
		return true;
    }

    bool PendController::startHook()
    {
    	dT = this->getPeriod();
		return true;

    }


    void PendController::updateHook()
    {
    	robotPos_inputPort.read(poseCurrent);
    	//s_t[2] = butterWorthLowpass((poseCurrent.position.x - originX - s_t[3])/dT, xrdot, xrdot_est);
    	//s_t[6] = butterWorthLowpass((poseCurrent.position.y - originY - s_t[7])/dT, yrdot, yrdot_est);
    	s_t[2] = (poseCurrent.position.x - originX - s_t[3])/dT;
    	s_t[6] = (poseCurrent.position.y - originY - s_t[7])/dT;
    	s_t[3] = poseCurrent.position.x -  originX;
    	s_t[7] = poseCurrent.position.y -  originY;

    	pendPos_inputPort.read(filteredPos);
    	s_t[1] = filteredPos.x;
    	s_t[5] = filteredPos.y;

    	pendVel_inputPort.read(filteredVel);
    	s_t[0] = filteredVel.x;
    	s_t[4] = filteredVel.y;

    	//controller (spits out xr, yr for the robot)
    	u_t[0] = 0.0; u_t[1]=0.0;
    	for(int i=0; i<4;i++){
    		u_t[0] += -gainLQR[i]*s_t[i];
    	}
    	for(int i=4; i<8;i++){
			u_t[1] += -gainLQR[i]*s_t[i];
		}

    	//updating position and velocity (order matters !) s_(t+1)
    	xr = s_t[3] + 0.5 * dT * (2 * s_t[2] + dT * u_t[0]); // xr_tplus1
    	yr  = s_t[7] + 0.5 * dT * (2 * s_t[6] + dT * u_t[1]); // yr_tplus1

    	stateLogger << s_t[0] << " " << s_t[1] << " " << s_t[2] << " " << s_t[3] << " " << s_t[4] << " " << s_t[5] << " " <<
    	    			s_t[6] << " " << s_t[7]  <<  " " << u_t[0] << " " << u_t[1] << " " << xr << " " << yr << endl;

    	//check for sanity
        if(abs(xr)>Xr || abs(yr)>Yr){
        	cout << "PendController::Commanded Value out of Range" << endl;
        		cout << "	xr = " << xr << endl;
        		cout << "	yr = " << yr << endl;
        	//this->stop();
        	xr = 0;
        	yr = 0;
        }

		//Send Pose to Robot
		geometry_msgs::Pose pose;

		//Always point up
		pose.orientation.x = 0.0;
		pose.orientation.y = 0.0;
		pose.orientation.z = 0.0;
		pose.orientation.w = 1.0;

		pose.position.x = originX + xr;
		pose.position.y = originY + yr;
		pose.position.z = originZ;

		commandedState[0]=pose.position.x;
		commandedState[1]=pose.position.y;
		commandedState[2]=pose.position.z;
		commandedState[3]=pose.orientation.x;
		commandedState[4]=pose.orientation.y;
		commandedState[5]=pose.orientation.z;
		commandedState[6]=pose.orientation.w;

		commandedState[7] = s_t[2];
		commandedState[8] = s_t[6];
		commandedState[9] = 0.0;
		commandedState[10] = 0.0;
		commandedState[11] = 0.0;
		commandedState[12] = 0.0;

		m_position_desi.write(commandedState);
    }

/*
    void PendController::updateHook()
    {
    	robotPos_inputPort.read(poseCurrent);

    	xr_msr = poseCurrent.position.x -  originX;
    	yr_msr = poseCurrent.position.y -  originY;

    	xr_comm = 0.0; //0.2*sin(3.14*t);
    	yr_comm = 0.2*cos(3.14*t);
    	xrdot_comm = 0.0; //(0.2*3.14)*cos(3.14*t);
    	yrdot_comm = -(0.2*3.14)*sin(3.14*t);

    	t += dT;

    	stateLogger << xr_comm << " " << xr_msr << " " << yr_comm << " " << yr_msr  << endl;

		//Send Pose to Robot
		geometry_msgs::Pose pose;

		//Always point up
		pose.orientation.x = 0.0;
		pose.orientation.y = 0.0;
		pose.orientation.z = 0.0;
		pose.orientation.w = 1.0;

		pose.position.x = originX + xr_comm;
		pose.position.y = originY + yr_comm;
		pose.position.z = originZ;

    	commandedState[0]=pose.position.x;
    	commandedState[1]=pose.position.y;
    	commandedState[2]=pose.position.z;
    	commandedState[3]=pose.orientation.x;
    	commandedState[4]=pose.orientation.y;
    	commandedState[5]=pose.orientation.z;
    	commandedState[6]=pose.orientation.w;

    	commandedState[7] = xrdot_comm;
    	commandedState[8] = yrdot_comm;
    	commandedState[9] = 0.0;
    	commandedState[10] = 0.0;
    	commandedState[11] = 0.0;
    	commandedState[12] = 0.0;

		m_position_desi.write(commandedState);
    }*/

    void PendController::stopHook()
    {
    	stateLogger.close();
    }

    void PendController::cleanupHook()
    {
    }

    bool PendController::moveToInitialPose(){
		geometry_msgs::Pose pose;

		cout << "moveToInitialPose starting .. " << endl;

		// Always point up
		pose.orientation.x = 0.0;
		pose.orientation.y = 0.0;
		pose.orientation.z = 0.0;
		pose.orientation.w = 1.0;

		pose.position.x = originX;
		pose.position.y = originY;
		pose.position.z = originZ;

    	commandedState[0]=pose.position.x;
    	commandedState[1]=pose.position.y;
    	commandedState[2]=pose.position.z;
    	commandedState[3]=pose.orientation.x;
    	commandedState[4]=pose.orientation.y;
    	commandedState[5]=pose.orientation.z;
    	commandedState[6]=pose.orientation.w;

    	commandedState[7] = 0.0;
    	commandedState[8] = 0.0;
    	commandedState[9] = 0.0;
    	commandedState[10] = 0.0;
    	commandedState[11] = 0.0;
    	commandedState[12] = 0.0;

		m_position_desi.write(commandedState);
		cout << "command sent to KUKA_IK" << endl;

		return true;
    }

    double PendController::butterWorthLowpass(double input, std::vector<double> &x, std::vector<double> &x_est){
		x[0] = x[1]; x[1] = x[2]; x[2] = x[3]; x[3] = x[4];
		x[4] = input / GAIN;
		x_est[0] = x_est[1]; x_est[1] = x_est[2]; x_est[2] = x_est[3]; x_est[3] = x_est[4];
        x_est[4] =   (x[0] + x[4]) + 4 * (x[1] + x[3]) + 6 * x[2]
                     + ( -0.1873794924 * x_est[0]) + ( -1.0546654059 * x_est[1])
                     + ( -2.3139884144 * x_est[2]) + ( -2.3695130072 * x_est[3]);
		return x_est[4];
    }
}//namespace
