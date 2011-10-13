#include "PendController.hpp"
#include <ocl/Component.hpp>
ORO_CREATE_COMPONENT(simplePendulum::PendController);

using namespace std;

namespace simplePendulum
{

    using namespace RTT;
    using namespace KDL;
    using namespace std;

    PendController::PendController(string name)
        : TaskContext(name,PreOperational)
    {

    	//Hardcoding values. (Herman Alert!)
    	originX = 0.150;
    	originY = 0.0;
    	originZ = 0.830;

    	Xr = 0.08;
    	Yr = 0.08;


        //Creating TaskContext
        //Adding Ports
    	this->addEventPort("PendProjPointInput", pendProjPoint_inputPort,
    			boost::bind(&PendController::processPendProjPoint, this, _1));
    	this->addPort("CartesianPoseOutput", m_position_desi);
    	this->addOperation("moveToInitialPose",&PendController::moveToInitialPose,this,OwnThread);

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

		return true;

    }

    void PendController::updateHook()
    {

    	//generate random xr, yr;
        xr = -Xr + 2*Xr*(double)rand()/(double)RAND_MAX;
        yr = -Yr + 2*Yr*(double)rand()/(double)RAND_MAX;

        if(abs(xr)>Xr || abs(yr)>Yr){
        	cout << "PendController::Commanded Value out of Range" << endl;
        		cout << "	xr = " << xr << endl;
        		cout << "	yr = " << yr << endl;
        	this->stop();
        }else{
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

			m_position_desi.write(pose);
        }

    }

    void PendController::stopHook()
    {
    }

    void PendController::cleanupHook()
    {
    }

    bool PendController::moveToInitialPose(){
		geometry_msgs::Pose pose;

		// Always point up
		pose.orientation.x = 0.0;
		pose.orientation.y = 0.0;
		pose.orientation.z = 0.0;
		pose.orientation.w = 1.0;

		pose.position.x = originX;
		pose.position.y = originY;
		pose.position.z = originZ;

		m_position_desi.write(pose);

		return true;
    }

    bool PendController::processPendProjPoint(RTT::base::PortInterface* portInterface){
    	geometry_msgs::Point pendTipProj;
    	pendProjPoint_inputPort.read(pendTipProj);

    	cout << "x = " << pendTipProj.x << endl << "y = " << pendTipProj.y << endl;

    	return true;
    }
}//namespace
