#include "PendFilter.hpp"
#include <ocl/Component.hpp>
ORO_LIST_COMPONENT_TYPE(simplePendulum::PendFilter);
//ORO_CREATE_COMPONENT(simplePendulum::PendFilter);

//Butterworth
#define NZEROS 4
#define NPOLES 4
#define GAIN   1.397824236e+02


using namespace std;

namespace simplePendulum
{

    using namespace RTT;
    using namespace KDL;
    using namespace std;

    PendFilter::PendFilter(string name)
        : TaskContext(name,PreOperational)
    {

        //Adding Ports
/*    	this->addEventPort("PendProjPointInput", pendProjPoint_inputPort,
    			boost::bind(&PendFilter::processPendProjPoint, this, _1));*/
    	this->addEventPort("PendProjPointInput", pendProjPoint_inputPort);
    	this->addPort("pendPosFiltered", pendProjPoint_outputPort);
    	this->addPort("pendVelFiltered", pendProjVelocity_outputPort);

    	//Butterworth - Lowpass
    	xr_est = std::vector<double>(NPOLES+1, 0.0);
    	xr = std::vector<double>(NZEROS+1, 0.0);
    	yr_est = std::vector<double>(NPOLES+1, 0.0);
    	yr = std::vector<double>(NZEROS+1, 0.0);

    	//Butterworth - Lowpass derivatives
    	xpdot_est = std::vector<double>(NPOLES+1, 0.0);
    	xpdot = std::vector<double>(NZEROS+1, 0.0);
    	ypdot_est = std::vector<double>(NPOLES+1, 0.0);
    	ypdot = std::vector<double>(NZEROS+1, 0.0);

    	xp_old = 0.0;
    	yp_old = 0.0;

    }

    PendFilter::~PendFilter()
    {

    }

    bool PendFilter::configureHook()
    {
    	dT = this->getPeriod();
		return true;
    }

    bool PendFilter::startHook()
    {

		return true;

    }

    void PendFilter::updateHook()
    {
    	pendProjPoint_inputPort.read(pendPosBWFiltered);
    	pendPosBWFiltered.x = butterWorthLowpass(pendPosBWFiltered.x, xr, xr_est);
    	pendPosBWFiltered.y = butterWorthLowpass(pendPosBWFiltered.y, yr, yr_est);
    	pendProjPoint_outputPort.write(pendPosBWFiltered);

    	pendVelBWFiltered.x = butterWorthLowpass((pendPosBWFiltered.x-xp_old)/dT, xpdot, xpdot_est);
    	pendVelBWFiltered.y = butterWorthLowpass((pendPosBWFiltered.y-yp_old)/dT, ypdot, ypdot_est);
    	xp_old = pendPosBWFiltered.x; yp_old = pendPosBWFiltered.y;
    	pendProjVelocity_outputPort.write(pendVelBWFiltered);
    }

    void PendFilter::stopHook()
    {
    }

    void PendFilter::cleanupHook()
    {
    }

    double PendFilter::butterWorthLowpass(double input, std::vector<double> &x, std::vector<double> &x_est){
		x[0] = x[1]; x[1] = x[2]; x[2] = x[3]; x[3] = x[4];
		x[4] = input / GAIN;
		x_est[0] = x_est[1]; x_est[1] = x_est[2]; x_est[2] = x_est[3]; x_est[3] = x_est[4];
		x_est[4] =   (x[0] + x[4]) + 4 * (x[1] + x[3]) + 6 * x[2]
				+ ( -0.1508387651 * x_est[0]) + (  0.8784455006 * x_est[1])
				+ ( -2.0129032148 * x_est[2]) + (  2.1708328750 * x_est[3]);
		//return x_est[4];
		return input;
    }

/*    bool PendFilter::processPendProjPoint(RTT::base::PortInterface* portInterface){
    	pendProjPoint_inputPort.read(pendPosBWFiltered);
    	pendPosBWFiltered.x = butterWorthLowpass(pendPosBWFiltered.x, xr, xr_est);
    	pendPosBWFiltered.y = butterWorthLowpass(pendPosBWFiltered.y, yr, yr_est);

    	pendPosBWFiltered.x = butterWorthLowpass((pendPosBWFiltered.x - xr_tm1)*89/1000, xrdot, xrdot_est);
    	pendPosBWFiltered.y = butterWorthLowpass((pendPosBWFiltered.y - yr_tm1)*89/1000, yrdot, yrdot_est);

    	xr_tm1 = pendPosBWFiltered.x;
    	yr_tm1 = pendPosBWFiltered.y;

    	pendProjPoint_outputPort.write(pendPosBWFiltered);
    	return true;
    }*/
}
