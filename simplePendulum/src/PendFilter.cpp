#include "PendFilter.hpp"
#include <ocl/Component.hpp>
ORO_LIST_COMPONENT_TYPE(simplePendulum::PendFilter);
//ORO_CREATE_COMPONENT(simplePendulum::PendFilter);

//Filter Specific
#define NZEROS 10
#define NPOLES 10
#define GAIN   8.585184890e+03


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
    	this->addEventPort("PendProjPointInput", pendProjPoint_inputPort,
    			boost::bind(&PendFilter::processPendProjPoint, this, _1));
    	this->addPort("pendPosToROS", pendProjPoint_outputPort);
    	this->addPort("pendPos_outputPort", pendPos_outputPort);

    	//Butterworth - Lowpass
    	xr_est = std::vector<double>(NPOLES+1, 0.0);
    	xr = std::vector<double>(NZEROS+1, 0.0);
    	yr_est = std::vector<double>(NPOLES+1, 0.0);
    	yr = std::vector<double>(NZEROS+1, 0.0);
    }

    PendFilter::~PendFilter()
    {

    }

    bool PendFilter::configureHook()
    {
		return true;
    }

    bool PendFilter::startHook()
    {

		return true;

    }

    void PendFilter::updateHook()
    {
    }

    void PendFilter::stopHook()
    {
    }

    void PendFilter::cleanupHook()
    {
    }

    bool PendFilter::butterWorthLowpass(){
    	//order	 =	 10
    	//sample rate	 =	 60
    	//corner1	 =	 10

    	//Butterworth - Lowpass on xr
		xr[0] = xr[1]; xr[1] = xr[2]; xr[2] = xr[3]; xr[3] = xr[4]; xr[4] = xr[5]; xr[5] = xr[6]; xr[6] = xr[7]; xr[7] = xr[8]; xr[8] = xr[9]; xr[9] = xr[10];
		xr[10] = pendPosFromROS.x / GAIN;
		xr_est[0] = xr_est[1]; xr_est[1] = xr_est[2]; xr_est[2] = xr_est[3]; xr_est[3] = xr_est[4]; xr_est[4] = xr_est[5]; xr_est[5] = xr_est[6]; xr_est[6] = xr_est[7]; xr_est[7] = xr_est[8]; xr_est[8] = xr_est[9]; xr_est[9] = xr_est[10];
		xr_est[10] =   (xr[0] + xr[10]) + 10 * (xr[1] + xr[9]) + 45 * (xr[2] + xr[8])
					 + 120 * (xr[3] + xr[7]) + 210 * (xr[4] + xr[6]) + 252 * xr[5]
					 + ( -0.0008011625 * xr_est[0]) + (  0.0133611574 * xr_est[1])
					 + ( -0.1030105408 * xr_est[2]) + (  0.4849789050 * xr_est[3])
					 + ( -1.5511214242 * xr_est[4]) + (  3.5381534158 * xr_est[5])
					 + ( -5.8765157243 * xr_est[6]) + (  7.0737013796 * xr_est[7])
					 + ( -6.0201305924 * xr_est[8]) + (  3.3221093450 * xr_est[9]);
		pendPosBWFiltered.x = xr_est[10];

    	//Butterworth - Lowpass on yr
		yr[0] = yr[1]; yr[1] = yr[2]; yr[2] = yr[3]; yr[3] = yr[4]; yr[4] = yr[5]; yr[5] = yr[6]; yr[6] = yr[7]; yr[7] = yr[8]; yr[8] = yr[9]; yr[9] = yr[10];
		yr[10] = pendPosFromROS.y / GAIN;
		yr_est[0] = yr_est[1]; yr_est[1] = yr_est[2]; yr_est[2] = yr_est[3]; yr_est[3] = yr_est[4]; yr_est[4] = yr_est[5]; yr_est[5] = yr_est[6]; yr_est[6] = yr_est[7]; yr_est[7] = yr_est[8]; yr_est[8] = yr_est[9]; yr_est[9] = yr_est[10];
		yr_est[10] =   (yr[0] + yr[10]) + 10 * (yr[1] + yr[9]) + 45 * (yr[2] + yr[8])
					 + 120 * (yr[3] + yr[7]) + 210 * (yr[4] + yr[6]) + 252 * yr[5]
					 + ( -0.0008011625 * yr_est[0]) + (  0.0133611574 * yr_est[1])
					 + ( -0.1030105408 * yr_est[2]) + (  0.4849789050 * yr_est[3])
					 + ( -1.5511214242 * yr_est[4]) + (  3.5381534158 * yr_est[5])
					 + ( -5.8765157243 * yr_est[6]) + (  7.0737013796 * yr_est[7])
					 + ( -6.0201305924 * yr_est[8]) + (  3.3221093450 * yr_est[9]);
		pendPosBWFiltered.y = yr_est[10];

		return true;
    }


    bool PendFilter::processPendProjPoint(RTT::base::PortInterface* portInterface){
    	pendProjPoint_inputPort.read(pendPosFromROS);
    	//butterWorthLowpass();
    	pendProjPoint_outputPort.write(pendPosBWFiltered);
    	return true;
    }
}
