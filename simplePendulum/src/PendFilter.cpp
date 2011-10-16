#include "PendFilter.hpp"
#include <ocl/Component.hpp>
ORO_LIST_COMPONENT_TYPE(simplePendulum::PendFilter);
//ORO_CREATE_COMPONENT(simplePendulum::PendFilter);

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
    	this->addPort("pendPos_outputPort", pendPos_outputPort);
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

    bool PendFilter::processPendProjPoint(RTT::base::PortInterface* portInterface){
    	pendProjPoint_inputPort.read(pendPosFromROS);

    	return true;
    }
}
