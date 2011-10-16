#include <rtt/RTT.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/os/TimeService.hpp>

#include <kdl/velocityprofile_trap.hpp>
#include <kdl/kdl.hpp>
#include <kdl/frames.hpp>

#include <ocl/OCL.hpp>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>


using namespace std;
namespace simplePendulum
{
    class PendFilter : public RTT::TaskContext
    {
    public:
    	PendFilter(std::string name);
        virtual ~PendFilter();

        virtual bool configureHook();
        virtual bool startHook();
        virtual void updateHook();
        virtual void stopHook();
        virtual void cleanupHook();


    private:
        bool processPendProjPoint(RTT::base::PortInterface* portInterface);
        bool butterWorthLowpass();
        geometry_msgs::Point pendPosFromROS;
        geometry_msgs::Point pendPosBWFiltered;

        std::vector<double> xr, xr_est, yr, yr_est;


    protected:
      RTT::InputPort<geometry_msgs::Point>   		pendProjPoint_inputPort;
      RTT::OutputPort<geometry_msgs::Point>   		pendProjPoint_outputPort;
      RTT::OutputPort<std::vector<double> >			pendPos_outputPort;

  }; // class
}//namespace
