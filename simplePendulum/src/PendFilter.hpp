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
        static double butterWorthLowpass(double, std::vector<double> &, std::vector<double> &);

    private:
        /*bool processPendProjPoint(RTT::base::PortInterface* portInterface);*/
        geometry_msgs::Point pendPosFromROS;
        geometry_msgs::Point pendPosBWFiltered;
        geometry_msgs::Point pendVelBWFiltered;

        double xp_old, yp_old;
        std::vector<double> xr, xr_est, yr, yr_est;
        std::vector<double> xpdot, xpdot_est, ypdot, ypdot_est;
        double xr_tm1, yr_tm1;
        double dT;


    protected:
      RTT::InputPort<geometry_msgs::Point>   		pendProjPoint_inputPort;
      RTT::OutputPort<geometry_msgs::Point>   		pendProjPoint_outputPort;
      RTT::OutputPort<geometry_msgs::Point>   		pendProjVelocity_outputPort;

  }; // class
}//namespace
