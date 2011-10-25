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

#include "PendFilter.hpp"

//Butterworth
#define NZEROS 4
#define NPOLES 4
#define GAIN   2.310287053e+00

using namespace std;
namespace simplePendulum
{
    class PendController : public RTT::TaskContext
    {
    public:
    	PendController(std::string name);
        virtual ~PendController();

        virtual bool configureHook();
        virtual bool startHook();
        virtual void updateHook();
        virtual void stopHook();
        virtual void cleanupHook();

        bool moveToInitialPose();
        double butterWorthLowpass(double input, std::vector<double> &, std::vector<double> &);

    private:
        double originX, originY, originZ;
        double Xr, Yr;
        geometry_msgs::Point filteredPos, filteredVel;
        geometry_msgs::Pose poseCurrent;

        //temporary variables
        double xr, yr;

        std::vector<double> s_t, s_tm1, u_t, u_tm1;
        static const double gainLQR [];
 //       double gainLQR[];
        double dT;
        double xr_old, yr_old;
        std::vector<double> xrdot, xrdot_est, yrdot, yrdot_est;


        ofstream stateLogger;

        //debugging
        double xr_comm, yr_comm, xrdot_comm, yrdot_comm, xr_msr, yr_msr, t;
        std::vector<double> commandedState;

    protected:
      RTT::InputPort<geometry_msgs::Point >   		pendPos_inputPort;
      RTT::InputPort<geometry_msgs::Point >   		pendVel_inputPort;
      RTT::InputPort<geometry_msgs::Pose >   		robotPos_inputPort;
      RTT::OutputPort<std::vector<double> >			m_position_desi;

  }; // class
}//namespace
