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

//#include "VelocityProfile_NonZeroInit.hpp"
using namespace std;
namespace simplePendulum
{
    /**
     * This class implements a TaskContext that creates a path in
     * Cartesian space between the current cartesian position and a
     * new desired cartesian position. It uses trapezoidal
     * velocity-profiles for every dof using a maximum velocity and a
     * maximum acceleration. It generates frame and twist setpoints
     * which can be used by OCL::CartesianControllerPos,
     * OCL::CartesianControllerPosVel or OCL::CartesianControllerVel.
     *
     */
    class PendController : public RTT::TaskContext
    {
    public:
        /**
         * Constructor of the class.
         *
         * @param name name of the TaskContext
         */
    	PendController(std::string name);
        virtual ~PendController();

        virtual bool configureHook();
        virtual bool startHook();
        virtual void updateHook();
        virtual void stopHook();
        virtual void cleanupHook();

        bool moveToInitialPose();
//        bool updateCG();

    private:
        bool processPendProjPoint(RTT::base::PortInterface* portInterface);
        double originX, originY, originZ;
        double Xr, Yr;

        //temporary variables
        double xr, yr;


    protected:
      RTT::InputPort< geometry_msgs::Point >   		pendProjPoint_inputPort;
      RTT::OutputPort<geometry_msgs::Pose >			m_position_desi;

  }; // class
}//namespace
