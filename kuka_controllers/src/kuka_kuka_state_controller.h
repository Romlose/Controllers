#include <urdf/model.h>
#include <hardware_interface/joint_command_interface.h>
#include <controller_interface/controller.h>
#include <ros/node_handle.h>
#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <realtime_tools/realtime_publisher.h>

#include <kdl/tree.hpp>
#include <kdl/kdl.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/frames.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include "sensor_msgs/JointState.h"
#include <geometry_msgs/WrenchStamped.h>

#include <boost/thread/condition.hpp>
#include <boost/scoped_ptr.hpp>

#include "kuka_controllers/KukaState.h"

namespace kuka_controllers{
        class KukaStateController: public controller_interface::Controller<hardware_interface::EffortJointInterface>
    {
        public:

            KukaStateController();
            ~KukaStateController();

            bool init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n);
            void starting(const ros::Time& time);
            void update(const ros::Time& time, const ros::Duration& period);
            void stopping(const ros::Time& time);
        
        private:
            
            void ext_wrench_cb(const geometry_msgs::WrenchStamped::ConstPtr &wrench_msg);
            void jsCallback(const sensor_msgs::JointState::ConstPtr& msg);

        protected:

            ros::NodeHandle nh_;
            KDL::Chain kdl_chain_;
            KDL::Tree kdl_tree_;
            std::vector<std::string> joint_names_;

            std::vector<KDL::Wrench> Wrenches;
            boost::scoped_ptr<KDL::JntArray> q; 
            boost::scoped_ptr<KDL::JntArray> q_dot;
            boost::scoped_ptr<KDL::JntArray> q_dot_dot;
            boost::scoped_ptr<KDL::JntArray> js_torques_;
            boost::scoped_ptr<KDL::ChainIdSolver_RNE> id_solver_;
            boost::scoped_ptr<KDL::JntArray> torques_;
            boost::scoped_ptr<KDL::Jacobian> jacobian_;

            boost::shared_ptr<realtime_tools::RealtimePublisher<kuka_controllers::KukaState> > realtime_pub_;
    };
PLUGINLIB_EXPORT_CLASS(kuka_controllers::KukaStateController, controller_interface::ControllerBase)
}