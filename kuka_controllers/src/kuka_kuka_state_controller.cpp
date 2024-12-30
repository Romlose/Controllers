#include "kuka_kuka_state_controller.h"

namespace kuka_controllers{

    KukaStateController::KukaStateController()
    : q(NULL)
    , q_dot(NULL)
    , q_dot_dot(NULL)
    , joint_names_()
    , js_torques_(NULL)
    , id_solver_(NULL)
    , torques_(NULL)
    {}

    KukaStateController::~KukaStateController(){}

    bool KukaStateController::init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n){
         nh_ = n;

        // get URDF and name of root and tip from the parameter server
        std::string robot_description, root_name, tip_name;

        if (!ros::param::search(n.getNamespace(),"robot_description", robot_description))
        {
            ROS_ERROR_STREAM("InverseDynamicController: No robot description (URDF) found on parameter server ("<<n.getNamespace()<<"/robot_description)");
            return false;
        }

        if (!nh_.getParam("root_name", root_name))
        {
            ROS_ERROR_STREAM("InverseDynamicController: No root name found on parameter server ("<<n.getNamespace()<<"/root_name)");
            return false;
        }

        tip_name = "tool0"; 

        // Construct an URDF model from the xml string
        std::string xml_string;

        if (n.hasParam(robot_description))
            n.getParam(robot_description.c_str(), xml_string);
        else
        {
            ROS_ERROR("Parameter %s not set, shutting down node...", robot_description.c_str());
            n.shutdown();
            return false;
        }

        if (xml_string.size() == 0)
        {
            ROS_ERROR("Unable to load robot model from parameter %s",robot_description.c_str());
            n.shutdown();
            return false;
        }

        ROS_DEBUG("%s content\n%s", robot_description.c_str(), xml_string.c_str());
        
        // Get urdf model out of robot_description
        urdf::Model model;
        if (!model.initString(xml_string))
        {
            ROS_ERROR("Failed to parse urdf file");
            n.shutdown();
            return false;
        }
        ROS_INFO("Successfully parsed urdf file");
        
        KDL::Tree kdl_tree_;
        if (!kdl_parser::treeFromUrdfModel(model, kdl_tree_))
        {
            ROS_ERROR("Failed to construct kdl tree");
            n.shutdown();
            return false;
        }

        // Populate the KDL chain
        if(!kdl_tree_.getChain(root_name, tip_name, kdl_chain_))
        {
            ROS_ERROR_STREAM("Failed to get KDL chain from tree: ");
            ROS_ERROR_STREAM("  "<<root_name<<" --> "<<tip_name);
            ROS_ERROR_STREAM("  Tree has "<<kdl_tree_.getNrOfJoints()<<" joints");
            ROS_ERROR_STREAM("  Tree has "<<kdl_tree_.getNrOfSegments()<<" segments");
            ROS_ERROR_STREAM("  The segments are:");

            KDL::SegmentMap segment_map = kdl_tree_.getSegments();
            KDL::SegmentMap::iterator it;

            for( it=segment_map.begin(); it != segment_map.end(); it++ )
              ROS_ERROR_STREAM( "    "<<(*it).first);

            return false;
        }

        ROS_DEBUG("Number of segments: %d", kdl_chain_.getNrOfSegments());
        ROS_DEBUG("Number of joints in chain: %d", kdl_chain_.getNrOfJoints());

        q.reset(new KDL::JntArray(kdl_chain_.getNrOfJoints()));
        q_dot.reset(new KDL::JntArray(kdl_chain_.getNrOfJoints()));
        q_dot_dot.reset(new KDL::JntArray(kdl_chain_.getNrOfJoints()));
        js_torques_.reset(new KDL::JntArray(kdl_chain_.getNrOfJoints()));
        id_solver_.reset( new KDL::ChainIdSolver_RNE( kdl_chain_, KDL::Vector(0.0, 0.0, -9.81)) );
        jacobian_.reset( new KDL::Jacobian( kdl_chain_.getNrOfJoints()));
        torques_.reset(new KDL::JntArray(kdl_chain_.getNrOfJoints()));
        Wrenches.clear();

        ros::Subscriber ext_wrenches_sub = nh_.subscribe<geometry_msgs::WrenchStamped>
                                            ("ext_wrenches", 10, &KukaStateController::ext_wrench_cb, this);
        
        ros::Subscriber js_callback_sub = nh_.subscribe<sensor_msgs::JointState>
                                            ("angles", 10, &KukaStateController::jsCallback, this);

        realtime_pub_.reset(new realtime_tools::RealtimePublisher<kuka_controllers::KukaState>(n, "arm_state", 4));

        realtime_pub_->msg_.torque_error.resize(kdl_chain_.getNrOfJoints());

        return true;
    }

    void KukaStateController::starting(const ros::Time& time){}

    void KukaStateController::update(const ros::Time &time, const ros::Duration &period){

        if(realtime_pub_->trylock()){

            if(!id_solver_->CartToJnt(*q, *q_dot, *q_dot_dot, Wrenches, *torques_) != 0){
                ROS_ERROR("Could not compute joint torques! Setting all torques to zero!");
                realtime_pub_->unlock();
            }
            realtime_pub_->msg_.header.stamp = time;
            for(unsigned int i = 0; i < 6; i ++ ){
                realtime_pub_->msg_.torque_error[i] = (*js_torques_)(i) - (*torques_)(i);
                realtime_pub_->msg_.joint_name[i] = joint_names_[i];
            }

            realtime_pub_->unlockAndPublish();
        }
    }

    void KukaStateController::stopping(const ros::Time& time){}

    void KukaStateController::ext_wrench_cb(const geometry_msgs::WrenchStamped::ConstPtr &msg){
        KDL::Wrench w(KDL::Vector(msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z), 
                  KDL::Vector(msg->wrench.torque.x, msg->wrench.torque.y, msg->wrench.torque.z));
        Wrenches.push_back(w);
    }

    void KukaStateController::jsCallback(const sensor_msgs::JointState::ConstPtr& msg){
        for(size_t i = 0; i < msg->name.size(); i++){
            joint_names_.push_back(msg->name[i]);
            (*q)(i) = msg->position[i];
            (*q_dot)(i) = msg->velocity[i]; 
            (*js_torques_)(i) = msg->effort[i];
            (*q_dot_dot)(i) = 0;
        }
	    ROS_INFO("updated joint_states");
    }
}