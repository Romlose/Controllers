#include <iostream>
#include <vector>
#include <string>
#include <math.h>
#include "ros/ros.h"

#include "kdl/jntarray.hpp"
#include "kdl/chain.hpp"
#include <kdl/tree.hpp>
#include <kdl/jacobian.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>

#include "sensor_msgs/JointState.h"
#include <geometry_msgs/WrenchStamped.h>

std::vector<std::string> joint_names;
KDL::JntArray q(6);
KDL::JntArray q_dot(6);
KDL::JntArray torque(6);
KDL::JntArray q_dot_dot(6);
std::vector<KDL::Wrench> Wrenches;

void ftCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg){
    KDL::Wrench w(KDL::Vector(msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z), 
                  KDL::Vector(msg->wrench.torque.x, msg->wrench.torque.y, msg->wrench.torque.z));
    Wrenches.push_back(w); 
    ros::Duration(1.0).sleep();
}

void WPRNT(const KDL::JntArray info){
    ROS_INFO_STREAM("Angles: " << info(0) << ",  " <<  info(1) << ",  " << info(2) << ",  "
                    << info(3) << ",  " << info(4) << ",  " << info(5) << ".");
}

void JACPRNT(const KDL::Jacobian JAC){  
    int rows = JAC.rows(); // Получаем количество строк
    int cols = JAC.columns(); // Получаем количество столбцов

    for (int i = 0; i < rows; ++i) {
        std::stringstream ss; // Используем stringstream для формирования строки
        for (int j = 0; j < cols; ++j) {
            ss << JAC(i, j) << ", "; // Добавляем элементы строки в стрим
        }
        ROS_INFO_STREAM("JAC: " << ss.str()); // Выводим строку
    }
}

void jsCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    for(size_t i = 0; i < msg->name.size(); i++){
        joint_names.push_back(msg->name[i]);
        q(i) = msg->position[i];
        q_dot(i) = msg->velocity[i]; 
        torque(i) = msg->effort[i];
    }
}


int main(int argc, char **argv)
{	
	ros::init(argc, argv, "KUKA");
	ros::NodeHandle n;
	
    ros::Subscriber sub = n.subscribe("/joint_states", 1, jsCallback);
    ros::Subscriber sub_ft_a1 = n.subscribe("FT_a1", 1, ftCallback);
    ros::Subscriber sub_ft_a2 = n.subscribe("FT_a2", 1, ftCallback);
    ros::Subscriber sub_ft_a3 = n.subscribe("FT_a3", 1, ftCallback);
    ros::Subscriber sub_ft_a4 = n.subscribe("FT_a4", 1, ftCallback);
    ros::Subscriber sub_ft_a5 = n.subscribe("FT_a5", 1, ftCallback);
    ros::Subscriber sub_ft_a6 = n.subscribe("FT_a6", 1, ftCallback);

    ros::Rate loop_rate(1);
    
    KDL::Tree my_tree;
    ros::NodeHandle node;
    std::string robot_desc_string;
    node.param("robot_description", robot_desc_string, std::string());
    if (!kdl_parser::treeFromString(robot_desc_string, my_tree)){
       ROS_ERROR("Failed to construct kdl tree");
       return false;
    }

    KDL::Chain chain;
    if(!my_tree.getChain("base_link", "tool0", chain)){
        ROS_ERROR("Failed to construct kdl chain");
        return false;
    }
    while (ros::ok()){
       ROS_INFO("KDL_KUKA_node working");

       KDL::ChainIdSolver_RNE solver(chain, KDL::Vector(0.0, 0.0, -9.81));
       KDL::JntArray calc_torques(6);

       solver.CartToJnt(q, q_dot, q_dot_dot, Wrenches, calc_torques);

       KDL::JntArray error;
       Subtract(calc_torques, torque, error);
    
       WPRNT(q);

       KDL::ChainJntToJacSolver jac_solver(chain);
       KDL::Jacobian jacobian(chain.getNrOfJoints());
       jac_solver.JntToJac(q, jacobian);

       JACPRNT(jacobian);
       ros::spinOnce();  
       loop_rate.sleep();
   }
   return 0;
}
