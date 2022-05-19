//Some contents from this code is referenced from: https://github.com/dairal/ur5-tcp-position-control/blob/main/src/ur5_cartesian_position_controller.cpp
#include "ros/ros.h"
#include <ros/package.h>
#include "std_msgs/String.h"

#include "std_msgs/Float64.h"
#include "control_msgs/JointControllerState.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64MultiArray.h"
#include "control_msgs/FollowJointTrajectoryActionGoal.h"
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/frames.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/jntarray.hpp>

const int Joints = 6; // joints in ur5
KDL::JntArray jnt_pos_start(Joints);
const int loop_rate_val = 100; //loop rate for publishing
control_msgs::FollowJointTrajectoryActionGoal pubmsg;

//variables for force in x,y,z direction and time it will take to run the simulation
float force_x; 
float force_y;
float force_z;
float time_d;

//Subscriber function that is used to fill in joint names and current joint positions from topic 'joint_states'
void Chatter(const sensor_msgs::JointState msg){
	pubmsg.goal.trajectory.joint_names=msg.name;
	pubmsg.header.frame_id="";
	jnt_pos_start(0) = msg.position[0];
	jnt_pos_start(1) = msg.position[1];
	jnt_pos_start(2) = msg.position[2];
	jnt_pos_start(3) = msg.position[3];
	jnt_pos_start(4) = msg.position[4];
	jnt_pos_start(5) = msg.position[5];
}

//Function to compute next position step for all joints
float compute_linear(double q_start, double q_goal, float t, float t_max) {
	return((q_goal - q_start) * (t/t_max) + q_start);
}

//function to get input from launch file and compute goal position
void get_goal_tcp_and_time(KDL::Frame tcp_pos_start, KDL::Vector* vec_tcp_pos_goal, float* t_max) {
	//Get user input
	float x,y,z;
	x=force_x;
	y=force_y;
	z=force_z;
	*t_max=time_d;

	//Compute goal position
	(*vec_tcp_pos_goal)(0) = (tcp_pos_start.p(0) + x);
	(*vec_tcp_pos_goal)(1) = (tcp_pos_start.p(1) + y);
	(*vec_tcp_pos_goal)(2) = (tcp_pos_start.p(2) + z);
}


int main(int argv, char **argc){
	//get urdf from path, urdf.xacro converted to urdf and then copied to urdf folder in joint_pub	
	std::string urdf_path = ros::package::getPath("joint_pub");
	if(urdf_path.empty()) {
		ROS_ERROR("ur5-joint-position-control package path was not found");
	}
	urdf_path += "/urdf/ur5.urdf";

	//initializing node and node handle
	ros::init(argv,argc,"Task2_Publisher");
	ros::NodeHandle nodeHandler;

	//getting parameters and inputting into force and time variables from launch file
	nodeHandler.getParam("force_x", force_x);
	nodeHandler.getParam("force_y", force_y);
	nodeHandler.getParam("force_z", force_z);
	nodeHandler.getParam("time_d", time_d);
	
	//initilizing rate
	ros::Rate loop_rate(loop_rate_val);

	//resizing pubmsg	
	pubmsg.goal.trajectory.points.resize(1);
	pubmsg.goal.trajectory.points[0].positions.resize(6);
	pubmsg.goal.trajectory.points[0].velocities.resize(6);

	//Create subscriber for joint states
	ros::Subscriber sub = nodeHandler.subscribe("joint_states",1000,Chatter);

	//Create publisher to send position commands to all joints	
	ros::Publisher publisherObject = nodeHandler.advertise<control_msgs::FollowJointTrajectoryActionGoal>("/arm_controller/follow_joint_trajectory/goal",1000);

	//Parse urdf model and generate KDL tree
	KDL::Tree ur5_tree;
	if (!kdl_parser::treeFromFile(urdf_path, ur5_tree)){
		ROS_ERROR("Failed to construct kdl tree");
   		return false;
	}

	//Generate a kinematic chain from the robot base to its tcp
	KDL::Chain ur5_chain;
	ur5_tree.getChain("base_link", "wrist_3_link", ur5_chain);

	//Create solvers
	KDL::ChainFkSolverPos_recursive fk_solver(ur5_chain);
	KDL::ChainIkSolverVel_pinv vel_ik_solver(ur5_chain, 0.0001, 1000);
	KDL::ChainIkSolverPos_NR ik_solver(ur5_chain, fk_solver, vel_ik_solver, 1000);

	//Make sure we have received proper joint angles already
	for(int i=0; i< 2; i++) {
		ros::spinOnce();
	 	loop_rate.sleep();
	}

	const float t_step = 1/((float)loop_rate_val);
	int count = 0;
	
	//variable for time reached and hence loop exit
	bool reached=false;
	while ((ros::ok()) && (reached==false)) {

		//Compute current tcp position
		KDL::Frame tcp_pos_start;
		fk_solver.JntToCart(jnt_pos_start, tcp_pos_start);

		ROS_INFO("Current tcp Position/Twist KDL:");		
		ROS_INFO("Position: %f %f %f", tcp_pos_start.p(0), tcp_pos_start.p(1), tcp_pos_start.p(2));		
		ROS_INFO("Orientation: %f %f %f", tcp_pos_start.M(0,0), tcp_pos_start.M(1,0), tcp_pos_start.M(2,0));

		//get input from launch file
		float t_max;
		KDL::Vector vec_tcp_pos_goal(0.0, 0.0, 0.0);
		get_goal_tcp_and_time(tcp_pos_start, &vec_tcp_pos_goal, &t_max);

		KDL::Frame tcp_pos_goal(tcp_pos_start.M, vec_tcp_pos_goal);

		//Compute inverse kinematics
		KDL::JntArray jnt_pos_goal(Joints);
		ik_solver.CartToJnt(jnt_pos_start, tcp_pos_goal, jnt_pos_goal);
		
		float t = 0.0;
		while(t<t_max) {
			std_msgs::Float64 position[6];
			//Compute next position step for all joints
			for(int i=0; i<Joints; i++) {
				pubmsg.goal.trajectory.points[0].velocities[i] = 0;
				pubmsg.goal.trajectory.points[0].positions[i] = compute_linear(jnt_pos_start(i), jnt_pos_goal(i), t, t_max);
			}
			pubmsg.header.stamp = ros::Time::now();
			pubmsg.header.seq ++;
			pubmsg.goal.trajectory.points[0].time_from_start = ros::Duration(t);
			publisherObject.publish(pubmsg);			
			std::cout << pubmsg <<std::endl;
			ros::spinOnce();
			loop_rate.sleep();
			++count;
			t += t_step;	
		}
		reached=true;		
	}	
	return 0;
}
