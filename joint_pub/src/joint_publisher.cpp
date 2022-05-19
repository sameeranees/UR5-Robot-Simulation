#include "ros/ros.h"
#include "math.h"
#include "ros/time.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64MultiArray.h"
#include "control_msgs/FollowJointTrajectoryActionGoal.h"

#define PI 3.14159265
using namespace std;
bool reached=false;

//topic type to publish on
control_msgs::FollowJointTrajectoryActionGoal pubmsg;

//subscriber for joint_states to get joint names
void Chatter(const sensor_msgs::JointState msg){
	pubmsg.header.frame_id="";
	pubmsg.goal.trajectory.joint_names=msg.name;
}

int main(int argv, char **argc){
	//initilizing ros node and handle
	ros::init(argv,argc,"Task1_Publisher");
	ros::NodeHandle nodeHandler;
	
	//subscriber for joint_states
	ros::Subscriber sub = nodeHandler.subscribe("joint_states",100,Chatter);
	
	//LOOP RATE	
	const int loop_rate_val = 100;		
	ros::Rate loop_rate(loop_rate_val);
	
	//Make sure we have received proper joint angles already
	for(int i=0; i< 2; i++) {
		ros::spinOnce();
	 	loop_rate.sleep();
	}	
	
	float angle = 0.0;
	float ch_deg=0;
	const float t_step = 1/((float)loop_rate_val);
	float tmax=3;	
	
	//resizing joint positions and velocities arrays	
	pubmsg.goal.trajectory.points.resize(1);
	pubmsg.goal.trajectory.points[0].positions.resize(6);
	pubmsg.goal.trajectory.points[0].velocities.resize(6);
	
	//publisher to publish joint positions on	
	ros::Publisher publisherObject = nodeHandler.advertise<control_msgs::FollowJointTrajectoryActionGoal>("/arm_controller/follow_joint_trajectory/goal",100);

	while ((ros::ok()) && (reached==false)) {
		float t = 0.0;
		while(t<tmax){
			//calculating joint angles based on sin function
			angle= sin(ch_deg*PI/180);
	
			for(int i=0;i<6;i++){
				pubmsg.goal.trajectory.points[0].positions[i] = angle;
				pubmsg.goal.trajectory.points[0].velocities[i] = 0;		

			}
			pubmsg.header.stamp = ros::Time::now();
			pubmsg.header.seq ++;
			pubmsg.goal.trajectory.points[0].time_from_start = ros::Duration(t);
			std::cout << pubmsg << std::endl;
			publisherObject.publish(pubmsg);
			ros::spinOnce();
			loop_rate.sleep();
			t+= t_step;		
			ch_deg ++;
		}
		reached=true;
    	}
	return 0;
}


