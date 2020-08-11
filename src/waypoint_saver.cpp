#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf/transform_datatypes.h>

#include <fstream>
#include <string>
#include <math.h>

using namespace std;

class WaypointSaver {
protected:
	ros::NodeHandle nh_;
	ros::Subscriber pose_sub_;

	ofstream os_;
	
	double dist_;
	int min_dist_;

	double ex_xpos_;
	double ex_ypos_;
	
	double xpos_;
	double ypos_;
	
	double cur_vel_;

	int count_;

	geometry_msgs::Point state_switching_point_[8];
	
public:
	WaypointSaver() {
		ROS_INFO("WAYPOINT SAVER INITIALIZED.");	
		initSetup();
	}
	
	~WaypointSaver() {
		os_.close();
		ROS_INFO("WAYPOINT SAVER TERMAINATED.");
	}
	
	void initSetup() {
		min_dist_ = 1;
	
		ex_xpos_ = 0;
		ex_ypos_ = 0;

		count_ = 0;

		pose_sub_ = nh_.subscribe("current_pose", 10, &WaypointSaver::PoseCallback, this);
	
		os_.open("/home/so-yeon/data.csv", ios::app);
		os_<<setprecision(numeric_limits<double>::digits10+2);
	}
	
	void PoseCallback(const geometry_msgs::PoseStamped::ConstPtr &pose_msg) {
		xpos_ = pose_msg->pose.position.x;
		ypos_ = pose_msg->pose.position.y;
	
		geometry_msgs::Point cur_pose;
		cur_pose.x = xpos_;
		cur_pose.y = ypos_;

		if(ex_xpos_ == 0 && ex_ypos_ ==0) {
			ex_xpos_ = xpos_;
			ex_ypos_ = ypos_;
		
			ROS_INFO("ADD STARTING POINT XPOS = %f, YPOS = %f", xpos_, ypos_);	
			os_<<count_<<","<<(double)xpos_<<","<<(double)ypos_<<","<<"\n";
			count_++;
			return;
		}
		
		dist_ = sqrtf(powf(ex_xpos_ - xpos_, 2) + powf(ex_ypos_ - ypos_, 2));
	
		// ROS_INFO("CURRENT XPOS = %f", xpos_);
		// ROS_INFO("CURRENT YPOS = %f", ypos_);
	
		if(dist_ > min_dist_) {
			ex_xpos_ = xpos_;
			ex_ypos_ = ypos_;
			ROS_INFO("ADD NEW WAYPOINT XPOS = %f, YPOS = %f", xpos_, ypos_);
			os_<<count_<<","<<(double)xpos_<<","<<(double)ypos_<<","<<"\n";
			count_++;
		}
	}

	double calcPlaneDist(const geometry_msgs::Point pos1, const geometry_msgs::Point pos2) {
		double dist = sqrtf(powf(pos1.x - pos2.x, 2) + powf(pos1.y - pos2.y, 2));
		return dist;
	}
};

int main(int argc, char** argv) {
	ros::init(argc, argv, "waypoint_saver");

	WaypointSaver ws;
	ros::spin();

	return 0;	
}
