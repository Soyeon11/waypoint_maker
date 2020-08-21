#include <ros/ros.h>
#include <waypoint_maker/Lane.h>
#include <waypoint_maker/Waypoint.h>
#include <waypoint_maker/State.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <vector>

#include <fstream>
#include <string>


#include <sys/types.h>
#include <dirent.h>


using namespace std;

class WaypointLoader {
protected:
	ros::NodeHandle nh_;
	ros::NodeHandle private_nh_;

	// publisher	
	ros::Publisher waypoint_pub_;
	
	// subscriber
	ros::Subscriber pose_sub_;
	ros::Subscriber state_sub_;

	ifstream is_;
	
	std::vector<std::string> all_csv_;
	vector< vector<waypoint_maker::Waypoint> > all_new_waypoints_;
	vector<waypoint_maker::Waypoint> new_waypoints_;
	vector<int> lane_size_;
	vector< vector<int> > all_state_index_;
	vector<int> state_index_;

	int size_;
	int final_size_;
	int state_inspection;	

	vector<waypoint_maker::Waypoint> final_waypoints_;

	float max_search_dist_;
	float min_search_dist_;
	int closest_waypoint_;
	int ex_closest_waypoint_;
	int lane_number_;
	int ex_lane_number_;

	waypoint_maker::Lane lane_msg_;
	
public:
	WaypointLoader() {
		initSetup();
		ROS_INFO("WAYPOINT LOADER INITIALIZED.");
	}
	
	~WaypointLoader() {
		new_waypoints_.clear();
		all_new_waypoints_.clear();
		final_waypoints_.clear();
		ROS_INFO("WAYPOINT LOADER TERMINATED.");
	}
	
	void initSetup() {
		private_nh_.getParam("/waypoint_loader_node/state_inspection", state_inspection);
	
		final_size_ = 30;


		max_search_dist_ = 10.0;
		min_search_dist_ = 0.5;

		closest_waypoint_ = 0;
		
		get_csvs_inDirectory();
		getNewWaypoints();

		waypoint_pub_ = nh_.advertise<waypoint_maker::Lane>("final_waypoints", 1);
		pose_sub_ = nh_.subscribe("odom", 10, &WaypointLoader::poseCallback, this);
		state_sub_ = nh_.subscribe("target_state",10, &WaypointLoader::stateCallback,this);
		lane_number_ = 0;
		ex_lane_number_ = 0;
	
	}
	
	void stateCallback(const waypoint_maker::Waypoint::ConstPtr &msg){
		state_inspection = msg->mission_state;
		lane_number_ = msg->lane_number;
	}

	void poseCallback(const nav_msgs::Odometry::ConstPtr &msg) {
		final_waypoints_.clear();
		geometry_msgs::PoseStamped current_pose;
		current_pose.pose.position = msg->pose.pose.position;

		getClosestWaypoint(current_pose);

		ROS_INFO("CLOSEST WAYPOINT INDEX=%d, X=%f, Y=%f", closest_waypoint_, new_waypoints_[closest_waypoint_].pose.pose.position.x, new_waypoints_[closest_waypoint_].pose.pose.position.y);
	
		if((size_ - closest_waypoint_) < final_size_) final_size_ = size_ - closest_waypoint_;
		
	 	for(int i=0;i<final_size_;i++) {
			final_waypoints_.push_back(new_waypoints_[closest_waypoint_ + i]);
		}
	
		lane_msg_.header = msg->header;
		lane_msg_.waypoints = final_waypoints_;
	
		waypoint_pub_.publish(lane_msg_);

		int final_waypoints_size = lane_msg_.waypoints.size();
		
		ROS_INFO("FINAL WAYPOINTS NUMBER=%d PUBLISHED.", final_waypoints_size);
	}
	
	void get_csvs_inDirectory()
	{
	
		DIR* dirp = opendir("/home/so-yeon/data/");
		struct dirent* dp;
		
		if (dirp == NULL){
			perror("UNABLE TO OPEN FOLDER");
			return;
			
		}


		while((dp = readdir(dirp)) != NULL){
			string address("/home/so-yeon/data/");

			string filename (dp->d_name);

			address.append(filename);
			if (filename.size() > 2){			
			all_csv_.push_back(address);
			ROS_INFO("SAVED CSV");
			}
			
		} 
		
		sort(all_csv_.begin(),all_csv_.end());
		closedir(dirp);
 
	}


	void getNewWaypoints() {
		string str_buf;

		int pos1;
		int pos2;
		int pos3;
		int pos4;

		for(std::vector<std::string>::size_type i = 0; i<all_csv_.size();i++)
		{
						
			is_.open(all_csv_.at(i));
			string csv(all_csv_.at(i));
			cout << csv;

			ROS_INFO("OPEN CSV");			
			
			vector<waypoint_maker::Waypoint> new_waypoints;
			waypoint_maker::Waypoint temp_waypoint;
			temp_waypoint.mission_state = 0;
	
			while(!is_.eof()) {
				getline(is_, str_buf);
				if(str_buf != "") {
					pos1 = str_buf.find(",");
					temp_waypoint.waypoint_index = stoi(str_buf.substr(0, pos1));
					string str_buf2 = str_buf.substr(pos1+1);
					pos2 = str_buf2.find(","); 
					temp_waypoint.pose.pose.position.x = stof(str_buf2.substr(0, pos2));
					string str_buf3 = str_buf2.substr(pos2+1);
					pos3 = str_buf3.find(",");
					temp_waypoint.pose.pose.position.y = stof(str_buf3.substr(0, pos3));
					string str_buf4 = str_buf3.substr(pos3+1);
					pos4 = str_buf4.find(",");
					temp_waypoint.twist.twist.linear.x = stof(str_buf4.substr(0, pos4));
					if(temp_waypoint.mission_state != stof(str_buf4.substr(pos4+1))){
					state_index_.push_back(temp_waypoint.waypoint_index);
					}
					temp_waypoint.mission_state = stof(str_buf4.substr(pos4+1));
					
					new_waypoints.push_back(temp_waypoint);

				}
			}
			is_.close();
			
			size_ = new_waypoints.size();
			lane_size_.push_back(size_);
				
			ROS_INFO("%d WAY POINTS HAVE BEEN SAVED.", size_);

			all_new_waypoints_.push_back(new_waypoints);
			all_state_index_.push_back(state_index_);
			new_waypoints.clear();
		}


		new_waypoints_.assign(all_new_waypoints_[0].begin(),all_new_waypoints_[0].end());
		size_ = lane_size_[0];
		

	}

	void getClosestWaypoint(geometry_msgs::PoseStamped current_pose) {
		

		if(lane_number_ != ex_lane_number_){
			private_nh_.setParam("/waypoint_follower_node/lane_final", lane_number_);
			private_nh_.setParam("/waypoint_follower_node/first_state_index", all_state_index_[lane_number_].at(0));
			private_nh_.setParam("/waypoint_follower_node/second_state_index", all_state_index_[lane_number_].at(1));
			private_nh_.setParam("/waypoint_follower_node/third_state_index", all_state_index_[lane_number_].at(2));
			new_waypoints_.clear();
			new_waypoints_.assign(all_new_waypoints_[lane_number_].begin(),all_new_waypoints_[lane_number_].end());
			size_ = lane_size_[lane_number_];
			ex_lane_number_ = lane_number_;
			if(lane_number_ ==0){
			state_inspection == 3;
			}
			else state_inspection == 0;
		}


		vector<int> closest_waypoint_candidates;
		for(int i=0;i<size_;i++) {
			float dist = calcPlaneDist(current_pose, new_waypoints_[i].pose);
			if(state_inspection==0||state_inspection==1||state_inspection==2){
				if(dist < max_search_dist_ && (new_waypoints_[i].mission_state==state_inspection)) closest_waypoint_candidates.push_back(i);
			}
			else if(dist < max_search_dist_ && abs((new_waypoints_[i].mission_state - state_inspection)) <= 1 ) closest_waypoint_candidates.push_back(i);
		}
		if(!closest_waypoint_candidates.empty()) {
			int waypoint_min = -1;
			float dist_min = max_search_dist_;
			for(int i=0;i<closest_waypoint_candidates.size();i++) {
				float dist = calcPlaneDist(current_pose, new_waypoints_[closest_waypoint_candidates[i]].pose);
				if(dist < dist_min) {
					dist_min = dist;
					waypoint_min = closest_waypoint_candidates[i];
				}
			}
			
			closest_waypoint_ = waypoint_min;
			closest_waypoint_candidates.clear();
		}
		else ROS_INFO("THERE IS NO CLOSEST WAYPOINT CANDIDATE.");
	}

	float calcPlaneDist(const geometry_msgs::PoseStamped pose1, const geometry_msgs::PoseStamped pose2) {
		float dist = sqrtf(powf(pose1.pose.position.x - pose2.pose.position.x, 2) + powf(pose1.pose.position.y - pose2.pose.position.y, 2));
		return dist;
	}  
};

int main(int argc, char** argv) {
	ros::init(argc, argv, "waypoint_loader");
	WaypointLoader wl;
	ros::Rate loop_rate(10);
	
	while(ros::ok()) {
		ros::spinOnce();
		loop_rate.sleep();	
	}
	return 0;
}
