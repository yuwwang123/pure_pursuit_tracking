#include <vector> 
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <math.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <iostream>
#include <fstream>
#include <visualization_msgs/Marker.h>
#include <yuwei_pure_pursuit/CSVReader.h>

//
//#define LOOK_AHEAD_DIST 1.0
//#define P_gain 0.4

using namespace std;
class PurePursuit{

	private:

		ros::Subscriber _odom_sub;
		ros::Publisher _nav_pub;
		ros::Publisher _setpoint_pub;
		vector<tf::Vector3> _waypoints;
		int _current_ind;
		tf::Vector3 _trans_current_wp;

		bool _first_time_run;
        bool _end_of_waypoints;
        float _P_gain;
        float _LOOK_AHEAD_DIST;
        float _velocity;

	public:
		PurePursuit(ros::NodeHandle& n, string file_name){
		    n.param<float>("/pure_pursuit/LOOK_AHEAD_DIST", _LOOK_AHEAD_DIST,1.0);
            n.param<float>("/pure_pursuit/P_gain", _P_gain,0.4);
            n.param<float>("/pure_pursuit/velocity", _velocity,3);

//			_odom_sub = n.subscribe("pf/pose/odom", 1, &PurePursuit::odom_callback, this);
            _odom_sub = n.subscribe("/odom", 1, &PurePursuit::odom_callback, this);

            _nav_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>("/nav", 1);
//             _nav_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>("/vesc/high_level/ackermann_cmd_mux/input/nav_1",1);

			_setpoint_pub = n.advertise<visualization_msgs::Marker>("set_point", 10);
            read_waypoints(file_name);
			_first_time_run = true;
            _end_of_waypoints = false;
		}

		void read_waypoints(string file_name){
            _waypoints.clear();
            CSVReader reader(file_name);

            // Get the data from CSV File
            std::vector<std::vector<std::string> > dataList = reader.getData();

            // Print the content of row by row on screen
            for(std::vector<std::string> vec : dataList){
                string x, y;
                x = vec.at(0);
                y = vec.at(1);
                _waypoints.push_back(tf::Vector3(stof(x),stof(y),0.0));
            }

            for (int i = 0; i<_waypoints.size(); i++){
                visualization_msgs::Marker p;
                p.header.frame_id = "map";
                p.header.stamp = ros::Time::now();
                p.id = i;

                p.type = visualization_msgs::Marker::SPHERE;
                p.action = visualization_msgs::Marker::ADD;

                p.pose.position.x = _waypoints[i].getX();
                p.pose.position.y = _waypoints[i].getY();
                p.pose.orientation.w = 1;
                p.pose.orientation.x = 0;
                p.pose.orientation.y = 0;
                p.pose.orientation.z = 0;

                p.scale.x = p.scale.y = p.scale.z = 0.2;

                p.color.a = p.color.r = 1.0;
                p.color.g = p.color.b = 0.0;

                _setpoint_pub.publish(p);
            }
		}

		void odom_callback(const nav_msgs::Odometry::ConstPtr &odom_msg) {
		    tf::Quaternion q_tf;
			tf::quaternionMsgToTF(odom_msg->pose.pose.orientation, q_tf);
			tf::Vector3 origin;
			origin = tf::Vector3(odom_msg->pose.pose.position.x, odom_msg->pose.pose.position.y, 0.0);
			tf::Transform tf;
			tf.setOrigin(origin);
			tf.setRotation(q_tf);
			tf = tf.inverse();  // transform points in map frame to car frame

			if(_first_time_run){
                _end_of_waypoints =false;
				int min_ind;
				float min_dist = 10000.0;
			 	for (int i=0; i<_waypoints.size(); i++){
			 		tf::Vector3 diff;
			 		diff = _waypoints.at(i) - origin;
			 		if (diff.length()<min_dist){
			 			min_ind = i;
			 			min_dist = diff.length();
			 		}
			 	}
			 	_current_ind = min_ind;
			 	_first_time_run = false;
			 	cout<<"waypoints size: "<<_waypoints.size()<<endl;
			 //	cout<<"min index: "<<min_ind<<endl;
			}

			bool stop = false;
			while (!stop && !_end_of_waypoints){
                if (_current_ind >= _waypoints.size()-1){
                    _end_of_waypoints = true;
                    _first_time_run =true;
                    cout<<"end of waypoints"<<endl;
                    break;
                }

				tf::Vector3 trans_wp = tf * _waypoints.at(_current_ind); // transform into car's frame. This is a copy constructor call.
				if (trans_wp.length() > _LOOK_AHEAD_DIST){
				    stop = true;
				    //cout<< "trans_wp length: "<<trans_wp.length()<<endl;
				}

				++_current_ind;
			}

			interpolate_points(tf*_waypoints.at(_current_ind-2), tf*_waypoints.at(_current_ind-1));
			float curvature = 2*abs(_trans_current_wp.getY())/(_LOOK_AHEAD_DIST * _LOOK_AHEAD_DIST);

            float steering_cmd =  _P_gain * curvature;
            if (_trans_current_wp.getY()<0){steering_cmd *= -1.0;}
            steering_cmd = min(steering_cmd, 0.41f);
            steering_cmd = max(steering_cmd, -0.41f);
			publish_cmds(steering_cmd);

		}

		void interpolate_points(const tf::Vector3& p1, const tf::Vector3& p2){
			_trans_current_wp = (p1+p2)/2;
		}

		void publish_cmds(float steering_cmd){ 
       
	        ackermann_msgs::AckermannDriveStamped ack_msg;
	        ack_msg.header.stamp = ros::Time::now();
	        float velo = -_velocity/M_PI*steering_cmd+ _velocity;
	        ack_msg.drive.speed = _velocity;
//	        if (_end_of_waypoints){ack_msg.drive.speed = 0.0;}
	        ack_msg.drive.steering_angle = steering_cmd;
//	        ack_msg.drive.steering_angle_velocity = 1.0;
	        _nav_pub.publish(ack_msg);
    }

};

int main(int argc, char ** argv) {
    
    ros::init(argc, argv, "pure_pursuit_node");
    ros::NodeHandle n;

    string file_name = "/home/yuwei/rcws/logs/yuwei_wp.csv";
    PurePursuit pp(n, file_name);
    ros::spin();

    return 0;
}



