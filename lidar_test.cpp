	#include "rclcpp/rclcpp.hpp"
	#include "sensor_msgs/msg/laser_scan.hpp"
	#include "geometry_msgs/msg/twist.hpp"
	#include "std_msgs/msg/string.hpp"

	class LidarSubscriberNode : public rclcpp::Node {
	public:
	    LidarSubscriberNode() : Node("lidar_subscriber") {
				 subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>("scan", 10, std::bind(&LidarSubscriberNode::laserCallback, this, std::placeholders::_1));

	       publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
				 action_publisher_ = this->create_publisher<std_msgs::msg::String>("dog_action", 10);
	   		 timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&LidarSubscriberNode::control, this));
	   		 have_right_obs = false;
	 }

	private:
	 void turn_to_far() {
				if(ahead_dis > 1.5) {				    //recover();
						if(ahead_dis > 3) {
								auto message = std_msgs::msg::String();
								message.data = "swing";
								action_publisher_->publish(message);
								action_show = true;
						}
						step = 0;
				    return;
				}
			auto twist_msg = std::make_shared<geometry_msgs::msg::Twist>();

				twist_msg->linear.x = 0;
				twist_msg->linear.y = 0;
	    	twist_msg->linear.z = 0;
	    	twist_msg->angular.x = 0;
	    	twist_msg->angular.y = 0;
	    	twist_msg->angular.z = 0.2;
				publisher_->publish(*twist_msg);
	  }


	  void stop() {
	     	auto twist_msg = std::make_shared<geometry_msgs::msg::Twist>();

				twist_msg->linear.y = 0;
	    	twist_msg->linear.z = 0;
	    	twist_msg->angular.x = 0;
	    	twist_msg->angular.y = 0;
	    	twist_msg->angular.z = 0;
	     	twist_msg->linear.x = 0.0;

			 publisher_->publish(*twist_msg);
	  }

	 void recover() {
	     	auto twist_msg = std::make_shared<geometry_msgs::msg::Twist>();

				twist_msg->linear.y = 0;
	    	twist_msg->linear.z = 0;
	    	twist_msg->angular.x = 0;
	    	twist_msg->angular.y = 0;
	    	twist_msg->angular.z = -0.2;
	     	twist_msg->linear.x = 0.0;

			 publisher_->publish(*twist_msg);

	     }

	  void check_and_walk() {
	      auto twist_msg = std::make_shared<geometry_msgs::msg::Twist>();
				RCLCPP_INFO(this->get_logger(), "ahead_dis = %f side_obs = %d, back_dis = %f",ahead_dis,side_obs,back_dis);

				if(back_dis != 0 && back_dis < 0.3) {
						auto message = std_msgs::msg::String();
						message.data = "pee";
						action_publisher_->publish(message);
						action_show = true;
						step = 0;
						return;
				}

	     	if((int)ahead_dis == -1 || ahead_dis == 0.0)
							return;
				if(ahead_dis > 1) {
	     	   twist_msg->linear.y = 0;
	    	   twist_msg->linear.z = 0;
	    	   twist_msg->angular.x = 0;
	    	   twist_msg->angular.y = 0;
	    	   twist_msg->angular.z =  0; //0.1 * side_obs;   //0
	     	   twist_msg->linear.x = 0.2;

			 	 	publisher_->publish(*twist_msg);
	     	}
	     	else
	     	   step = 1;
	  }

	void control()  {
		 if(action_show) {
				 action_count++;
				 if(action_count < 100)  //10 seconds
				 	  return;
				 else {
					 action_count = 0;
					 action_show = false;
					 auto message = std_msgs::msg::String();
					 message.data = "restore";
					 action_publisher_->publish(message);
				 }
		 }

		 if(step == 0)
		   check_and_walk();

	   else if(step == 1)   //if
		    turn_to_far();

		 else
			   step = 0;


	}


void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
		float max_range = 0.0;
		int i = 0;
		float temp;

		float deg = msg->angle_increment * 57.2957795;  //1 degree
		float theta;

		ahead_dis = -1;
		have_right_obs = false;

					for (float range : msg->ranges) {
					   theta = deg * i;  //theta increment in 1 degree each

					    if(std::isfinite(range) && i == 0)
					    	ahead_dis = range;

					    if(std::isfinite(range) && (int)theta == 90 && range != 0 && range < 0.3)
					        side_obs = -1;
					    else if(std::isfinite(range) && (int)theta == 270 && range != 0 && range < 0.3)
					        side_obs = 1;

							if(std::isfinite(range) && (int)theta == 180)
							   back_dis = range;
							 // have_right_obs = true;
							  //back_dis = range;
								//have_right_obs = true;

					    if((int)theta < 90 || (int)theta > 270)  {
							    if (std::isfinite(range) && range > max_range) {
											max_range = range;
											far_pos = (int)theta;
							    }
					    }
					    i++;
					}
										//RCLCPP_INFO(this->get_logger(), "have_right_obs = %d",have_right_obs);
						far_dis = max_range;
	    }

	    int robot_dir = -1;  //0 1 2 3 -> fw bw L R
	    //bool dead = false;
	    int far_pos;
	    float far_dis;
	    float ahead_dis;
			float back_dis = 0;
	    int count = 0;
	    bool turn_ok = false;
	    int step = 0;
	    int walk_count = 0;
	    int side_obs = 0;   //0 - left, 1 -right
	    bool have_right_obs = false;
      bool action_show = false;
			int action_count = 0;

	    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
			rclcpp::Publisher<std_msgs::msg::String>::SharedPtr action_publisher_;
	    rclcpp::TimerBase::SharedPtr timer_;
	    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
	};

	int main(int argc, char **argv) {
	    rclcpp::init(argc, argv);
	    auto node = std::make_shared<LidarSubscriberNode>();

	    rclcpp::spin(node);
	    rclcpp::shutdown();
	    return 0;
	}

