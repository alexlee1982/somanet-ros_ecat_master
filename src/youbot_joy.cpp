#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Joy.h>

class TeleopYouBot
{
public:
  TeleopYouBot();
  ros::Publisher vel_pub_;
  geometry_msgs::TwistStamped vel;

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  
  ros::NodeHandle nh_;

  int linear_X_, linear_Y_, angular_;
  double l_scale_, a_scale_;
  
  ros::Subscriber joy_sub_;
  
};


TeleopYouBot::TeleopYouBot()
{

  nh_.param("axis_linear_X", linear_X_, linear_X_);
  nh_.param("axis_linear_Y", linear_Y_, linear_Y_);
  nh_.param("axis_angular", angular_, angular_);
  nh_.param("scale_angular", a_scale_, a_scale_);
  nh_.param("scale_linear", l_scale_, l_scale_);

  vel_pub_ = nh_.advertise<geometry_msgs::TwistStamped>("command_velocity_joy", 1);

  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 1, &TeleopYouBot::joyCallback, this);

}

void TeleopYouBot::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  vel.twist.linear.x = l_scale_*joy->axes[linear_X_];
  vel.twist.linear.y = l_scale_*joy->axes[linear_Y_];
  vel.twist.angular.z = a_scale_*joy->axes[angular_];

}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_youbot");
  TeleopYouBot teleop_youbot;
  ros::Rate loop_rate(10);
   
  while (ros::ok())
  {
	teleop_youbot.vel.header.stamp = ros::Time::now();  	
	teleop_youbot.vel_pub_.publish(teleop_youbot.vel);
	ros::spinOnce();
	loop_rate.sleep();  
  }
}
