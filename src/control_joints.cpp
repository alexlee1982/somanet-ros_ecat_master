#include "ros/ros.h"
#include "ros_ecat_master/MoveJoint.h"

#include <sstream>

int main(int argc, char **argv)
{

  ros::init(argc, argv, "joints_controller");

  ros::NodeHandle n;

  ros::Publisher joint_pub = n.advertise<ros_ecat_master::MoveJoint>("/move_joints", 1);

  ros::Rate loop_rate(10);

  while (ros::ok())
  {

    ros_ecat_master::MoveJoint msg;
    
    msg.TurnDegree.push_back(300.0);
    msg.TurnDegree.push_back(200.0);
    msg.TurnDegree.push_back(100.0);
    msg.TurnDegree.push_back(350.0);

    msg.VelocityRPM.push_back(350);
    msg.VelocityRPM.push_back(350);
    msg.VelocityRPM.push_back(350);
    msg.VelocityRPM.push_back(350);


    joint_pub.publish(msg);

    ros::spinOnce();
    loop_rate.sleep();
  }


  return 0;
}
