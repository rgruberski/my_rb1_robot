#include "ros/forwards.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf/tf.h>

#include <my_rb1_msgs/Rotate.h>

class RobotRotateService {

public:
  // ROS Objects
  ros::NodeHandle nh_;

  // ROS Services
  ros::ServiceServer robot_rotate_service;

  // ROS Subscribers
  ros::Subscriber odom_sub;

  // ROS Publishers
  ros::Publisher vel_pub;

  // ROS Messages
  geometry_msgs::Twist vel_msg;

  RobotRotateService() {

    robot_rotate_service = nh_.advertiseService(
        "/rotate_robot", &RobotRotateService::my_callback, this);
    ROS_INFO("The Service /rotate_robot is READY");

    odom_sub =
        nh_.subscribe("odom", 1000, &RobotRotateService::odomCallback, this);

    vel_pub = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  }

  void odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
    tf::Pose pose;
    tf::poseMsgToTF(msg->pose.pose, pose);
    yaw = tf::getYaw(pose.getRotation());
  }

  bool my_callback(my_rb1_msgs::Rotate::Request &req,
                   my_rb1_msgs::Rotate::Response &res) {
    ROS_INFO("The Service /rotate_robot has been called");

    double distance = 0.0;

    vel_msg.angular.z = -0.5 * ((float)req.degrees / std::abs(req.degrees));
    vel_pub.publish(vel_msg);

    yaw_old = std::abs(yaw);

    while (distance < std::abs(req.degrees)) {
      double distance_rad = std::abs(std::abs(yaw) - yaw_old);
      distance += (180 / M_PI) * distance_rad;
      yaw_old = std::abs(yaw);

      ros::spinOnce();
    }

    vel_msg.angular.z = 0.0;
    vel_pub.publish(vel_msg);

    res.result = "true";
    ROS_INFO("Finished service /rotate_robot");
    return true;
  }

private:
  double yaw;
  double yaw_old;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "rotate_robot_service_node");

  RobotRotateService service;

  ros::spin();

  return 0;
}
