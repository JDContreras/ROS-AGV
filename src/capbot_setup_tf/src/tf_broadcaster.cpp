#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

//enviar la informacion de tf del robot
int main(int argc, char** argv){
  ros::init(argc, argv, "capbot_tf_publisher");
  ros::NodeHandle n;

  ros::Rate r(200);

  tf::TransformBroadcaster broadcaster;
  ROS_INFO("running");
  while(n.ok()){
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.0, 0.3)),
        ros::Time::now(),"base_link", "laser"));
    r.sleep();
  }
}
