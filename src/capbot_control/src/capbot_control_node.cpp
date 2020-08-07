#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <realtime_tools/realtime_buffer.h>



class capbot_control : public hardware_interface::RobotHW
{
public:
  capbot_control(){

    //enviar velocidad inicial al robot

    //valores iniciales para las variables de comandos
    pos_[0] = 0.0; pos_[1] = 0.0;
    vel_[0] = 0.0; vel_[1] = 0.0;
    eff_[0] = 0.0; eff_[1] = 0.0;
    cmd_[0] = 0.0; cmd_[1] = 0.0;


    hardware_interface::JointStateHandle state_joint_1("R_joint", &pos_[0], &vel_[0], &eff_[0]);
    jnt_state_interface_.registerHandle(state_handle_1);

    hardware_interface::JointStateHandle state_joint_2("L_joint", &pos_[1], &vel_[1], &eff_[1]);
    jnt_state_interface_.registerHandle(state_handle_2);

    registerInterface(&jnt_state_interface_);

    hardware_interface::JointHandle vel_handle_1(jnt_state_interface_.getHandle("R_joint"), &cmd_[0]);
    jnt_vel_interface_.registerHandle(vel_handle_1);

    hardware_interface::JointHandle vel_handle_2(jnt_state_interface_.getHandle("L_joint"), &cmd_[1]);
    jnt_vel_interface_.registerHandle(vel_handle_2);

    registerInterface(&jnt_vel_interface_);
  }

  //crear funcion para parar

    
    ros::Time getTime() const {return ros::Time::now();}
    ros::Duration getPeriod() const {return ros::Duration(0.01);}
  
  //funcion de reconectar

  void read(){
    ROS_INFO_STREAM("Steppers vel: " << st_vel[0] << ", " << st_vel[1]);
    //cmd_[1], -cmd_[0]
   for (unsigned int i = 0; i < 2; ++i)
    {
      pos_[i] += 1 //leer el ingremento desde arduino
      vel_[i] = 1  //leer la velocidad actual desde arduino
    }
  }

  void write(){
    ROS_INFO_STREAM("Commands for joints: " << cmd_[0] << ", " << -cmd_[1]);

    //escribir los comandos a la arduino


  }

private:
  hardware_interface::JointStateInterface    jnt_state_interface_;
  hardware_interface::VelocityJointInterface jnt_vel_interface_;
  double cmd_[2];
  double pos_[2];
  double vel_[2];
  double eff_[2];
};

int main(int argc, char **argv)
{
  double x, y, theta;
  
  ros::init(argc, argv, "icart_mini");
  ros::NodeHandle nh;
    
  TFrog robot;
  ROS_INFO_STREAM("period: " << robot.getPeriod().toSec());
  controller_manager::ControllerManager cm(&robot, nh);

  ros::Rate rate(1.0 / robot.getPeriod().toSec());
  ros::AsyncSpinner spinner(1);
  spinner.start();

  while(ros::ok())
  {
    int state = YP_get_error_state();
    
    if(state == 0){
        robot.read();
        robot.write();
    }else{
        ROS_WARN("Disconnected T-frog driver");
        robot.reopen();
    }
    
    cm.update(robot.getTime(), robot.getPeriod());
    rate.sleep();
  }
  spinner.stop();

  return 0;
}
