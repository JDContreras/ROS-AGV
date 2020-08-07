//ROS
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int64.h>
#include <ros/callback_queue.h>
//ROS_CONTROL
#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <realtime_tools/realtime_buffer.h>



class capbot_control : public hardware_interface::RobotHW
{
public:
  capbot_control(){ //constructor

    //valores iniciales para las variables de comandos
    pos_[0] = 0.0; pos_[1] = 0.0; //posicion que lee de las ruedas
    vel_[0] = 0.0; vel_[1] = 0.0; //velocidad que "lee" de las ruedas
    eff_[0] = 0.0; eff_[1] = 0.0; //torque o corriente--no se usa en este robot
    cmd_[0] = 0.0; cmd_[1] = 0.0; //variable para los comandos que llegan del roscontrol a las ruedas
    max_speed = 18.85; //maxima velocidad en r/s --> 3 rps ---> 180 rpm

    //interfaz al roscontrol para actualice el estado de las variables leidas del robot
    hardware_interface::JointStateHandle state_joint_1("R_joint", &pos_[0], &vel_[0], &eff_[0]);
    jnt_state_interface_.registerHandle(state_joint_1);

    hardware_interface::JointStateHandle state_joint_2("L_joint", &pos_[1], &vel_[1], &eff_[1]);
    jnt_state_interface_.registerHandle(state_joint_2);

    registerInterface(&jnt_state_interface_);

    //interfaz al roscontrol para obtener el valor de velocidad enciado del diff_driver
    hardware_interface::JointHandle vel_handle_1(jnt_state_interface_.getHandle("R_joint"), &cmd_[0]);
    jnt_vel_interface_.registerHandle(vel_handle_1);

    hardware_interface::JointHandle vel_handle_2(jnt_state_interface_.getHandle("L_joint"), &cmd_[1]);
    jnt_vel_interface_.registerHandle(vel_handle_2);

    registerInterface(&jnt_vel_interface_);
    
    //inicializar pub & sub
    position_left_sub = nh.subscribe("left_count", 1, &capbot_control::left_count_callback, this);
    position_right_sub = nh.subscribe("right_count", 1, &capbot_control::right_count_callback, this);
    speed_left_pub  =  nh.advertise<std_msgs::Float32>("left_vel", 1);
    speed_right_pub = nh.advertise<std_msgs::Float32>("right_vel", 1);

  }

  //crear funcion para parar

    
    ros::Time getTime() const {return ros::Time::now();}
    ros::Duration getPeriod() const {return ros::Duration(0.02);}
  
  //funcion de reconectar

  void read(const ros::Duration &period){

    double distance_left = wheel_angle[0]*0.015708;
    double distance_right = wheel_angle[1]*0.015708;
    pos_[0] +=  distance_left;
    pos_[1] +=  distance_right;
    vel_[0] +=  distance_left / period.toSec();
    vel_[1] +=  distance_right / period.toSec();
  }

  void write(){
    ROS_INFO_STREAM("Commands for joints: " << cmd_[0] << ", " << cmd_[1]);
    double speed_left = cmd_[0];
    double speed_right = cmd_[1];
    double speed = std::max(std::abs(speed_left),std::abs(speed_right));
    if (speed > max_speed){ 
            speed_left *=  max_speed/speed;
            speed_right *=  max_speed/speed;   
    }//if
    //inicializa los objetos para mensajes
    std_msgs::Float32 speed_left_msgs;
    std_msgs::Float32 speed_right_msgs;
    //actualiza el valor de cada mensaje
    speed_left_msgs.data = speed_left;
    speed_right_msgs.data =  speed_right;
    //publica los mensajes
    speed_left_pub.publish(speed_left_msgs);
    speed_right_pub.publish(speed_left_msgs);

  }//write

private:
//inicaliar las variables
    double cmd_[2];
    double pos_[2];
    double vel_[2];
    double eff_[2];
    double max_speed;
    long int wheel_angle[2];
//inicializar objetos
    hardware_interface::JointStateInterface    jnt_state_interface_;
    hardware_interface::VelocityJointInterface jnt_vel_interface_;
    ros::Subscriber position_left_sub;
    ros::Subscriber position_right_sub;
    ros::Publisher speed_left_pub;
    ros::Publisher speed_right_pub;
    ros::NodeHandle nh;

//Callbacks

    void left_count_callback(const std_msgs::Int64& msg) {
        wheel_angle[0] = msg.data;
    }

    void right_count_callback(const std_msgs::Int64& msg) {
        wheel_angle[1] = msg.data;
    }
};

void controlLoop(capbot_control &hw, controller_manager::ControllerManager &cm, std::chrono::system_clock::time_point &last_time)
{ //creditos de este controlLoo a eborghi10
    std::chrono::system_clock::time_point current_time = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_time = current_time - last_time;
    ros::Duration elapsed(elapsed_time.count());
    last_time = current_time;

    hw.read(elapsed);
    cm.update(ros::Time::now(), elapsed);
    hw.write();
}

int main(int argc, char **argv)
{
  //iniciar el nodo y el nodehandle
  ros::init(argc, argv, "capbot_hardware_interface_node");
  ros::NodeHandle nh;
   
//iniciar el hardware interface
  capbot_control robot;
  ROS_INFO_STREAM("period: " << robot.getPeriod().toSec());

//iniciar el controller manager
  controller_manager::ControllerManager cm(&robot, nh);
    
    double control_frequency ;
    nh.param<double>("control_frequency", control_frequency, 10.0);

    ros::CallbackQueue robot_queue;
    ros::AsyncSpinner spinner(2, &robot_queue);     

    std::chrono::system_clock::time_point last_time = std::chrono::system_clock::now();

    ros::TimerOptions control_timer(
        ros::Duration(1 / control_frequency), 
        std::bind(controlLoop, std::ref(robot), std::ref(cm), std::ref(last_time)), 
        &robot_queue);

    ros::Timer control_loop = nh.createTimer(control_timer);

    spinner.start();

    ros::spin();   

    return 0;                                 

}
