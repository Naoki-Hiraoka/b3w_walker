#include<controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

class MyRobot : public hardware_interface::RobotHW
{
public:
  MyRobot()
  {
    // connect and register the joint state interface
    hardware_interface::JointStateHandle state_handle_a("ld_hinge", &pos[0], &vel[0], &eff[0]);
    jnt_state_interface.registerHandle(state_handle_a);

    hardware_interface::JointStateHandle state_handle_b("lu_hinge", &pos[1], &vel[1], &eff[1]);
    jnt_state_interface.registerHandle(state_handle_b);

    registerInterface(&jnt_state_interface);

    // connect and register the joint position interface
    hardware_interface::JointHandle pos_handle_a(jnt_state_interface.getHandle("ld_hinge"), &cmd[0]);
    jnt_pos_interface.registerHandle(pos_handle_a);

    hardware_interface::JointHandle pos_handle_b(jnt_state_interface.getHandle("lu_hinge"), &cmd[1]);
    jnt_pos_interface.registerHandle(pos_handle_b);

    registerInterface(&jnt_pos_interface);

    pre_ts = ros::Time::now();
  }

  void read(){
    ROS_INFO_STREAM("read"<<" "<<pos[0]<<" "<<pos[1]<<std::endl);
  }

  void write(){
    ROS_INFO_STREAM("write"<<" "<<cmd[0]<<" "<<cmd[1]<<std::endl);
  }

  ros::Time get_time(){
    ts=ros::Time::now();
    return ts;
  }
  ros::Duration get_period(){
    //ros::Duration d = ts-pre_ts;
    ros::Duration d{0.1};
    pre_ts=ts;
    return d;
  }
  
private:
  hardware_interface::JointStateInterface jnt_state_interface;
  hardware_interface::PositionJointInterface jnt_pos_interface;
  double cmd[2]={0,0};

  double pos[2]={0,0};
  double vel[2]={0,0};
  double eff[2]={0,0};

  ros::Time pre_ts{};
  ros::Time ts{};
};
main(int argc, char **argv)
{
  ros::init(argc, argv, "sampleHW");
  ros::NodeHandle n;
  MyRobot robot;
  controller_manager::ControllerManager cm(&robot,n);
  ros::Rate loop_rate(10);

  ros::AsyncSpinner spinner{1};
  spinner.start();
  while (ros::ok())
    {
      robot.read();
      cm.update(robot.get_time(), robot.get_period());
      robot.write();
      //ros::spinOnce(); これを使うとコントローラを追加しようとするとプログラムが停止する。
      loop_rate.sleep();
    }
  spinner.stop();
}
