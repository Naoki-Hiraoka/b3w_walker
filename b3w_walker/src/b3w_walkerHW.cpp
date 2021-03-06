#include<controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

static const std::vector<std::string> joint_names{"r_shoulder_pitch",
    "r_shoulder_roll",
    "r_elbow_roll",
    "l_shoulder_pitch",
    "l_shoulder_roll",
    "l_elbow_roll",
    "r_footjoint_roll",
    "r_footjoint_pitch",
    "r_ankle_pitch",
    "r_ankle_roll",
    "l_footjoint_roll",
    "l_footjoint_pitch",
    "l_ankle_pitch",
    "l_ankle_roll"
    };


class MyRobot : public hardware_interface::RobotHW
{
public:
  MyRobot():
    cmd(joint_names.size(),0.0),
    pos(joint_names.size(),0.0),
    vel(joint_names.size(),0.0),
    eff(joint_names.size(),0.0)
  {
    // connect and register the joint state interface
    for(int i=0;i<joint_names.size();i++){
      hardware_interface::JointStateHandle state_handle(joint_names[i], &pos[i], &vel[i], &eff[i]);
      jnt_state_interface.registerHandle(state_handle);
    }
    registerInterface(&jnt_state_interface);

    // connect and register the joint position interface
    for(int i=0;i<joint_names.size();i++){
      hardware_interface::JointHandle pos_handle(jnt_state_interface.getHandle(joint_names[i]), &cmd[i]);
      jnt_pos_interface.registerHandle(pos_handle);
    }
    registerInterface(&jnt_pos_interface);

    ts = ros::Time::now();

    //wiringpi
    
  }

  void read(){//センサーが無いので、適当に作る
    std::vector<double> new_pos(joint_names.size());
    std::vector<double> new_vel(joint_names.size());
    std::vector<double> new_eff(joint_names.size());
    for(int i=0;i<joint_names.size();i++){
      new_pos[i]=cmd[i];
    }
    if(get_period().toSec()!=0){
      for(int i=0;i<joint_names.size();i++){
	new_vel[i]=(new_pos[i]-pos[i])/get_period().toSec();
      }
    }else{
      for(int i=0;i<joint_names.size();i++){
	new_vel[i]=0.0;
      }
    }
    for(int i=0;i<joint_names.size();i++){
      new_eff[i]=0.0;
    }

    for(int i=0;i<joint_names.size();i++){
      pos[i]=new_pos[i];
      vel[i]=new_vel[i];
      eff[i]=new_eff[i];
    }
  }

  void write(){    
    ROS_INFO_STREAM("write"<<" "<<cmd[0]<<" "<<cmd[1]<<" "<<cmd[2]<<" "<<cmd[3]<<" "<<cmd[4]<<" "<<cmd[5]<<" "<<cmd[6]<<" "<<cmd[7]<<" "<<cmd[8]<<" "<<cmd[9]<<" "<<cmd[10]<<" "<<cmd[11]<<" "<<cmd[12]<<" "<<cmd[13]);
  }

  ros::Time get_time(){
    pre_ts=ts;
    ts=ros::Time::now();
    return ts;
  }
  ros::Duration get_period(){
    ros::Duration d = ts-pre_ts;
    //ros::Duration d{0.1};    
    return d;
  }
  
private:
  hardware_interface::JointStateInterface jnt_state_interface;
  hardware_interface::PositionJointInterface jnt_pos_interface;
  std::vector<double> cmd;

  std::vector<double> pos;
  std::vector<double> vel;
  std::vector<double> eff;

  ros::Time pre_ts{};
  ros::Time ts{};
};
main(int argc, char **argv)
{
  ros::init(argc, argv, "sampleHW");
  ros::NodeHandle n;
  MyRobot robot;
  controller_manager::ControllerManager cm(&robot,n);
  ros::Rate loop_rate(30);

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
