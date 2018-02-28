/*
 * For robot teleoperation using the haptic device.
 *
 * Author: Jack Liu <jacknlliu@gmail.com>
 */
#ifndef TELEOP_LISTENER_H_
#define TELEOP_LISTENER_H_

#include <mutex>
#include <condition_variable>
#include <thread>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/WrenchStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "dhdc.h"

#define TELEOP_MODE 0
#define AUTO_MODE 1
#define SHARED_AUTONOMY_MODE 2

// declaration the class
class TeleopRobot {
public:
  TeleopRobot(int device_id, int operator_mode); // init and setup callback and new thread
  ~TeleopRobot();

  // callback function for receiving topics to update robot state
  // void jointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg);
  // void wrenchCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg);
  // void toolCallback(const geometry_msgs::TwistStamped::ConstPtr& msg);
  ros::Timer timer;
  void sendPositionTimer(const ros::TimerEvent& event);
  void doAutonomyOP();
//  void robotStateCallback(const sensor_msgs::JointState::ConstPtr& joint_msg, const geometry_msgs::WrenchStamped::ConstPtr& wrench_msg);
  void jointStatesCallback(const sensor_msgs::JointState::ConstPtr& joint_msg);
  void wrechCallback(const geometry_msgs::WrenchStamped::ConstPtr& wrench_msg);
  void toolCallback(const geometry_msgs::PoseStamped::ConstPtr& tool_msg);

  void velocityCallback(const geometry_msgs::TwistStamped::ConstPtr& vel_msg);  

  // publish script msg to topic
  int speedL(double end_relative_pose[]);
  int moveEndL(double pose[], double a=1.2, double v=0.25, double t=0.02, double r=0);
  int moveEndRelativeL(double pose[], double a=1.2, double v=0.25, double t=0.02, double r=0);
  // int getRobotJointStates(double state[], int size);
  int getJointStates(double angle[], int length);
  int getWrenchState(double wrech[], int length);
  int getToolPose(double tool_pose[], int length);
  int setToolPose(double tool_pose[], int length);
  int halt(); // join the listenHandlerThread
  int moveServoj(double pose[], double a, double v, double t=0);


  void setHandlerLockFlag(bool flag);
  bool isHandlerLocked();


protected:
  ros::NodeHandle nh_;
  ros::Subscriber sub;
  ros::Subscriber sub_vel;
  ros::Subscriber sub_wrench;
  ros::Subscriber sub_tool;
  ros::ServiceClient gmm_client;

  ros::Publisher urscript_sub_;

  int operator_mode=TELEOP_MODE;

  std::thread* listen_handler_thread;
  void listenHandlerThread();
  std::thread* handler_get_thread;
  void handlerGetThread();

private:
  bool keepalive_ = true;

  // locks the state variables while unpack the data
//  std::mutex val_lock_;
  std::mutex val_lock1_; // protect joints state, one lock may be not good?
  std::mutex val_lock2_;
  std::mutex val_lock3_;
  std::mutex val_lock4_;
  std::condition_variable msg_cond_; // send lock signal to who concerned

  // store the current robot state
  double robot_joints_pose_[6];
  double robot_end_pose_[6];
  double robot_end_vel_[6];
  double robot_end_force_[6];
  double robot_joints_force_[6];

  // store current haptic device data
  bool handler_lock_flag = true;  // the handler could not be used to send data
  std::mutex flag_lock_;  // for handler_lock_flag sync
  std::mutex handler_pose_lock;  // for handler pose data sync

  int deviceID;
  double handler_pose_[3];
  double handler_pose_buf_[3];
  double handler_force_[3]; // not used
  double handler_strech_[3];
};

#endif
