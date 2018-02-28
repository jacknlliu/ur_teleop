#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

// for timer
#include <sys/time.h>
#include <signal.h>
#include <string.h>

#include <math.h>

#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/WrenchStamped.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include "dhdc.h"
#include "teleop_listener.h"
#include "ur_teleop/GmmRegression.h"

#include <seds/GMR.h>
using namespace MathLib;

// for log file
#include <fstream>
#include <iostream>
using namespace std;

using namespace sensor_msgs;
using namespace message_filters;

ofstream control_cmd_file;
GaussianMixture gmm_model;

// for keyboard listen
int kbhit(void);
char getKbhit();

// for force feedback
int sign_float(double x);
void setForceFeedback(double x, double y, double z, int id, double limit);


// /*
//  * This tutorial demonstrates simple receipt of messages over the ROS system.
//  */
// void jointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg)
// {
//   ROS_INFO("I heard position message at: [%i]", msg->header.stamp.sec);
//   for (unsigned int i = 0; i < msg->position.size(); i++) {
//       ROS_INFO("position %i: %f", i,msg->position[i]);
//   }
//
//   for (unsigned int i = 0; i < msg->velocity.size(); i++) {
//       ROS_INFO("velocity %i: %f", i,msg->velocity[i]);
//   }
// }
//
// void wrechCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg)
// {
//   ROS_INFO("I heard wrench message at: [%i]", msg->header.stamp.sec);
//   ROS_INFO("wrech force x: %f", msg->wrench.force.x);
//   ROS_INFO("wrech force y: %f", msg->wrench.force.y);
//   ROS_INFO("wrech force z: %f", msg->wrench.force.z);
//
//   ROS_INFO("wrech torque x: %f", msg->wrench.torque.x);
//   ROS_INFO("wrech torque y: %f", msg->wrench.torque.y);
//   ROS_INFO("wrech torque z: %f", msg->wrench.torque.z);
// }

TeleopRobot::TeleopRobot(int device_id, int op_mode): deviceID(device_id), operator_mode(op_mode)
{
  // init data member
  for (int32_t i = 0; i < 6; i++) {
    robot_joints_pose_[i]   = 0.0;
    robot_end_pose_[i]      = 0.0;
    robot_end_force_[i]     = 0.0;
    robot_joints_force_[i]  = 0.0;
  }

  for (int32_t i = 0; i < 3; i++) {
    handler_pose_[i]   = 0.0;
    handler_pose_buf_[i] = 0.0;
    handler_force_[i]  = 0.0;
    /* set 10.0 always works, but 1.0-5.0 will more stable, 5.0 test most,
     * but a little more lookahead, then 2.0 a little less lookahead
     * and 1.0 more little.
     */
    handler_strech_[i] = 1.0;
  }

  // setup ros message callback
//  message_filters::Subscriber<sensor_msgs::JointState> joint_states_sub(nh_, "joint_states", 1);
//  message_filters::Subscriber<geometry_msgs::WrenchStamped> wrench_sub(nh_, "wrench", 1);
//  message_filters::Subscriber<geometry_msgs::PoseStamped> tool_pose_sub(nh_, "tool_pose", 1);
//  TimeSynchronizer<sensor_msgs::JointState, geometry_msgs::WrenchStamped> sync(joint_states_sub, wrench_sub,10);
//  sync.registerCallback(boost::bind(&TeleopRobot::robotStateCallback, this, _1, _2));
  sub = nh_.subscribe("joint_states", 20, &TeleopRobot::jointStatesCallback, this);

  sub_vel = nh_.subscribe("tool_velocity", 20, &TeleopRobot::velocityCallback, this);

  sub_wrench = nh_.subscribe("wrench", 20, &TeleopRobot::wrechCallback, this);

  sub_tool = nh_.subscribe("tool_pose", 20, &TeleopRobot::toolCallback, this);

  gmm_client = nh_.serviceClient<ur_teleop::GmmRegression>("gmr_srv");

  // publisher
  urscript_sub_ = nh_.advertise<std_msgs::String>(
      "ur_driver/URScript", 1);

  // set timer
  // struct sigaction tact;
  // tact.sa_handler = TeleopRobot::sendPositionTimer;
  // tact.sa_flags = 0;
  //
  // sigemptyset(&tact.sa_mask);
  // sigaction(SIGALRM, &tact, NULL);
  //
  // struct itimerval value;
  // value.it_value.tv_sec = 0;
  // value.it_value.tv_usec = 20000;
  // value.it_interval = value.it_value;
  // setitimer(ITIMER_REAL, &value, NULL);

  // new thread
  listen_handler_thread = new std::thread(
      boost::bind(&TeleopRobot::listenHandlerThread, this));
  handler_get_thread = new std::thread(boost::bind(&TeleopRobot::handlerGetThread, this));

// we send position to robot by handler every 0.01s once
  // timer = nh_.createTimer(ros::Duration(0.01), &TeleopRobot::sendPositionTimer, this);
  // TODO: set send period as class TeleopRobot member, send_period
  timer = nh_.createTimer(ros::Duration(0.008), &TeleopRobot::sendPositionTimer, this);


  // TODO: set default execute time for velocity mode, position mode and force mode.


  // TODO: replay use force compliance control. action sequence

  keepalive_ = true;
}

TeleopRobot::~TeleopRobot()
{
  // terminate thread
  keepalive_ = false;
  listen_handler_thread->join();
  handler_get_thread->join();
  // clear data
}


bool isEqual(double a, double b)
{
    if ((a-b < 0.00001 && a-b >= 0.0) || (a-b > -0.00001 && a-b <= 0.0) ){
        return true;
    } else {
        return false;
    }
}


int TeleopRobot::speedL(double end_velocity[])
{
  char cmd[1024];
  std_msgs::String joint_msg;
  double period = 0.01; // unit: s, send period, not execute period
  double current_end_vel[6] = {0.0,0.0, 0.0, 0.0, 0.0, 0.0};
 
  double  t = period;

  double average_vel= 0.0;

  for (int i=0;i<6;i++) {
    current_end_vel[i] = robot_end_vel_[i];
  }

  val_lock4_.lock();

  for (int i = 0; i < 6; i++)
  {
        average_vel = average_vel + fabs(end_velocity[i]-current_end_vel[i]);
  }

  val_lock4_.unlock();

  average_vel= average_vel/6.0;

   // acceleration should be difference between the reference velocity and the current velocity
  double  accel = average_vel/t;

// try to add a protect speed
  if (fabs(accel) > 1.0 ) {
    accel = 1.0;
  }

  // for pose:=zero move
  if (isEqual(end_velocity[0], 0.0) && isEqual(end_velocity[1], 0.0) && isEqual(end_velocity[2], 0.0) &&
          isEqual(end_velocity[3], 0.0) && isEqual(end_velocity[4], 0.0) && isEqual(end_velocity[5], 0.0)) {
    accel = 0.000001;
    t = 0.008;
  }

  sprintf(cmd, "speedl([%f, %f, %f, %f, %f, %f], %f, %f)\n",
          end_velocity[0], end_velocity[1],end_velocity[2], end_velocity[3], end_velocity[4], end_velocity[5], accel, t);

  joint_msg.data = cmd;
  urscript_sub_.publish(joint_msg);

// log control command
  double cmd_timestamp =ros::Time::now().toSec();
  control_cmd_file<<std::fixed<<setprecision(4)<<cmd_timestamp;
  control_cmd_file<<setprecision(std::cout.precision())<<" "<<end_velocity[0]<<" "<< end_velocity[1]<<" "<<end_velocity[2]<<" "<<accel<<" "<<t<<endl;

  return 0;

}


int TeleopRobot::moveEndL(double pose[], double a, double v, double t, double r)
{
  char cmd[1024];
  std_msgs::String joint_msg;
  double period = 0.02; // unit: s

  // for pose:=zero move
  if (isEqual(pose[0], 0.0) && isEqual(pose[1], 0.0) && isEqual(pose[2], 0.0) &&
          isEqual(pose[3], 0.0) && isEqual(pose[4], 0.0) && isEqual(pose[5], 0.0)) {
    a = 0.000001;
    v = 0.000001;
    t = 0.008;
  }
    double max_pose = fabs(pose[0])>fabs(pose[1])?(fabs(pose[0])>fabs(pose[2])?fabs(pose[0]):fabs(pose[2])):(fabs(pose[1])>fabs(pose[2])?fabs(pose[1]):fabs(pose[2]));
    v = max_pose/period;
    if (v>0.2) {
        v = 0.2;
    }

    double min_pose = fabs(pose[0])<fabs(pose[1])?(fabs(pose[0])<fabs(pose[2])?fabs(pose[0]):fabs(pose[2])):(fabs(pose[1])<fabs(pose[2])?fabs(pose[1]):fabs(pose[2]));
    a = v/period;
    if (a>0.2) {
        a = 0.2;
    }

    r = 0;

    t = period*2.0; // this parameter also can be test. we set 2.0 as default for continuality.

    double lookaheadtime = 0.03; // default at least 0.03 we test. 
    double gain = 100;
//  sprintf(cmd,
//          "movel([%1.5f, %1.5f, %1.5f, %1.5f, %1.5f, %1.5f], %f, %f, %f, %f)\n",
//          pose[0], pose[1], pose[2], pose[3], pose[4], pose[5], a, v, t, r);
  sprintf(cmd, "servoj(get_inverse_kin(pose_trans(get_forward_kin(),p[%1.5f, %1.5f, %1.5f, %1.5f, %1.5f, %1.5f])), %f, %f, %f, %f, %f)\n",
          pose[0], pose[1], pose[2], pose[3], pose[4], pose[5], a, v, t, lookaheadtime, gain);

  joint_msg.data = cmd;
  urscript_sub_.publish(joint_msg);

// log control command
  double cmd_timestamp =ros::Time::now().toSec();
  control_cmd_file<<std::fixed<<setprecision(4)<<cmd_timestamp;
  control_cmd_file<<setprecision(std::cout.precision())<<" "<<pose[0]<<" "<< pose[1]<<" "<<pose[2]<<" "<<a<<" "<<v<<" "<<t<<endl;

  return 0;
}


int TeleopRobot::moveEndRelativeL(double pose[], double a, double v, double t, double r)
{
  val_lock3_.lock();
  for (int i = 0; i < 6; i++) {
    pose[i] = pose[i] + robot_end_pose_[i];
  }

//  ROS_INFO("Origin: [%1.5f, %1.5f, %1.5f, %1.5f, %1.5f, %1.5f]", robot_end_pose_[0], robot_end_pose_[1],
//          robot_end_pose_[2],robot_end_pose_[3], robot_end_pose_[4], robot_end_pose_[5]);
  val_lock3_.unlock();

//  ROS_INFO("Send: [%f, %f, %f, %f, %f, %f]", pose[0], pose[1],
//          pose[2],pose[3], pose[4], pose[5]);

  // comment for test
  moveEndL(pose, a, v, t, r);

  return 0;
}


int TeleopRobot::getJointStates(double angle[], int length)
{
  if (6 == length) {
    val_lock1_.lock();

    for (int i = 0; i < 6; i++) {
      angle[i] = robot_joints_pose_[i];
    }

    val_lock1_.unlock();
    return 0;
  } else {
    return -1;
  }
}


int TeleopRobot::getWrenchState(double wrech[], int length)
{
  if (6 == length) {
    val_lock2_.lock();

    for (int i = 0; i < 6; i++) {
      wrech[i] = robot_end_force_[i];
    }

    val_lock2_.unlock();
    return 0;
  } else {
    return -1;
  }
}


int TeleopRobot::getToolPose(double tool_pose[], int length)
{
  if (6 == length) {
    val_lock3_.lock();

    for (int i = 0; i < 6; i++) {
      tool_pose[i] = robot_end_pose_[i];
    }

    val_lock3_.unlock();
    return 0;
  } else {
    return -1;
  }
}


int TeleopRobot::setToolPose(double tool_pose[], int length)
{
  return 0;
}


int TeleopRobot::halt()
{
  keepalive_ = false;

  if (listen_handler_thread!=NULL) {
      listen_handler_thread->join();
  }

  if (handler_get_thread!=NULL) {
      handler_get_thread->join();
  }

  return 0;
}

// must new this thread
void TeleopRobot::handlerGetThread()
{
  // get handler pose data
  while(ros::ok() && keepalive_ ) {
    if (deviceID >= 0) {
	    handler_pose_lock.lock();
	    dhdGetPosition(&handler_pose_[0], &handler_pose_[1], &handler_pose_[2], deviceID);
	    handler_pose_lock.unlock();
    }
    usleep(1000);  // 1ms to get a new position of handler
  }
}

void TeleopRobot::listenHandlerThread()
{
  // while(ros::ok && not 'q') for handler listen and using keyboard P to get pose
  // use time delay not timer callback to do it
  // get handler pose & update stored last time data
   if (ros::ok() && keepalive_ ) {
    static int count = 0;
    double a[3] = {0.0, 0.0, 0.0};
    double init_vel[6]={0.0, 0.0, 0.0,0.0, 0.0, 0.0};
    char input;

    while(ros::ok() && 'q' != (input=getKbhit())) {
      if (input == 'p') {
        count = (count+1)%2;
        speedL(init_vel);
        if (count%2) {
          // start to listen
          //  dhdGetPosition(&a[0], &a[1], &a[2], deviceID);
           handler_pose_lock.lock();
           handler_pose_buf_[0] = handler_pose_[0];
           handler_pose_buf_[1] = handler_pose_[1];
           handler_pose_buf_[2] = handler_pose_[2];
           handler_pose_lock.unlock();
           setHandlerLockFlag(false);
           ROS_INFO("Start getting handler data...");
        } else {
           setHandlerLockFlag(true);
           ROS_INFO("Stop getting handler data!");
        }
      }
      usleep(1000); // for avoiding cpu 100%
    }

    speedL(init_vel);
    ROS_INFO("Exit listen handler data");
    setHandlerLockFlag(true);

    // let's shutdown everything!
    keepalive_ = false;
    ros::shutdown();
  }
}


void TeleopRobot::doAutonomyOP()
{
        double map_pos[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        // compute the regression data

        // get target_pose from the vision input
        // double target_pose[6] = {-0.2876,0.6366,0.2900,-0.9019,-2.9250,0.4525};
        double target_pose[6] = {-0.08974,0.62254,-0.03570,1.2600,2.2420,-0.7666};

        //  define  a input feature vector
        double current_tool_pose[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        getToolPose(current_tool_pose, 6);
        
        Matrix  test_vector(1,6);
        test_vector(0,0) = current_tool_pose[0] - target_pose[0];
        test_vector(0,1) = current_tool_pose[1] - target_pose[1];
        test_vector(0,2) = current_tool_pose[2] - target_pose[2];
        test_vector(0,3) = current_tool_pose[3] - target_pose[3];
        test_vector(0,4) = current_tool_pose[4] - target_pose[4];
        test_vector(0,5) = current_tool_pose[5] - target_pose[5];

        Matrix SigmaOut;

        Matrix output = gmm_model.doRegression(test_vector, &SigmaOut);

        // use service to get remote call result
        ur_teleop::GmmRegression srv;
        for (int i=0;i<6;i++) {
          srv.request.pose[i] = test_vector(0,i);
        }

        if (gmm_client.call(srv))
        {
             ROS_INFO("receive something: %f", srv.response.velocity[0]);
        }
        else
        {
            ROS_ERROR("Failed to call services");
        } //end remote call

        map_pos[0] = output(0,0);
        map_pos[1] = output(0,1);
        map_pos[2] = output(0,2);
        map_pos[3] = output(0,3);
        map_pos[4] = output(0,4);
        map_pos[5] = output(0,5);

        ROS_INFO("send pos:[%1.5f, %1.5f, %1.5f, %1.5f, %1.5f, %1.5f]", map_pos[0], map_pos[1], map_pos[2], map_pos[3], map_pos[4], map_pos[5]);
      // moveEndL(map_pos, 0.001, 0.001, 0.02, 0);
       speedL(map_pos);
}


void TeleopRobot::sendPositionTimer(const ros::TimerEvent& event)
{
  // delta handler pose add the end tool pose
  // get the new data from handler
  double new_pos[3] = {0.0, 0.0, 0.0};
  double send[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  double map_pos[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};


  if (!handler_lock_flag) {
      // get new position of handler

      /* get data */
      if (deviceID >= 0) {
      // for TELEOP_MODE
      handler_pose_lock.lock(); // handler_pose_ used by another thread so should sync
      new_pos[0] = handler_pose_[0];
      new_pos[1] = handler_pose_[1];
      new_pos[2] = handler_pose_[2];

      for (int i=0;i<3;i++) {
        send[i] =  (new_pos[i] - handler_pose_buf_[i]) * handler_strech_[i];
      }

      for (int i=0;i<3;i++) {
          handler_pose_buf_[i] = new_pos[i];
      }
      handler_pose_lock.unlock();
      ROS_INFO("get handler pos:[%1.5f, %1.5f, %1.5f]", new_pos[0], new_pos[1], new_pos[2]);

      // map_pos[0] = -send[1]; // left, right
      // map_pos[2] = send[0];  // forward, backward
      // map_pos[1] = -send[2]; // up, down

      map_pos[0] = send[1]; 
      map_pos[1] = - send[0];
      map_pos[2] = send[2];

      // use velocity control interface
      double sample_period = 0.001; // unit: s.

      // We use sample_period as velocity computing time, since we always
      // get/send position difference with the nearest sample data. 
      map_pos[0] = map_pos[0]/sample_period;
      map_pos[1] = map_pos[1]/sample_period;
      map_pos[2] = map_pos[2]/sample_period;

      speedL(map_pos);
      } else {
        // for AUTO_MODE
        if (operator_mode == AUTO_MODE)
        {
                  doAutonomyOP();
        }
      }
    }
}


int TeleopRobot::moveServoj(double pose[], double a, double v, double t)
{
    char cmd[1024];
    std_msgs::String joint_msg;

    pose[0] = 0.0;
    pose[1] = 0.0;
    pose[2] = 0.0;
    pose[3] = 0.0;
    pose[4] = 0.0;
    pose[5] = 0.002;

    sprintf(cmd,
            "servoj([%1.5f, %1.5f, %1.5f, %1.5f, %1.5f, %1.5f], %f, %f, %f)\n",
            pose[0], pose[1], pose[2], pose[3], pose[4], pose[5], a, v, t);

    joint_msg.data = cmd;
    urscript_sub_.publish(joint_msg);

    return 0;
}


void TeleopRobot::setHandlerLockFlag(bool flag)
{
  	flag_lock_.lock();
    handler_lock_flag = flag;
    flag_lock_.unlock();
}


bool TeleopRobot::isHandlerLocked()
{
  flag_lock_.lock();
  return handler_lock_flag;
  flag_lock_.unlock();
}


void TeleopRobot::jointStatesCallback(const sensor_msgs::JointState::ConstPtr& joint_msg)
{
    val_lock1_.lock();
    // joint position
    for (int i = 0; i < joint_msg->position.size(); i++) {
//        ROS_INFO("position %i: %f", i,joint_msg->position[i]);
    }


    for (int i=0; i < joint_msg->position.size(); i++) {
      robot_joints_pose_[i] =  joint_msg->position[i];
    }

  
  double joint_state_timestamp =ros::Time::now().toSec()*1000.0;

    val_lock1_.unlock();
}


void TeleopRobot::velocityCallback(const geometry_msgs::TwistStamped::ConstPtr& vel_msg)
{
      val_lock4_.lock();
      
      robot_end_vel_[0] =  vel_msg->twist.linear.x;
      robot_end_vel_[1] =  vel_msg->twist.linear.y;
      robot_end_vel_[2] =  vel_msg->twist.linear.z;
      robot_end_vel_[3] =  vel_msg->twist.angular.x;
      robot_end_vel_[4] =  vel_msg->twist.angular.y;
      robot_end_vel_[5] =  vel_msg->twist.angular.z;


    val_lock4_.unlock();
}


void TeleopRobot::wrechCallback(const geometry_msgs::WrenchStamped::ConstPtr& wrench_msg)
{
    val_lock2_.lock();
    // wrench force
//    ROS_INFO("I heard wrench message at: [%i]", wrench_msg->header.stamp.sec);
//    ROS_INFO("wrech force x: %f", wrench_msg->wrench.force.x);
//    ROS_INFO("wrech force y: %f", wrench_msg->wrench.force.y);
//    ROS_INFO("wrech force z: %f", wrench_msg->wrench.force.z);

//    ROS_INFO("wrech torque x: %f", wrench_msg->wrench.torque.x);
//    ROS_INFO("wrech torque y: %f", wrench_msg->wrench.torque.y);
//    ROS_INFO("wrech torque z: %f", wrench_msg->wrench.torque.z);

    robot_end_force_[0] = wrench_msg->wrench.force.x;
    robot_end_force_[1] = wrench_msg->wrench.force.y;
    robot_end_force_[2] = wrench_msg->wrench.force.z;
    robot_end_force_[3] = wrench_msg->wrench.torque.x;
    robot_end_force_[4] = wrench_msg->wrench.torque.y;
    robot_end_force_[5] = wrench_msg->wrench.torque.z;

    // set force feedback

    setForceFeedback(robot_end_force_[0], robot_end_force_[1],
                     robot_end_force_[2],deviceID, 1.0);

    val_lock2_.unlock();
}


void setForceFeedback(double x, double y, double z, int id, double limit)
{
    if (fabs(x) >= limit) {
        x = sign_float(x)*limit;
    }

    if (fabs(y) >= limit) {
        y = sign_float(y)*limit;
    }

    if (fabs(z) >= limit) {
        z = sign_float(z)*limit;
    }

    // set force feedback
    dhdSetForce(x,y,z,id);
}


int sign_float(double x)
{
    if (x >= 0) {
        return  1;
    } else {
        return -1;
    }
}


void TeleopRobot::toolCallback(const geometry_msgs::PoseStamped::ConstPtr& tool_msg)
{
    val_lock3_.lock();
//    ROS_INFO("tool position x: %f", tool_msg->pose.position.x);
//    ROS_INFO("tool position y: %f", tool_msg->pose.position.y);
//    ROS_INFO("tool position z: %f", tool_msg->pose.position.z);

//    ROS_INFO("tool orientation x: %f", tool_msg->pose.orientation.x);
//    ROS_INFO("tool orientation y: %f", tool_msg->pose.orientation.y);
//    ROS_INFO("tool orientation z: %f", tool_msg->pose.orientation.z);

    robot_end_pose_[0] = tool_msg->pose.position.x;
    robot_end_pose_[1] = tool_msg->pose.position.y;
    robot_end_pose_[2] = tool_msg->pose.position.z;
    robot_end_pose_[3] = tool_msg->pose.orientation.x;
    robot_end_pose_[4] = tool_msg->pose.orientation.y;
    robot_end_pose_[5] = tool_msg->pose.orientation.z;
    val_lock3_.unlock();
}

//void TeleopRobot::robotStateCallback(const sensor_msgs::JointState::ConstPtr& joint_msg,
//                                     const geometry_msgs::WrenchStamped::ConstPtr& wrench_msg /*,
//                                     const geometry_msgs::PoseStamped::ConstPtr& tool_msg */)
//{
//    val_lock_.lock();

//    // joint position
//    for (int i = 0; i < joint_msg->position.size(); i++) {
//        ROS_INFO("position %i: %f", i,joint_msg->position[i]);
//    }


//    for (int i=0; i < joint_msg->position.size(); i++) {
//      robot_joints_pose_[i] =  joint_msg->position[i];
//    }

//    // wrench force
//    ROS_INFO("I heard wrench message at: [%i]", wrench_msg->header.stamp.sec);
//    ROS_INFO("wrech force x: %f", wrench_msg->wrench.force.x);
//    ROS_INFO("wrech force y: %f", wrench_msg->wrench.force.y);
//    ROS_INFO("wrech force z: %f", wrench_msg->wrench.force.z);

//    ROS_INFO("wrech torque x: %f", wrench_msg->wrench.torque.x);
//    ROS_INFO("wrech torque y: %f", wrench_msg->wrench.torque.y);
//    ROS_INFO("wrech torque z: %f", wrench_msg->wrench.torque.z);

//    robot_joints_force_[0] = wrench_msg->wrench.force.x;
//    robot_joints_force_[1] = wrench_msg->wrench.force.y;
//    robot_joints_force_[2] = wrench_msg->wrench.force.z;
//    robot_joints_force_[3] = wrench_msg->wrench.torque.x;
//    robot_joints_force_[4] = wrench_msg->wrench.torque.y;
//    robot_joints_force_[5] = wrench_msg->wrench.torque.z;


//    // tool position
////    ROS_INFO("tool position x: %f", tool_msg->pose.position.x);
////    ROS_INFO("tool position y: %f", tool_msg->pose.position.y);
////    ROS_INFO("tool position z: %f", tool_msg->pose.position.z);

////    ROS_INFO("tool orientation x: %f", tool_msg->pose.orientation.x);
////    ROS_INFO("tool orientation y: %f", tool_msg->pose.orientation.y);
////    ROS_INFO("tool orientation z: %f", tool_msg->pose.orientation.z);

////    robot_end_pose_[0] = tool_msg->pose.position.x;
////    robot_end_pose_[1] = tool_msg->pose.position.y;
////    robot_end_pose_[2] = tool_msg->pose.position.z;
////    robot_end_pose_[3] = tool_msg->pose.orientation.x;
////    robot_end_pose_[4] = tool_msg->pose.orientation.y;
////    robot_end_pose_[5] = tool_msg->pose.orientation.z;

//    val_lock_.unlock();
//}


int kbhit(void)
{
  struct termios oldt, newt;
  int ch;
  int oldf;

  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

  ch = getchar();
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  fcntl(STDIN_FILENO, F_SETFL, oldf);
  if(ch != EOF) {
    ungetc(ch, stdin);
    return 1;
  }

  return 0;
}

char getKbhit()
{
  char ch;
  if (kbhit()){
    ch = getchar();
    return ch;
  } else {
    return -1;
  }
}


int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "teleop_listener");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */
  // ros::Subscriber sub = n.subscribe("joint_states", 100, jointStatesCallback);
  //
  // ros::Subscriber sub_wrech = n.subscribe("wrench", 100, wrechCallback);

  //comment for test
  #define HANDLER_ON
  #ifdef HANDLER_ON
  int deviceCount = dhdGetDeviceCount();
  int deviceID = 0;
  if (1 == deviceCount) {
    if ((deviceID = dhdOpenID(0)) < 0) {
      ROS_INFO("error: handler device: %s\n", dhdErrorGetLastStr());
      return -1;
    }
  } else {
    ROS_INFO("No handler device find! %s\n", dhdErrorGetLastStr());
    return -1;
  }
  #else
  int deviceCount = 0;
  int deviceID = -1;
  #endif
//  int deviceCount = 1;
//  int deviceID = 0;
  // end comment


  double a = 0.0;
  double b = 0.0;
  double c = 0.0;

  for (int i = 0; i < 10; i++) {
      // comment for test
    #ifdef HANDLER_ON
    if (deviceID >= 0) {
    dhdGetPosition(&a, &b, &c, deviceID);
    }
    #endif
    ROS_INFO("handler position: %f, %f, %f", a, b,c);
  }


  // load GMM params which trained from MATLAB
  ROS_INFO("start read params");
  bool file_state = gmm_model.loadParams("pose-velctl-gmm_parameters.txt");
  if (file_state) {
    ROS_INFO("We get the gmm_model data,now we will print it");
  gmm_model.sigma[0].Print();
  gmm_model.sigma[1].Print();
  gmm_model.sigma[2].Print();
  }

  control_cmd_file.open("robot-log/cmd_log.txt");
  if (control_cmd_file.is_open()) {
    ROS_INFO("start cmd log");
  }

  // TeleopRobot teleop(deviceID, AUTO_MODE);
  TeleopRobot teleop(deviceID, TELEOP_MODE);
  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */

  ros::AsyncSpinner spinner(3);
  spinner.start();

  ros::waitForShutdown();

  // teleop.halt(); // teleop seems have been delete!

  // control_cmd_file.close();

  return 0;
}
