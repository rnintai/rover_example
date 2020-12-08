#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/HomePosition.h>
#include <mavros_msgs/Altitude.h>
#include <geometry_msgs/Vector3.h>
#include <vector>

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

class Rover{
public:
  Rover();
  ~Rover();
  void init();
  bool arm();
  bool offBoard();
  float degtoRad(float deg);
//   void offbVelControl(float vx, float vy, float vz, float yaw, std::size_t count);
  void run();
  void pubVelocity();
  void velCallBack(const geometry_msgs::TwistConstPtr& twist_);
  void getHomeGeoPoint();
  void setHomeGeoPointCB(const mavros_msgs::HomePositionConstPtr& home);
  void getAltitude();
  void getAltitudeCB(const mavros_msgs::AltitudeConstPtr& altitude);

private:
  ros::NodeHandle nh_;
  ros::Subscriber state_sub;
  ros::Publisher cmd_vel_pub_;
  ros::Publisher set_vel_pub_;
  ros::ServiceClient arming_client;
  ros::ServiceClient set_mode_client;
  ros::Subscriber cmd_vel_sub_;
  ros::Subscriber home_sub_;

  bool is_arm_ = false;

  mavros_msgs::HomePosition home_{};
  bool home_set_ = false;
  mavros_msgs::Altitude altitude_{};
  bool altitude_received_ = false;
  float altitude_in_amsl_ = 0.0f;
  
  geometry_msgs::Vector3 lin_vel_;
  geometry_msgs::Vector3 ang_vel_;
};

Rover::Rover(){
    // getHomeGeoPoint();
    // getAltitude();
    home_sub_ = nh_.subscribe<mavros_msgs::HomePosition>("mavros/home_position/home", 1,
                                        boost::bind(&Rover::setHomeGeoPointCB, this, _1));
    state_sub = nh_.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    arming_client = nh_.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    set_mode_client = nh_.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");    
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::TwistStamped>
            ("mavros/setpoint_velocity/cmd_vel",5);
    cmd_vel_sub_ = nh_.subscribe<geometry_msgs::Twist>
            ("cmd_vel",10,&Rover::velCallBack,this);
  }

Rover::~Rover(){}

bool Rover::arm(){
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    if(!current_state.armed){
        if (arming_client.call(arm_cmd) && arm_cmd.response.success)
            return true;
        else
            return false;
    }
    else return true;
}

bool Rover::offBoard(){
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    if(current_state.mode != "OFFBOARD"){
        if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
            return true;
        else
            return false;
    }
    else return true;
}

void Rover::run(){
    if(!this->arm()){
        ROS_INFO("arm Failed.");
    }
    // if(is_arm_){
        if(!this->offBoard()){
            ROS_INFO("OffBoard Failed.");
        }
    // }
}

// void Rover::pubVelocity(){
//     geometry_msgs::TwistStamped cmd_vel_;
//     cmd_vel_.twist.linear.x = lin_vel_.x;
//     cmd_vel_.twist.angular.z = ang_vel_.z;

//     cmd_vel_pub_.publish(cmd_vel_);
// }

void Rover::velCallBack(const geometry_msgs::TwistConstPtr& _twist){
    // lin_vel_ = _twist->linear;
    // ang_vel_ = _twist->angular;
    geometry_msgs::TwistStamped cmd_vel_;

    cmd_vel_.twist.linear.x = _twist->linear.x;
    cmd_vel_.twist.angular.z = _twist->angular.z;
    cmd_vel_pub_.publish(cmd_vel_);

    // ROS_INFO("lin: %.3f, ang: %.3f",lin_vel_,ang_vel_);
}


float Rover::degtoRad(float deg){
    return (deg /180.0f * M_PI);
}


// void Rover::getHomeGeoPoint()
// {
//   // FCU Home position: See http://docs.ros.org/api/sensor_msgs/html/msg/NavSatFix.html
//   ros::Rate rate_(20.0);
//   ROS_INFO("Waiting for Aero FC Home to be set...");
//   while (ros::ok() && !home_set_)
//   {
//     ros::spinOnce();
//     rate_.sleep();
//   }
// }
// Callback that gets called periodically from MAVROS notifying Global Poistion of Aero FCU
void Rover::setHomeGeoPointCB(const mavros_msgs::HomePositionConstPtr& home)
{
  home_ = *home;
  home_set_ = true;
  ROS_INFO("Received Home (WGS84 datum): %lf, %lf, %lf", home_.geo.latitude, home_.geo.longitude, home_.geo.altitude);
}

// void Rover::getAltitude()
// {
//   ros::Subscriber altitude_sub =
//       nh_.subscribe<mavros_msgs::Altitude>("mavros/altitude", 1, boost::bind(&Rover::getAltitudeCB, this, _1));
//   while (ros::ok() && !altitude_received_)
//   {
//     ros::spinOnce();
//     rate_.sleep();
//   }
// }

// void Rover::getAltitudeCB(const mavros_msgs::AltitudeConstPtr& altitude)
// {
//   altitude_ = *altitude;
//   altitude_received_ = true;
//   altitude_in_amsl_ = altitude_.amsl;
// }

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rover_example_node");
    Rover rover;
    ros::Rate rate(20);

    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }


    // if(!rover.arm()){
    //     ROS_INFO("Arming Failed.");
    //     // exit(0);
    // }

    // if(current_state.connected){
    //      if(!rover.arm()){
    //     ROS_INFO("Arming Failed.");
    //     // exit(0);
    //     }else{
    //     ROS_INFO("Armed Succesfully");
    //     }
    // }
    while(ros::ok()){
        rover.run();
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
    
/*
 * setbool.srv을 통해 trigger. 
 * simple server 구현하여 trigger을 줬을때에 arm이 되고, offboard mode인지 체크하며 모드 변경.
 * $ rosservice call std_srvs/setBool?
 * 
 * turtlebot teleop key로 테스트. cmd_vel topic을 받아서 작동하도록.
 * cmd_vel을 setpoint vel로.
 * 
 * home position??
 * 
 * 
 * offboard, arm의 service call이 불필요하게 되고있음.
 * main문 제외 rate는 없애고, velcallback에서 publish까지.
 */

