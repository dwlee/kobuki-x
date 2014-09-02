 /*
 * waiter_node.hpp
 *
 *  Created on: Apr 6, 2013
 *      Author: jorge
 */

#ifndef WAITER_NODE_HPP_
#define WAITER_NODE_HPP_

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

#include <std_msgs/String.h>
 
#include <kobuki_msgs/SensorState.h>
#include <kobuki_msgs/DigitalInputEvent.h>

#include <simple_delivery_msgs/DeliveryStatus.h>
#include <simple_delivery_msgs/DeliveryOrder.h>
#include <simple_delivery_msgs/RobotDeliveryOrderAction.h>
#include <yocs_msgs/TableList.h>

#include "waiterbot_ctrl_office/ar_markers.hpp"
#include "waiterbot_ctrl_office/nav_watchdog.hpp"
#include "waiterbot_ctrl_office/navigator.hpp"
#include "waiterbot_ctrl_office/waiter_sound.hpp"

namespace waiterbot
{

class WaiterNode
{
public:

  WaiterNode(std::string name) :
    as_(nh_, "delivery_order", false),
    node_name_(name),
    SPOT_BASE_MARKER_TIMEOUT(10.0),
    SPOT_POSE_MARKER_TIMEOUT(15.0),
    blink_frequency_(2.0),
    last_blink_time_(0.0),
    last_blink_led_(1)
  {
  }

  ~WaiterNode(void)
  {
  }

  bool init();
  void spin();
  bool wakeUp();
  bool leaveNest();

  void odometryCB(const nav_msgs::Odometry::ConstPtr& msg);
  void digitalInputCB(const kobuki_msgs::DigitalInputEvent::ConstPtr& msg);
  void coreSensorsCB(const kobuki_msgs::SensorState::ConstPtr& msg);
  void tablePosesCB(const yocs_msgs::TableList::ConstPtr& msg);

  void deliverOrderCB();
  void preemptOrderCB();

protected:
  const double SPOT_BASE_MARKER_TIMEOUT;
  const double SPOT_POSE_MARKER_TIMEOUT;

  ros::NodeHandle nh_;
  std::string node_name_;

  // NodeHandle instance must be created before this line. Otherwise strange error may occur
  actionlib::SimpleActionServer<simple_delivery_msgs::RobotDeliveryOrderAction> as_;

  // create messages that are used to published feedback/result
  simple_delivery_msgs::RobotDeliveryOrderFeedback feedback_;
  simple_delivery_msgs::RobotDeliveryOrderResult   result_;

  /*********************
  ** Publishers
  **********************/
  ros::Publisher led_1_pub_;
  ros::Publisher led_2_pub_;
  ros::Publisher sound_pub_;
  ros::Publisher table_marker_pub_;

  /*********************
  ** Subscribers
  **********************/
  ros::Subscriber digital_input_sub_;
  ros::Subscriber core_sensors_sub_;
  ros::Subscriber table_poses_sub_;

  ARMarkersCafe   ar_markers_;
  NavWatchdog nav_watchd_;
  Navigator   navigator_;

  geometry_msgs::PoseStamped pickup_pose_;
  yocs_msgs::TableList       table_poses_;
  kobuki_msgs::SensorState   core_sensors_;
  simple_delivery_msgs::DeliveryStatus           delivery_order_;
  std::string                global_frame_;
  std::string                resources_path_;

  // LED blinking attributes; TODO make a separate HRI class
  double blink_frequency_;
  double last_blink_time_;
  uint8_t last_blink_led_;

  boost::thread order_process_thread_;

  bool debug_mode_;
  bool initialized_;
  bool initialized_table_;

  bool wait_for_button_;

  bool processOrder(std::vector<std::string> locations);
  bool getReadyToWork();
  bool waitForPoses();
  bool waitForButton();
  bool gotoTable(int table_id);
  void sendFeedback(int feedback_status);
  bool setSucceeded(std::string message);
  bool setFailure(std::string message);
  void playSound(const std::string& wav_file); 

  bool cleanupAndSuccess();
  bool cleanupAndError();

  const std::string toStr(int16_t status)
  {
    return std::string(toCStr(status));
  }

  const char* toCStr(int16_t status)
  {
    switch (status)
    {
      case simple_delivery_msgs::DeliveryStatus::IDLE                          : return "idle";
      case simple_delivery_msgs::DeliveryStatus::GO_TO_FRONTDESK                 : return "going to front desk";
      case simple_delivery_msgs::DeliveryStatus::ARRIVAL_AT_FRONTDESK                : return "arrived to front desk";
      case simple_delivery_msgs::DeliveryStatus::WAITING_FOR_FRONTDESK           : return "waiting for front desk";
      case simple_delivery_msgs::DeliveryStatus::GO_TO_RECEIVER                   : return "going to receiver";
      case simple_delivery_msgs::DeliveryStatus::WAITING_CONFIRM_RECEIVER : return "waiting for receiver";
      case simple_delivery_msgs::DeliveryStatus::COMPLETE_DELIVERY             : return "delivery completed";
      case simple_delivery_msgs::DeliveryStatus::COMPLETE_ALL_DELIVERY             : return "all delivery completed";
      case simple_delivery_msgs::DeliveryStatus::RETURN_TO_DOCK            : return "going to dock";
      case simple_delivery_msgs::DeliveryStatus::COMPELTE_RETURN                         : return "complete return";
      case simple_delivery_msgs::DeliveryStatus::ERROR                         : return "error";
      default                                               : return "UNRECOGNIZED STATUS";
    }
  }

  //< DEBUG
  void fakeOrderForEasyDebugging(int order_id, int table_id);
  //> DEBUG
};

} /* namespace waiterbot */

#endif /* WAITER_NODE_HPP_ */
