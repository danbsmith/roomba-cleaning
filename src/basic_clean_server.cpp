#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <roomba_clean_actions/basic_cleanAction.h>
#include <roomba_serial/SendButton.h>
#include <roomba_serial/SetMode.h>
#include <roomba_serial/Sensors.h>
#include <ctime>

class basic_cleanAction {
protected:
  ros::NodeHandle nh_;
  ros::ServiceClient client = nh_.serviceClient<roomba_serial::Sensors>("GetSensors");
  actionlib::SimpleActionServer<roomba_clean_actions::basic_cleanAction> cleanserv_;
  std::string action_name_;
  roomba_clean_actions::basic_cleanFeedback feedback_;
  roomba_clean_actions::basic_cleanResult result_;
  ros::Publisher button_pub = nh_.advertise<roomba_serial::SendButton>("BUTTON_OUT", 100);
  ros::Publisher mode_pub = nh_.advertise<roomba_serial::SetMode>("MODE_CHANGES", 100);

public:
  basic_cleanAction(std::string name) :
  cleanserv_(nh_, name, boost::bind(&basic_cleanAction::executeCB, this, _1), false), action_name_(name) {
    cleanserv_.start();
    ROS_INFO("Clean server started.");
  }

  ~basic_cleanAction(void) {
  }

  void executeCB(const roomba_clean_actions::basic_cleanGoalConstPtr &goal) {
    ROS_INFO("Got goal of %d seconds", goal->seconds);
    ros::Rate r(0.5);
    roomba_serial::SendButton dock;
    dock.buttoncode = 10;
    roomba_serial::Sensors sensorServer;
    int distance = 0;
    float charge;
    sensorServer.request.request = 0;
    bool success = true;
    time_t base = time(NULL);
    time_t curr = base;
    roomba_serial::SetMode mode;
    roomba_serial::SendButton button;
    mode.modecode = 2; // Safe mode, to start cleaning cycle
    button.buttoncode = 2;
    bool running = false;
    mode_pub.publish(mode);
    r.sleep();
    while(!running) {
      if(cleanserv_.isPreemptRequested() || !ros::ok()) {
        button_pub.publish(dock);
        ROS_INFO("%s: Preempted, now docking", action_name_.c_str());
        cleanserv_.setPreempted();
        success = false;
        break;
      }
      button_pub.publish(button);
      r.sleep();
      client.call(sensorServer);
      ROS_INFO("Current into battery is %d mA", sensorServer.response.current);
      for(int i = 0; i < 5; i++) {
        r.sleep();
      }
      running = (sensorServer.response.current < -100);
    }
    ROS_INFO("Roomba Away!");
    while(curr < base + goal->seconds) {
      if(cleanserv_.isPreemptRequested() || !ros::ok()) {
        button_pub.publish(dock);
        ROS_INFO("%s: Preempted, now docking", action_name_.c_str());
        cleanserv_.setPreempted();
        success = false;
        break;
      }
      curr = time(NULL);
      if(client.call(sensorServer)) {
        distance = distance + sensorServer.response.distance;
        charge = (float) (sensorServer.response.charge / sensorServer.response.capacity);
      }
      else {
        ROS_INFO("%s: Didn't get sensor response, sending stale data.", action_name_.c_str());
      }
      feedback_.seconds = (uint16_t) (curr - base);
      feedback_.millimeters = distance;
      feedback_.battery_charge = charge;
      cleanserv_.publishFeedback(feedback_);
      r.sleep();
    }
    button_pub.publish(dock);
    bool charging = false;
    while(!charging) {
      while(!client.call(sensorServer)) {r.sleep();}
      charging = (sensorServer.response.current > 0);
      feedback_.millimeters = distance + sensorServer.response.distance;
      feedback_.battery_charge = (float) (sensorServer.response.charge / sensorServer.response.capacity);
      feedback_.seconds = time(NULL) - base;
      cleanserv_.publishFeedback(feedback_);
      r.sleep();
    }
    result_.seconds = feedback_.seconds;
    result_.millimeters = feedback_.millimeters;
    result_.battery_charge = feedback_.battery_charge;
    cleanserv_.setSucceeded(result_);
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "basic_clean");
  basic_cleanAction bc(ros::this_node::getName());
  ros::spin();
}
