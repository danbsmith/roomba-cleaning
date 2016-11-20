#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <roomba_clean_actions/basic_cleanAction.h>

void doneCb(const actionlib::SimpleClientGoalState& state, const roomba_clean_actions::basic_cleanResultConstPtr& result) {
    ROS_INFO("Finished clean in state %s", state.toString().c_str());
    ROS_INFO("    Duration: %d, distance travelled: %d.  %f%% charge remaining.", result->seconds, result->millimeters, result->battery_charge);
    ros::shutdown();
}

void activeCb() {
    ROS_INFO("Cleaning task active.");
}

void feedbackCb(const roomba_clean_actions::basic_cleanFeedbackConstPtr& feedback) {
    //ROS_INFO("    Duration: %d, distance travelled: %d.  %f%% charge remaining.", feedback->seconds, feedback->millimeters, feedback->battery_charge);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "run_clean");
    actionlib::SimpleActionClient<roomba_clean_actions::basic_cleanAction> ac("basic_clean", true);
    ROS_INFO("Waiting for server...");
    ac.waitForServer();
    roomba_clean_actions::basic_cleanGoal goal;
    goal.seconds = 1800;
    ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
    ros::spin();
    return 0;
}
