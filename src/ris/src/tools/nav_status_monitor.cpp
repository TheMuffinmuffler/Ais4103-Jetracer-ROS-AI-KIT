#include <actionlib_msgs/GoalStatusArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <move_base_msgs/MoveBaseActionFeedback.h>
#include <move_base_msgs/MoveBaseActionResult.h>
#include <ros/ros.h>

class NavStatusMonitor
{
public:
  NavStatusMonitor()
  {
    status_sub_ = nh_.subscribe("/move_base/status", 10, &NavStatusMonitor::statusCallback, this);
    feedback_sub_ = nh_.subscribe("/move_base/feedback", 10, &NavStatusMonitor::feedbackCallback, this);
    result_sub_ = nh_.subscribe("/move_base/result", 10, &NavStatusMonitor::resultCallback, this);
    amcl_sub_ = nh_.subscribe("/amcl_pose", 10, &NavStatusMonitor::amclCallback, this);

    timer_ = nh_.createTimer(ros::Duration(2.0), &NavStatusMonitor::timerCallback, this);
  }

private:
  void statusCallback(const actionlib_msgs::GoalStatusArray::ConstPtr& msg)
  {
    if (msg->status_list.empty())
      return;

    const auto& s = msg->status_list.back();
    last_status_ = s.status;
    last_text_ = s.text;
  }

  void feedbackCallback(const move_base_msgs::MoveBaseActionFeedback::ConstPtr& msg)
  {
    last_feedback_x_ = msg->feedback.base_position.pose.position.x;
    last_feedback_y_ = msg->feedback.base_position.pose.position.y;
    have_feedback_ = true;
  }

  void resultCallback(const move_base_msgs::MoveBaseActionResult::ConstPtr& msg)
  {
    ROS_INFO("move_base result status=%u text=%s", msg->status.status, msg->status.text.c_str());
  }

  void amclCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
  {
    amcl_x_ = msg->pose.pose.position.x;
    amcl_y_ = msg->pose.pose.position.y;
    have_amcl_ = true;
  }

  void timerCallback(const ros::TimerEvent&)
  {
    ROS_INFO_STREAM("nav_status_monitor | status=" << static_cast<int>(last_status_)
                    << " text=\"" << last_text_ << "\""
                    << " feedback=(" << last_feedback_x_ << ", " << last_feedback_y_ << ")"
                    << " amcl=(" << amcl_x_ << ", " << amcl_y_ << ")");
  }

  ros::NodeHandle nh_;
  ros::Subscriber status_sub_;
  ros::Subscriber feedback_sub_;
  ros::Subscriber result_sub_;
  ros::Subscriber amcl_sub_;
  ros::Timer timer_;

  uint8_t last_status_{0};
  std::string last_text_{"none"};
  double last_feedback_x_{0.0};
  double last_feedback_y_{0.0};
  double amcl_x_{0.0};
  double amcl_y_{0.0};
  bool have_feedback_{false};
  bool have_amcl_{false};
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "nav_status_monitor");
  NavStatusMonitor node;
  ros::spin();
  return 0;
}