#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>

#include <tf/transform_datatypes.h>

class RSJRobotTestNode
{
private:
  ros::Subscriber sub_odom_;
  ros::Subscriber sub_scan_;
  ros::Publisher pub_twist_;

  sensor_msgs::LaserScan latest_scan_;
  float left_prev_;
  geometry_msgs::Twist cmd_vel_;

  void cbOdom(const nav_msgs::Odometry::ConstPtr &msg)
  {
  }
  void cbScan(const sensor_msgs::LaserScan::ConstPtr &msg)
  {
    // 受け取ったメッセージをコピーしておく
    latest_scan_ = *msg;
  }
public:
  RSJRobotTestNode()
  {
    ros::NodeHandle nh;
    pub_twist_ = nh.advertise<geometry_msgs::Twist>(
        "cmd_vel", 5);
    sub_odom_ = nh.subscribe("odom", 5,
        &RSJRobotTestNode::cbOdom, this);
    sub_scan_ = nh.subscribe("scan", 5, 
        &RSJRobotTestNode::cbScan, this);
  }
  void mainloop()
  {
    ROS_INFO("Hello ROS World!");

    ros::Rate rate(10.0);
    left_prev_ = FLT_MAX;
    while (ros::ok())
    {
      ros::spinOnce();

      if (latest_scan_.ranges.size() > 0)
      {
        // LaserScanメッセージをすでに受け取っている場合

        float front = FLT_MAX, left = FLT_MAX;

        // theta-range 座標系から x-y 座標系に変換
        for(unsigned int i = 0; i < latest_scan_.ranges.size(); i ++)
        {
          if (!(latest_scan_.ranges[i] < latest_scan_.range_min ||
              latest_scan_.ranges[i] > latest_scan_.range_max ||
              std::isnan(latest_scan_.ranges[i])))
          {
            // 距離値がエラーでない場合

            float theta = latest_scan_.angle_min + i * latest_scan_.angle_increment;
            // x-y 座標系に変換
            float x = latest_scan_.ranges[i] * cosf(theta);
            float y = latest_scan_.ranges[i] * sinf(theta);

            if (fabs(y) < 0.25 && x > 0.05)
            {
              // ロボット正面方向のデータについて最小距離を計算
              if (front > x) front = x;
            }
            else if (fabs(x) < 0.25 && y > 0.25)
            {
              // ロボット左方向のデータについて最小距離を計算
              if (left > y) left = y;
            }
          }
        }

        if (front > 0.5)
        {
          // 正面の距離に余裕がある場合

          ROS_INFO("Following left wall (distance %0.3f)", left);
          cmd_vel_.linear.x = 0.1;

          if (left > 1.0) left = 1.0;
          // 角速度指令値を0に近づけるようにフィードバック
          cmd_vel_.angular.z += -cmd_vel_.angular.z * 0.01;
          // 左方向の距離を0.5mに近づけるようにフィードバック
          cmd_vel_.angular.z += (left - 0.5) * 0.02;
          // 距離の変化量(壁の向きを表す)を0に近づけるようにフィードバック
          if (left_prev_ < 1.0)
            cmd_vel_.angular.z += (left - left_prev_) * 4.0;

          if (cmd_vel_.angular.z > 0.3) cmd_vel_.angular.z = 0.3;
          else if (cmd_vel_.angular.z < -0.3) cmd_vel_.angular.z = -0.3;
        }
        else
        {
          ROS_INFO("Something in front");
          cmd_vel_.linear.x = 0.0;
          cmd_vel_.angular.z = -0.2;
        }
        left_prev_ = left;

        pub_twist_.publish(cmd_vel_);
      }

      rate.sleep();
    }
  }
};

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "rsj_robot_test_node");

    RSJRobotTestNode robot_test;

    robot_test.mainloop();
}