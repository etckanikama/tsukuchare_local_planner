#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <pcl_ros/point_cloud.h>
#include <iostream>
#include <cmath>

double  STEP_ANGLE = 0.36;
int CENTER_ANGLE_INDEX = 363;

// // HSV -> RGB変換
// void hsv2rgb(float h, float s, float v, int &_r, int &_g, int &_b) {

//     float r = static_cast<float>(v);
//     float g = static_cast<float>(v);
//     float b = static_cast<float>(v);
//     if (s > 0.0f) {
//         h *= 6.0f;
//         const int i = (int) h;
//         const float f = h - (float) i;
//         switch (i) {
//             default:
//             case 0:
//                 g *= 1 - s * (1 - f);
//                 b *= 1 - s;
//                 break;
//             case 1:
//                 r *= 1 - s * f;
//                 b *= 1 - s;
//                 break;
//             case 2:
//                 r *= 1 - s;
//                 b *= 1 - s * (1 - f);
//                 break;
//             case 3:
//                 r *= 1 - s;
//                 g *= 1 - s * f;
//                 break;
//             case 4:
//                 r *= 1 - s * (1 - f);
//                 g *= 1 - s;
//                 break;
//             case 5:
//                 g *= 1 - s;
//                 b *= 1 - s * f;
//                 break;
//         }
//     }
//     _r = static_cast<int>(r * 255);
//     _g = static_cast<int>(g * 255);
//     _b = static_cast<int>(b * 255);
// }

sensor_msgs::LaserScan scan;


void scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan_)
{
    float nandeyanen = scan_->range_max;
    scan.header = scan_->header;
    scan.angle_min = scan_->angle_min;
    scan.angle_max = scan_->angle_max;
    scan.angle_increment = scan_->angle_increment;
    scan.time_increment = scan_->time_increment;
    scan.scan_time = scan_->scan_time;
    scan.range_min = scan_->range_min;
    scan.range_max = scan_->range_max;
    scan.ranges = scan_->ranges;
    scan.intensities = scan_->intensities;
    //std::cout << "msg: " << scan_->angle_max << "\n";
    //std::cout << "msg: " << scan.angle_max << "\n";
    //ROS_INFO("glob: %f \n",scan.angle_max);
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "dummy_point_publisher"); //node名　rosrun pottential_planner dummy_point_publisher (CMakeLists.txtにも書かれている)
    ros::NodeHandle nh;

    ros::Rate rate(10);
    
    std::string topic_name = "points";
    // float range = 3.0;
    // int number_of_points = 100000;
    int cnt = 0;


    // pcl:PointCloud型のPublisher
    // 実際にTopicとして流れるのは sensor_msgs::PointCloud2 になる
    // テンプレートの中身を変えればXYZIとかXYZとかに変更可能
    ros::Publisher pc_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ> >(topic_name.c_str(),10);
    ros::Subscriber scan_sub = nh.subscribe("scan",10, scanCallback);
    ros::Publisher scan_pub = nh.advertise<sensor_msgs::LaserScan>("scan_pub", 10); //rvizへのpub用

    while (ros::ok()) {
        ros::spinOnce();

        // std::cout << scan.ranges[363] << std::endl;
        if (cnt > 5)
        {
            // ROS_INFO("AA%ld \n",scan.ranges.size());
            pcl::PointCloud<pcl::PointXYZ> dummy_cloud;
            for (int idx=0; idx < scan.ranges.size(); idx++)
            {
                pcl::PointXYZ new_point;
                if (std::isnan(scan.ranges[idx])) continue;
                int cuurent_index = idx;
                double distance = scan.ranges[idx];
                int angle_diff = CENTER_ANGLE_INDEX - cuurent_index;
                double theta = STEP_ANGLE * (M_PI / 180) * angle_diff;
                double angle_tate = distance* std::cos(theta);
                double angle_yoko = distance* std::sin(theta);

                
                std::cout << "idx: " << idx  << " " << scan.ranges[idx] << "\n";
                new_point.x = angle_tate;
                new_point.y = angle_yoko;
                dummy_cloud.points.push_back(new_point);
            }
            // auto msg = dummy_cloud.makeShared();

            /*clustering*/
	        /*kd-treeクラスを宣言*/
            pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
            /*探索する点群をinput*/
            tree->setInputCloud(dummy_cloud);
            /*クラスタリング後のインデックスが格納されるベクトル*/
            std::vector<pcl::PointIndices> cluster_indices;
            /*今回の主役*/
            pcl::EuclideanClusterExtraction<pcl::PointXYZ> ece;
            /*距離の閾値を設定*/
            ece.setClusterTolerance(cluster_tolerance);
            /*各クラスタのメンバの最小数を設定*/
            ece.setMinClusterSize(min_cluster_size);
            /*各クラスタのメンバの最大数を設定*/
            ece.setMaxClusterSize(cloud->points.size());
            /*探索方法を設定*/
            ece.setSearchMethod(tree);
            /*クラスリング対象の点群をinput*/
            ece.setInputCloud(cloud);
            /*クラスリング実行*/
            ece.extract(cluster_indices);
            // ヘッダ情報はココでつめる
            msg->header.frame_id = "laser";
            pcl_conversions::toPCL(ros::Time::now(), msg->header.stamp);
            pc_pub.publish(msg);



        }


            // if (scan.ranges.size() == 0) continue;
            
                  // ダミー点群の準備
            // pcl::PointCloud<pcl::PointXYZ> dummy_cloud;

            // for (int i = 0; i < number_of_points; i++) {

            //     pcl::PointXYZ new_point;
            //     new_point.x = range - (rand() * range * 2) / RAND_MAX;
            //     new_point.y = range - (rand() * range * 2) / RAND_MAX;
            //     // new_point.z = range - (rand() * range * 2) / RAND_MAX;
            //     new_point.z = 0.0;
            //     // float distance = std::sqrt(new_point.x * new_point.x + new_point.y * new_point.y + new_point.z * new_point.z);

            //     int r, g, b;
            //     // hsv2rgb(std::fmod(distance / 3.0, 1.0), 1.0, 1.0, r, g, b);
            //     // new_point.r = r;
            //     // new_point.g = g;
            //     // new_point.b = b;

            //     // pcl::PointCloudはpush_backで足せば良いだけなので楽ちん
            //     dummy_cloud.points.push_back(new_point);
            // }

            // auto msg = dummy_cloud.makeShared();

            // // ヘッダ情報はココでつめる
            // msg->header.frame_id = "laser";
            // pcl_conversions::toPCL(ros::Time::now(), msg->header.stamp);
            // pc_pub.publish(msg);



  

        cnt++;
        // rvizへとscanの値をそのままPublish
        // scan_pub.publish(scan);

        rate.sleep();
    }

    return 0;
}