#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_ros/point_cloud.h>
#include <iostream>

#include <stdio.h>
#include <cmath>
#include <random>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
// #include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
// #include <pcl/visualization/cloud_viewer.h>

double STEP_ANGLE = 0.36;
int CENTER_ANGLE_INDEX = 363;

// PCLでユークリッドクラスタリングをするプログラム
// roslaunch urg_node 〜〜〜　でurgeを起動
// rosrun potential_planner dummy_point_publisher　（このプログラムのノード名を起動）
// rvizで確認する

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
    // std::cout << "msg: " << scan_->angle_max << "\n";
    // std::cout << "msg: " << scan.angle_max << "\n";
    // ROS_INFO("glob: %f \n",scan.angle_max);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dummy_point_publisher"); // node名　rosrun pottential_planner dummy_point_publisher (CMakeLists.txtにも書かれている)
    ros::NodeHandle nh;

    ros::Rate rate(10);

    std::string topic_name = "points";
    // float range = 3.0;
    // int number_of_points = 100000;
    double cluster_tolerance = 0.1;
    int min_cluster_size = 1;

    // gnuplot
    FILE *gnu;

    gnu = popen("gnuplot -persist", "w");
    // fprintf(gnu, "unset key\n");
    fprintf(gnu, "set xrange [-4.5:4.5]\n"); // x範囲はここで変える-6:6
    fprintf(gnu, "set yrange [-4.5:4.5]\n"); // y範囲はここで変える-5:5
    fprintf(gnu, "set xlabel 'x(m)'\n");
    fprintf(gnu, "set ylabel 'y(m)'\n");
    fprintf(gnu, "set size ratio -1\n");
    fprintf(gnu, "set xlabel font 'Arial,20'\n");
    fprintf(gnu, "set ylabel font 'Arial,20'\n");

    float x[1080] = {0};
    float y[1080] = {0};

    // pcl:PointCloud型のPublisher
    // 実際にTopicとして流れるのは sensor_msgs::PointCloud2 になる
    // テンプレートの中身を変えればXYZIとかXYZとかに変更可能
    ros::Publisher pc_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>(topic_name.c_str(), 10);
    ros::Subscriber scan_sub = nh.subscribe("scan", 10, scanCallback);
    ros::Publisher scan_pub = nh.advertise<sensor_msgs::LaserScan>("scan_pub", 10); // rvizへのpub用

    while (ros::ok())
    {
        ros::spinOnce();

        if (scan.ranges.size() > 0)
        {
            // LaserScanからPointcloud型に変換してdummy_cloudという箱にいれる
            pcl::PointCloud<pcl::PointXYZ>::Ptr dummy_cloud{new pcl::PointCloud<pcl::PointXYZ>};

            for (unsigned int i = 0; i < scan.ranges.size(); i++)
            {
                pcl::PointXYZ new_point;
                if (!(scan.ranges[i] < scan.range_min || scan.ranges[i] > scan.range_max || std::isnan(scan.ranges[i])))
                {
                    std::cout << "angle_min " << scan.angle_min << std::endl;
                    std::cout << "angle_max " << scan.angle_max << std::endl;
                    std::cout << "angle_inclement " << scan.angle_increment << std::endl;
                    float theta = scan.angle_min + i * scan.angle_increment;
                    x[i] = scan.ranges[i] * cosf(theta);
                    y[i] = scan.ranges[i] * sinf(theta);

                    std::cout << "x " << i << ":" << x[i] << std::endl;
                    std::cout << "y " << i << ":" << y[i] << std::endl;
                    new_point.x = x[i];
                    new_point.y = y[i];
                    dummy_cloud->points.push_back(new_point);
                }
            }
            sensor_msgs::PointCloud2 msg;
            pcl::toROSMsg(*dummy_cloud, msg);
            msg.header.frame_id = "laser";
            pc_pub.publish(msg);

            // fprintf(gnu, "plot '-' title 'Robot1 location' w p pt 7 lc rgb \"black\"\n");
            // for (int k = 0; k <= scan.ranges.size(); k++)

            // {
            //     fprintf(gnu, "%f %f\n", x[k], y[k]);
            // }
            // fprintf(gnu, "e\n");
            // fflush(gnu); // gnuplot

            // for (int idx=0; idx < scan.ranges.size(); idx++)
            // {
            //     pcl::PointXYZ new_point;

            //     if (std::isnan(scan.ranges[idx])) continue;
            //     int cuurent_index = idx;
            //     double distance = scan.ranges[idx];
            //     int angle_diff = CENTER_ANGLE_INDEX - cuurent_index;
            //     double theta = STEP_ANGLE * (M_PI / 180) * angle_diff;
            //     double angle_tate = distance* std::cos(theta);
            //     double angle_yoko = distance* std::sin(theta);

            //     // std::cout << "idx: " << idx  << " " << scan.ranges[idx] << "\n";
            //     std::cout << "idx: " << CENTER_ANGLE_INDEX  << " " << scan.ranges[CENTER_ANGLE_INDEX] << "\n";
            //     std::cout << "idx: " << 0  << " " << scan.ranges[0] << "\n";
            //     std::cout << "idx: " << 725  << " " << scan.ranges[725] << "\n";
            //     std::cout << "x " << angle_tate << std::endl;
            //     std::cout << "y " << angle_yoko << std::endl;
            //     new_point.x = angle_yoko;
            //     new_point.y = angle_tate;
            //     // new_point.z = -1;
            //     dummy_cloud->points.push_back(new_point);
            // }

            // std::cout << "1: " << std::endl;

            // if (dummy_cloud->points.size() < 1) continue;
            // std::cout << "dummy_cloud with " << dummy_cloud->width << std::endl;
            // std::cout << "dummy_cloud height " << dummy_cloud->height << std::endl;
            // std::cout << "dummy_cloud point size " << dummy_cloud->points.size() << std::endl;
            // std::cout << "dummy_cloud point[0]_x " << dummy_cloud->points[365].x << std::endl;
            // std::cout << "dummy_cloud point[0]_y " << dummy_cloud->points[365].y << std::endl;

            // // sensor_msgs::PointCloud2 msg;
            // // pcl::toROSMsg(*dummy_cloud, msg);
            // // msg.header.frame_id = "laser";
            // // pc_pub.publish(msg);

            // /*clustering*/
            // /*kd-treeクラスを宣言*/
            // pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
            // /*探索する点群をinput*/
            // tree->setInputCloud(dummy_cloud);
            // /*クラスタリング後のインデックスが格納されるベクトル*/
            // std::vector<pcl::PointIndices> cluster_indices;
            // /*今回の主役*/
            // pcl::EuclideanClusterExtraction<pcl::PointXYZ> ece;
            // /*距離の閾値を設定*/
            // ece.setClusterTolerance(cluster_tolerance);
            // /*各クラスタのメンバの最小数を設定*/
            // ece.setMinClusterSize(min_cluster_size);
            // /*各クラスタのメンバの最大数を設定*/
            // ece.setMaxClusterSize(dummy_cloud->points.size());
            // /*探索方法を設定*/
            // ece.setSearchMethod(tree);
            // /*クラスリング対象の点群をinput*/
            // ece.setInputCloud(dummy_cloud);
            // /*クラスリング実行*/
            // ece.extract(cluster_indices);

            // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster_cloud {new pcl::PointCloud<pcl::PointXYZRGB>};

            // // /*dividing（クラスタごとに点群を分割）*/
            // pcl::ExtractIndices<pcl::PointXYZ> ei;
            // ei.setInputCloud(dummy_cloud);
            // ei.setNegative(false);

            // std::random_device rd; // 乱数生成
            // std::default_random_engine eng(rd()); //疑似乱数の指定
            // std::uniform_int_distribution<int> distr(0, 255); //0 ~255の一様乱数を生成
            // int cloud_cnt = 0;
            // // std::cout << "3: " << std::endl;

            // cluster_cloud->points.resize(dummy_cloud->points.size());

            // for(size_t i=0;i<cluster_indices.size();i++){
            //     /*extract*/

            //     u_int8_t cluster_red =  distr(eng); // 乱数生成0~255
            //     u_int8_t cluster_blue =  distr(eng); // 乱数生成0~255
            //     u_int8_t cluster_green =  distr(eng); // 乱数生成0~255

            //     pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_clustered_points (new pcl::PointCloud<pcl::PointXYZ>);
            //     pcl::PointIndices::Ptr tmp_clustered_indices (new pcl::PointIndices);
            //     *tmp_clustered_indices = cluster_indices[i];
            //     ei.setIndices(tmp_clustered_indices);
            //     ei.filter(*tmp_clustered_points);
            //     // std::cout << "4: " << std::endl;

            //     for (size_t j=0;j< tmp_clustered_points->points.size();j++)
            //     {
            //         cluster_cloud->points[cloud_cnt].x = tmp_clustered_points->points[j].x;
            //         cluster_cloud->points[cloud_cnt].y = tmp_clustered_points->points[j].y;
            //         cluster_cloud->points[cloud_cnt].z = tmp_clustered_points->points[j].z;
            //         cluster_cloud->points[cloud_cnt].r = cluster_red;
            //         cluster_cloud->points[cloud_cnt].g = cluster_green;
            //         cluster_cloud->points[cloud_cnt].b = cluster_blue;
            //         cloud_cnt++;

            //     }

            // }

            // sensor_msgs::PointCloud2 msg;
            // pcl::toROSMsg(*cluster_cloud, msg);
            // msg.header.frame_id = "laser";
            // pc_pub.publish(msg);
        }

        // rate.sleep();
    }
    fclose(gnu);

    return 0;
}