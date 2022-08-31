# readme
* クラスタリングの説明
* laser_to_clustering.cppの説明
    * laserScanからポイントクラウド型に変換してクラスタリングしている
* 起動方法
  * roscoreの起動
  * roslaunch urg_node urg_lidar.launch
  * rosrun potential_planner dummy_point_publisher
  * rvizで確認
    * laser
    * pointcloud2をaddで確認
* ソースコードの解説</br>
今後書きます

* pcl::PointCloud<pcl::PointXYZ> のドキュメントはhttps://pointclouds.org/documentation/singletonpcl_1_1_point_cloud.html

