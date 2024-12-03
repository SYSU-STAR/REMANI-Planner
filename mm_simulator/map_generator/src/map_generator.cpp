#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
// #include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <iostream>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>
#include <Eigen/Eigen>
#include <random>
#include <std_msgs/Int32.h>


using namespace std; 

pcl::KdTreeFLANN<pcl::PointXYZ> kdtreeLocalMap;
random_device rd;
default_random_engine eng;
uniform_real_distribution<double> rand_x;
uniform_real_distribution<double> rand_y;
uniform_real_distribution<double> rand_z;

ros::Publisher _all_map_pub;

int _obs_num, _float_obs_num;
int _map_type;
double _x_size, _y_size, _z_size;
double _x_l, _x_h, _y_l, _y_h;
double _resolution, _pub_rate;
double _min_dist;

bool _map_ok = false;
bool _has_odom = false;

uniform_real_distribution<double> rand_z_;
uniform_real_distribution<double> rand_box_x_;
uniform_real_distribution<double> rand_box_y_;
uniform_real_distribution<double> rand_box_z_;

sensor_msgs::PointCloud2 globalMap_pcd;
pcl::PointCloud<pcl::PointXYZ> cloudMap;

void GenerateWall(double x_l, double x_h, 
                  double y_l, double y_h, 
                  double z_l, double z_h, 
                  pcl::PointCloud<pcl::PointXYZ>& cloudMap){
  int x_num, y_num, z_num;
  double resolution = _resolution / 2.0;
  x_num = ceil((x_h - x_l)/resolution);
  y_num = ceil((y_h - y_l)/resolution);
  z_num = ceil((z_h - z_l)/resolution);
  pcl::PointXYZ pt;
  for (int i=0; i<x_num; i++)
    for (int j=0; j<y_num; j++)
      for (int k=0; k<z_num; k++){
        pt.x = x_l + i * resolution;
        pt.y = y_l + j * resolution;
        pt.z = z_l + k * resolution;
        cloudMap.points.push_back(pt);
      }
}

void GenerateBox(Eigen::Vector3d pos, Eigen::Vector3d box_size, pcl::PointCloud<pcl::PointXYZ>& cloudMap){
  GenerateWall(pos(0) - box_size(0) / 2, pos(0) + box_size(0) / 2, 
               pos(1) - box_size(1) / 2, pos(1) + box_size(1) / 2,
               pos(2) - box_size(2) / 2, pos(2) + box_size(2) / 2, 
               cloudMap);
}

void GenerateWall(Eigen::Vector3d pos, double theta, Eigen::Vector3d wall_size, pcl::PointCloud<pcl::PointXYZ>& cloudMap){
  Eigen::Vector2d dirx(cos(theta), sin(theta));
  Eigen::Vector2d diry(cos(theta + M_PI_2), sin(theta + M_PI_2));
  double resolution = _resolution / 3.0;
  int x_num = ceil(wall_size(0) / resolution);
  int y_num = ceil(wall_size(1) / resolution);
  int z_num = ceil(wall_size(2) / resolution);
  Eigen::Vector2d posx;
  Eigen::Vector2d posy;
  pcl::PointXYZ pt;
  for (int i=0; i<x_num; i++){
    posx = pos.head(2) + (double)i * resolution * dirx;
    pt.x = posx(0);
    for (int j=0; j<y_num; j++){
      posy = posx + (double)j * resolution * diry;
      pt.y = posy(1);
      for (int k=0; k<z_num; k++){
        pt.z = pos(2) + (double)k * resolution;
        cloudMap.points.push_back(pt);
      }
    }
  }
}

void GenerateBridge(){
  pcl::PointXYZ pt_random;
  vector<Eigen::Vector2d> obs_position, obs_size;
  double x, y;
  Eigen::Vector3d bridge_size, box_size;
  Eigen::Vector2d bridge_pos;
  bridge_size << 0.8, 1.8, 0.4;
  box_size << 0.2, 0.2, 0.2;
  int bridge_num = 1;

  rand_x = uniform_real_distribution<double>(_x_l * 0.75, _x_h * 0.75);
  rand_y = uniform_real_distribution<double>(_y_l, _y_h);
  rand_z_ = uniform_real_distribution<double>(-0.1, 0.1);

  rand_box_x_ = uniform_real_distribution<double>(0.1, 0.8);
  rand_box_y_ = uniform_real_distribution<double>(0.1, 0.8);
  rand_box_z_ = uniform_real_distribution<double>(0.8, 3.0);

  // generate wall
  GenerateWall(_x_l, _x_h, _y_l, _y_l + _resolution, 0.0, 0.2, cloudMap);
  GenerateWall(_x_l, _x_h, _y_h - _resolution, _y_h, 0.0, 0.2, cloudMap);
  GenerateWall(_x_l, _x_l + _resolution, _y_l, _y_h, 0.0, 0.2, cloudMap);
  GenerateWall(_x_h - _resolution, _x_h, _y_l, _y_h, 0.0, 0.2, cloudMap);

  GenerateWall(-_resolution, _resolution, _y_l, -0.75, 0.0, 0.2, cloudMap);
  GenerateWall(-_resolution, _resolution, 0.75, _y_h, 0.0, 0.2, cloudMap);

  std::vector<Eigen::Vector2d> bridge_pos_list;
  std::vector<Eigen::Vector3d> bridge_size_list;
  for(int i = 0; i < bridge_num; ++i){
    switch(i){
      case 0 :{
        bridge_pos << 0.0, 0.0;
        bridge_size << 0.6, 1.5, 0.7;
        break;
      }
    }
    bridge_pos_list.push_back(bridge_pos);
    bridge_size_list.push_back(bridge_size);
  }

  for (int i = 0; i < bridge_num && ros::ok(); i++){
    x = bridge_pos_list[i](0);
    y = bridge_pos_list[i](1);
    bridge_size = bridge_size_list[i];

    obs_position.push_back(Eigen::Vector2d(x, y));

    x = floor(x / _resolution) * _resolution + _resolution / 2.0;
    y = floor(y / _resolution) * _resolution + _resolution / 2.0;

    auto pos = Eigen::Vector2d(x, y);
      // board
    GenerateWall(pos(0) - bridge_size(0) / 2.0, pos(0) + bridge_size(0) / 2.0,
                pos(1) - bridge_size(1) / 2.0, pos(1) + bridge_size(1) / 2.0,
                bridge_size(2), bridge_size(2) + 1e-3 + 2 * _resolution,
                cloudMap);
  // feet
    std::vector<Eigen::Vector2d> corner_list;
    Eigen::Vector2d corner;
    corner << pos(0) - bridge_size(0) / 2, pos(1) - bridge_size(1) / 2;
    corner_list.push_back(corner);
    corner << pos(0) - bridge_size(0) / 2, pos(1) + bridge_size(1) / 2;
    corner_list.push_back(corner);
    corner << pos(0) + bridge_size(0) / 2, pos(1) - bridge_size(1) / 2;
    corner_list.push_back(corner);
    corner << pos(0) + bridge_size(0) / 2, pos(1) + bridge_size(1) / 2;
    corner_list.push_back(corner);

    GenerateWall(pos(0) - bridge_size(0) / 2, pos(0) + bridge_size(0) / 2, 
                pos(1) - bridge_size(1) / 2, pos(1) - bridge_size(1) / 2 + 2 * _resolution,
                0.0, bridge_size(2), cloudMap);
    GenerateWall(pos(0) - bridge_size(0) / 2, pos(0) + bridge_size(0) / 2 , 
                pos(1) + bridge_size(1) / 2 - 2 * _resolution, pos(1) + bridge_size(1) / 2,
                0.0, bridge_size(2), cloudMap);
  }

  GenerateWall(_x_l, _x_h, _y_l, _y_l + _resolution, 0.0, 0.2, cloudMap);
  GenerateWall(_x_l, _x_h, _y_h - _resolution, _y_h, 0.0, 0.2, cloudMap);
  GenerateWall(_x_l, _x_l + _resolution, _y_l, _y_h, 0.0, 0.2, cloudMap);
  GenerateWall(_x_h - _resolution, _x_h, _y_l, _y_h, 0.0, 0.2, cloudMap);

  GenerateBox(Eigen::Vector3d(_x_l + _resolution, _y_l + _resolution, 0.9), Eigen::Vector3d(0.1, 0.1, 1.8), cloudMap);
  GenerateBox(Eigen::Vector3d(_x_l + _resolution, _y_h - _resolution, 0.9), Eigen::Vector3d(0.1, 0.1, 1.8), cloudMap);
  GenerateBox(Eigen::Vector3d(_x_h - _resolution, _y_l + _resolution, 0.9), Eigen::Vector3d(0.1, 0.1, 1.8), cloudMap);
  GenerateBox(Eigen::Vector3d(_x_h - _resolution, _y_h - _resolution, 0.9), Eigen::Vector3d(0.1, 0.1, 1.8), cloudMap);

  cloudMap.width = cloudMap.points.size();
  cloudMap.height = 1;
  cloudMap.is_dense = true;


  kdtreeLocalMap.setInputCloud(cloudMap.makeShared());
  _map_ok = true;
  ROS_WARN("Finished generate Bridge Map ");
}

void GenerateCuboids(){
    pcl::PointXYZ pt_random;
  vector<Eigen::Vector2d> obs_position, obs_size;
  double x, y, z;
  Eigen::Vector3d bridge_size, box_size;
  Eigen::Vector2d bridge_pos;
  bridge_size << 0.8, 1.8, 0.4;
  box_size << 0.2, 0.2, 0.2;
  int bridge_num = 0;

  std::vector<Eigen::Vector2d> bridge_pos_list;
  std::vector<Eigen::Vector3d> bridge_size_list;
  for(int i = 0; i < bridge_num; ++i){
    switch(i){
      case 0 :{
        bridge_pos << 4.7, 2.2;
        bridge_size << 0.8, 1.8, 0.4;
        break;
      }
    }
    bridge_pos_list.push_back(bridge_pos);
    bridge_size_list.push_back(bridge_size);
  }


  rand_x = uniform_real_distribution<double>(_x_l * 0.75, _x_h * 0.75);
  rand_y = uniform_real_distribution<double>(_y_l, _y_h);
  rand_z_ = uniform_real_distribution<double>(-0.1, 0.1);

  auto rand_float_x = uniform_real_distribution<double>(_x_l * 0.6, _x_h * 0.6);
  auto rand_float_y = uniform_real_distribution<double>(_y_l, _y_h);
  auto rand_float_z = uniform_real_distribution<double>(0.6, 1.1);


  rand_box_x_ = uniform_real_distribution<double>(0.1, 0.8);
  rand_box_y_ = uniform_real_distribution<double>(0.1, 0.8);
  rand_box_z_ = uniform_real_distribution<double>(0.8, 3.0);

  auto rand_float_box_x = uniform_real_distribution<double>(0.3, 0.6);
  auto rand_float_box_y = uniform_real_distribution<double>(0.3, 0.6);
  auto rand_float_box_z = uniform_real_distribution<double>(0.3, 0.6);

  // generate wall
  GenerateWall(_x_l, _x_h, _y_l, _y_l + _resolution, 0.0, 0.2, cloudMap);
  GenerateWall(_x_l, _x_h, _y_h - _resolution, _y_h, 0.0, 0.2, cloudMap);
  GenerateWall(_x_l, _x_l + _resolution, _y_l, _y_h, 0.0, 0.2, cloudMap);
  GenerateWall(_x_h - _resolution, _x_h, _y_l, _y_h, 0.0, 0.2, cloudMap);

  // generate random box
  obs_position.clear();
  obs_size.clear();
  for (int i = 0; i < _obs_num && ros::ok(); ++i){
    x = rand_x(eng);
    y = rand_y(eng);
    z = rand_z_(eng);
    double size_x = rand_box_x_(eng);
    double size_y = rand_box_y_(eng);
    double size_z = rand_box_z_(eng);

    box_size << size_x, size_y, size_z;

    x = floor(x / _resolution) * _resolution + _resolution / 2.0;
    y = floor(y / _resolution) * _resolution + _resolution / 2.0;
    z = floor(z / _resolution) * _resolution + _resolution / 2.0;

    bool flag_continue = false;
    for (auto p : obs_position)
      if ((Eigen::Vector2d(x, y) - p).norm() < _min_dist /*metres*/)
      {
        i--;
        flag_continue = true;
        break;
      }
    if (flag_continue)
      continue;

    obs_position.push_back(Eigen::Vector2d(x, y));
    obs_size.push_back(Eigen::Vector2d(size_x, size_y));

    GenerateBox(Eigen::Vector3d(x, y, z), box_size, cloudMap);
  }

  obs_position.clear();
  obs_size.clear();
  for (int i = 0; i < _float_obs_num && ros::ok(); ++i){
    x = rand_float_x(eng);
    y = rand_float_y(eng);
    z = rand_float_z(eng);
    double size_x = rand_float_box_x(eng);
    double size_y = rand_float_box_y(eng);
    double size_z = rand_float_box_z(eng);

    box_size << size_x, size_y, size_z;

    x = floor(x / _resolution) * _resolution + _resolution / 2.0;
    y = floor(y / _resolution) * _resolution + _resolution / 2.0;
    z = floor(z / _resolution) * _resolution + _resolution / 2.0;

    bool flag_continue = false;
    for (auto p : obs_position)
      if ((Eigen::Vector2d(x, y) - p).norm() < 1.0 /*metres*/)
      {
        i--;
        flag_continue = true;
        break;
      }
    if (flag_continue)
      continue;

    obs_position.push_back(Eigen::Vector2d(x, y));
    obs_size.push_back(Eigen::Vector2d(size_x, size_y));

    GenerateBox(Eigen::Vector3d(x, y, z), box_size, cloudMap);
  }

  GenerateBox(Eigen::Vector3d(_x_l + _resolution, _y_l + _resolution, 0.9), Eigen::Vector3d(0.1, 0.1, 1.8), cloudMap);
  GenerateBox(Eigen::Vector3d(_x_l + _resolution, _y_h - _resolution, 0.9), Eigen::Vector3d(0.1, 0.1, 1.8), cloudMap);
  GenerateBox(Eigen::Vector3d(_x_h - _resolution, _y_l + _resolution, 0.9), Eigen::Vector3d(0.1, 0.1, 1.8), cloudMap);
  GenerateBox(Eigen::Vector3d(_x_h - _resolution, _y_h - _resolution, 0.9), Eigen::Vector3d(0.1, 0.1, 1.8), cloudMap);

  cloudMap.width = cloudMap.points.size();
  cloudMap.height = 1;
  cloudMap.is_dense = true;


  kdtreeLocalMap.setInputCloud(cloudMap.makeShared());
  _map_ok = true;
  ROS_WARN("Finished generate Cuboids Map ");
}

void pubPoints(){
  while (ros::ok())
  {
    ros::spinOnce();
    if (_map_ok)
      break;
  }
  pcl::toROSMsg(cloudMap, globalMap_pcd);
  globalMap_pcd.header.frame_id = "world";
  _all_map_pub.publish(globalMap_pcd);
}

int main(int argc, char **argv){
  
  ros::init(argc, argv, "random_map_sensing");
  ros::NodeHandle n("~");

  _all_map_pub = n.advertise<sensor_msgs::PointCloud2>("/map_generator/global_cloud", 1);

  n.param("map/x_size", _x_size, 50.0);
  n.param("map/y_size", _y_size, 50.0);
  n.param("map/z_size", _z_size, 5.0);
  n.param("map/obs_num", _obs_num, 30);
  n.param("map/float_obs_num", _float_obs_num, 30);
  n.param("map/map_type", _map_type, 0);
  n.param("map/resolution", _resolution, 0.1);

  int seed = 0;
  n.param("map/seed", seed, 0);
  if(seed == 0){seed = rd();} 
  eng.seed(seed);

  n.param("pub_rate", _pub_rate, 10.0);
  n.param("min_distance", _min_dist, 1.0);

  _x_l = -_x_size / 2.0;
  _x_h = +_x_size / 2.0;

  _y_l = -_y_size / 2.0;
  _y_h = +_y_size / 2.0;

  _obs_num = min(_obs_num, (int)_x_size * 10);
  
  if(_map_type == 0){
    GenerateCuboids();
  }else if(_map_type == 1){
    GenerateBridge();
  }

  ros::Rate loop_rate(_pub_rate);
  while (ros::ok()){
    pubPoints();   
    ros::spinOnce();
    loop_rate.sleep();
  }

}