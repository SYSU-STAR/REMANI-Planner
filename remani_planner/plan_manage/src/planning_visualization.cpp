#include <plan_manage/planning_visualization.h>

using std::cout;
using std::endl;
namespace remani_planner
{
  PlanningVisualization::PlanningVisualization(ros::NodeHandle &nh){
    node = nh;

    goal_point_pub = nh.advertise<visualization_msgs::Marker>("goal_point", 2);
    global_traj_pub = nh.advertise<visualization_msgs::Marker>("global_traj", 2);
    init_ctrl_pts_pub = nh.advertise<visualization_msgs::Marker>("init_ctrl_pts", 2);
    optmizing_traj_pub = nh.advertise<visualization_msgs::Marker>("optmizing_traj", 2);
    init_waypoints_pub = nh.advertise<visualization_msgs::Marker>("init_waypoints", 2);
    optimal_ctrl_pts_pub = nh.advertise<visualization_msgs::Marker>("optimal_ctrl_pts", 2);
    optimal_waypoints_pub = nh.advertise<visualization_msgs::Marker>("optimal_waypoints", 2);
    failed_list_pub = nh.advertise<visualization_msgs::Marker>("failed_list", 2);
    a_star_list_pub = nh.advertise<visualization_msgs::Marker>("a_star_list", 20);
    init_list_debug_pub = nh.advertise<visualization_msgs::Marker>("init_debug_list",2);
    
    intermediate_pt0_pub = nh.advertise<visualization_msgs::Marker>("pt0_dur_opt", 10);
    intermediate_grad0_pub = nh.advertise<visualization_msgs::MarkerArray>("grad0_dur_opt", 10);
    intermediate_pt1_pub = nh.advertise<visualization_msgs::Marker>("pt1_dur_opt", 10);
    intermediate_grad1_pub = nh.advertise<visualization_msgs::MarkerArray>("grad1_dur_opt", 10);
    intermediate_grad_smoo_pub = nh.advertise<visualization_msgs::MarkerArray>("smoo_grad_dur_opt", 10);
    intermediate_grad_dist_pub = nh.advertise<visualization_msgs::MarkerArray>("dist_grad_dur_opt", 10);
    intermediate_grad_feas_pub = nh.advertise<visualization_msgs::MarkerArray>("feas_grad_dur_opt", 10);
    
    t_init = ros::Time::now();
  }

  // // real ids used: {id, id+1000}
  void PlanningVisualization::displayMarkerList(ros::Publisher &pub, const vector<Eigen::Vector3d> &list, double scale,
                                                Eigen::Vector4d color, int id, bool show_sphere /* = true */ )
  {
    visualization_msgs::Marker sphere, line_strip;
    sphere.header.frame_id = line_strip.header.frame_id = "world";
    sphere.header.stamp = line_strip.header.stamp = ros::Time::now();
    sphere.type = visualization_msgs::Marker::SPHERE_LIST;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    sphere.action = line_strip.action = visualization_msgs::Marker::ADD;
    sphere.id = id;
    line_strip.id = id + 1000;

    sphere.pose.orientation.w = line_strip.pose.orientation.w = 1.0;
    sphere.color.r = line_strip.color.r = color(0);
    sphere.color.g = line_strip.color.g = color(1);
    sphere.color.b = line_strip.color.b = color(2);
    sphere.color.a = line_strip.color.a = color(3) > 1e-5 ? color(3) : 1.0;
    sphere.scale.x = scale;
    sphere.scale.y = scale;
    sphere.scale.z = scale;
    line_strip.scale.x = scale / 2;
    geometry_msgs::Point pt;
    for (int i = 0; i < int(list.size()); i++)
    {
      pt.x = list[i](0);
      pt.y = list[i](1);
      pt.z = list[i](2);
      if (show_sphere) sphere.points.push_back(pt);
      else line_strip.points.push_back(pt);
    }
    if (show_sphere) pub.publish(sphere);
    else pub.publish(line_strip);
  }

  // real ids used: {id, id+1}
  void PlanningVisualization::generatePathDisplayArray(visualization_msgs::MarkerArray &array,
                                                       const vector<Eigen::Vector3d> &list, double scale, Eigen::Vector4d color, int id)
  {
    visualization_msgs::Marker sphere, line_strip;
    sphere.header.frame_id = line_strip.header.frame_id = "world";
    sphere.header.stamp = line_strip.header.stamp = ros::Time::now();
    sphere.type = visualization_msgs::Marker::SPHERE_LIST;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    sphere.action = line_strip.action = visualization_msgs::Marker::ADD;
    sphere.id = id;
    line_strip.id = id + 1;

    sphere.pose.orientation.w = line_strip.pose.orientation.w = 1.0;
    sphere.color.r = line_strip.color.r = color(0);
    sphere.color.g = line_strip.color.g = color(1);
    sphere.color.b = line_strip.color.b = color(2);
    sphere.color.a = line_strip.color.a = color(3) > 1e-5 ? color(3) : 1.0;
    sphere.scale.x = scale;
    sphere.scale.y = scale;
    sphere.scale.z = scale;
    line_strip.scale.x = scale / 3;
    geometry_msgs::Point pt;
    for (int i = 0; i < int(list.size()); i++)
    {
      pt.x = list[i](0);
      pt.y = list[i](1);
      pt.z = list[i](2);
      sphere.points.push_back(pt);
      line_strip.points.push_back(pt);
    }
    array.markers.push_back(sphere);
    array.markers.push_back(line_strip);
  }

  // real ids used: {1000*id ~ (arrow nums)+1000*id}
  void PlanningVisualization::generateArrowDisplayArray(visualization_msgs::MarkerArray &array,
                                                        const vector<Eigen::Vector3d> &list, double scale, Eigen::Vector4d color, int id)
  {
    visualization_msgs::Marker arrow;
    arrow.header.frame_id = "world";
    arrow.header.stamp = ros::Time::now();
    arrow.type = visualization_msgs::Marker::ARROW;
    arrow.action = visualization_msgs::Marker::ADD;

    // geometry_msgs::Point start, end;
    // arrow.points

    arrow.color.r = color(0);
    arrow.color.g = color(1);
    arrow.color.b = color(2);
    arrow.color.a = color(3) > 1e-5 ? color(3) : 1.0;
    arrow.scale.x = scale;
    arrow.scale.y = 2 * scale;
    arrow.scale.z = 2 * scale;

    geometry_msgs::Point start, end;
    for (int i = 0; i < int(list.size() / 2); i++)
    {
      // arrow.color.r = color(0) / (1+i);
      // arrow.color.g = color(1) / (1+i);
      // arrow.color.b = color(2) / (1+i);

      start.x = list[2 * i](0);
      start.y = list[2 * i](1);
      start.z = list[2 * i](2);
      end.x = list[2 * i + 1](0);
      end.y = list[2 * i + 1](1);
      end.z = list[2 * i + 1](2);
      arrow.points.clear();
      arrow.points.push_back(start);
      arrow.points.push_back(end);
      arrow.id = i + id * 1000;

      array.markers.push_back(arrow);
    }
  }

  void PlanningVisualization::displayGoalPoint(Eigen::Vector2d goal_point, Eigen::Vector4d color, const double scale, int id)
  {
    visualization_msgs::Marker sphere;
    sphere.header.frame_id = "world";
    sphere.header.stamp = ros::Time::now();
    sphere.type = visualization_msgs::Marker::SPHERE;
    sphere.action = visualization_msgs::Marker::ADD;
    sphere.id = id;

    sphere.pose.orientation.w = 1.0;
    sphere.color.r = color(0);
    sphere.color.g = color(1);
    sphere.color.b = color(2);
    sphere.color.a = color(3);
    sphere.scale.x = scale;
    sphere.scale.y = scale;
    sphere.scale.z = scale;
    sphere.pose.position.x = goal_point(0);
    sphere.pose.position.y = goal_point(1);
    sphere.pose.position.z = 0.0;

    goal_point_pub.publish(sphere);
  }

  void PlanningVisualization::displayGlobalTraj(vector<Eigen::Vector2d> init_pts, const double scale, int id)
  {

    if (global_traj_pub.getNumSubscribers() == 0)
    {
      return;
    }

    vector<Eigen::Vector3d> init_pts_3d;
    for(unsigned int i = 0; i < init_pts.size(); ++i){
      init_pts_3d.push_back(Eigen::Vector3d(init_pts[i](0), init_pts[i](1), 0.0));
    }

    Eigen::Vector4d color(0, 0, 1, 1);
    displayMarkerList(global_traj_pub, init_pts_3d, scale, color, id);
  }

  void PlanningVisualization::displayOptimizingTraj(vector<vector<Eigen::Vector2d>> trajs, const double scale)
  {

    if (optmizing_traj_pub.getNumSubscribers() == 0)
    {
      return;
    }

    static int last_nums = 0;

    for ( int id=0; id<last_nums; id++ )
    {
      Eigen::Vector4d color(0, 0, 0, 0);
      vector<Eigen::Vector3d> blank;
      displayMarkerList(optmizing_traj_pub, blank, scale, color, id, false);
      ros::Duration(0.001).sleep();
    }
    last_nums = 0;

    for ( int id=0; id<(int)trajs.size(); id++ )
    {
      vector<Eigen::Vector3d> init_pts_3d;
      for(unsigned int i = 0; i < trajs[id].size(); ++i){
        init_pts_3d.push_back(Eigen::Vector3d(trajs[id][i](0), trajs[id][i](1), 0.0));
      }
      Eigen::Vector4d color(0, 0, 1, 0.7);
      displayMarkerList(optmizing_traj_pub, init_pts_3d, scale, color, id, false);
      ros::Duration(0.001).sleep();
      last_nums++;
    }

  }

  void PlanningVisualization::displayInitCtrlPts(vector<Eigen::Vector2d> init_pts, const double scale, int id)
  {
    if (init_ctrl_pts_pub.getNumSubscribers() == 0)
    {
      return;
    }

    vector<Eigen::Vector3d> pts_3d;
    for(unsigned int i = 0; i < init_pts.size(); ++i){
      pts_3d.push_back(Eigen::Vector3d(init_pts[i](0), init_pts[i](1), 0.0));
    }

    Eigen::Vector4d color(0, 0, 1, 1);
    displayMarkerList(init_ctrl_pts_pub, pts_3d, scale, color, id);
  }

  void PlanningVisualization::displayInitWaypoints(vector<Eigen::Vector2d> pts, const double scale, int id)
  {
    if (init_waypoints_pub.getNumSubscribers() == 0)
    {
      return;
    }

    vector<Eigen::Vector3d> pts_3d;
    for(unsigned int i = 0; i < pts.size(); ++i){
      pts_3d.push_back(Eigen::Vector3d(pts[i](0), pts[i](1), 0.0));
    }

    Eigen::Vector4d color(0, 0, 0, 1);
    displayMarkerList(init_waypoints_pub, pts_3d, scale, color, id, true);
  }

  void PlanningVisualization::displayOptWaypoints(vector<Eigen::Vector2d> pts, const double scale, int id)
  {
    if (optimal_waypoints_pub.getNumSubscribers() == 0)
    {
      return;
    }
    
    if(id == 0){
      visualization_msgs::Marker MarkerDelete;
      MarkerDelete.action = visualization_msgs::Marker::DELETEALL;
      optimal_waypoints_pub.publish(MarkerDelete);
    }

    vector<Eigen::Vector3d> opt_pts_3d;
    for(unsigned int i = 0; i < pts.size(); ++i){
      opt_pts_3d.push_back(Eigen::Vector3d(pts[i](0), pts[i](1), 0.0));
    }

    Eigen::Vector4d color(0, 1, 1, 1);
    displayMarkerList(optimal_waypoints_pub, opt_pts_3d, scale, color, id);
  }

  void PlanningVisualization::displayInitPathListDebug(vector<Eigen::Vector2d> init_pts, const double scale, int id)
  {

    if (init_list_debug_pub.getNumSubscribers() == 0)
    {
      return;
    }

    vector<Eigen::Vector3d> init_pts_3d;
    for(unsigned int i = 0; i < init_pts.size(); ++i){
      init_pts_3d.push_back(Eigen::Vector3d(init_pts[i](0), init_pts[i](1), 0.0));
    }

    Eigen::Vector4d color(1, 1, 0, 1);
    displayMarkerList(init_list_debug_pub, init_pts_3d, scale, color, id);
  }

  void PlanningVisualization::displayOptimalCtrlPts(std::vector<Eigen::MatrixXd> optimal_pts_list, int id)
  {

    if (optimal_ctrl_pts_pub.getNumSubscribers() == 0)
    {
      return;
    }

    if(id == 0){
      visualization_msgs::Marker MarkerDelete;
      MarkerDelete.action = visualization_msgs::Marker::DELETEALL;
      optimal_ctrl_pts_pub.publish(MarkerDelete);
    }

    vector<Eigen::Vector3d> list;
    for(unsigned int j = 0; j < optimal_pts_list.size(); ++j){
      Eigen::MatrixXd optimal_pts = optimal_pts_list[j];
      for (int i = 0; i < (int)optimal_pts.cols(); i++)
      {
        Eigen::Vector3d pt;
        pt.setZero();
        pt.head(2) = optimal_pts.col(i).transpose().head(2);
        list.push_back(pt);
      }
    }
    
    Eigen::Vector4d color(1.0, 0.0, 0, 0.6);
    displayMarkerList(optimal_ctrl_pts_pub, list, 0.08, color, id);
  }

  void PlanningVisualization::displayFailedList(Eigen::MatrixXd failed_pts, int id)
  {

    if (failed_list_pub.getNumSubscribers() == 0)
    {
      return;
    }

    if(id == 0){
      visualization_msgs::Marker MarkerDelete;
      MarkerDelete.action = visualization_msgs::Marker::DELETEALL;
      failed_list_pub.publish(MarkerDelete);
    }

    vector<Eigen::Vector3d> list;
    Eigen::Vector3d pt;
    pt.setZero();
    for (int i = 0; i < failed_pts.cols(); i++)
    {
      pt.head(2) = failed_pts.col(i).head(2);
      list.push_back(pt);
    }
    Eigen::Vector4d color(0.3, 0, 0, 1);
    displayMarkerList(failed_list_pub, list, 0.05, color, id);
  }

  void PlanningVisualization::displayAStarList(std::vector<std::vector<Eigen::Vector2d>> a_star_paths, int id /* = Eigen::Vector4d(0.5,0.5,0,1)*/)
  {

    if (a_star_list_pub.getNumSubscribers() == 0)
    {
      return;
    }

    int i = 0;
    vector<Eigen::Vector3d> list;

    Eigen::Vector4d color = Eigen::Vector4d(0.5 + ((double)rand() / RAND_MAX / 2), 0.5 + ((double)rand() / RAND_MAX / 2), 0, 1); // make the A star pathes different every time.
    double scale = 0.05 + (double)rand() / RAND_MAX / 10;

    for (auto block : a_star_paths)
    {
      
      for (auto pt : block)
      {
        list.push_back(Eigen::Vector3d(pt(0), pt(1), 0.1));
      }
      //Eigen::Vector4d color(0.5,0.5,0,1);
      
      i++;
    }
    displayMarkerList(a_star_list_pub, list, scale, color, id); // real ids used: [ id ~ id+a_star_paths.size() ]
  }

  void PlanningVisualization::displayArrowList(ros::Publisher &pub, const vector<Eigen::Vector3d> &list, double scale, Eigen::Vector4d color, int id)
  {
    visualization_msgs::MarkerArray array;
    // clear
    pub.publish(array);

    generateArrowDisplayArray(array, list, scale, color, id);

    pub.publish(array);
  }

  void PlanningVisualization::displayIntermediatePt(std::string type, Eigen::MatrixXd &pts, int id, Eigen::Vector4d color)
  {
    std::vector<Eigen::Vector3d> pts_;
    pts_.reserve(pts.cols());
    for ( int i=0; i<pts.cols(); i++ )
    {
      pts_.emplace_back(pts.col(i));
    }

    if ( !type.compare("0") )
    {
      displayMarkerList(intermediate_pt0_pub, pts_, 0.1, color, id);
    }
    else if ( !type.compare("1") )
    {
      displayMarkerList(intermediate_pt1_pub, pts_, 0.1, color, id);
    }
  }

  void PlanningVisualization::displayIntermediateGrad(std::string type, Eigen::MatrixXd &pts, Eigen::MatrixXd &grad, int id, Eigen::Vector4d color)
  {
    if ( pts.cols() != grad.cols() )
    {
      ROS_ERROR("pts.cols() != grad.cols()");
      return;
    }
    std::vector<Eigen::Vector3d> arrow_;
    arrow_.reserve(pts.cols()*2);
    if ( !type.compare("swarm") )
    {
      for ( int i=0; i<pts.cols(); i++ )
      {
        arrow_.emplace_back(pts.col(i));
        arrow_.emplace_back(grad.col(i));
      }
    }
    else
    {
      for ( int i=0; i<pts.cols(); i++ )
      {
        arrow_.emplace_back(pts.col(i));
        arrow_.emplace_back(pts.col(i)+grad.col(i));
      }
    }
    

    if ( !type.compare("grad0") )
    {
      displayArrowList(intermediate_grad0_pub, arrow_, 0.05, color, id);
    }
    else if ( !type.compare("grad1") )
    {
      displayArrowList(intermediate_grad1_pub, arrow_, 0.05, color, id);
    }
    else if ( !type.compare("dist") )
    {
      displayArrowList(intermediate_grad_dist_pub, arrow_, 0.05, color, id);
    }
    else if ( !type.compare("smoo") )
    {
      displayArrowList(intermediate_grad_smoo_pub, arrow_, 0.05, color, id);
    }
    else if ( !type.compare("feas") )
    {
      displayArrowList(intermediate_grad_feas_pub, arrow_, 0.05, color, id);
    }
    
  }

  // PlanningVisualization::
} // namespace remani_planner