#ifndef _PLAN_CONTAINER_H_
#define _PLAN_CONTAINER_H_

#include <Eigen/Eigen>
#include <vector>
#include <ros/ros.h>

#include "traj_utils/poly_traj_utils.hpp"

using std::vector;

namespace remani_planner
{
  struct CarFlatTrajData{
    int singul;
    std::vector<Eigen::Vector3d> traj_pts; //某单个方向上均匀采样后的所有坐标点：x,y,t  有终点没有起点
    std::vector<double> thetas;    // 储存均匀采样后的yaw
    Eigen::MatrixXd start_state;   // start flat state pva
    Eigen::MatrixXd final_state;   // end flat state (2, 3)
  };

  struct FlatTrajData{
    int singul;
    std::vector<Eigen::VectorXd> traj_pts; // 某单个方向上均匀采样后的所有坐标点：x, y, theta 1-6  有终点没有起点
    std::vector<double> traj_times; // 时间分配
    std::vector<double> thetas;     // 储存均匀采样后的yaw
    Eigen::MatrixXd start_state;    // start flat state pvaj (8, 4)
    Eigen::MatrixXd final_state;    // end flat state 
  };

  struct GlobalTrajData
  {
    poly_traj::Trajectory<7> traj;
    double global_start_time; // world time
    double duration;

    /* Global traj time. 
       The corresponding global trajectory time of the current local target.
       Used in local target selection process */
    double glb_t_of_lc_tgt;
    /* Global traj time. 
       The corresponding global trajectory time of the last local target.
       Used in initial-path-from-last-optimal-trajectory generation process */
    double last_glb_t_of_lc_tgt;
  };

  struct LocalTrajData
  {
    poly_traj::Trajectory<7> traj;
    int drone_id; // A negative value indicates no received trajectories.
    int traj_id;
    double duration;
    double start_time; // world time
    double end_time;   // world time
    Eigen::VectorXd start_pos;
    double init_angle;
    int singul;
  };

  typedef std::vector<LocalTrajData> SingulTraj;
  typedef std::vector<FlatTrajData> KinoTraj;

  struct SingulTrajData
  {
    SingulTraj singul_traj;
    int traj_id = 0;
    double duration = 0;
    double start_time;
    ros::Time start_time_ros;
    
    void addSingulTraj(const poly_traj::Trajectory<7> &trajectory, const double &world_time){
      if(traj_id == 0) start_time = world_time;
      LocalTrajData local_traj;
      local_traj.drone_id = -1;
      local_traj.traj_id = ++traj_id;
      local_traj.duration = trajectory.getTotalDuration();
      local_traj.start_pos = trajectory.getJuncPos(0);
      local_traj.start_time = world_time;
      local_traj.end_time = world_time + local_traj.duration;
      local_traj.traj = trajectory;
      local_traj.singul = trajectory.getPiece(0).getSingul();

      duration += local_traj.duration;
      singul_traj.push_back(local_traj);
    }

    void clearSingulTraj(){
      singul_traj.clear();
      traj_id = 0;
      duration = 0;
    }

    int locatePieceIdx(double &t) const
    {
      int N = singul_traj.size();
      int idx;
      double dur;
      for (idx = 0; idx < N && t > (dur = singul_traj[idx].duration); idx++)
      {
        t -= dur;
      }
      if (idx == N)
      {
        idx--;
        t += singul_traj[idx].duration;
      }
      return idx;
    }

    int getPieceIdx(double t) const
    {
      int N = singul_traj.size();
      int idx;
      double dur;
      for (idx = 0; idx < N && t > (dur = singul_traj[idx].duration); idx++)
      {
        t -= dur;
      }
      if (idx == N)
      {
        idx--;
        t += singul_traj[idx].duration;
      }
      return idx;
    }

    Eigen::VectorXd getPos(double t) const{
        int pieceIdx = locatePieceIdx(t);
        return singul_traj[pieceIdx].traj.getPos(t);
    }

    Eigen::VectorXd getVel(double t) const
    {
      int pieceIdx = locatePieceIdx(t);
      return singul_traj[pieceIdx].traj.getVel(t);
    }

    Eigen::VectorXd getAcc(double t) const
    {
      int pieceIdx = locatePieceIdx(t);
      return singul_traj[pieceIdx].traj.getAcc(t);
    }

    Eigen::VectorXd getJer(double t) const
    {
      int pieceIdx = locatePieceIdx(t);
      return singul_traj[pieceIdx].traj.getJer(t);
    }

    double getCarAngle(double t) const
    {
      int pieceIdx = locatePieceIdx(t);
      return singul_traj[pieceIdx].traj.getCarAngle(t);
    }

    int getSingul(double t) const
    {
      int pieceIdx = locatePieceIdx(t);
      return singul_traj[pieceIdx].singul;
    }


  };

  class TrajContainer
  {
  public:
    GlobalTrajData global_traj;
    SingulTrajData singul_traj_data;

    TrajContainer(){}
    ~TrajContainer(){}

    void setGlobalTraj(const poly_traj::Trajectory<7> &trajectory, const double &world_time)
    {
      global_traj.traj = trajectory;
      global_traj.duration = trajectory.getTotalDuration();
      global_traj.global_start_time = world_time;
      global_traj.glb_t_of_lc_tgt = world_time;
      global_traj.last_glb_t_of_lc_tgt = -1.0;
    }
  };

  struct PlanParameters
  {
    /* planning algorithm parameters */
    double max_vel_, max_acc_;     // physical limits
    double max_mani_vel_;
    double polyTraj_piece_length;  // 
    double polyTraj_piece_time;
    double feasibility_tolerance_; // permitted ratio of vel/acc exceeding limits
    double planning_horizen_;
    int drone_id; // single drone: drone_id <= -1, swarm: drone_id >= 0

    /* processing time */
    double time_search_ = 0.0;
    double time_optimize_ = 0.0;
    double time_adjust_ = 0.0;

    int mobile_base_dim_, manipulator_dim_, traj_dim_;
    double mobile_base_non_singul_vel_;
  };

} // namespace remani_planner

#endif