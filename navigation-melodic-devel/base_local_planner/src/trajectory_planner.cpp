/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Eitan Marder-Eppstein
*********************************************************************/

#include <base_local_planner/trajectory_planner.h>
#include <costmap_2d/footprint.h>
#include <string>
#include <sstream>
#include <math.h>
#include <angles/angles.h>

#include <boost/algorithm/string.hpp>

#include <ros/console.h>

//for computing path distance
#include <queue>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/utils.h>

using namespace std;
using namespace costmap_2d;

//trajectory_planner.cpp是轨迹生成基本功能的实现程序
//不同的局部轨迹规划方法: navfn::NavfnROS
//                      carrot_planner::CarrotPlanner
//                      dwa_local_planner::DWAPlannerROS
//                      base_local_planner::TrajectoryPlannerROS
//很多都要用到这个基本类中的函数

namespace base_local_planner{

  void TrajectoryPlanner::reconfigure(BaseLocalPlannerConfig &cfg){
      BaseLocalPlannerConfig config(cfg);

      boost::mutex::scoped_lock l(configuration_mutex_);

      acc_lim_x_ = config.acc_lim_x;
      acc_lim_y_ = config.acc_lim_y;
      acc_lim_theta_ = config.acc_lim_theta;

      max_vel_x_ = config.max_vel_x;
      min_vel_x_ = config.min_vel_x;
      
      max_vel_th_ = config.max_vel_theta;
      min_vel_th_ = config.min_vel_theta;
      min_in_place_vel_th_ = config.min_in_place_vel_theta;

      sim_time_ = config.sim_time;
      sim_granularity_ = config.sim_granularity;
      angular_sim_granularity_ = config.angular_sim_granularity;

      path_distance_bias_ = config.path_distance_bias;
      goal_distance_bias_ = config.goal_distance_bias;
      occdist_scale_ = config.occdist_scale;

      if (meter_scoring_) {
        //if we use meter scoring, then we want to multiply the biases by the resolution of the costmap
        double resolution = costmap_.getResolution();
        goal_distance_bias_ *= resolution;
        path_distance_bias_ *= resolution;
      }

      oscillation_reset_dist_ = config.oscillation_reset_dist;
      escape_reset_dist_ = config.escape_reset_dist;
      escape_reset_theta_ = config.escape_reset_theta;

      vx_samples_ = config.vx_samples;
      vtheta_samples_ = config.vtheta_samples;

      if (vx_samples_ <= 0) {
          config.vx_samples = 1;
          vx_samples_ = config.vx_samples;
          ROS_WARN("You've specified that you don't want any samples in the x dimension. We'll at least assume that you want to sample one value... so we're going to set vx_samples to 1 instead");
      }
      if(vtheta_samples_ <= 0) {
          config.vtheta_samples = 1;
          vtheta_samples_ = config.vtheta_samples;
          ROS_WARN("You've specified that you don't want any samples in the theta dimension. We'll at least assume that you want to sample one value... so we're going to set vtheta_samples to 1 instead");
      }

      heading_lookahead_ = config.heading_lookahead;

      holonomic_robot_ = config.holonomic_robot;
      
      backup_vel_ = config.escape_vel;

      dwa_ = config.dwa;

      heading_scoring_ = config.heading_scoring;
      heading_scoring_timestep_ = config.heading_scoring_timestep;

      simple_attractor_ = config.simple_attractor;

      //y-vels
      string y_string = config.y_vels;
      vector<string> y_strs;
      boost::split(y_strs, y_string, boost::is_any_of(", "), boost::token_compress_on);

      vector<double>y_vels;
      for(vector<string>::iterator it=y_strs.begin(); it != y_strs.end(); ++it) {
          istringstream iss(*it);
          double temp;
          iss >> temp;
          y_vels.push_back(temp);
          //ROS_INFO("Adding y_vel: %e", temp);
      }
      y_vels_ = y_vels;
  }

  TrajectoryPlanner::TrajectoryPlanner(WorldModel& world_model,
                                       const Costmap2D& costmap,
                                       std::vector<geometry_msgs::Point> footprint_spec,
                                       double acc_lim_x, double acc_lim_y, double acc_lim_theta,
                                       double sim_time, double sim_granularity,
                                       int vx_samples, int vtheta_samples,
                                       double path_distance_bias, double goal_distance_bias, double occdist_scale,
                                       double heading_lookahead, double oscillation_reset_dist,
                                       double escape_reset_dist, double escape_reset_theta,
                                       bool holonomic_robot,
                                       double max_vel_x, double min_vel_x,
                                       double max_vel_th, double min_vel_th, double min_in_place_vel_th,
                                       double backup_vel,
                                       bool dwa, bool heading_scoring, double heading_scoring_timestep, 
                                       bool meter_scoring, bool simple_attractor,
                                       vector<double> y_vels, double stop_time_buffer, double sim_period, 
                                       double angular_sim_granularity)
    : path_map_(costmap.getSizeInCellsX(), costmap.getSizeInCellsY()),
      goal_map_(costmap.getSizeInCellsX(), costmap.getSizeInCellsY()),
      costmap_(costmap),
      world_model_(world_model), footprint_spec_(footprint_spec),
      sim_time_(sim_time), sim_granularity_(sim_granularity), 
      angular_sim_granularity_(angular_sim_granularity),
      vx_samples_(vx_samples), vtheta_samples_(vtheta_samples),
      path_distance_bias_(path_distance_bias), goal_distance_bias_(goal_distance_bias), 
      occdist_scale_(occdist_scale),
      acc_lim_x_(acc_lim_x), acc_lim_y_(acc_lim_y), acc_lim_theta_(acc_lim_theta),
      prev_x_(0), prev_y_(0), escape_x_(0), escape_y_(0), escape_theta_(0), 
      heading_lookahead_(heading_lookahead),
      oscillation_reset_dist_(oscillation_reset_dist), escape_reset_dist_(escape_reset_dist),
      escape_reset_theta_(escape_reset_theta), holonomic_robot_(holonomic_robot),
      max_vel_x_(max_vel_x), min_vel_x_(min_vel_x),
      max_vel_th_(max_vel_th), min_vel_th_(min_vel_th), min_in_place_vel_th_(min_in_place_vel_th),
      backup_vel_(backup_vel),
      dwa_(dwa), heading_scoring_(heading_scoring), heading_scoring_timestep_(heading_scoring_timestep),
      simple_attractor_(simple_attractor), y_vels_(y_vels), 
      stop_time_buffer_(stop_time_buffer), sim_period_(sim_period){
    //the robot is not stuck to begin with
    stuck_left = false;
    stuck_right = false;
    stuck_left_strafe = false;
    stuck_right_strafe = false;
    rotating_left = false;
    rotating_right = false;
    strafe_left = false;
    strafe_right = false;

    escaping_ = false;
    final_goal_position_valid_ = false;


    costmap_2d::calculateMinAndMaxDistances(footprint_spec_, inscribed_radius_, circumscribed_radius_);
  }

  TrajectoryPlanner::~TrajectoryPlanner(){}

  //获取每个栅格的代价值，由三个代价值构成
  bool TrajectoryPlanner::getCellCosts(int cx, int cy, 
                                       float &path_cost, float &goal_cost, float &occ_cost, 
                                       float &total_cost) {
    MapCell cell = path_map_(cx, cy);
    MapCell goal_cell = goal_map_(cx, cy);
    if (cell.within_robot) {
        return false;
    }
    occ_cost = costmap_.getCost(cx, cy);
    if (cell.target_dist == path_map_.obstacleCosts() ||
        cell.target_dist == path_map_.unreachableCellCosts() ||
        occ_cost >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
        return false;
    }
    path_cost = cell.target_dist;
    goal_cost = goal_cell.target_dist;
    total_cost = path_distance_bias_ * path_cost 
               + goal_distance_bias_ * goal_cost 
               + occdist_scale_ * occ_cost;
    return true;
  }

  //根据给定的速度和角速度采样生成单条路径和其代价(该函数被scoreTrajectory和createTrajectories调用)
  void TrajectoryPlanner::generateTrajectory(double x, double y, double theta,
                                             double vx, double vy, double vtheta,
                                             double vx_samp, double vy_samp, double vtheta_samp,
                                             double acc_x, double acc_y, double acc_theta,
                                             double impossible_cost, Trajectory& traj){
    //make sure the configuration doesn't change mid run
    boost::mutex::scoped_lock l(configuration_mutex_);

    double x_i = x;
    double y_i = y;
    double theta_i = theta;

    double vx_i, vy_i, vtheta_i;
    vx_i = vx;
    vy_i = vy;
    vtheta_i = vtheta;

    //计算线速度的大小(速度合成后)
    double vmag = hypot(vx_samp, vy_samp);

    //计算仿真步数(这里的计算原理是什么 下面的公式量纲都不同)
    int num_steps;
    if(!heading_scoring_) {
      //在这里sim_granularity_表示仿真点之间的距离间隔
      //仿真步数 = max(速度模×总仿真时间/距离间隔，角速度/角速度间隔)，四舍五入(fabs函数求绝对值)
      num_steps = int(max((vmag * sim_time_) / sim_granularity_, fabs(vtheta_samp) / angular_sim_granularity_) + 0.5);
    } else {
      //在这里sim_granularity_代表仿真的时间间隔
      num_steps = int(sim_time_ / sim_granularity_ + 0.5);
    }

    //至少选取一步,即使不会移动我们也会对当前位置进行评分
    if(num_steps == 0) {
      num_steps = 1;
    }

    //每一步的时间
    double dt = sim_time_ / num_steps;
    double time = 0.0;

    //初始化一条轨迹
    traj.resetPoints();
    traj.xv_ = vx_samp;
    traj.yv_ = vy_samp;
    traj.thetav_ = vtheta_samp;
    traj.cost_ = -1.0;

    //初始化轨迹的代价
    double path_dist = 0.0;
    double goal_dist = 0.0;
    double occ_cost = 0.0; //障碍物代价
    double heading_diff = 0.0; //航向角

    //循环生成轨迹，并计算轨迹对应的代价值
    for(int i = 0; i < num_steps; ++i){
      unsigned int cell_x, cell_y;
      //防止路径跑出已知地图
      //当前位置(x_i,y_i)转换到地图上，如果无法转换，说明该路径点不在地图上，将其代价设置为-1.0，并return
      if(!costmap_.worldToMap(x_i, y_i, cell_x, cell_y)){
        traj.cost_ = -1.0;
        return;
      }

      //考虑机器人的大小，把当前点扩张到机器人在该点的足迹范围
      //获得机器人在该点时它的足迹所对应的代价,如果足迹遇障，直接返回-1
      double footprint_cost = footprintCost(x_i, y_i, theta_i);

      //机器人在路径上遇障
      if(footprint_cost < 0){
        traj.cost_ = -1.0;
        return;
        //TODO: Really look at getMaxSpeedToStopInTime... dues to discretization errors and high acceleration limits,
        //it can actually cause the robot to hit obstacles. There may be something to be done to fix, but I'll have to
        //come back to it when I have time. Right now, pulling it out as it'll just make the robot a bit more conservative,
        //but safe.
        /*
        double max_vel_x, max_vel_y, max_vel_th;
        //we want to compute the max allowable speeds to be able to stop
        //to be safe... we'll make sure we can stop some time before we actually hit
        getMaxSpeedToStopInTime(time - stop_time_buffer_ - dt, max_vel_x, max_vel_y, max_vel_th);

        //check if we can stop in time
        if(fabs(vx_samp) < max_vel_x && fabs(vy_samp) < max_vel_y && fabs(vtheta_samp) < max_vel_th){
          ROS_ERROR("v: (%.2f, %.2f, %.2f), m: (%.2f, %.2f, %.2f) t:%.2f, st: %.2f, dt: %.2f", vx_samp, vy_samp, vtheta_samp, max_vel_x, max_vel_y, max_vel_th, time, stop_time_buffer_, dt);
          //if we can stop... we'll just break out of the loop here.. no point in checking future points
          break;
        }
        else{
          traj.cost_ = -1.0;
          return;
        }
        */
      }

      //更新occ_cost：把所有路径点的最大障碍物代价设置为路径的occ_cost
      occ_cost = std::max(std::max(occ_cost, footprint_cost), double(costmap_.getCost(cell_x, cell_y)));

      //这里感觉不管是简单的追踪策略还是复杂一些考虑goal_dist和path_dist甚至考虑heading_diff的策略
      //实际上都是只考虑了当前仿真轨迹的最后一点的cost，这样是不是比较草率?
      //还是说我理解不到位，当前仿真轨迹的最后一点的cost会受到前面各点的影响?

      //比较简单的追踪策略，只考虑与目标点之间的直线距离(只更新goal_dist)
      if (simple_attractor_) {
        goal_dist = (x_i - global_plan_[global_plan_.size() -1].pose.position.x) *
                    (x_i - global_plan_[global_plan_.size() -1].pose.position.x) +
                    (y_i - global_plan_[global_plan_.size() -1].pose.position.y) *
                    (y_i - global_plan_[global_plan_.size() -1].pose.position.y);
      //借助goal_map_和path_map_获取该点与目标点及全局规划路径之间的距离(更新goal_dist和path_dist)
      } else {
        bool update_path_and_goal_distances = true;

        //如果为朝向打分
        if (heading_scoring_) {
          //heading_scoring_timestep_是给朝向打分时在时间上要看多远
          //也就是在路径上走过一个特定时刻(heading_scoring_timestep_)后，才为朝向打分一次
          if (time >= heading_scoring_timestep_ && time < heading_scoring_timestep_ + dt) {
            //headingDiff函数的具体过程是：
            //  从全局路径终点(目标点)开始迭代，当前点与全局路径上的各点依次连线获得cost
            //  cost为正(无障碍)则计算:当前点与迭代到的点间的连线方向与当前点的姿态之差，返回并停止继续连线；
            //  若所有连线cost都为负，返回极大值
            heading_diff = headingDiff(cell_x, cell_y, x_i, y_i, theta_i);
          } else {
            update_path_and_goal_distances = false;
          }
        }

        //如果需要为朝向打分，则同样也等到为朝向打分的特定时刻才更新path_dist和goal_dist
        if (update_path_and_goal_distances) {
          //更新路径距离与目标距离(只考虑当前轨迹的终点cell)
          path_dist = path_map_(cell_x, cell_y).target_dist;
          goal_dist = goal_map_(cell_x, cell_y).target_dist;

          //如果目标距离或路径距离 ≥impossible_cost(地图尺寸)，代价设置为-2.0
          if(impossible_cost <= goal_dist || impossible_cost <= path_dist){
            traj.cost_ = -2.0;
            return;
          }
        }
      }

      //若该点足迹不遇障，且该点的goal_dist与path_dist存在，加入轨迹
      traj.addPoint(x_i, y_i, theta_i);

      //计算该点的速度
      //速度计算函数使当前速度在dt时间内以加速度acc_x向采样速度靠近，到达采样速度后将不再改变
      //所以实际上每条轨迹都是一个由当前速度趋向并稳定在采样速度的过程
      //所以说采样速度在这个函数中，是该函数的仿真过程的目标速度
      //而这里调用computeNewVelocity函数的返回值是仿真过程的每个step的速度(因为无法在一个step就达到采样速度)
      //需要注意的是，这里对速度的计算与我们发布给机器人的速度无关，这里的速度只是为了推算下一个点，获得路径
      //而我们真正发布给机器人的速度是采样速度。真实世界里机器人由当前速度–>采样速度的过程对应我们地图上本次仿真的轨迹
      vx_i = computeNewVelocity(vx_samp, vx_i, acc_x, dt);
      vy_i = computeNewVelocity(vy_samp, vy_i, acc_y, dt);
      vtheta_i = computeNewVelocity(vtheta_samp, vtheta_i, acc_theta, dt);

      //通过计算出的速度计算下一个位置、姿态(使用匀速运动公式)
      x_i = computeNewXPosition(x_i, vx_i, vy_i, theta_i, dt);
      y_i = computeNewYPosition(y_i, vx_i, vy_i, theta_i, dt);
      theta_i = computeNewThetaPosition(theta_i, vtheta_i, dt);

      //增加时间
      time += dt;
    }

    //ROS_INFO("OccCost: %f, vx: %.2f, vy: %.2f, vtheta: %.2f", occ_cost, vx_samp, vy_samp, vtheta_samp);
    double cost = -1.0;
    //如果打分不考虑航向
    if (!heading_scoring_) {
      cost = path_distance_bias_ * path_dist + goal_distance_bias_ * goal_dist + occdist_scale_ * occ_cost;
    } else {
      cost = path_distance_bias_ * path_dist + goal_distance_bias_ * goal_dist + occdist_scale_ * occ_cost 
           + 0.3 * heading_diff;
    }
    traj.cost_ = cost;
  }

  //在从机器人当前cell到全局路径上所有点的连线中，找到一条最远的直线(不被障碍阻挡->cost为正)(最靠近全局目标点)
  double TrajectoryPlanner::headingDiff(int cell_x, int cell_y, double x, double y, double heading){
    unsigned int goal_cell_x, goal_cell_y;
    for (int i = global_plan_.size() - 1; i >=0; --i) {
      if (costmap_.worldToMap(global_plan_[i].pose.position.x, global_plan_[i].pose.position.y, 
                              goal_cell_x, goal_cell_y)) {
        //计算当前点与全局路径上的各点连线的cost
        if (lineCost(cell_x, goal_cell_x, cell_y, goal_cell_y) >= 0) {
          double gx, gy;
          costmap_.mapToWorld(goal_cell_x, goal_cell_y, gx, gy);
          return fabs(angles::shortest_angular_distance(heading, atan2(gy - y, gx - x)));
        }
      }
    }
    return DBL_MAX;
  }

  //计算两点连线的cost
  double TrajectoryPlanner::lineCost(int x0, int x1, int y0, int y1){
    //Bresenham Ray-Tracing
    int deltax = abs(x1 - x0);        // The difference between the x's
    int deltay = abs(y1 - y0);        // The difference between the y's
    int x = x0;                       // Start x off at the first pixel
    int y = y0;                       // Start y off at the first pixel

    int xinc1, xinc2, yinc1, yinc2;
    int den, num, numadd, numpixels;

    double line_cost = 0.0;
    double point_cost = -1.0;

    // The x-values are increasing
    if (x1 >= x0){
      xinc1 = 1;
      xinc2 = 1;
    }
    // The x-values are decreasing
    else{
      xinc1 = -1;
      xinc2 = -1;
    }

    // The y-values are increasing
    if (y1 >= y0){
      yinc1 = 1;
      yinc2 = 1;
    }
    // The y-values are decreasing
    else{
      yinc1 = -1;
      yinc2 = -1;
    }

    // There is at least one x-value for every y-value
    if (deltax >= deltay){
      xinc1 = 0;                  // Don't change the x when numerator >= denominator
      yinc2 = 0;                  // Don't change the y for every iteration
      den = deltax;
      num = deltax / 2;
      numadd = deltay;
      numpixels = deltax;         // There are more x-values than y-values
    } else {                      // There is at least one y-value for every x-value
      xinc2 = 0;                  // Don't change the x for every iteration
      yinc1 = 0;                  // Don't change the y when numerator >= denominator
      den = deltay;
      num = deltay / 2;
      numadd = deltax;
      numpixels = deltay;         // There are more y-values than x-values
    }

    for (int curpixel = 0; curpixel <= numpixels; curpixel++) {
      point_cost = pointCost(x, y); //Score the current point

      if (point_cost < 0) {
        return -1;
      }

      if (line_cost < point_cost) {
        line_cost = point_cost;
      }

      num += numadd;              // Increase the numerator by the top of the fraction
      if (num >= den) {           // Check if numerator >= denominator
        num -= den;               // Calculate the new numerator value
        x += xinc1;               // Change the x as appropriate
        y += yinc1;               // Change the y as appropriate
      }
      x += xinc2;                 // Change the x as appropriate
      y += yinc2;                 // Change the y as appropriate
    }

    return line_cost;
  }

  double TrajectoryPlanner::pointCost(int x, int y){
    unsigned char cost = costmap_.getCost(x, y);
    //if the cell is in an obstacle the path is invalid
    if(cost == LETHAL_OBSTACLE || cost == INSCRIBED_INFLATED_OBSTACLE || cost == NO_INFORMATION){
      return -1;
    }
    return cost;
  }

  //Movebase调用全局规划器生成全局路径后，传入TrajectoryPlannerROS封装类
  //再通过这个函数传入真正的局部规划器TrajectoryPlanner类中，并且将全局路径的最终点作为目标点final_goal
  void TrajectoryPlanner::updatePlan(const vector<geometry_msgs::PoseStamped>& new_plan, bool compute_dists){
    global_plan_.resize(new_plan.size());
    for(unsigned int i = 0; i < new_plan.size(); ++i){
      global_plan_[i] = new_plan[i];
    }

    //判断全局路径是否有效
    if(global_plan_.size() > 0){
      geometry_msgs::PoseStamped& final_goal_pose = global_plan_[ global_plan_.size() - 1 ];
      final_goal_x_ = final_goal_pose.pose.position.x;
      final_goal_y_ = final_goal_pose.pose.position.y;
      final_goal_position_valid_ = true;
    } else {
      final_goal_position_valid_ = false;
    }

    //compute_dists默认为false，即本地规划器在更新全局plan时，不重新计算path_map_和goal_map_
    if (compute_dists) {
      //reset the map for new operations
      path_map_.resetPathDist();
      goal_map_.resetPathDist();

      //make sure that we update our path based on the global plan and compute costs
      path_map_.setTargetCells(costmap_, global_plan_);
      goal_map_.setLocalGoal(costmap_, global_plan_);
      ROS_DEBUG("Path/Goal distance computed");
    }
  }

  //checkTrajectory调用scoreTrajectory
  bool TrajectoryPlanner::checkTrajectory(double x, double y, double theta, double vx, double vy,
                                          double vtheta, double vx_samp, double vy_samp, double vtheta_samp){
    Trajectory t;
    double cost = scoreTrajectory(x, y, theta, vx, vy, vtheta, vx_samp, vy_samp, vtheta_samp);

    //如果路径的代价有效 则通过检验
    if(cost >= 0) {
      return true;
    }
    ROS_WARN("Invalid Trajectory %f, %f, %f, cost: %f", vx_samp, vy_samp, vtheta_samp, cost);

    //否则检验失败
    return false;
  }

  //scoreTrajectory调用generateTrajectory,生成单条路径并返回代价
  double TrajectoryPlanner::scoreTrajectory(double x, double y, double theta, double vx, double vy,
                                            double vtheta, double vx_samp, double vy_samp, double vtheta_samp) {
    Trajectory t;
    double impossible_cost = path_map_.obstacleCosts();
    generateTrajectory(x, y, theta,
                       vx, vy, vtheta,
                       vx_samp, vy_samp, vtheta_samp,
                       acc_lim_x_, acc_lim_y_, acc_lim_theta_,
                       impossible_cost, t);

    return double( t.cost_ );
  }

  //传入当前位姿、速度、加速度限制,生成合理速度范围内的轨迹,并进行打分,找到代价最低的轨迹返回
  Trajectory TrajectoryPlanner::createTrajectories(double x, double y, double theta,
                                                   double vx, double vy, double vtheta,
                                                   double acc_x, double acc_y, double acc_theta) {
    //计算可行的线速度和角速度范围

    double max_vel_x = max_vel_x_, max_vel_theta;
    double min_vel_x, min_vel_theta;

    //如果最终的目标是有效的(全局规划非空)
    if(final_goal_position_valid_){
      //计算当前位置和目标位置之间的距离：final_goal_dist
      double final_goal_dist = hypot(final_goal_x_ - x, final_goal_y_ - y);
      //最大速度:考虑预设的最大速度和"起点与目标直线距离/总仿真时间"
      max_vel_x = min(max_vel_x, final_goal_dist / sim_time_);
    }

    //继续计算线速度与角速度的上下限，使用的限制是:在一段时间内，由最大加减速度所能达到的速度范围

    //dwa: dynamic window approach
    //如果使用dwa法，则用的是轨迹前向模拟的周期sim_period_ (专用于dwa法计算速度的一个时间间隔)
    if (dwa_) {
      max_vel_x = max(min(max_vel_x, vx + acc_x * sim_period_), min_vel_x_);
      min_vel_x = max(min_vel_x_, vx - acc_x * sim_period_);

      max_vel_theta = min(max_vel_th_, vtheta + acc_theta * sim_period_);
      min_vel_theta = max(min_vel_th_, vtheta - acc_theta * sim_period_);
    //如果不使用dwa法，则用的是整段仿真时间sim_time_ 
    } else {
      max_vel_x = max(min(max_vel_x, vx + acc_x * sim_time_), min_vel_x_);
      min_vel_x = max(min_vel_x_, vx - acc_x * sim_time_);

      max_vel_theta = min(max_vel_th_, vtheta + acc_theta * sim_time_);
      min_vel_theta = max(min_vel_th_, vtheta - acc_theta * sim_time_);
    }

    //根据预设的线速度与角速度的采样数，和上面计算得到的范围，分别计算出速度的采样间隔(也就是速度的分辨率)
    double dvx = (max_vel_x - min_vel_x) / (vx_samples_ - 1);
    double dvtheta = (max_vel_theta - min_vel_theta) / (vtheta_samples_ - 1);

    //把范围内最小的线速度和角速度作为初始采样速度
    double vx_samp = min_vel_x;
    double vtheta_samp = min_vel_theta;
    //y向速度不进行采样遍历
    double vy_samp = 0.0;

    //为了迭代比较不同采样速度生成的不同路径的代价，声明best_traj和comp_traj并都将其代价初始化为-1
    Trajectory* best_traj = &traj_one;
    best_traj->cost_ = -1.0;
    Trajectory* comp_traj = &traj_two;
    comp_traj->cost_ = -1.0;

    //用于交换的指针
    Trajectory* swap = NULL;

    //任何cell的代价值都不可能大于地图的尺寸
    double impossible_cost = path_map_.obstacleCosts();

    //机器人没有处于逃逸状态,遍历所有线速度和角速度,调用generateTrajectory函数用可能的线速度和角速度生成轨迹
    if (!escaping_) {
      //循环所有x速度
      for(int i = 0; i < vx_samples_; ++i) {
        //在遍历时，单独拎出角速度=0，即直线前进的情况，避免由于采样间隔的设置而跃过了这种特殊情况
        vtheta_samp = 0;
        //调用generateTrajectory函数生成轨迹,并将轨迹保存在comp_traj中
        generateTrajectory(x, y, theta, vx, vy, vtheta, vx_samp, vy_samp, vtheta_samp,
                           acc_x, acc_y, acc_theta, impossible_cost, *comp_traj);

        //如果新生成的轨迹的代价更小,则将其放到best_traj
        if(comp_traj->cost_ >= 0 && (comp_traj->cost_ < best_traj->cost_ || best_traj->cost_ < 0)){
          swap = best_traj;
          best_traj = comp_traj;
          comp_traj = swap;
        }

        //角速度=最小值
        vtheta_samp = min_vel_theta;
        //迭代循环生成所有角速度的路径,并打分
        for(int j = 0; j < vtheta_samples_ - 1; ++j){
          generateTrajectory(x, y, theta, vx, vy, vtheta, vx_samp, vy_samp, vtheta_samp,
                             acc_x, acc_y, acc_theta, impossible_cost, *comp_traj);

          if(comp_traj->cost_ >= 0 && (comp_traj->cost_ < best_traj->cost_ || best_traj->cost_ < 0)){
            swap = best_traj;
            best_traj = comp_traj;
            comp_traj = swap;
          }
          vtheta_samp += dvtheta;
        }
        vx_samp += dvx;
      }

      //只对holonomic robots迭代循环y速度，一般的机器人没有y速度      
      if (holonomic_robot_) {
        //这也只是考虑了低速斜向45度移动的情况
        vx_samp = 0.1;
        vy_samp = 0.1;
        vtheta_samp = 0.0;
        generateTrajectory(x, y, theta, vx, vy, vtheta, vx_samp, vy_samp, vtheta_samp,
                           acc_x, acc_y, acc_theta, impossible_cost, *comp_traj);

        if(comp_traj->cost_ >= 0 && (comp_traj->cost_ < best_traj->cost_ || best_traj->cost_ < 0)){
          swap = best_traj;
          best_traj = comp_traj;
          comp_traj = swap;
        }

        vx_samp = 0.1;
        vy_samp = -0.1;
        vtheta_samp = 0.0;
        generateTrajectory(x, y, theta, vx, vy, vtheta, vx_samp, vy_samp, vtheta_samp,
                           acc_x, acc_y, acc_theta, impossible_cost, *comp_traj);

        if(comp_traj->cost_ >= 0 && (comp_traj->cost_ < best_traj->cost_ || best_traj->cost_ < 0)){
          swap = best_traj;
          best_traj = comp_traj;
          comp_traj = swap;
        }
      }
    }

    //继续考虑线速度=0（原地旋转）的情况

    //讨论一下这种情况什么时候会发生:
    //1、当TrajectoryPlannerROS中,位置已经到达目标,姿态已达,则直接发送0速;姿态未达,则调用降速函数和原地旋转函数,并调用checkTrajectory函数检查合法性,直到旋转至目标姿态。而checkTrajectory函数调用的是scoreTrajectory和generateTrajectory,不会调用createTrajectory函数，所以，快要到达目标附近时的原地旋转，不会进入到这个函数的这部分来处理。
    //2、并且，由于局部规划器的路径打分依据(后述)是:“与目标点的距离”和“与全局路径的偏离”,这两项打分都只考虑路径终点的cell(这里是指生成的局部轨迹的终点)，而不是考虑路径上所有cell的综合效果，机器人运动到一个cell上，哪怕有任何一条能向目标再前进的无障碍路径，它的最终得分一定是要比原地旋转的路径得分来得高的。
    //因此: 这里的原地自转，是行进过程中的、未达目标附近时的原地自转，并且，是机器人行进过程中遇到障碍、前方无路可走只好原地自转，或是连原地自转都不能满足，要由逃逸状态后退一段距离，再原地自转调整方向，准备接下来的行动。一种可能情况是机器人行进前方遇到了突然出现而不在地图上的障碍。
    //机器人原地旋转时的角速度限制范围要比运动时的角速度限制范围更严格，底盘无法处理过小的原地转速，故要注意处理这层限制。

    vtheta_samp = min_vel_theta;
    vx_samp = 0.0;
    vy_samp = 0.0;

    //let's try to rotate toward open space
    double heading_dist = DBL_MAX;

    //循环所有角速度
    for(int i = 0; i < vtheta_samples_; ++i) {
      //强制最小原地旋转速度,因为底盘无法处理过小的原地转速(相对前进过程中转向来说)
      double vtheta_samp_limited = vtheta_samp > 0 ? max(vtheta_samp, min_in_place_vel_th_)
                                                   : min(vtheta_samp, -1.0 * min_in_place_vel_th_);
      //产生遍历角速度的路径
      generateTrajectory(x, y, theta, vx, vy, vtheta, vx_samp, vy_samp, vtheta_samp_limited,
                         acc_x, acc_y, acc_theta, impossible_cost, *comp_traj);

      //如果新生成的轨迹的代价更小,则将其放到best_traj
      //注意如果能找到合法的原地旋转,相比之下,我们就不希望选择以y向速度进行移动,而是选择进行原地旋转
      if(comp_traj->cost_ >= 0
          && (comp_traj->cost_ <= best_traj->cost_ || best_traj->cost_ < 0 || best_traj->yv_ != 0.0)
          && (vtheta_samp > dvtheta || vtheta_samp < -1 * dvtheta)){
        double x_r, y_r, th_r;
        //获取新路径的终点(原地)(因为是原地旋转)
        comp_traj->getEndpoint(x_r, y_r, th_r);
        //计算沿旋转后的朝向前进heading_lookahead_距离后的位置
        x_r += heading_lookahead_ * cos(th_r);
        y_r += heading_lookahead_ * sin(th_r);

        unsigned int cell_x, cell_y;
        //转换到地图坐标系
        if (costmap_.worldToMap(x_r, y_r, cell_x, cell_y)) {
          //计算到目标点的距离
          double ahead_gdist = goal_map_(cell_x, cell_y).target_dist;
          //取距离最小的放进best_traj(heading_dist初始为无穷大)
          if (ahead_gdist < heading_dist) {
            //结合后面的抑制震荡部分代码(防止机器人在一个小范围内左右来回乱转)
            //只有stuck_left为假(机器人上一时刻没有向左旋转)的时候,才允许使用向右的角速度更新best_traj
            if (vtheta_samp < 0 && !stuck_left) {
              swap = best_traj;
              best_traj = comp_traj;
              comp_traj = swap;
              heading_dist = ahead_gdist;
            }
            else if(vtheta_samp > 0 && !stuck_right) {
              swap = best_traj;
              best_traj = comp_traj;
              comp_traj = swap;
              heading_dist = ahead_gdist;
            }
          }
        }
      }
      vtheta_samp += dvtheta;
    }

    //抑制震荡影响: 当机器人在某方向移动时，对下一个周期的与其相反方向标记为无效
    //直到机器人从标记震荡的位置处离开一定距离，返回最佳轨迹

    //如果最优轨迹的代价大于0(有效)
    if (best_traj->cost_ >= 0) {
      //当找到的最优轨迹的线速度为负时(逃逸模式)
      //正常情况线速度的遍历范围都是正的 因此如果发现最优轨迹的线速度为<=0 说明上一次的轨迹无效 发布了后退的指令(逃逸模式)
      if (!(best_traj->xv_ > 0)) {
        //角速度为负 标记正在向右旋转(rotating_right)
        //这里要想印证确实是这样理解，需要检查角速度为负是否真的对应机器人向右旋转
        if (best_traj->thetav_ < 0) {
          //再一次发现向右旋转 标记stuck_right
          if (rotating_right){
            stuck_right = true;
          }
          rotating_right = true;
        } 
        //角速度为正 标记正在向左旋转(rotating_left)
        else if (best_traj->thetav_ > 0) {
          //再一次发现向左旋转 标记stuck_left(禁止下一时刻直接向右旋转,导致左右乱转出现振荡)
          if (rotating_left){
            stuck_left = true;
          }
          rotating_left = true;
        }
        //y向速度为正 标记正在向右平移
        else if(best_traj->yv_ > 0) {
          if (strafe_right) {
            stuck_right_strafe = true;
          }
          strafe_right = true;
        }
        //y向速度为负 标记正在向左平移 
        else if(best_traj->yv_ < 0){
          if (strafe_left) {
            stuck_left_strafe = true;
          }
          strafe_left = true;
        }

        //记录当前的位置
        prev_x_ = x;
        prev_y_ = y;
      }

      //必须远离上面记录的位置一段距离后才恢复标志位
      double dist = hypot(x - prev_x_, y - prev_y_);
      if (dist > oscillation_reset_dist_) {
        rotating_left = false;
        rotating_right = false;
        strafe_left = false;
        strafe_right = false;
        stuck_left = false;
        stuck_right = false;
        stuck_left_strafe = false;
        stuck_right_strafe = false;
      }

      //判断是否退出逃逸状态
      dist = hypot(x - escape_x_, y - escape_y_);
      if(dist > escape_reset_dist_ ||
          fabs(angles::shortest_angular_distance(escape_theta_, theta)) > escape_reset_theta_){
        escaping_ = false;
      }
      //注意这里直接返回了 不再对后面产生影响
      return *best_traj;
    }

    //holonomic robots可以有y向速度
    if (holonomic_robot_) {
      //if we can't rotate in place or move forward... maybe we can move sideways and rotate
      vtheta_samp = min_vel_theta;
      vx_samp = 0.0;

      //循环所有的y向速度
      for(unsigned int i = 0; i < y_vels_.size(); ++i){
        vtheta_samp = 0;
        vy_samp = y_vels_[i];
        //产生遍历y方向的路径
        generateTrajectory(x, y, theta, vx, vy, vtheta, vx_samp, vy_samp, vtheta_samp,
                           acc_x, acc_y, acc_theta, impossible_cost, *comp_traj);

        if(comp_traj->cost_ >= 0 && (comp_traj->cost_ <= best_traj->cost_ || best_traj->cost_ < 0)){
          double x_r, y_r, th_r;
          comp_traj->getEndpoint(x_r, y_r, th_r);
          x_r += heading_lookahead_ * cos(th_r);
          y_r += heading_lookahead_ * sin(th_r);
          unsigned int cell_x, cell_y;

          //转换到地图坐标系
          if(costmap_.worldToMap(x_r, y_r, cell_x, cell_y)) {
            //计算到目标点的距离
            double ahead_gdist = goal_map_(cell_x, cell_y).target_dist;
            //取距离最小的放进best_traj(heading_dist初始为无穷大)
            if (ahead_gdist < heading_dist) {
              //只有stuck_left_strafe为假(机器人上一时刻没有向左平移)的时候,才允许使用向右的y向速度更新best_traj
              if (vy_samp > 0 && !stuck_left_strafe) {
                swap = best_traj;
                best_traj = comp_traj;
                comp_traj = swap;
                heading_dist = ahead_gdist;
              }
              //只有stuck_right_strafe为假(机器人上一时刻没有向右平移)的时候,才允许使用向左的y向速度更新best_traj
              else if(vy_samp < 0 && !stuck_right_strafe) {
                swap = best_traj;
                best_traj = comp_traj;
                comp_traj = swap;
                heading_dist = ahead_gdist;
              }
            }
          }
        }
      }
    }

    //这一段代码不懂什么意思(好像和前面类似的代码段有些赋值是相反的)
    if (best_traj->cost_ >= 0) {
      if (!(best_traj->xv_ > 0)) {
        if (best_traj->thetav_ < 0) {
          if (rotating_right){
            stuck_right = true;
          }
          rotating_left = true;
        } 
        else if(best_traj->thetav_ > 0) {
          if(rotating_left){
            stuck_left = true;
          }
          rotating_right = true;
        } 
        else if(best_traj->yv_ > 0) {
          if(strafe_right){
            stuck_right_strafe = true;
          }
          strafe_left = true;
        } 
        else if(best_traj->yv_ < 0) {
          if(strafe_left){
            stuck_left_strafe = true;
          }
          strafe_right = true;
        }

        //记录当前的位置
        prev_x_ = x;
        prev_y_ = y;
      }

      //必须远离上面记录的位置一段距离后才恢复标志位
      double dist = hypot(x - prev_x_, y - prev_y_);
      if(dist > oscillation_reset_dist_) {
        rotating_left = false;
        rotating_right = false;
        strafe_left = false;
        strafe_right = false;
        stuck_left = false;
        stuck_right = false;
        stuck_left_strafe = false;
        stuck_right_strafe = false;
      }

      dist = hypot(x - escape_x_, y - escape_y_);
      if(dist > escape_reset_dist_ || fabs(angles::shortest_angular_distance(escape_theta_, theta)) > escape_reset_theta_) {
        escaping_ = false;
      }

      return *best_traj;
    }

    //当轨迹cost为负即无效时，执行接下来的部分，设置一个负向速度，产生让机器人缓慢退后的轨迹
    vtheta_samp = 0.0;
    vx_samp = backup_vel_;
    vy_samp = 0.0;
    generateTrajectory(x, y, theta, vx, vy, vtheta, vx_samp, vy_samp, vtheta_samp,
                       acc_x, acc_y, acc_theta, impossible_cost, *comp_traj);

    //if the new trajectory is better... let's take it
    /*
       if(comp_traj->cost_ >= 0 && (comp_traj->cost_ < best_traj->cost_ || best_traj->cost_ < 0)){
       swap = best_traj;
       best_traj = comp_traj;
       comp_traj = swap;
       }
       */

    //即使静态地图显示后面为阻塞,仍允许机器人缓慢向后移动
    //也就是不需要判断后退时的轨迹是否是best,只要给了向后的速度,那么默认就是best(因为都是不得已才给的)
    swap = best_traj;
    best_traj = comp_traj;
    comp_traj = swap;

    double dist = hypot(x - prev_x_, y - prev_y_);
    if (dist > oscillation_reset_dist_) {
      rotating_left = false;
      rotating_right = false;
      strafe_left = false;
      strafe_right = false;
      stuck_left = false;
      stuck_right = false;
      stuck_left_strafe = false;
      stuck_right_strafe = false;
    }

    //若后退速度生成的轨迹的终点有效(>-2.0)(为什么这个条件就是有效)，进入逃逸状态(后退一段距离)
    //逃逸状态起始就是不再前进，不进入if(!escaping_)的分支
    if (!escaping_ && best_traj->cost_ > -2.0) {
      escape_x_ = x;
      escape_y_ = y;
      escape_theta_ = theta;
      escaping_ = true;
    }

    //判断是否退出逃逸状态
    dist = hypot(x - escape_x_, y - escape_y_);
    if (dist > escape_reset_dist_ ||
        fabs(angles::shortest_angular_distance(escape_theta_, theta)) > escape_reset_theta_) {
      escaping_ = false;
    }

    //若后退轨迹遇障，还是继续后退，因为后退一点后立刻就会进入原地自转模式
    if(best_traj->cost_ == -1.0)
      best_traj->cost_ = 1.0;

    return *best_traj;
  }

  //局部规划的整个流程体现在findBestPath函数中
  //该函数能够在范围内生成下一步的可能路线，选择出最优路径，并返回该路径对应的下一步的速度
  Trajectory TrajectoryPlanner::findBestPath(const geometry_msgs::PoseStamped& global_pose,
                                             geometry_msgs::PoseStamped& global_vel, 
                                             geometry_msgs::PoseStamped& drive_velocities){

    //将当前机器人位置和方向转变成float形式的vector
    Eigen::Vector3f pos(global_pose.pose.position.x, global_pose.pose.position.y, 
                        tf2::getYaw(global_pose.pose.orientation));
    Eigen::Vector3f vel(global_vel.pose.position.x, global_vel.pose.position.y, 
                        tf2::getYaw(global_vel.pose.orientation));

    //重置地图 清除所有障碍物信息以及地图内容
    path_map_.resetPathDist();
    goal_map_.resetPathDist();

    //利用机器人当前位姿，获得机器人footpoint(足迹/覆盖位置)
    std::vector<base_local_planner::Position2DInt> footprint_list = footprint_helper_.getFootprintCells(pos, footprint_spec_, costmap_, true);

    //标记机器人初始footprint内的所有cell为within_robot
    for (unsigned int i = 0; i < footprint_list.size(); ++i) {
      path_map_(footprint_list[i].x, footprint_list[i].y).within_robot = true;
    }

    //更新路径地图和目标地图
    path_map_.setTargetCells(costmap_, global_plan_);
    goal_map_.setLocalGoal(costmap_, global_plan_);
    ROS_DEBUG("Path/Goal distance computed");

    //调用createTrajectories函数,传入当前位姿、速度、加速度限制
    //生成合理速度范围内的轨迹,并进行打分,找到代价最低的轨迹返回至best
    Trajectory best = createTrajectories(pos[0], pos[1], pos[2], vel[0], vel[1], vel[2],
                                         acc_lim_x_, acc_lim_y_, acc_lim_theta_);
    ROS_DEBUG("Trajectories created");

    /*
    //If we want to print a ppm file to draw goal dist
    char buf[4096];
    sprintf(buf, "base_local_planner.ppm");
    FILE *fp = fopen(buf, "w");
    if(fp){
      fprintf(fp, "P3\n");
      fprintf(fp, "%d %d\n", map_.size_x_, map_.size_y_);
      fprintf(fp, "255\n");
      for(int j = map_.size_y_ - 1; j >= 0; --j){
        for(unsigned int i = 0; i < map_.size_x_; ++i){
          int g_dist = 255 - int(map_(i, j).goal_dist);
          int p_dist = 255 - int(map_(i, j).path_dist);
          if(g_dist < 0)
            g_dist = 0;
          if(p_dist < 0)
            p_dist = 0;
          fprintf(fp, "%d 0 %d ", g_dist, 0);
        }
        fprintf(fp, "\n");
      }
      fclose(fp);
    }
    */

    //如果找到的best轨迹的代价为负，表示说明所有的路径都不可用
    if(best.cost_ < 0){
      drive_velocities.pose.position.x = 0;
      drive_velocities.pose.position.y = 0;
      drive_velocities.pose.position.z = 0;
      drive_velocities.pose.orientation.w = 1;
      drive_velocities.pose.orientation.x = 0;
      drive_velocities.pose.orientation.y = 0;
      drive_velocities.pose.orientation.z = 0;
    }
    //若代价非负，表示找到有效路径，为drive_velocities填充速度后返回
    else{
      drive_velocities.pose.position.x = best.xv_;
      drive_velocities.pose.position.y = best.yv_;
      drive_velocities.pose.position.z = 0;
      tf2::Quaternion q;
      q.setRPY(0, 0, best.thetav_);
      tf2::convert(q, drive_velocities.pose.orientation);
    }
    //返回最优轨迹
    return best;
  }

  //we need to take the footprint of the robot into account when we calculate cost to obstacles
  double TrajectoryPlanner::footprintCost(double x_i, double y_i, double theta_i){
    //check if the footprint is legal
    return world_model_.footprintCost(x_i, y_i, theta_i, footprint_spec_, inscribed_radius_, circumscribed_radius_);
  }

  void TrajectoryPlanner::getLocalGoal(double& x, double& y){
    x = path_map_.goal_x_;
    y = path_map_.goal_y_;
  }
};


