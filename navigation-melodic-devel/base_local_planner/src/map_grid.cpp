/*********************************************************************
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
 *********************************************************************/
#include <base_local_planner/map_grid.h>
#include <costmap_2d/cost_values.h>
using namespace std;

namespace base_local_planner{

  //MapGrid和MapCell类用于局部规划的计算

  //MapGrid是“地图”，地图大小为size_x_×size_y_
  //它包含一个由MapCell类对象数组的成员，即cell数组
  //goal_map和path_map都是MapGrid类实例,分别称这两张地图为 “目标地图” 和 “路径地图”

  MapGrid::MapGrid()
    : size_x_(0), size_y_(0){}

  MapGrid::MapGrid(unsigned int size_x, unsigned int size_y) 
    : size_x_(size_x), size_y_(size_y){
    commonInit();
  }

  MapGrid::MapGrid(const MapGrid& mg){
    size_y_ = mg.size_y_;
    size_x_ = mg.size_x_;
    map_ = mg.map_;
  }

  void MapGrid::commonInit(){
    //don't allow construction of zero size grid
    ROS_ASSERT(size_y_ != 0 && size_x_ != 0);

    map_.resize(size_y_ * size_x_);

    //make each cell aware of its location in the grid
    for(unsigned int i = 0; i < size_y_; ++i){
      for(unsigned int j = 0; j < size_x_; ++j){
        unsigned int id = size_x_ * i + j;
        map_[id].cx = j;
        map_[id].cy = i;
      }
    }
  }

  size_t MapGrid::getIndex(int x, int y){
    return size_x_ * y + x;
  }

  MapGrid& MapGrid::operator= (const MapGrid& mg){
    size_y_ = mg.size_y_;
    size_x_ = mg.size_x_;
    map_ = mg.map_;
    return *this;
  }

  //检查路径地图尺寸是否和costmap相同，若不同，则按照costmap尺寸对其重新设置
  //并且检查MapCell在MapGrid中的index是否和它本身的坐标索引一致
  void MapGrid::sizeCheck(unsigned int size_x, unsigned int size_y){
    if(map_.size() != size_x * size_y)
      map_.resize(size_x * size_y);

    if(size_x_ != size_x || size_y_ != size_y){
      size_x_ = size_x;
      size_y_ = size_y;

      for(unsigned int i = 0; i < size_y_; ++i){
        for(unsigned int j = 0; j < size_x_; ++j){
          int index = size_x_ * i + j;
          map_[index].cx = j;
          map_[index].cy = i;
        }
      }
    }
  }

  //以current_cell为父节点更新check_cell的值
  inline bool MapGrid::updatePathCell(MapCell* current_cell, MapCell* check_cell,
                                      const costmap_2d::Costmap2D& costmap){

    //若发现它是障碍物或未知cell或它在机器人足迹内，直接返回false，设置该cell的dist值为最大
    //该cell将不会进入循环队列，也就是不会由它继续向下传播
    unsigned char cost = costmap.getCost(check_cell->cx, check_cell->cy);
    if(!getCell(check_cell->cx, check_cell->cy).within_robot && (cost == costmap_2d::LETHAL_OBSTACLE ||
                                                                 cost == costmap_2d::INSCRIBED_INFLATED_OBSTACLE ||
                                                                 cost == costmap_2d::NO_INFORMATION)){
      check_cell->target_dist = obstacleCosts();
      return false;
    }

    //待计算的cell的新的dist值 = 当前cell的dist值+1(相当于通过当前cell走到待计算的cell[类似Dijkstra算法的思路])
    double new_target_dist = current_cell->target_dist + 1;
    //如果计算出来的新的dist值 < 待计算的cell原本的dist值,则更新这个cell的dist值
    if (new_target_dist < check_cell->target_dist) {
      check_cell->target_dist = new_target_dist;
    }
    return true;
  }

  //reset the path_dist and goal_dist fields for all cells
  void MapGrid::resetPathDist(){
    for(unsigned int i = 0; i < map_.size(); ++i) {
      map_[i].target_dist = unreachableCellCosts();
      map_[i].target_mark = false;
      map_[i].within_robot = false;
    }
  }

  //对global_plan的分辨率进行调整，使其达到costmap的分辨率
  void MapGrid::adjustPlanResolution(const std::vector<geometry_msgs::PoseStamped>& global_plan_in,
                                     std::vector<geometry_msgs::PoseStamped>& global_plan_out, 
                                     double resolution) {
    if (global_plan_in.size() == 0) {
      return;
    }
    double last_x = global_plan_in[0].pose.position.x;
    double last_y = global_plan_in[0].pose.position.y;
    global_plan_out.push_back(global_plan_in[0]);

    double min_sq_resolution = resolution * resolution;

    for (unsigned int i = 1; i < global_plan_in.size(); ++i) {
      double loop_x = global_plan_in[i].pose.position.x;
      double loop_y = global_plan_in[i].pose.position.y;
      double sqdist = (loop_x - last_x) * (loop_x - last_x) + (loop_y - last_y) * (loop_y - last_y);
      if (sqdist > min_sq_resolution) {
        int steps = ceil((sqrt(sqdist)) / resolution);
        // add a points in-between
        double deltax = (loop_x - last_x) / steps;
        double deltay = (loop_y - last_y) / steps;
        // TODO: Interpolate orientation
        for (int j = 1; j < steps; ++j) {
          geometry_msgs::PoseStamped pose;
          pose.pose.position.x = last_x + j * deltax;
          pose.pose.position.y = last_y + j * deltay;
          pose.pose.position.z = global_plan_in[i].pose.position.z;
          pose.pose.orientation = global_plan_in[i].pose.orientation;
          pose.header = global_plan_in[i].header;
          global_plan_out.push_back(pose);
        }
      }
      global_plan_out.push_back(global_plan_in[i]);
      last_x = loop_x;
      last_y = loop_y;
    }
  }

  //处理路径地图:根据global_plan更新哪些cell被视为路径
  void MapGrid::setTargetCells(const costmap_2d::Costmap2D& costmap,
                               const std::vector<geometry_msgs::PoseStamped>& global_plan) {
    //检查路径地图尺寸以及索引是否正确
    sizeCheck(costmap.getSizeInCellsX(), costmap.getSizeInCellsY());

    bool started_path = false;

    //用于储存全局路径上的MapCell
    queue<MapCell*> path_dist_queue;

    std::vector<geometry_msgs::PoseStamped> adjusted_global_plan;
    //传入的全局规划是global系下的，调用adjustPlanResolution函数对其分辨率进行调整，使其达到costmap的分辨率
    adjustPlanResolution(global_plan, adjusted_global_plan, costmap.getResolution());
    if (adjusted_global_plan.size() != global_plan.size()) {
      ROS_DEBUG("Adjusted global plan resolution, added %zu points", adjusted_global_plan.size() - global_plan.size());
    }

    unsigned int i;
    //将全局路径的点转换到路径地图上
    for (i = 0; i < adjusted_global_plan.size(); ++i) {
      double g_x = adjusted_global_plan[i].pose.position.x;
      double g_y = adjusted_global_plan[i].pose.position.y;
      unsigned int map_x, map_y;
      //如果成功把一个全局规划上的点的坐标转换到地图坐标(map_x,map_y)上,且在代价地图上这一点不是未知的
      if (costmap.worldToMap(g_x, g_y, map_x, map_y) && costmap.getCost(map_x, map_y) != costmap_2d::NO_INFORMATION) {
        MapCell& current = getCell(map_x, map_y);
        //将这个点的target_dist(到path的距离)设置为0,即在全局路径上
        //setTargetCells和setLocalGoal这两个函数:(此函数是setTargetCells)
        //    前者在“路径地图”上将全局路径点target_dist标记为0
        //    后者在“目标地图”上将目标点target_dist标记为0
        current.target_dist = 0.0;
        //标记已经计算了距离
        current.target_mark = true;
        //把该点放进path_dist_queue队列中
        path_dist_queue.push(&current);
        //标记已经开始把点转换到地图坐标
        started_path = true;
      }
      //当代价地图上这一点的代价不存在了(规划路径已经到达了代价地图的边界) 
      //并且标记了已经开始转换，退出循环
      //(这个else if写的有一点迷惑性,注意首先需要不满足if条件,才会判断elseif的条件)
      else if (started_path) {
          break;
      }
    }
    //如果循环结束后,开始转换标志(started_path)还没有置位 则报错
    if (!started_path) {
      ROS_ERROR("None of the %d first of %zu (%zu) points of the global plan were in the local costmap and free", i, adjusted_global_plan.size(), global_plan.size());
      return;
    }
    //计算路径地图上的每一个cell与规划路径之间的距离
    computeTargetDistance(path_dist_queue, costmap);
  }

  //通过迭代找到全局路径的终点，即目标点
  //但如果迭代过程当中到达了局部规划costmap的边际或经过障碍物，立即退出迭代，将上一个有效点作为终点
  void MapGrid::setLocalGoal(const costmap_2d::Costmap2D& costmap,
                             const std::vector<geometry_msgs::PoseStamped>& global_plan) {
    //检查地图尺寸和索引
    sizeCheck(costmap.getSizeInCellsX(), costmap.getSizeInCellsY());

    int local_goal_x = -1;
    int local_goal_y = -1;
    bool started_path = false;

    std::vector<geometry_msgs::PoseStamped> adjusted_global_plan;
    //调整分辨率
    adjustPlanResolution(global_plan, adjusted_global_plan, costmap.getResolution());

    //逐个跳过全局路径中的点,一直到碰到了局部规划costmap的边界
    //则立即退出迭代，将上一个有效点作为终点
    //否则一直迭代到全局路径的终点(目标点)
    for (unsigned int i = 0; i < adjusted_global_plan.size(); ++i) {
      double g_x = adjusted_global_plan[i].pose.position.x;
      double g_y = adjusted_global_plan[i].pose.position.y;
      unsigned int map_x, map_y;
      if (costmap.worldToMap(g_x, g_y, map_x, map_y) && costmap.getCost(map_x, map_y) != costmap_2d::NO_INFORMATION) {
        local_goal_x = map_x;
        local_goal_y = map_y;
        started_path = true;
      } else {
        if (started_path) {
          break;
        }// else we might have a non pruned path, so we just continue
      }
    }
    if (!started_path) {
      ROS_ERROR("None of the points of the global plan were in the local costmap, global plan points too far from robot");
      return;
    }

    queue<MapCell*> path_dist_queue;
    if (local_goal_x >= 0 && local_goal_y >= 0) {
      MapCell& current = getCell(local_goal_x, local_goal_y);
      costmap.mapToWorld(local_goal_x, local_goal_y, goal_x_, goal_y_);
      //将迭代得到的目标点对应在“目标地图”上的cell的target_dist标记为0
      //setTargetCells和setLocalGoal这两个函数:(此函数是setLocalGoal)
      //    前者在“路径地图”上将全局路径点target_dist标记为0
      //    后者在“目标地图”上将目标点target_dist标记为0
      current.target_dist = 0.0;
      current.target_mark = true;
      path_dist_queue.push(&current);
    }
    //计算目标地图上的每一个cell与规划路径之间的距离
    computeTargetDistance(path_dist_queue, costmap);
  }

  //传入target_dist=0的cell队列，然后从它们开始向四周开始传播
  void MapGrid::computeTargetDistance(queue<MapCell*>& dist_queue, const costmap_2d::Costmap2D& costmap){
    MapCell* current_cell;
    MapCell* check_cell;
    unsigned int last_col = size_x_ - 1;
    unsigned int last_row = size_y_ - 1;

    //依次将队列中的cell作为"父cell",检查"父cell"四周的cell,对其调用updatePathCell函数，得到该cell的值
    //如果该cell的值有效，则加入循环队列里，弹出“父cell”，四周的cell成为新的“父cell”

    //C++中的queue(FIFO): 只能访问queue<T>容器适配器的第一个和最后一个元素,只能在末尾添加新元素,只能从头部移除元素

    while(!dist_queue.empty()){
      current_cell = dist_queue.front();
      //将current_cell弹出
      dist_queue.pop();

      if(current_cell->cx > 0){
        check_cell = current_cell - 1;
        if(!check_cell->target_mark){
          //标记该cell被访问过了
          check_cell->target_mark = true;
          if(updatePathCell(current_cell, check_cell, costmap)) {
            dist_queue.push(check_cell);
          }
        }
      }

      if(current_cell->cx < last_col){
        check_cell = current_cell + 1;
        if(!check_cell->target_mark){
          check_cell->target_mark = true;
          if(updatePathCell(current_cell, check_cell, costmap)) {
            dist_queue.push(check_cell);
          }
        }
      }

      if(current_cell->cy > 0){
        check_cell = current_cell - size_x_;
        if(!check_cell->target_mark){
          check_cell->target_mark = true;
          if(updatePathCell(current_cell, check_cell, costmap)) {
            dist_queue.push(check_cell);
          }
        }
      }

      if(current_cell->cy < last_row){
        check_cell = current_cell + size_x_;
        if(!check_cell->target_mark){
          check_cell->target_mark = true;
          if(updatePathCell(current_cell, check_cell, costmap)) {
            dist_queue.push(check_cell);
          }
        }
      }
      //直到地图上所有cell的dist值都被计算出来
    }
  }

};
