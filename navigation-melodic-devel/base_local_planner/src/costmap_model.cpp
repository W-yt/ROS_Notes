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
#include <base_local_planner/line_iterator.h>
#include <base_local_planner/costmap_model.h>
#include <costmap_2d/cost_values.h>

using namespace std;
using namespace costmap_2d;

namespace base_local_planner {

  //CostmapModel类派生自WorldModel类
  //在TrajectoryPlanner中被使用，承担局部规划器与局部规划Costmap之间的桥梁工作
  
  //CostmapModel类帮助局部规划器在Costmap上进行计算
  //footprintCost、lineCost、pointCost三个函数分别能通过Costmap计算出：
  //    机器人足迹范围的代价、两个cell连线的代价、单个cell的代价，并将值返回给局部规划器

  CostmapModel::CostmapModel(const Costmap2D& ma) : costmap_(ma) {}

  double CostmapModel::footprintCost(const geometry_msgs::Point& position, 
                                     const std::vector<geometry_msgs::Point>& footprint,
                                     double inscribed_radius, double circumscribed_radius){
    // returns:
    //  -1 if footprint covers at least a lethal obstacle cell, or
    //  -2 if footprint covers at least a no-information cell, or
    //  -3 if footprint is [partially] outside of the map, or
    //  a positive value for traversable space

    unsigned int cell_x, cell_y;

    //获取机器人中心点的cell坐标，存放在cell_x cell_y中
    //如果得不到坐标，说明不在地图上，直接返回-3
    if(!costmap_.worldToMap(position.x, position.y, cell_x, cell_y))
      return -3.0;

    //如果脚印点数小于三，默认机器人形状为圆形，不考虑脚印，只考虑中心
    if(footprint.size() < 3){
      unsigned char cost = costmap_.getCost(cell_x, cell_y);
      //如果中心位于未知代价的cell上，返回-2
      if(cost == NO_INFORMATION)
        return -2.0;
      //如果中心位于致命障碍cell上，返回-1 (这几个宏是在costvalue中定义的，是灰度值)
      if(cost == LETHAL_OBSTACLE || cost == INSCRIBED_INFLATED_OBSTACLE)
        return -1.0;
      //如果机器人位置既不是未知也不是致命，返回它的代价
      return cost;
    }

    //如果脚印点数小于三，需要考虑机器人的形状，把足迹视为多边形
    unsigned int x0, x1, y0, y1;
    double line_cost = 0.0;
    double footprint_cost = 0.0;

    //we need to rasterize each line in the footprint
    for(unsigned int i = 0; i < footprint.size() - 1; ++i){
      //获取第一个点的cell坐标
      if(!costmap_.worldToMap(footprint[i].x, footprint[i].y, x0, y0))
        return -3.0;
      //获取第二个点的cell坐标
      if(!costmap_.worldToMap(footprint[i + 1].x, footprint[i + 1].y, x1, y1))
        return -3.0;

      //得到两点连线的代价
      line_cost = lineCost(x0, x1, y0, y1);
      footprint_cost = std::max(line_cost, footprint_cost);

      //如果某条边缘线段代价<0(碰到了障碍)，直接停止生成代价，返回这个负代价
      if(line_cost < 0)
        return line_cost;
    }

    //再把footprint的最后一个点和第一个点连起来，形成封闭图形
    if(!costmap_.worldToMap(footprint.back().x, footprint.back().y, x0, y0))
      return -3.0;
    if(!costmap_.worldToMap(footprint.front().x, footprint.front().y, x1, y1))
      return -3.0;
    line_cost = lineCost(x0, x1, y0, y1);
    footprint_cost = std::max(line_cost, footprint_cost);
    if(line_cost < 0)
      return line_cost;

    //如果所有边缘线的代价都是合法的，那么返回足迹的代价
    return footprint_cost;
  }

  //计算两点连线的代价
  double CostmapModel::lineCost(int x0, int x1, int y0, int y1) const {
    double line_cost = 0.0;
    double point_cost = -1.0;

    for(LineIterator line( x0, y0, x1, y1 ); line.isValid(); line.advance()){
      //LineIterator类的advance函数就是取线上的下一个点,然后getX和getY函数获取点的坐标
      point_cost = pointCost(line.getX(), line.getY());

      if(point_cost < 0)
        return point_cost;

      //两点连线的代价就是线上点的最大代价?
      if(line_cost < point_cost)
        line_cost = point_cost;
    }

    return line_cost;
  }

  double CostmapModel::pointCost(int x, int y) const {
    unsigned char cost = costmap_.getCost(x, y);
    //if the cell is in an obstacle the path is invalid
    if(cost == NO_INFORMATION)
      return -2;
    if(cost == LETHAL_OBSTACLE)
      return -1;

    return cost;
  }

};
