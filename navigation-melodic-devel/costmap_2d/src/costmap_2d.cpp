/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
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
 *         David V. Lu!!
 *********************************************************************/
#include <costmap_2d/costmap_2d.h>
#include <cstdio>

using namespace std;

namespace costmap_2d{
  Costmap2D::Costmap2D(unsigned int cells_size_x, unsigned int cells_size_y, double resolution,
                       double origin_x, double origin_y, unsigned char default_value) : 
        size_x_(cells_size_x), size_y_(cells_size_y), resolution_(resolution), 
        origin_x_(origin_x), origin_y_(origin_y), costmap_(NULL), default_value_(default_value){
    access_ = new mutex_t();

    // create the costmap
    initMaps(size_x_, size_y_);
    resetMaps();
  }

  void Costmap2D::deleteMaps(){
    // clean up data
    boost::unique_lock<mutex_t> lock(*access_);
    delete[] costmap_;
    costmap_ = NULL;
  }

  void Costmap2D::initMaps(unsigned int size_x, unsigned int size_y){
    boost::unique_lock<mutex_t> lock(*access_);
    delete[] costmap_;
    costmap_ = new unsigned char[size_x * size_y];
  }

  void Costmap2D::resizeMap(unsigned int size_x, unsigned int size_y, double resolution,
                            double origin_x, double origin_y){
    size_x_ = size_x;
    size_y_ = size_y;
    resolution_ = resolution;
    origin_x_ = origin_x;
    origin_y_ = origin_y;

    initMaps(size_x, size_y);

    // reset our maps to have no information
    resetMaps();
  }

  void Costmap2D::resetMaps(){
    boost::unique_lock<mutex_t> lock(*access_);
    memset(costmap_, default_value_, size_x_ * size_y_ * sizeof(unsigned char));
  }

  void Costmap2D::resetMap(unsigned int x0, unsigned int y0, unsigned int xn, unsigned int yn){
    boost::unique_lock<mutex_t> lock(*(access_));
    unsigned int len = xn - x0;
    for (unsigned int y = y0 * size_x_ + x0; y < yn * size_x_ + x0; y += size_x_)
      memset(costmap_ + y, default_value_, len * sizeof(unsigned char));
  }

  bool Costmap2D::copyCostmapWindow(const Costmap2D& map, double win_origin_x, double win_origin_y, 
                                    double win_size_x, double win_size_y){
    // check for self windowing
    if (this == &map){
      // ROS_ERROR("Cannot convert this costmap into a window of itself");
      return false;
    }

    // clean up old data
    deleteMaps();

    // compute the bounds of our new map
    unsigned int lower_left_x, lower_left_y, upper_right_x, upper_right_y;
    if (!map.worldToMap(win_origin_x, win_origin_y, lower_left_x, lower_left_y) || 
        !map.worldToMap(win_origin_x + win_size_x, win_origin_y + win_size_y, upper_right_x, upper_right_y)){
      // ROS_ERROR("Cannot window a map that the window bounds don't fit inside of");
      return false;
    }

    size_x_ = upper_right_x - lower_left_x;
    size_y_ = upper_right_y - lower_left_y;
    resolution_ = map.resolution_;
    origin_x_ = win_origin_x;
    origin_y_ = win_origin_y;

    // initialize our various maps and reset markers for inflation
    initMaps(size_x_, size_y_);

    // copy the window of the static map and the costmap that we're taking
    copyMapRegion(map.costmap_, lower_left_x, lower_left_y, map.size_x_, costmap_, 0, 0, size_x_, size_x_, size_y_);
    return true;
  }

  Costmap2D& Costmap2D::operator=(const Costmap2D& map){
    // check for self assignement
    if (this == &map)
      return *this;

    // clean up old data
    deleteMaps();

    size_x_ = map.size_x_;
    size_y_ = map.size_y_;
    resolution_ = map.resolution_;
    origin_x_ = map.origin_x_;
    origin_y_ = map.origin_y_;

    // initialize our various maps
    initMaps(size_x_, size_y_);

    // copy the cost map
    memcpy(costmap_, map.costmap_, size_x_ * size_y_ * sizeof(unsigned char));

    return *this;
  }

  Costmap2D::Costmap2D(const Costmap2D& map) :
      costmap_(NULL){
    access_ = new mutex_t();
    *this = map;
  }

  // just initialize everything to NULL by default
  Costmap2D::Costmap2D() :
      size_x_(0), size_y_(0), resolution_(0.0), origin_x_(0.0), origin_y_(0.0), costmap_(NULL){
    access_ = new mutex_t();
  }

  Costmap2D::~Costmap2D(){
    deleteMaps();
    delete access_;
  }

  unsigned int Costmap2D::cellDistance(double world_dist){
    double cells_dist = max(0.0, ceil(world_dist / resolution_));
    return (unsigned int)cells_dist;
  }

  unsigned char* Costmap2D::getCharMap() const{
    return costmap_;
  }

  unsigned char Costmap2D::getCost(unsigned int mx, unsigned int my) const {
    return costmap_[getIndex(mx, my)];
  }

  void Costmap2D::setCost(unsigned int mx, unsigned int my, unsigned char cost){
    costmap_[getIndex(mx, my)] = cost;
  }

  void Costmap2D::mapToWorld(unsigned int mx, unsigned int my, double& wx, double& wy) const{
    wx = origin_x_ + (mx + 0.5) * resolution_;
    wy = origin_y_ + (my + 0.5) * resolution_;
  }

  bool Costmap2D::worldToMap(double wx, double wy, unsigned int& mx, unsigned int& my) const{
    if (wx < origin_x_ || wy < origin_y_)
      return false;

    mx = (int)((wx - origin_x_) / resolution_);
    my = (int)((wy - origin_y_) / resolution_);

    if (mx < size_x_ && my < size_y_)
      return true;

    return false;
  }

  void Costmap2D::worldToMapNoBounds(double wx, double wy, int& mx, int& my) const{
    mx = (int)((wx - origin_x_) / resolution_);
    my = (int)((wy - origin_y_) / resolution_);
  }

  void Costmap2D::worldToMapEnforceBounds(double wx, double wy, int& mx, int& my) const{
    // Here we avoid doing any math to wx,wy before comparing them to
    // the bounds, so their values can go out to the max and min values
    // of double floating point.
    if (wx < origin_x_){
      mx = 0;
    }
    else if (wx >= resolution_ * size_x_ + origin_x_){
      mx = size_x_ - 1;
    }else{
      mx = (int)((wx - origin_x_) / resolution_);
    }

    if (wy < origin_y_){
      my = 0;
    }
    else if (wy >= resolution_ * size_y_ + origin_y_){
      my = size_y_ - 1;
    }else{
      my = (int)((wy - origin_y_) / resolution_);
    }
  }

  void Costmap2D::updateOrigin(double new_origin_x, double new_origin_y){
    // project the new origin into the grid
    int cell_ox, cell_oy;
    cell_ox = int((new_origin_x - origin_x_) / resolution_);
    cell_oy = int((new_origin_y - origin_y_) / resolution_);

    // Nothing to update
    if (cell_ox == 0 && cell_oy == 0)
      return;

    // compute the associated world coordinates for the origin cell
    // because we want to keep things grid-aligned
    double new_grid_ox, new_grid_oy;
    new_grid_ox = origin_x_ + cell_ox * resolution_;
    new_grid_oy = origin_y_ + cell_oy * resolution_;

    // To save casting from unsigned int to int a bunch of times
    int size_x = size_x_;
    int size_y = size_y_;

    // we need to compute the overlap of the new and existing windows
    int lower_left_x, lower_left_y, upper_right_x, upper_right_y;
    lower_left_x = min(max(cell_ox, 0), size_x);
    lower_left_y = min(max(cell_oy, 0), size_y);
    upper_right_x = min(max(cell_ox + size_x, 0), size_x);
    upper_right_y = min(max(cell_oy + size_y, 0), size_y);

    unsigned int cell_size_x = upper_right_x - lower_left_x;
    unsigned int cell_size_y = upper_right_y - lower_left_y;

    // we need a map to store the obstacles in the window temporarily
    unsigned char* local_map = new unsigned char[cell_size_x * cell_size_y];

    // copy the local window in the costmap to the local map
    copyMapRegion(costmap_, lower_left_x, lower_left_y, size_x_, local_map, 0, 0, cell_size_x, cell_size_x, cell_size_y);

    // now we'll set the costmap to be completely unknown if we track unknown space
    resetMaps();

    // update the origin with the appropriate world coordinates
    origin_x_ = new_grid_ox;
    origin_y_ = new_grid_oy;

    // compute the starting cell location for copying data back in
    int start_x = lower_left_x - cell_ox;
    int start_y = lower_left_y - cell_oy;

    // now we want to copy the overlapping information back into the map, but in its new location
    copyMapRegion(local_map, 0, 0, cell_size_x, costmap_, start_x, start_y, size_x_, cell_size_x, cell_size_y);

    // make sure to clean up
    delete[] local_map;
  }

  //设置机器人足迹内cell的cost
  bool Costmap2D::setConvexPolygonCost(const std::vector<geometry_msgs::Point>& polygon, unsigned char cost_value){
    //输入的参数polygon是多边形顶点集合，cost_value是要设定的cost值
    //先将世界系下的多边形顶点转换到地图坐标系，并存放进map_polygon数组中
    std::vector<MapLocation> map_polygon;
    for (unsigned int i = 0; i < polygon.size(); ++i){
      MapLocation loc;
      if (!worldToMap(polygon[i].x, polygon[i].y, loc.x, loc.y)){
        // ("Polygon lies outside map bounds, so we can't fill it");
        return false;
      }
      map_polygon.push_back(loc);
    }

    std::vector<MapLocation> polygon_cells;

    //调用convexFillCells函数
    //通过机器人顶点坐标数组map_polygon得到多边形边缘及内部的全部cell，存放在polygon_cells中
    convexFillCells(map_polygon, polygon_cells);

    //把多边形边缘及内部的所有cell的cost设置为cost_value
    for (unsigned int i = 0; i < polygon_cells.size(); ++i){
      unsigned int index = getIndex(polygon_cells[i].x, polygon_cells[i].y);
      costmap_[index] = cost_value;
    }
    return true;
  }

  //获取多边形边上的cell
  void Costmap2D::polygonOutlineCells(const std::vector<MapLocation>& polygon, std::vector<MapLocation>& polygon_cells){
    PolygonOutlineCells cell_gatherer(*this, costmap_, polygon_cells);
    //遍历顶点数组，循环调用raytraceLine函数
    for (unsigned int i = 0; i < polygon.size()-1; ++i){
      //对于离散的平面点，指定两个点，raytraceLine函数可以找到两个点之间的其他点，使得这些中间组成一个尽可能趋近直线的点集
      //该函数定义在costmap_2d.h中
      raytraceLine(cell_gatherer, polygon[i].x, polygon[i].y, polygon[i + 1].x, polygon[i + 1].y);
    }
    if (!polygon.empty()){
      unsigned int last_index = polygon.size() - 1;
      //将最后一点和第一点连接起来，形成闭合区域
      raytraceLine(cell_gatherer, polygon[last_index].x, polygon[last_index].y, polygon[0].x, polygon[0].y);
    }
  }

  //通过机器人顶点坐标数组得到多边形边缘及内部的全部cell
  void Costmap2D::convexFillCells(const std::vector<MapLocation>& polygon, std::vector<MapLocation>& polygon_cells){
    //确保给定的多边形顶点不少于3个
    if (polygon.size() < 3)
      return;

    //调用polygonOutlineCells函数，通过给定的顶点提取多边形的轮廓上的cell
    polygonOutlineCells(polygon, polygon_cells);

    //对多边形轮廓上的的cell点的x做排序，使其按x坐标升序排列(快速冒泡排序)
    MapLocation swap;
    unsigned int i = 0;
    while (i < polygon_cells.size() - 1){
      if (polygon_cells[i].x > polygon_cells[i + 1].x){
        swap = polygon_cells[i];
        polygon_cells[i] = polygon_cells[i + 1];
        polygon_cells[i + 1] = swap;
        if (i > 0)
          --i;
      }
      else
        ++i;
    }

    i = 0;
    MapLocation min_pt;
    MapLocation max_pt;
    unsigned int min_x = polygon_cells[0].x;
    unsigned int max_x = polygon_cells[polygon_cells.size() - 1].x;

    //遍历所有x，对每个相同的x(其实就是每一列)，检查y，获得y最大和最小的polygon cell
    //将范围内的所有cell填充进polygon_cells，从而获得多边形边缘及内部的所有cell
    for (unsigned int x = min_x; x <= max_x; ++x){
      if (i >= polygon_cells.size() - 1)
        break;

      //因为polygon_cells是按x生序排列的，因此这样取出两个的cell，一般都是一列上的(x相同)
      //(但理论上也可能出现恰好某一列上只有一个cell的情况,这是不是没考虑)
      if (polygon_cells[i].y < polygon_cells[i + 1].y){
        min_pt = polygon_cells[i];
        max_pt = polygon_cells[i + 1];
      }else{
        min_pt = polygon_cells[i + 1];
        max_pt = polygon_cells[i];
      }

      i += 2;
      //对横坐标为x的列进行循环，找到该列的最大和最小的y
      while (i < polygon_cells.size() && polygon_cells[i].x == x){
        if (polygon_cells[i].y < min_pt.y)
          min_pt = polygon_cells[i];
        else if (polygon_cells[i].y > max_pt.y)
          max_pt = polygon_cells[i];
        ++i;
      }

      MapLocation pt;
      //把横坐标为x的列的cell都添加到polygon_cells中
      for (unsigned int y = min_pt.y; y < max_pt.y; ++y){
        pt.x = x;
        pt.y = y;
        polygon_cells.push_back(pt);
      }
    }
  }

  unsigned int Costmap2D::getSizeInCellsX() const{
    return size_x_;
  }

  unsigned int Costmap2D::getSizeInCellsY() const{
    return size_y_;
  }

  double Costmap2D::getSizeInMetersX() const{
    return (size_x_ - 1 + 0.5) * resolution_;
  }

  double Costmap2D::getSizeInMetersY() const{
    return (size_y_ - 1 + 0.5) * resolution_;
  }

  double Costmap2D::getOriginX() const{
    return origin_x_;
  }

  double Costmap2D::getOriginY() const{
    return origin_y_;
  }

  double Costmap2D::getResolution() const{
    return resolution_;
  }

  bool Costmap2D::saveMap(std::string file_name){
    FILE *fp = fopen(file_name.c_str(), "w");

    if (!fp){
      return false;
    }

    fprintf(fp, "P2\n%u\n%u\n%u\n", size_x_, size_y_, 0xff);
    for (unsigned int iy = 0; iy < size_y_; iy++){
      for (unsigned int ix = 0; ix < size_x_; ix++){
        unsigned char cost = getCost(ix, iy);
        fprintf(fp, "%d ", cost);
      }
      fprintf(fp, "\n");
    }
    fclose(fp);
    return true;
  }

}  // namespace costmap_2d
