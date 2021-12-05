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
*         Mike Phillips (put the planner in its own thread)
*********************************************************************/
#include <move_base/move_base.h>
#include <move_base_msgs/RecoveryStatus.h>
#include <cmath>

#include <boost/algorithm/string.hpp>
#include <boost/thread.hpp>

#include <geometry_msgs/Twist.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

/* 在Movebase主体中，各层地图的更新被启动，Action的回调函数触发全局规划线程，若成功，则将全局规划结果传入局部规划器，循环进行局部
 * 规划，得到速度指令，控制机器人前进，直到到达目标。其间，需要判断机器人是否到达终点（若是则规划停止）、机器人是否状态异常如发生
 * 震荡行为（若是则进入恢复行为）、机器人是否超时（若是则停止规划发布零速，否则重新规划）等等。这个主体是一个大的调用框架，保证了
 * 运动规划的正常运行，具体算法在各子过程中分别实现。*/

namespace move_base {

  //MoveBase类的构造函数
  MoveBase::MoveBase(tf2_ros::Buffer& tf):
    tf_(tf),
    as_(NULL),
    planner_costmap_ros_(NULL), controller_costmap_ros_(NULL),
    //加载了baseGlobalPlanner的类库
    bgp_loader_("nav_core", "nav_core::BaseGlobalPlanner"),
    //加载了baseLocalPlanner的类库
    blp_loader_("nav_core", "nav_core::BaseLocalPlanner"),
    //加载了recoveryBehaviour的类库
    recovery_loader_("nav_core", "nav_core::RecoveryBehavior"),
    planner_plan_(NULL), latest_plan_(NULL), controller_plan_(NULL),
    runPlanner_(false), setup_(false), p_freq_change_(false), c_freq_change_(false), new_global_plan_(false) {
    //一直到这里都是构造函数的初始化列表（从冒号开始）

    //新建Action服务器，绑定回调函数MoveBase::executeCb，这个函数是move_base的核心
    //as_指向Action服务器，当执行as_->start()时调用MoveBase::executeCb函数（收到Action的goal后调用该回调函数）
    as_ = new MoveBaseActionServer(ros::NodeHandle(), "move_base", boost::bind(&MoveBase::executeCb, this, _1), false);

    //这是定义了两个namespace（但是具体在使用过程中这两个namespace有什么区别呢？）
    ros::NodeHandle private_nh("~");
    ros::NodeHandle nh;

    recovery_trigger_ = PLANNING_R;

    //这些参数在私有namespace上，其他节点无法访问？
    //从参数服务器加载用户输入的参数，如果没有就设置为默认值（第三个参数就是默认值）
    std::string global_planner, local_planner;
    //设置全局规划器的插件名称，默认navfn/NavfnROS（navigation核心购件——全局规划器的具体实现）
    private_nh.param("base_global_planner", global_planner, std::string("navfn/NavfnROS"));
    //设置局部规划器的插件名称，默认TrajectoryPlannerROS（navigation核心购件——局部规划器的具体实现）
    private_nh.param("base_local_planner", local_planner, std::string("base_local_planner/TrajectoryPlannerROS"));
    //robot_base_frame，默认base_link
    private_nh.param("global_costmap/robot_base_frame", robot_base_frame_, std::string("base_link"));
    //global_frame，默认/map坐标系
    private_nh.param("global_costmap/global_frame", global_frame_, std::string("map"));
    //全局路径规划器的循环速率 设置为0.0表示当收到新目标点或者局部路径规划器上报路径不通时 全局路径规划器才会启动
    //也就是说全局路径规划不会自动循环不断地执行，只有必要的时候才执行
    private_nh.param("planner_frequency", planner_frequency_, 0.0);
    //发布底盘控制命令的控制频率
    private_nh.param("controller_frequency", controller_frequency_, 20.0);
    //空间清理操作执行前 路径规划器等待有效规划的时间（秒）
    private_nh.param("planner_patience", planner_patience_, 5.0);
    //空间清理操作执行前 控制器等待有效控制命令的时间（秒）
    private_nh.param("controller_patience", controller_patience_, 15.0);
    //恢复操作之前尝试规划的次数 -1表示无上限地不断尝试
    //机器人如果发生状态异常如发生震荡行为 则进入恢复行为
    private_nh.param("max_planning_retries", max_planning_retries_, -1);
    //执行恢复操作之前允许的震荡时间 0表示永远不超时
    private_nh.param("oscillation_timeout", oscillation_timeout_, 0.0);
    //机器人需要移动该距离才可认为没有震荡 移动完毕后重置定时器参数oscillation_timeout
    private_nh.param("oscillation_distance", oscillation_distance_, 0.5);

    //parameters of make_plan service
    private_nh.param("make_plan_clear_costmap", make_plan_clear_costmap_, true);
    private_nh.param("make_plan_add_unreachable_goal", make_plan_add_unreachable_goal_, true);

    //初始化三个plan的缓冲池数组
    //geometry_msgs::PoseStamped is A Pose with reference coordinate frame and timestamp
    //其中的pose是一个三维的点表示位置，一个四元数表示姿态
    //也就说，plan的结果其实就是geometry_msgs::PoseStamped类型的一个队列，即一个位姿队列
    planner_plan_ = new std::vector<geometry_msgs::PoseStamped>();
    latest_plan_ = new std::vector<geometry_msgs::PoseStamped>();
    controller_plan_ = new std::vector<geometry_msgs::PoseStamped>();

    //planner_thread_线程所绑定的planThread函数是move_base的一个重点，即全局规划线程
    //创建规划器线程，在该线程里运行planThread函数
    planner_thread_ = new boost::thread(boost::bind(&MoveBase::planThread, this));

    //创建发布速度话题的publisher
    vel_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    //创建发布即时目标话题的publisher
    current_goal_pub_ = private_nh.advertise<geometry_msgs::PoseStamped>("current_goal", 0 );

    //我理解的不断创建新的ros::NodeHandle的作用就是为了给发布的一些话题等数据不同的namespace
    //比如这里 就是要将下面这个Action服务器的消息放在move_base这个namespace上
    ros::NodeHandle action_nh("move_base");
    //创建发布MoveBaseActionGoal消息的publisher，发布到/move_base/goal话题上
    action_goal_pub_ = action_nh.advertise<move_base_msgs::MoveBaseActionGoal>("goal", 1);
    //创建发布RecoveryStatus消息的publisher，发布到/move_base/recovery_status话题上
    recovery_status_pub_= action_nh.advertise<move_base_msgs::RecoveryStatus>("recovery_status", 1);

    ros::NodeHandle simple_nh("move_base_simple");
    //这里提供一种机制：如果你发布geometry_msgs::PoseStamped消息到/move_base_simple/goal话题上，
    //回调函数goalCB会处理在/move_base_simple/goal话题上接收到的消息
    //这种机制相当于是对原本的Action进行了简化，发布goal，它不会反馈信息，但是可以供nav_view和rviz等仿真工具使用
    //这里是在订阅/move_base_simple/goal话题上接收到的消息 也就是开发者向这个简化topic上发布消息 在这里被navigation订阅
    goal_sub_ = simple_nh.subscribe<geometry_msgs::PoseStamped>("goal", 1, boost::bind(&MoveBase::goalCB, this, _1));

    //加载代价地图的参数：内切、外接、清理半径等
    //假设机器人的半径和costmap规定的一致
    private_nh.param("local_costmap/inscribed_radius", inscribed_radius_, 0.325);
    private_nh.param("local_costmap/circumscribed_radius", circumscribed_radius_, 0.46);
    private_nh.param("clearing_radius", clearing_radius_, circumscribed_radius_);
    private_nh.param("conservative_reset_dist", conservative_reset_dist_, 3.0);

    //一些标志位的设置
    private_nh.param("shutdown_costmaps", shutdown_costmaps_, false);
    private_nh.param("clearing_rotation_allowed", clearing_rotation_allowed_, true);
    private_nh.param("recovery_behavior_enabled", recovery_behavior_enabled_, true);

    //下面的两个部分初始化全局规划器和局部规划器的指针和各自的costmap

    //创建全局规划器的代价地图（规划器用到的地图实质上是Costmap2DROS类的实例，这个类是ROS对costmap的封装）
    planner_costmap_ros_ = new costmap_2d::Costmap2DROS("global_costmap", tf_);
    planner_costmap_ros_->pause();

    //初始化全局规划器，planner_指针（createInstance函数实现？）
    try {
      planner_ = bgp_loader_.createInstance(global_planner);
      planner_->initialize(bgp_loader_.getName(global_planner), planner_costmap_ros_);
    } catch (const pluginlib::PluginlibException& ex) {
      ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered and that the containing library is built? Exception: %s", global_planner.c_str(), ex.what());
      exit(1);
    }

    //创建本地规划器的代价地图
    controller_costmap_ros_ = new costmap_2d::Costmap2DROS("local_costmap", tf_);
    controller_costmap_ros_->pause();

    //创建本地规划器，tc_指针
    try {
      tc_ = blp_loader_.createInstance(local_planner);
      ROS_INFO("Created local_planner %s", local_planner.c_str());
      tc_->initialize(blp_loader_.getName(local_planner), &tf_, controller_costmap_ros_);
    } catch (const pluginlib::PluginlibException& ex) {
      ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered and that the containing library is built? Exception: %s", local_planner.c_str(), ex.what());
      exit(1);
    }

    //根据传感器数据动态更新全局和本地的代价地图
    //类函数start()会调用各层地图的active()函数，开始订阅传感器话题，对地图进行更新，这部分在代价地图部分详述。
    planner_costmap_ros_->start();
    controller_costmap_ros_->start();

    //创建一个用于响应获取全局路径规划请求的Server
    make_plan_srv_ = private_nh.advertiseService("make_plan", &MoveBase::planService, this);

    //创建一个用于响应清除代价地图请求的Server
    clear_costmaps_srv_ = private_nh.advertiseService("clear_costmaps", &MoveBase::clearCostmapsService, this);

    //if we shutdown our costmaps when we're deactivated... we'll do that now
    if(shutdown_costmaps_){
      ROS_DEBUG_NAMED("move_base","Stopping costmaps initially");
      planner_costmap_ros_->stop();
      controller_costmap_ros_->stop();
    }

    //加载恢复行为插件列表（可以自行指定恢复行为列表，如果没有指定或者指定有误，则加载默认的恢复行为）
    if(!loadRecoveryBehaviors(private_nh)){
      loadDefaultRecoveryBehaviors();
    }

    //在最开始的时候，需要进行一次全局路径规划，这里设置进入全局规划模式
    state_ = PLANNING;

    //恢复行为的索引，设置为0表示从索引为0的恢复行为开始依次执行
    recovery_index_ = 0;

    //Action服务器启动 as_指向Action服务器
    as_->start();

    //参数动态配置（参数服务器上参数发生变化后 调用回调函数reconfigureCB）
    dsrv_ = new dynamic_reconfigure::Server<move_base::MoveBaseConfig>(ros::NodeHandle("~"));
    dynamic_reconfigure::Server<move_base::MoveBaseConfig>::CallbackType cb = boost::bind(&MoveBase::reconfigureCB, this, _1, _2);
    dsrv_->setCallback(cb);
  }

  //该函数自动从参数服务器上读取修改的参数，并且进行一些参数变换的中间过渡处理（如base_global_planner规划插件变化）
  void MoveBase::reconfigureCB(move_base::MoveBaseConfig &config, uint32_t level){
    boost::recursive_mutex::scoped_lock l(configuration_mutex_);

    //第一次进入该函数，只是加载初始的参数
    if(!setup_)
    {
      last_config_ = config;
      default_config_ = config;
      setup_ = true;
      return;
    }

    //如果在参数服务器上设置了恢复默认参数
    if(config.restore_defaults) {
      config = default_config_;
      config.restore_defaults = false;
    }

    if(planner_frequency_ != config.planner_frequency)
    {
      planner_frequency_ = config.planner_frequency;
      p_freq_change_ = true;
    }

    if(controller_frequency_ != config.controller_frequency)
    {
      controller_frequency_ = config.controller_frequency;
      c_freq_change_ = true;
    }

    planner_patience_ = config.planner_patience;
    controller_patience_ = config.controller_patience;
    max_planning_retries_ = config.max_planning_retries;
    conservative_reset_dist_ = config.conservative_reset_dist;

    recovery_behavior_enabled_ = config.recovery_behavior_enabled;
    clearing_rotation_allowed_ = config.clearing_rotation_allowed;
    shutdown_costmaps_ = config.shutdown_costmaps;

    oscillation_timeout_ = config.oscillation_timeout;
    oscillation_distance_ = config.oscillation_distance;
    
    //全局规划器改变
    if(config.base_global_planner != last_config_.base_global_planner) {
      boost::shared_ptr<nav_core::BaseGlobalPlanner> old_planner = planner_;
      //initialize the global planner
      ROS_INFO("Loading global planner %s", config.base_global_planner.c_str());
      try {
        planner_ = bgp_loader_.createInstance(config.base_global_planner);

        //等待当前的规划器完成本次的规划
        boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);

        //先清除规划器中的现有数据，再初始化新的规划器
        planner_plan_->clear();
        latest_plan_->clear();
        controller_plan_->clear();
        resetState();
        planner_->initialize(bgp_loader_.getName(config.base_global_planner), planner_costmap_ros_);

        lock.unlock();
      } catch (const pluginlib::PluginlibException& ex) {
        ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered and that the \
                   containing library is built? Exception: %s", config.base_global_planner.c_str(), ex.what());
        //如果切换过程中出现问题了，就退回之前的规划器
        planner_ = old_planner;
        config.base_global_planner = last_config_.base_global_planner;
      }
    }

    //局部规划器改变（同理）
    if(config.base_local_planner != last_config_.base_local_planner){
      boost::shared_ptr<nav_core::BaseLocalPlanner> old_planner = tc_;
      //create a local planner
      try {
        tc_ = blp_loader_.createInstance(config.base_local_planner);
        // Clean up before initializing the new planner
        planner_plan_->clear();
        latest_plan_->clear();
        controller_plan_->clear();
        resetState();
        tc_->initialize(blp_loader_.getName(config.base_local_planner), &tf_, controller_costmap_ros_);
      } catch (const pluginlib::PluginlibException& ex) {
        ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered and that the \
                   containing library is built? Exception: %s", config.base_local_planner.c_str(), ex.what());
        tc_ = old_planner;
        config.base_local_planner = last_config_.base_local_planner;
      }
    }

    make_plan_clear_costmap_ = config.make_plan_clear_costmap;
    make_plan_add_unreachable_goal_ = config.make_plan_add_unreachable_goal;

    //注意保存上一次的配置，
    last_config_ = config;
  }

  //将PoseStamped包装在Action消息中，然后重新发送到服务器（仿真时用这个发布goal）
  void MoveBase::goalCB(const geometry_msgs::PoseStamped::ConstPtr& goal){
    ROS_DEBUG_NAMED("move_base","In ROS goal callback, wrapping the PoseStamped in the action message and re-sending to the server.");
    move_base_msgs::MoveBaseActionGoal action_goal;
    action_goal.header.stamp = ros::Time::now();
    action_goal.goal.target_pose = *goal;

    action_goal_pub_.publish(action_goal);
  }

  //提供给使用者一种指定goal的路径规划服务（在这种服务下如果无法达到goal 会自动搜索终点容许误差范围内的可达到点作为终点）
  bool MoveBase::planService(nav_msgs::GetPlan::Request &req, nav_msgs::GetPlan::Response &resp){
    if(as_->isActive()){
      ROS_ERROR("move_base must be in an inactive state to make a plan for an external user");
      return false;
    }
    //make sure we have a costmap for our planner
    if(planner_costmap_ros_ == NULL){
      ROS_ERROR("move_base cannot make a plan for you because it doesn't have a costmap");
      return false;
    }

    geometry_msgs::PoseStamped start;
    //如果使用者没有指定一个起始位姿（由空的frame-id标识），则使用机器人的当前位姿
    if(req.start.header.frame_id.empty()){
        geometry_msgs::PoseStamped global_pose;
        if(!getRobotPose(global_pose, planner_costmap_ros_)){
          ROS_ERROR("move_base cannot make a plan for you because it could not get the start pose of the robot");
          return false;
        }
        start = global_pose;
    }
    //若指定了起始位姿
    else{ 
        start = req.start;
    }

    //？没有整理到笔记中
    if (make_plan_clear_costmap_) {
      //update the copy of the costmap the planner uses
      clearCostmapWindows(2 * clearing_radius_, 2 * clearing_radius_);
    }

    //first try to make a plan to the exact desired goal
    std::vector<geometry_msgs::PoseStamped> global_plan;
    if(!planner_->makePlan(start, req.goal, global_plan) || global_plan.empty()){
      ROS_DEBUG_NAMED("move_base","Failed to find a plan to exact goal of (%.2f, %.2f), searching for a feasible goal within tolerance",
          req.goal.pose.position.x, req.goal.pose.position.y);

      //在规定的误差范围内向外寻找可行的目标位置
      geometry_msgs::PoseStamped p;
      p = req.goal;
      bool found_legal = false;
      float resolution = planner_costmap_ros_->getCostmap()->getResolution();
      float search_increment = resolution*3.0;
      if(req.tolerance > 0.0 && req.tolerance < search_increment) search_increment = req.tolerance;
      //在允许的范围内逐渐扩大搜索的半径
      for(float max_offset = search_increment; max_offset <= req.tolerance && !found_legal; max_offset += search_increment){
        for(float y_offset = 0; y_offset <= max_offset && !found_legal; y_offset += search_increment) {
          for(float x_offset = 0; x_offset <= max_offset && !found_legal; x_offset += search_increment) {

            //don't search again inside the current outer layer
            if(x_offset < max_offset-1e-9 && y_offset < max_offset-1e-9) continue;

            //search to both sides of the desired goal
            for(float y_mult = -1.0; y_mult <= 1.0 + 1e-9 && !found_legal; y_mult += 2.0) {

              //if one of the offsets is 0, -1*0 is still 0 (so get rid of one of the two)
              if(y_offset < 1e-9 && y_mult < -1.0 + 1e-9) continue;

              for(float x_mult = -1.0; x_mult <= 1.0 + 1e-9 && !found_legal; x_mult += 2.0) {
                if(x_offset < 1e-9 && x_mult < -1.0 + 1e-9) continue;

                p.pose.position.y = req.goal.pose.position.y + y_offset * y_mult;
                p.pose.position.x = req.goal.pose.position.x + x_offset * x_mult;

                if(planner_->makePlan(start, p, global_plan)){
                  if(!global_plan.empty()){
                    //当我们搜索到一个在目标附近的可达到的终点之后，从起始点到可达到点的路径规划结果就在global_plan队列中了
                    //我们可以选择当找到目标附近的可达到点之后，是否将原本不可达到的目标点也放到global_plan队列的末尾
                    //因为有可能当我们达到其附近的可达到点之后，原本不可达到的目标点也可以达到了
                    if (make_plan_add_unreachable_goal_) {
                      //adding the (unreachable) original goal to the end of the global plan, in case the local planner can get you there
                      //(the reachable goal should have been added by the global planner)
                      global_plan.push_back(req.goal);
                    }

                    found_legal = true;
                    ROS_DEBUG_NAMED("move_base", "Found a plan to point (%.2f, %.2f)", p.pose.position.x, p.pose.position.y);
                    break;
                  }
                }
                else{
                  ROS_DEBUG_NAMED("move_base","Failed to find a plan to point (%.2f, %.2f)", p.pose.position.x, p.pose.position.y);
                }
              }
            }
          }
        }
      }
    }

    //copy the plan into a message to send out
    resp.plan.poses.resize(global_plan.size());
    for(unsigned int i = 0; i < global_plan.size(); ++i){
      resp.plan.poses[i] = global_plan[i];
    }

    return true;
  }

  MoveBase::~MoveBase(){
    recovery_behaviors_.clear();

    delete dsrv_;

    if(as_ != NULL)
      delete as_;

    if(planner_costmap_ros_ != NULL)
      delete planner_costmap_ros_;

    if(controller_costmap_ros_ != NULL)
      delete controller_costmap_ros_;

    planner_thread_->interrupt();
    planner_thread_->join();

    delete planner_thread_;

    delete planner_plan_;
    delete latest_plan_;
    delete controller_plan_;

    planner_.reset();
    tc_.reset();
  }

  //正式执行全局路径规划算法，结果保存在planner_中（但是算法实现的不在这里 是NavFn中的一个同名函数makePlan实现的）
  bool MoveBase::makePlan(const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan){
    boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(planner_costmap_ros_->getCostmap()->getMutex()));

    //初始化空plan
    plan.clear();

    //如果没有全局代价地图，返回false，因为全局规划必须基于全局代价地图
    if(planner_costmap_ros_ == NULL) {
      ROS_ERROR("Planner costmap ROS is NULL, unable to create global plan");
      return false;
    }

    //如果得不到机器人的起始位姿，返回false
    geometry_msgs::PoseStamped global_pose;
    if(!getRobotPose(global_pose, planner_costmap_ros_)) {
      ROS_WARN("Unable to get starting pose of robot, unable to create global plan");
      return false;
    }

    const geometry_msgs::PoseStamped& start = global_pose;

    //接下来是实际进行全局规划的函数（这里调用的makePlan函数不是现在这个makePlan函数）
    //若规划失败或得到的plan为空，返回false，否则返回true。
    if(!planner_->makePlan(start, goal, plan) || plan.empty()){
      ROS_DEBUG_NAMED("move_base","Failed to find a  plan to point (%.2f, %.2f)", goal.pose.position.x, goal.pose.position.y);
      return false;
    }
    return true;
  }

  void MoveBase::publishZeroVelocity(){
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = 0.0;
    vel_pub_.publish(cmd_vel);
  }

  bool MoveBase::isQuaternionValid(const geometry_msgs::Quaternion& q){
    //检查四元数中是否有无穷值
    if(!std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z) || !std::isfinite(q.w)){
      ROS_ERROR("Quaternion has nans or infs... discarding as a navigation goal");
      return false;
    }

    tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);

    //检查四元数长度是否接近0
    if(tf_q.length2() < 1e-6){
      ROS_ERROR("Quaternion has length close to zero... discarding as navigation goal");
      return false;
    }

    //归一化四元数 并检查它是否可以将垂直向量正确转换
    tf_q.normalize();

    tf2::Vector3 up(0, 0, 1);

    double dot = up.dot(up.rotate(tf_q.getAxis(), tf_q.getAngle()));

    if(fabs(dot - 1) > 1e-3){
      ROS_ERROR("Quaternion is invalid... for navigation the z-axis of the quaternion must be close to vertical.");
      return false;
    }

    return true;
  }

  geometry_msgs::PoseStamped MoveBase::goalToGlobalFrame(const geometry_msgs::PoseStamped& goal_pose_msg){
    std::string global_frame = planner_costmap_ros_->getGlobalFrameID();
    geometry_msgs::PoseStamped goal_pose, global_pose;
    goal_pose = goal_pose_msg;

    //just get the latest available transform... for accuracy they should send goals in the frame of the planner
    goal_pose.header.stamp = ros::Time();

    try{
      tf_.transform(goal_pose_msg, global_pose, global_frame);
    }
    catch(tf2::TransformException& ex){
      ROS_WARN("Failed to transform the goal pose from %s into the %s frame: %s",
          goal_pose.header.frame_id.c_str(), global_frame.c_str(), ex.what());
      return goal_pose_msg;
    }

    return global_pose;
  }

  void MoveBase::wakePlanner(const ros::TimerEvent& event)
  {
    // we have slept long enough for rate
    planner_cond_.notify_one();
  }

  //planThread()的核心是调用makePlan函数，该函数中实际进行全局规划
  //当executeCB函数中唤醒planThread（调用planner_cond_.notify_one()）并将标志位runPlanner_设置为真
  //跳出内部的循环，继续进行下面部分
  void MoveBase::planThread(){
    ROS_DEBUG_NAMED("move_base_plan_thread","Starting planner thread...");
    ros::NodeHandle n;
    ros::Timer timer;
    bool wait_for_wake = false;
    boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
    while(n.ok()){
      //不断循环，直到wait_for_wake为假（上面行已置为假）且runPlanner_为真，跳出循环
      //check if we should run the planner (the mutex is locked)
      while(wait_for_wake || !runPlanner_){
        //if we should not be running the planner then suspend this thread
        ROS_DEBUG_NAMED("move_base_plan_thread","Planner thread is suspending");
        //调用wait函数时，函数会自动调用lock.unlock()释放锁，使得其他被阻塞在锁竞争上的线程得以继续执行
        planner_cond_.wait(lock);
        wait_for_wake = false;
      }
      ros::Time start_time = ros::Time::now();

      //把全局中被更新的全局目标planner_goal存储为临时目标
      geometry_msgs::PoseStamped temp_goal = planner_goal_;
      lock.unlock();
      ROS_DEBUG_NAMED("move_base_plan_thread","Planning...");

      //run planner
      //全局规划初始化 清空
      planner_plan_->clear();
      //调用MoveBase类的makePlan函数，如果成功为临时目标制定全局规划planner_plan_，则返回true
      bool gotPlan = n.ok() && makePlan(temp_goal, *planner_plan_);

      //如果成功为临时目标制定全局规划
      if(gotPlan){
        //提示成功制定全局规划，并打印规划路线上的点数
        ROS_DEBUG_NAMED("move_base_plan_thread","Got Plan with %zu points!", planner_plan_->size());
        //pointer swap the plans under mutex (the controller will pull from latest_plan_)
        std::vector<geometry_msgs::PoseStamped>* temp_plan = planner_plan_;

        //当线程中需要操作全局变量的时候就是上锁
        lock.lock();
        planner_plan_ = latest_plan_;
        latest_plan_ = temp_plan;
        //最近一次有效全局规划的时间设为当前时间
        last_valid_plan_ = ros::Time::now();
        planning_retries_ = 0;
        new_global_plan_ = true;

        ROS_DEBUG_NAMED("move_base_plan_thread","Generated a plan from the base_global_planner");

        //确保只有在我们还没到达目标时才启动controller以局部规划
        //如果runPlanner_在调用此函数时被置为真，将MoveBase状态设置为CONTROLLING（局部规划中）
        if(runPlanner_)
          state_ = CONTROLLING;
        //planner_frequency_ <= 0则全局规划就是触发机制的，不会自动循环
        if(planner_frequency_ <= 0)
          runPlanner_ = false;
        lock.unlock();
      }
      //如果全局规划失败并且MoveBase还在Planning状态，即机器人没有移动，则进入自转模式
      else if(state_==PLANNING){
        ROS_DEBUG_NAMED("move_base_plan_thread","No Plan...");
        //最迟制定出本次全局规划的时间 = 上次成功规划的时间 + 容忍时间
        ros::Time attempt_end = last_valid_plan_ + ros::Duration(planner_patience_);

        //检查时间和次数是否超过限制，若其中一项不满足限制，停止全局规划
        //如果次数限制是负数，则表示尝试次数为无限
        lock.lock();
        planning_retries_++;
        if(runPlanner_ &&
           (ros::Time::now() > attempt_end || planning_retries_ > uint32_t(max_planning_retries_))){
          //将MoveBase状态设置为恢复行为
          state_ = CLEARING;
          //停止全局规划 全局规划标志位置为假
          runPlanner_ = false;
          publishZeroVelocity();
          //恢复行为触发器状态设置为全局规划失败（触发机制：PLANNING_R、CONTROLLING_R、OSCILLATION_R）
          recovery_trigger_ = PLANNING_R;
        }
        lock.unlock();
      }
      //为下一次迭代获取互斥锁
      lock.lock();

      //如果设置了全局规划的自动循环频率 则计算休眠时间
      if(planner_frequency_ > 0){
        ros::Duration sleep_time = (start_time + ros::Duration(1.0/planner_frequency_)) - ros::Time::now();
        if (sleep_time > ros::Duration(0.0)){
          wait_for_wake = true;
          timer = n.createTimer(sleep_time, &MoveBase::wakePlanner, this);
        }
      }
    }
  }

  //executeCb是Action的回调函数，它是MoveBase控制流的主体
  //它调用了MoveBase内另外几个作为子部分的重要成员函数，先后完成了全局规划和局部规划。
  void MoveBase::executeCb(const move_base_msgs::MoveBaseGoalConstPtr& move_base_goal)
  {
    //检测收到的目标位置的旋转四元数是否有效
    if(!isQuaternionValid(move_base_goal->target_pose.pose.orientation)){
      as_->setAborted(move_base_msgs::MoveBaseResult(), "Aborting on goal because it was sent with an invalid quaternion");
      return;
    }

    //将目标位置转换到global坐标系下（那目标位置原本是在当前的机器人坐标系下么？）
    geometry_msgs::PoseStamped goal = goalToGlobalFrame(move_base_goal->target_pose);

    //规划的时候不要动
    publishZeroVelocity();

    //有了一个goal 启动全局规划
    boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
    //用接收到的目标goal来更新全局规划目标（全局变量），它在planThread中会被用来做全局规划的当前目标
    planner_goal_ = goal;
    runPlanner_ = true;
    //开始全局规划并于此处阻塞
    //在这里调用notify会直接启动全局规划器线程，进行全局路径规划
    //全局规划器线程绑定的函数plannerThread()里有planner_cond_对象的wait函数
    planner_cond_.notify_one();
    lock.unlock();

    //全局规划完成后 发布目标到current_goal话题上
    current_goal_pub_.publish(goal);
    //创建一个全局规划容器
    std::vector<geometry_msgs::PoseStamped> global_plan;

    //设置局部规划频率
    ros::Rate r(controller_frequency_);
    //如果代价地图是被关闭的 这里重启
    if(shutdown_costmaps_){
      ROS_DEBUG_NAMED("move_base","Starting up costmaps that were shut down previously");
      planner_costmap_ros_->start();
      controller_costmap_ros_->start();
    }

    //上一次有效的局部规划时间设为现在
    last_valid_control_ = ros::Time::now();
    //上一次有效的全局规划时间设为现在
    last_valid_plan_ = ros::Time::now();
    //上一次震荡重置时间设为现在
    last_oscillation_reset_ = ros::Time::now();
    //对同一目标的全局规划次数记录归为0
    planning_retries_ = 0;

    ros::NodeHandle n;
    //全局规划完成 接下来循环调用executeCycle函数来控制机器人进行局部规划 完成相应跟随
    while(n.ok()){
      if(c_freq_change_){
        ROS_INFO("Setting controller frequency to %.2f", controller_frequency_);
        r = ros::Rate(controller_frequency_);
        c_freq_change_ = false;
      }
      //如果action的服务器被抢占 可能是：局部规划进行过程中收到新的目标
      //                       也可能是：收到取消行动的命令
      if(as_->isPreemptRequested()){
        //如果是 局部规划进行过程中收到新的目标
        //那么放弃当前目标，重复上面对目标进行的操作，使用新目标，并重新全局规划
        if(as_->isNewGoalAvailable()){
          //如果获得了新目标，接收并存储新目标，并将上述过程重新进行一遍
          move_base_msgs::MoveBaseGoal new_goal = *as_->acceptNewGoal();

          if(!isQuaternionValid(new_goal.target_pose.pose.orientation)){
            as_->setAborted(move_base_msgs::MoveBaseResult(), "Aborting on goal because it was sent with an invalid quaternion");
            return;
          }

          //将新目标坐标转换到全局坐标系（默认/map）下
          goal = goalToGlobalFrame(new_goal.target_pose);

          //重设恢复行为索引位为0
          recovery_index_ = 0;
          //重设MoveBase状态为全局规划中
          state_ = PLANNING;

          //重新调用planThread进行全局规划
          lock.lock();
          planner_goal_ = goal;
          runPlanner_ = true;
          planner_cond_.notify_one();
          lock.unlock();

          //全局规划成功后，发布新目标到current_goal话题上
          ROS_DEBUG_NAMED("move_base","move_base has received a goal of x: %.2f, y: %.2f", goal.pose.position.x, goal.pose.position.y);
          current_goal_pub_.publish(goal);

          last_valid_control_ = ros::Time::now();
          last_valid_plan_ = ros::Time::now();
          last_oscillation_reset_ = ros::Time::now();
          planning_retries_ = 0;
        }
        //如果是 收到取消行动的命令
        else {
          //被取消了 则重置服务器状态
          resetState();

          //Action服务器清除相关内容，并调用setPreempted()函数（函数作用？）
          ROS_DEBUG_NAMED("move_base","Move base preempting the current goal");
          as_->setPreempted();

          return;
        }
      }

      //检查目标是否被转换到全局坐标系（/map）下，如果并没有转换过来 则进行转换 然后再次执行上面的操作
      if(goal.header.frame_id != planner_costmap_ros_->getGlobalFrameID()){
        goal = goalToGlobalFrame(goal);

        //恢复行为索引重置为0，MoveBase状态置为全局规划中
        recovery_index_ = 0;
        state_ = PLANNING;

        //唤醒全局规划线程
        lock.lock();
        planner_goal_ = goal;
        runPlanner_ = true;
        planner_cond_.notify_one();
        lock.unlock();

        //publish the goal point to the visualizer
        ROS_DEBUG_NAMED("move_base","The global frame for move_base has changed, new frame: %s, new goal position x: %.2f, y: %.2f", goal.header.frame_id.c_str(), goal.pose.position.x, goal.pose.position.y);
        current_goal_pub_.publish(goal);

        //make sure to reset our timeouts and counters
        last_valid_control_ = ros::Time::now();
        last_valid_plan_ = ros::Time::now();
        last_oscillation_reset_ = ros::Time::now();
        planning_retries_ = 0;
      }

      //记录开始局部规划的时刻为当前时间
      ros::WallTime start = ros::WallTime::now();

      //调用executeCycle函数进行局部规划，传入目标和全局规划路线
      bool done = executeCycle(goal, global_plan);

      if(done)  return;

      //记录从局部规划开始到这时的时间差
      ros::WallDuration t_diff = ros::WallTime::now() - start;
      ROS_DEBUG_NAMED("move_base","Full control cycle time: %.9f\n", t_diff.toSec());

      //用局部规划频率进行休眠
      r.sleep();
      //cycleTime用来获取从r实例初始化到r实例被调用sleep函数的时间间隔
      if(r.cycleTime() > ros::Duration(1 / controller_frequency_) && state_ == CONTROLLING)
        ROS_WARN("Control loop missed its desired rate of %.4fHz... the loop actually took %.4f seconds", controller_frequency_, r.cycleTime().toSec());
    }

    //为什么要唤醒全局规划线程 才能使它彻底退出（这里的它是指局部规划？）
    lock.lock();
    runPlanner_ = true;
    planner_cond_.notify_one();
    lock.unlock();

    //如果节点被关闭了，那么Action服务器也关闭并返回（这是什么意思？）
    as_->setAborted(move_base_msgs::MoveBaseResult(), "Aborting on the goal because the node has been killed");
    return;
  }

  double MoveBase::distance(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2)
  {
    return hypot(p1.pose.position.x - p2.pose.position.x, p1.pose.position.y - p2.pose.position.y);
  }

  //executeCycle函数的作用是进行局部规划，函数先声明了将要发布的速度，然后获取当前位姿并格式转换。
  bool MoveBase::executeCycle(geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& global_plan){
    boost::recursive_mutex::scoped_lock ecl(configuration_mutex_);

    geometry_msgs::Twist cmd_vel;
    geometry_msgs::PoseStamped global_pose;
    //update feedback to correspond to our curent position
    getRobotPose(global_pose, planner_costmap_ros_);
    //将当前姿态数据保存在current_position中
    const geometry_msgs::PoseStamped& current_position = global_pose;

    //feedback指的是从服务端周期反馈回客户端的信息，把当前位姿反馈给客户端
    move_base_msgs::MoveBaseFeedback feedback;
    feedback.base_position = current_position;
    as_->publishFeedback(feedback);

    //做几个判断，判断机器人是否被困住，若是，则进入恢复行为
    
    //检查机器人是否在振荡
    //如果机器人移动的距离超过了振荡的判定距离，则认为没有在振荡
    if(distance(current_position, oscillation_pose_) >= oscillation_distance_){
      //把最新的振荡时间和振荡位置重置为当前时间和当前位置
      last_oscillation_reset_ = ros::Time::now();
      oscillation_pose_ = current_position;

      //如果我们上一次的恢复行为是由振荡引起，我们就重新设置恢复行为的索引
      //为了确保每次进入恢复行为都是从索引为0的恢复行为开始依次执行
      if(recovery_trigger_ == OSCILLATION_R)
        recovery_index_ = 0;
    }

    //检查局部代价地图是否是当前的 若不是则制停机器人
    if(!controller_costmap_ros_->isCurrent()){
      ROS_WARN("[%s]:Sensor data is out of date, we're not going to allow commanding of the base for safety",ros::this_node::getName().c_str());
      publishZeroVelocity();
      return false;
    }

    //如果全局规划得出了新的路线
    if(new_global_plan_){
      new_global_plan_ = false;
      ROS_DEBUG_NAMED("move_base","Got a new plan...swap pointers");

      //然后通过指针交换，将latest_plan_（最新的全局规划结果）的值传递给controller_plan_即局部规划使用
      //然后将上一次的局部规划路线传递给latest_plan（这个有用么？）
      std::vector<geometry_msgs::PoseStamped>* temp_plan = controller_plan_;

      boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
      controller_plan_ = latest_plan_;
      latest_plan_ = temp_plan;
      lock.unlock();
      ROS_DEBUG_NAMED("move_base","pointers swapped!");

      //在实例tc_上调用局部规划器的类函数setPlan()把全局规划的结果传递给局部规划器，如果传递失败，退出并返回。
      if(!tc_->setPlan(*controller_plan_)){
        //ABORT and SHUTDOWN COSTMAPS
        ROS_ERROR("Failed to pass global plan to the controller, aborting.");
        resetState();

        //停止全局规划线程
        lock.lock();
        runPlanner_ = false;
        lock.unlock();

        //停止Action服务器，打印“将全局规划传递至局部规划器控制失败”
        as_->setAborted(move_base_msgs::MoveBaseResult(), "Failed to pass global plan to the controller.");
        return true;
      }

      //如果恢复行为的触发原因是全局规划失败，重置恢复行为索引（全局规划失败也会进行局部规划 因为恢复行为是在局部规划中进行的）
      //可能是因为recovery_trigger_的初始值就是PLANNING_R或者上一次全局规划是PLANNING_R触发的
      if(recovery_trigger_ == PLANNING_R)
        recovery_index_ = 0;
    }

    //对MoveBase状态进行判断，由于局部规划在全局规划结束后才调用，所以有以下几种结果：
    switch(state_){
      //PLANNING：全局规划还没完成，还没得到一个全局路线，那么唤醒一个全局规划线程去制定全局路线
      case PLANNING:
        {
          boost::recursive_mutex::scoped_lock lock(planner_mutex_);
          runPlanner_ = true;
          planner_cond_.notify_one();
        }
        ROS_DEBUG_NAMED("move_base","Waiting for plan, in the planning state.");
        break;

      //CONTROLLING：全局规划成功，得到全局路线，这里进行真正的局部规划：
      case CONTROLLING:
        ROS_DEBUG_NAMED("move_base","In controlling state.");

        //如果已经位于终点，结束局部规划；
        if(tc_->isGoalReached()){
          ROS_DEBUG_NAMED("move_base","Goal reached!");
          resetState();

          //结束全局规划线程
          boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
          runPlanner_ = false;
          lock.unlock();

          //Action返回成功
          as_->setSucceeded(move_base_msgs::MoveBaseResult(), "Goal reached.");
          return true;
        }

        //检查机器人是否被困住，如果是，则进入恢复行为
        if(oscillation_timeout_ > 0.0 &&
            last_oscillation_reset_ + ros::Duration(oscillation_timeout_) < ros::Time::now()){
          //如果振荡状态超时了，发布0速度
          publishZeroVelocity();
          //MoveBase状态置为恢复行为
          state_ = CLEARING;
          //恢复行为触发器置为OSCILLATION_R（振荡导致）
          recovery_trigger_ = OSCILLATION_R;
        }

        {//如果没到终点 且状态正常
          boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(controller_costmap_ros_->getCostmap()->getMutex()));

          //局部规划器实例tc_被传入了全局规划后，调用computeVelocityCommands函数计算速度存储在cmd_vel中
          if(tc_->computeVelocityCommands(cmd_vel)){
            ROS_DEBUG_NAMED( "move_base", "Got a valid command from the local planner: %.3lf, %.3lf, %.3lf",
                            cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z );
            //若成功计算速度，则说明本次局部规划成功， 将上一次有效局部控制的时间设为当前时间
            last_valid_control_ = ros::Time::now();
            //向底盘发送速度控制消息，一个循环只发一次速度命令
            vel_pub_.publish(cmd_vel);
            //如果恢复行为触发器值是局部规划失败，把索引置0
            if(recovery_trigger_ == CONTROLLING_R)
              recovery_index_ = 0;
          }
          //若速度计算失败
          else {
            ROS_DEBUG_NAMED("move_base", "The local planner could not find a valid plan.");
            //计算局部规划用时限制
            ros::Time attempt_end = last_valid_control_ + ros::Duration(controller_patience_);

            //若局部规划用时超过限制
            if(ros::Time::now() > attempt_end){
              //发布0速度，进入恢复行为，触发器置为局部规划失败
              publishZeroVelocity();
              state_ = CLEARING;
              recovery_trigger_ = CONTROLLING_R;
            }
            //若局部规划用时没超过限制 那就是找不到有效的控制路线（can't find a valid control），则再次回到全局规划
            else{
              //发布0速度，在机器人当前位置再次回到全局规划
              last_valid_plan_ = ros::Time::now();
              planning_retries_ = 0;
              state_ = PLANNING;
              publishZeroVelocity();

              //激活全局规划线程
              boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
              runPlanner_ = true;
              planner_cond_.notify_one();
              lock.unlock();
            }
          }
        }
        break;

      //CLEARING：全局规划失败，进入恢复行为
      case CLEARING:
        ROS_DEBUG_NAMED("move_base","In clearing/recovery state");
        //如果允许使用恢复行为，且恢复行为索引值小于恢复行为数组的大小
        if(recovery_behavior_enabled_ && recovery_index_ < recovery_behaviors_.size()){
          ROS_DEBUG_NAMED("move_base_recovery","Executing behavior %u of %zu", recovery_index_+1, recovery_behaviors_.size());

          move_base_msgs::RecoveryStatus msg;
          msg.pose_stamped = current_position;
          msg.current_recovery_number = recovery_index_;
          msg.total_number_of_recoveries = recovery_behaviors_.size();
          msg.recovery_behavior_name =  recovery_behavior_names_[recovery_index_];

          //发布恢复行为的相关信息
          recovery_status_pub_.publish(msg);

          //开始恢复行为，在executeCycle的循环中一次次迭代恢复行为（这里会阻塞吗？）
          recovery_behaviors_[recovery_index_]->runBehavior();

          //上一次震荡重置时间设为现在（虽然之前已经振荡了，但是这里重置，然后看一下他进行了一些恢复行为之后它还会不会振荡）
          last_oscillation_reset_ = ros::Time::now();

          //在进行恢复行为的时候需要不断尝试切换进入Planning模式，检查恢复行为是否产生了效果（使机器人脱困）
          ROS_DEBUG_NAMED("move_base_recovery","Going back to planning state");
          last_valid_plan_ = ros::Time::now();
          planning_retries_ = 0;
          state_ = PLANNING;

          //更新恢复行为索引
          recovery_index_++;
        }
        //若没有可用的恢复行为或所有恢复行为均无效
        else{
          ROS_DEBUG_NAMED("move_base_recovery","All recovery behaviors have failed, locking the planner and disabling it.");
          //关闭全局规划器
          boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
          runPlanner_ = false;
          lock.unlock();

          ROS_DEBUG_NAMED("move_base_recovery","Something should abort after this.");

          //反馈失败的具体信息
          //找不到可行的局部控制策略
          if(recovery_trigger_ == CONTROLLING_R){
            ROS_ERROR("Aborting because a valid control could not be found. Even after executing all recovery behaviors");
            as_->setAborted(move_base_msgs::MoveBaseResult(), "Failed to find a valid control. Even after executing recovery behaviors.");
          }
          //找不到可行的全局规划策略
          else if(recovery_trigger_ == PLANNING_R){
            ROS_ERROR("Aborting because a valid plan could not be found. Even after executing all recovery behaviors");
            as_->setAborted(move_base_msgs::MoveBaseResult(), "Failed to find a valid plan. Even after executing recovery behaviors.");
          }
          //机器人无法摆脱振荡状态
          else if(recovery_trigger_ == OSCILLATION_R){
            ROS_ERROR("Aborting because the robot appears to be oscillating over and over. Even after executing all recovery behaviors");
            as_->setAborted(move_base_msgs::MoveBaseResult(), "Robot is oscillating. Even after executing recovery behaviors.");
          }
          resetState();
          return true;
        }
        break;

      //其他错误状态
      default:
        ROS_ERROR("This case should never be reached, something is wrong, aborting");
        resetState();
        //关闭全局规划器
        boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
        runPlanner_ = false;
        lock.unlock();
        as_->setAborted(move_base_msgs::MoveBaseResult(), "Reached a case that should not be hit in move_base. This is a bug, please report it.");
        return true;
    }
    //we aren't done yet
    return false;
  }

  //加载恢复行为插件 可以自行指定恢复行为列表，如果没有指定或者指定有误，则加载默认的恢复行为
  bool MoveBase::loadRecoveryBehaviors(ros::NodeHandle node){
    XmlRpc::XmlRpcValue behavior_list;
    if(node.getParam("recovery_behaviors", behavior_list)){
      if(behavior_list.getType() == XmlRpc::XmlRpcValue::TypeArray){
        for(int i = 0; i < behavior_list.size(); ++i){
          if(behavior_list[i].getType() == XmlRpc::XmlRpcValue::TypeStruct){
            if(behavior_list[i].hasMember("name") && behavior_list[i].hasMember("type")){
              //check for recovery behaviors with the same name
              for(int j = i + 1; j < behavior_list.size(); j++){
                if(behavior_list[j].getType() == XmlRpc::XmlRpcValue::TypeStruct){
                  if(behavior_list[j].hasMember("name") && behavior_list[j].hasMember("type")){
                    std::string name_i = behavior_list[i]["name"];
                    std::string name_j = behavior_list[j]["name"];
                    if(name_i == name_j){
                      ROS_ERROR("A recovery behavior with the name %s already exists, this is not allowed. Using the default recovery behaviors instead.",
                          name_i.c_str());
                      return false;
                    }
                  }
                }
              }
            }
            else{
              ROS_ERROR("Recovery behaviors must have a name and a type and this does not. Using the default recovery behaviors instead.");
              return false;
            }
          }
          else{
            ROS_ERROR("Recovery behaviors must be specified as maps, but they are XmlRpcType %d. We'll use the default recovery behaviors instead.",
                behavior_list[i].getType());
            return false;
          }
        }

        //if we've made it to this point, we know that the list is legal so we'll create all the recovery behaviors
        for(int i = 0; i < behavior_list.size(); ++i){
          try{
            //check if a non fully qualified name has potentially been passed in
            if(!recovery_loader_.isClassAvailable(behavior_list[i]["type"])){
              std::vector<std::string> classes = recovery_loader_.getDeclaredClasses();
              for(unsigned int i = 0; i < classes.size(); ++i){
                if(behavior_list[i]["type"] == recovery_loader_.getName(classes[i])){
                  //if we've found a match... we'll get the fully qualified name and break out of the loop
                  ROS_WARN("Recovery behavior specifications should now include the package name. You are using a deprecated API. Please switch from %s to %s in your yaml file.",
                      std::string(behavior_list[i]["type"]).c_str(), classes[i].c_str());
                  behavior_list[i]["type"] = classes[i];
                  break;
                }
              }
            }

            boost::shared_ptr<nav_core::RecoveryBehavior> behavior(recovery_loader_.createInstance(behavior_list[i]["type"]));

            //shouldn't be possible, but it won't hurt to check
            if(behavior.get() == NULL){
              ROS_ERROR("The ClassLoader returned a null pointer without throwing an exception. This should not happen");
              return false;
            }

            //initialize the recovery behavior with its name
            behavior->initialize(behavior_list[i]["name"], &tf_, planner_costmap_ros_, controller_costmap_ros_);
            recovery_behavior_names_.push_back(behavior_list[i]["name"]);
            recovery_behaviors_.push_back(behavior);
          }
          catch(pluginlib::PluginlibException& ex){
            ROS_ERROR("Failed to load a plugin. Using default recovery behaviors. Error: %s", ex.what());
            return false;
          }
        }
      }
      else{
        ROS_ERROR("The recovery behavior specification must be a list, but is of XmlRpcType %d. We'll use the default recovery behaviors instead.",
            behavior_list.getType());
        return false;
      }
    }
    else{
      //if no recovery_behaviors are specified, we'll just load the defaults
      return false;
    }

    //if we've made it here... we've constructed a recovery behavior list successfully
    return true;
  }

  //加载默认的恢复行为插件
  void MoveBase::loadDefaultRecoveryBehaviors(){
    recovery_behaviors_.clear();
    try{
      //we need to set some parameters based on what's been passed in to us to maintain backwards compatibility
      ros::NodeHandle n("~");
      n.setParam("conservative_reset/reset_distance", conservative_reset_dist_);
      n.setParam("aggressive_reset/reset_distance", circumscribed_radius_ * 4);

      //保守的空间清理操作
      //first, we'll load a recovery behavior to clear the costmap
      boost::shared_ptr<nav_core::RecoveryBehavior> cons_clear(recovery_loader_.createInstance("clear_costmap_recovery/ClearCostmapRecovery"));
      cons_clear->initialize("conservative_reset", &tf_, planner_costmap_ros_, controller_costmap_ros_);
      recovery_behavior_names_.push_back("conservative_reset");
      recovery_behaviors_.push_back(cons_clear);

      //旋转
      //next, we'll load a recovery behavior to rotate in place
      boost::shared_ptr<nav_core::RecoveryBehavior> rotate(recovery_loader_.createInstance("rotate_recovery/RotateRecovery"));
      if(clearing_rotation_allowed_){
        rotate->initialize("rotate_recovery", &tf_, planner_costmap_ros_, controller_costmap_ros_);
        recovery_behavior_names_.push_back("rotate_recovery");
        recovery_behaviors_.push_back(rotate);
      }

      //激进的空间清理操作
      //next, we'll load a recovery behavior that will do an aggressive reset of the costmap
      boost::shared_ptr<nav_core::RecoveryBehavior> ags_clear(recovery_loader_.createInstance("clear_costmap_recovery/ClearCostmapRecovery"));
      ags_clear->initialize("aggressive_reset", &tf_, planner_costmap_ros_, controller_costmap_ros_);
      recovery_behavior_names_.push_back("aggressive_reset");
      recovery_behaviors_.push_back(ags_clear);

      //再次旋转
      //we'll rotate in-place one more time
      if(clearing_rotation_allowed_){
        recovery_behaviors_.push_back(rotate);
        recovery_behavior_names_.push_back("rotate_recovery");
      }
    }
    catch(pluginlib::PluginlibException& ex){
      ROS_FATAL("Failed to load a plugin. This should not happen on default recovery behaviors. Error: %s", ex.what());
    }
    return;
  }

  void MoveBase::resetState(){
    // Disable the planner thread
    boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
    runPlanner_ = false;
    lock.unlock();

    // Reset statemachine
    state_ = PLANNING;
    recovery_index_ = 0;
    recovery_trigger_ = PLANNING_R;
    publishZeroVelocity();

    //if we shutdown our costmaps when we're deactivated... we'll do that now
    if(shutdown_costmaps_){
      ROS_DEBUG_NAMED("move_base","Stopping costmaps");
      planner_costmap_ros_->stop();
      controller_costmap_ros_->stop();
    }
  }

  //获取机器人在指定的代价地图坐标系下的位姿
  bool MoveBase::getRobotPose(geometry_msgs::PoseStamped& global_pose, costmap_2d::Costmap2DROS* costmap){
    tf2::toMsg(tf2::Transform::getIdentity(), global_pose.pose);
    geometry_msgs::PoseStamped robot_pose;
    tf2::toMsg(tf2::Transform::getIdentity(), robot_pose.pose);
    robot_pose.header.frame_id = robot_base_frame_;
    robot_pose.header.stamp = ros::Time(); // latest available
    ros::Time current_time = ros::Time::now();  // save time for checking tf delay later

    // get robot pose on the given costmap frame
    try{
      tf_.transform(robot_pose, global_pose, costmap->getGlobalFrameID());
    }
    catch (tf2::LookupException& ex){
      ROS_ERROR_THROTTLE(1.0, "No Transform available Error looking up robot pose: %s\n", ex.what());
      return false;
    }
    catch (tf2::ConnectivityException& ex){
      ROS_ERROR_THROTTLE(1.0, "Connectivity Error looking up robot pose: %s\n", ex.what());
      return false;
    }
    catch (tf2::ExtrapolationException& ex){
      ROS_ERROR_THROTTLE(1.0, "Extrapolation Error looking up robot pose: %s\n", ex.what());
      return false;
    }

    //检查获取的机器人位姿数据时间戳是不是在代价地图的tf容许范围内（默认10秒）
    // check if global_pose time stamp is within costmap transform tolerance
    if (current_time.toSec() - global_pose.header.stamp.toSec() > costmap->getTransformTolerance()){
      ROS_WARN_THROTTLE(1.0, "Transform timeout for %s. " \
                        "Current time: %.4f, pose stamp: %.4f, tolerance: %.4f", costmap->getName().c_str(),
                        current_time.toSec(), global_pose.header.stamp.toSec(), costmap->getTransformTolerance());
      return false;
    }
    return true;
  }
};
