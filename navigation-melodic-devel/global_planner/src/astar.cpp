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
#include<global_planner/astar.h>
#include<costmap_2d/cost_values.h>

namespace global_planner {

    //A* 算法是策略寻路，不保证一定是最短路径
    //Dijkstra 算法是全局遍历，确保运算结果一定是最短路径
    //Dijkstra 算法需要载入全部数据，遍历搜索

    AStarExpansion::AStarExpansion(PotentialCalculator* p_calc, int xs, int ys) :
            Expander(p_calc, xs, ys) {
    }

    bool AStarExpansion::calculatePotentials(unsigned char* costs, 
                                             double start_x, double start_y, double end_x, double end_y,
                                             int cycles, float* potential) {
        //清空队列
        //queue_是启发式搜索到的向量队列<i,cost>
        queue_.clear();
        //toIndex函数获取索引值
        int start_i = toIndex(start_x, start_y);
        //将起点放入队列
        queue_.push_back(Index(start_i, 0));

        //std::fill：将一个区间的元素都赋予指定的值，即在（first, last)范围内填充指定值
        //将所有点的potential都设为一个极大值,potential就是估计值g,f=g+h
        //potential为g，也就是从起点到当前点的代价(Dijkstra算法中只有这个值)
        std::fill(potential, potential + ns_, POT_HIGH);
        //起点的potential值为0
        potential[start_i] = 0;

        //终点的索引
        int goal_i = toIndex(end_x, end_y);
        int cycle = 0;

        //循环目的：得到最小cost的索引，并删除它，如果索引指向goal则退出算法，返回true
        while (queue_.size() > 0 && cycle < cycles) {
            //将首元素放到最后，其他元素按照Cost值从小到大排列
            Index top = queue_[0];
            //pop_heap()是将堆顶元素与最后一个元素交换位置，之后用pop_back将最后一个元素删除
            //greater1()是自己定义的一个针对Index的比较函数，这里表示依据Index的小顶堆
            std::pop_heap(queue_.begin(), queue_.end(), greater1());
            queue_.pop_back();

            //小顶堆 所以top就是cost最小的点的索引
            int i = top.i;
            //如果到了目标点就结束
            if (i == goal_i)
                return true;

            //对前后左右四个点执行add函数，将代价最小点i周围点加入搜索队里并更新代价值
            add(costs, potential, potential[i], i + 1, end_x, end_y);
            add(costs, potential, potential[i], i - 1, end_x, end_y);
            add(costs, potential, potential[i], i + nx_, end_x, end_y);
            add(costs, potential, potential[i], i - nx_, end_x, end_y);

            cycle++;
        }
        return false;
    }

    //添加点并更新代价函数
    //如果是已经添加的点则忽略，根据costmap的值如果是障碍物的点也忽略
    //更新代价函数的公式：f(n) = g(n) + h(n)
    //potential[next_i]是起点到当前点的cost即g(n),
    //distance * neutral_cost_是当前点到目的点的cost即h(n)
    void AStarExpansion::add(unsigned char* costs, 
                             float* potential, float prev_potential, 
                             int next_i, 
                             int end_x, int end_y) {
        //越界了
        if (next_i < 0 || next_i >= ns_)    return;

        //未搜索的点cost为POT_HIGH，如小于该值，则为已搜索点，跳过
        if (potential[next_i] < POT_HIGH)   return;

        //障碍物或者无信息的点
        if(costs[next_i]>=lethal_cost_ && 
           !(unknown_ && costs[next_i]==costmap_2d::NO_INFORMATION))
            return;
        
        //potential[next_i]是起点到当前点的cost即g(n)
        //prev_potentia是父节点的pot值
        potential[next_i] = p_calc_->calculatePotential(potential, costs[next_i] + neutral_cost_, next_i, prev_potential);

        int x = next_i % nx_, y = next_i / nx_;
        //计算distance：从节点n到目标点最佳路径的估计代价，这里选用了曼哈顿距离（不能斜着走 且无视障碍物）
        float distance = abs(end_x - x) + abs(end_y - y);

        //distance只是格子个数，还有乘上每个格子的真实距离或是分辨率，所以最后h = distance*neutral_cost_
        //因此最后的f = h + g = potential[next_i] + distance*neutral_cost_
        //neutral_cost_默认值为50
        queue_.push_back(Index(next_i, potential[next_i] + distance * neutral_cost_));

        //插入小顶堆
        std::push_heap(queue_.begin(), queue_.end(), greater1());
    }
}
