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
#ifndef _POTENTIAL_CALCULATOR_H
#define _POTENTIAL_CALCULATOR_H

#include <algorithm>

//calculatePotential()计算根据use_quadratic的值有下面两个选择:
//  若为TRUE  则使用二次曲线计算
//  若为False 则采用简单方法计算
//简单方法即：return prev_potential + cost
//  即：costs[next_i] + neutral_cost_+ prev_potential)
//  翻译：地图代价+单格距离代价(初始化为50)+之前路径代价g

namespace global_planner {

    class PotentialCalculator {
        public:
            PotentialCalculator(int nx, int ny) {
                setSize(nx, ny);
            }

            virtual float calculatePotential(float* potential, unsigned char cost, int n, float prev_potential=-1){
                //如果父节点的pot值小于0(调用时没有赋值 使用了缺省值-1)
                //(在函数clearEndpoint中会出现这种缺省调用的情况)
                if(prev_potential < 0){
                    //则将周围四个点的pot的最小值当做父节点的pot值
                    float min_h = std::min(potential[n - 1], potential[n + 1])；
                    float min_v = std::min(potential[n - nx_], potential[n + nx_]);
                    prev_potential = std::min(min_h, min_v);
                }

                return prev_potential + cost;
            }

            virtual void setSize(int nx, int ny) {
                nx_ = nx;
                ny_ = ny;
                ns_ = nx * ny;
            }

        protected:
            inline int toIndex(int x, int y) {
                return x + nx_ * y;
            }

            int nx_, ny_, ns_;
    };

}
#endif
