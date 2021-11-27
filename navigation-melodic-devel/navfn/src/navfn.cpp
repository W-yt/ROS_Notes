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
//
// Navigation function computation
// Uses Dijkstra's method
// Modified for Euclidean-distance computation
// 
// Path calculation uses no interpolation when pot field is at max in
//   nearby cells
//
// Path calc has sanity check that it succeeded
//


#include <navfn/navfn.h>
#include <ros/console.h>

namespace navfn {

  //for A*-method
  int create_nav_plan_astar(COSTTYPE *costmap, int nx, int ny, int* goal, int* start, float *plan, int nplan){
      static NavFn *nav = NULL;

      if (nav == NULL)
        nav = new NavFn(nx,ny);

      if (nav->nx != nx || nav->ny != ny) // check for compatibility with previous call
      {
        delete nav;
        nav = new NavFn(nx,ny);      
      }

      nav->setGoal(goal);
      nav->setStart(start);

      nav->costarr = costmap;
      nav->setupNavFn(true);

      // calculate the nav fn and path
      nav->priInc = 2*COST_NEUTRAL;
      nav->propNavFnAstar(std::max(nx*ny/20,nx+ny));

      // path
      int len = nav->calcPath(nplan);

      if (len > 0)			// found plan
        ROS_DEBUG("[NavFn] Path found, %d steps\n", len);
      else
        ROS_DEBUG("[NavFn] No path found\n");

      if (len > 0)
      {
        for (int i=0; i<len; i++)
        {
          plan[i*2] = nav->pathx[i];
          plan[i*2+1] = nav->pathy[i];
        }
      }

      return len;
    }

  NavFn::NavFn(int xs, int ys){  
    // create cell arrays
    costarr = NULL;
    potarr = NULL;
    pending = NULL;
    gradx = grady = NULL;
    setNavArr(xs,ys);

    // priority buffers
    pb1 = new int[PRIORITYBUFSIZE];
    pb2 = new int[PRIORITYBUFSIZE];
    pb3 = new int[PRIORITYBUFSIZE];

    //传播阈值的增长单位
    priInc = 2*COST_NEUTRAL;	

    //目标点和起始点
    goal[0] = goal[1] = 0;
    start[0] = start[1] = 0;

    // display function
    displayFn = NULL;
    displayInt = 0;

    //生成路径缓冲区
    npathbuf = npath = 0;
    pathx = pathy = NULL;
    //步长(一般为0.5个分辨率)
    pathStep = 0.5;
  }

  NavFn::~NavFn(){
    if(costarr)
      delete[] costarr;
    if(potarr)
      delete[] potarr;
    if(pending)
      delete[] pending;
    if(gradx)
      delete[] gradx;
    if(grady)
      delete[] grady;
    if(pathx)
      delete[] pathx;
    if(pathy)
      delete[] pathy;
    if(pb1)
      delete[] pb1;
    if(pb2)
      delete[] pb2;
    if(pb3)
      delete[] pb3;
  }

  //设置goal目标点(对于navfn来说是起始点)
  void NavFn::setGoal(int *g){
      goal[0] = g[0];
      goal[1] = g[1];
      ROS_DEBUG("[NavFn] Setting goal to %d,%d\n", goal[0], goal[1]);
  }

  //设置起始点
  void NavFn::setStart(int *g){
      start[0] = g[0];
      start[1] = g[1];
      ROS_DEBUG("[NavFn] Setting start to %d,%d\n", start[0], start[1]);
  }

  //创建navfn的相关数组(cost数组、potential数组、pending数组[指示待办]、gradx与grady梯度数组)
  void NavFn::setNavArr(int xs, int ys){
      ROS_DEBUG("[NavFn] Array is %d x %d\n", xs, ys);

      nx = xs;
      ny = ys;
      ns = nx*ny;

      if(costarr)
        delete[] costarr;
      if(potarr)
        delete[] potarr;
      if(pending)
        delete[] pending;

      if(gradx)
        delete[] gradx;
      if(grady)
        delete[] grady;

      //cost数组
      costarr = new COSTTYPE[ns];
      memset(costarr, 0, ns*sizeof(COSTTYPE));
      //potential数组
      potarr = new float[ns];	
      //pending——待办的
      pending = new bool[ns];
      memset(pending, 0, ns*sizeof(bool));
      gradx = new float[ns];
      grady = new float[ns];
  }

  //将costmap翻译成costarr，根据代价值cost生成无向权重图
  void NavFn::setCostmap(const COSTTYPE *cmap, bool isROS, bool allow_unknown){
    //用指针cm指向数组costarr，costarr是全局规划用到的地图
    //COSTTYPE宏定义为unsigned char，即ROS中使用的地图Costmap2D中用于储存地图数据的成员类型
    COSTTYPE *cm = costarr;
    //地图为ROS map
    if (isROS){
      //在地图的长宽范围内进行迭代
      for (int i=0; i<ny; i++){
        //k值记录二重迭代的次数
        int k = i*nx;
        for (int j=0; j<nx; j++, k++, cmap++, cm++){
          //最小权重值为COST_NEUTRAL＝50(无障碍物的free栅格)
          //最大权重值为COST_OBS＝254(致命障碍被禁止的栅格)
          //次大权重值为COST_OBS_ROS＝253(膨胀型障碍的栅格)
          //未知权重值未COST_UNKNOWN_ROS=255(未知权重的栅格)
          *cm = COST_OBS;
          int v = *cmap;
          //若当前cell在costmap上的值 < COST_OBS_ROS(253)，即非膨胀型障碍
          if (v < COST_OBS_ROS){
            //重新将其赋值为COST_NEUTRAL(50)+当前cell在costmap上的值×比例0.8（最高253）
            //将原本[0~253]范围的数值变换到[50~253]范围
            v = COST_NEUTRAL + COST_FACTOR*v;
            //数值限制 防止将非膨胀型障碍赋值大于253 
            if (v >= COST_OBS)
              v = COST_OBS-1;
            //赋值给当前全局规划要使用的地图costarr
            *cm = v;
          }
          //若当前cell的值为COST_UNKNOWN_ROS(255)，未知区域
          else if(v == COST_UNKNOWN_ROS && allow_unknown){
            //统一设置为253（膨胀型障碍）
            v = COST_OBS-1;
            *cm = v;
          }
        }
      }
    }
    //地图不是ROS map，可能是PGM地图，同样进行翻译
    else{
      for (int i=0; i<ny; i++){
        int k=i*nx;
        for (int j=0; j<nx; j++, k++, cmap++, cm++){
          *cm = COST_OBS;
          //避免处理边界cell
          if (i<7 || i > ny-8 || j<7 || j > nx-8)
            continue;
          int v = *cmap;
          if (v < COST_OBS_ROS){
            v = COST_NEUTRAL+COST_FACTOR*v;
            if (v >= COST_OBS)
              v = COST_OBS-1;
            *cm = v;
          }
          else if(v == COST_UNKNOWN_ROS){
            v = COST_OBS-1;
            *cm = v;
          }
        }
      }
    }
  }

  //函数内完成了整个路径计算的流程，顺序调用了几个子部分的函数
  bool NavFn::calcNavFnDijkstra(bool atStart){
    //对翻译生成的costarr数组进行了边际设置等处理，并初始化了potarr数组和梯度数组gradx、grady
    setupNavFn(true);

    //从目标点开始传播计算pot值
    propNavFnDijkstra(std::max(nx*ny/20,nx+ny),atStart);

    //从起始点开始梯度下降搜索最优路径
    int len = calcPath(nx*ny/2);

    //如果找到了有效路径
    if (len > 0){
      ROS_DEBUG("[NavFn] Path found, %d steps\n", len);
      return true;
    }
    else{
      ROS_DEBUG("[NavFn] No path found\n");
      return false;
    }
  }

  //for A*-method
  bool NavFn::calcNavFnAstar()
    {
      setupNavFn(true);

      // calculate the nav fn and path
      propNavFnAstar(std::max(nx*ny/20,nx+ny));

      // path
      int len = calcPath(nx*4);

      if (len > 0)			// found plan
      {
        ROS_DEBUG("[NavFn] Path found, %d steps\n", len);
        return true;
      }
      else
      {
        ROS_DEBUG("[NavFn] No path found\n");
        return false;
      }
    }

  float *NavFn::getPathX() { return pathx; }
  float *NavFn::getPathY() { return pathy; }
  int    NavFn::getPathLen() { return npath; }

//将cell添加到curP、nextP、overP的操作
#define push_cur(n)  { if (n>=0 && n<ns && !pending[n] && \
    costarr[n]<COST_OBS && curPe<PRIORITYBUFSIZE) \
  { curP[curPe++]=n; pending[n]=true; }}

#define push_next(n) { if (n>=0 && n<ns && !pending[n] && \
    costarr[n]<COST_OBS && nextPe<PRIORITYBUFSIZE) \
  { nextP[nextPe++]=n; pending[n]=true; }}
  
#define push_over(n) { if (n>=0 && n<ns && !pending[n] && \
    costarr[n]<COST_OBS && overPe<PRIORITYBUFSIZE) \
  { overP[overPe++]=n; pending[n]=true; }}

  //对翻译生成的costarr数组进行了边际设置等处理，并初始化了potarr数组和梯度数组gradx、grady
  void NavFn::setupNavFn(bool keepit){
    //初始化potarr元素全部为POT_HIGH(1.0e10无穷大表示未赋值的)，并初始化梯度表初始值全部为0.0
    for (int i=0; i<ns; i++){
      //将pot数组初始化为最大值，默认起点到所有点的行走代价值都为最大
      potarr[i] = POT_HIGH;
      //这是什么情况使用的？
      if (!keepit) 
        costarr[i] = COST_NEUTRAL;
      //初始化x,y方向的梯度表
      gradx[i] = grady[i] = 0.0;
    }

    //设置costarr的四条边的cell的值为COST_OBS(致命层254)，封闭地图四周，以防产生边界以外的轨迹
    COSTTYPE *pc;
    //costarr第一行全部设置为COST_OBS
    pc = costarr;
    for (int i=0; i<nx; i++)
      *pc++ = COST_OBS;
    //costarr最后一行全部设置为COST_OBS
    pc = costarr + (ny-1)*nx;
    for (int i=0; i<nx; i++)
      *pc++ = COST_OBS;
    //costarr第一列全部设置为COST_OBS
    pc = costarr;
    for (int i=0; i<ny; i++, pc+=nx)
      *pc = COST_OBS;
    //costarr最后一列全部设置为COST_OBS
    pc = costarr + nx - 1;
    for (int i=0; i<ny; i++, pc+=nx)
      *pc = COST_OBS;

    //初始化一些用于迭代更新potarr的数据
    curT = COST_OBS;  //当前传播阈值
    curP = pb1; //当前用于传播的cell索引数组      curP记录当前正要访问的栅格
    curPe = 0;  //当前用于传播的cell的数量
    nextP = pb2;//用于下个传播过程的cell索引数组  nextP记录即将要访问的栅格中优先级较高的部分
    nextPe = 0; //用于下个传播过程的cell的数量
    overP = pb3;//传播界限外的cell索引数组       overP记录即将要访问的栅格中优先级较低的部分
    overPe = 0; //传播界限外的cell的数量

    //初始化pending数组为全0，即设置所有的cell状态都不是“待办状态”
    memset(pending, 0, ns*sizeof(bool));

    //k为目标goal_cell的索引
    int k = goal[0] + goal[1]*nx;
    //设置costarr的索引k（目标）的pot值为0
    //并对它四周的cell在pending数组中标记为“待办状态”，并把索引存放入curP数组
    initCost(k,0);

    //用nobs记录costarr中的致命障碍物cell的数量(但是实际好像没有用)
    pc = costarr;
    int ntot = 0;
    for (int i=0; i<ns; i++, pc++){
      if (*pc >= COST_OBS)
        ntot++;
    }
    nobs = ntot;
  }

  //初始化目标的pot值用于开始传播
  void NavFn::initCost(int k, float v){
    potarr[k] = v;
    push_cur(k+1);
    push_cur(k-1);
    push_cur(k-nx);
    push_cur(k+nx);
  }

#define INVSQRT2 0.707106781
  //该函数用于更新单个cell的Potential值
  inline void NavFn::updateCell(int n){
    //先获取当前cell四周邻点的potarr值
    float u,d,l,r;
    l = potarr[n-1];
    r = potarr[n+1];		
    u = potarr[n-nx];
    d = potarr[n+nx];

    //寻找左右邻点最小pot值与上下邻点最小pot值
    float ta, tc;
    if (l<r) tc=l; else tc=r;
    if (u<d) ta=u; else ta=d;

    //只有当当前cell不是致命障碍物时，才由它向四周传播，否则到它后停止，不传播
    if (costarr[n] < COST_OBS){
      //获取当前点的cost值
      float hf = (float)costarr[n];
      float dc = tc-ta;
      if (dc < 0){
        //dc为左右邻点最小pot值与上下邻点最小pot值之差的绝对值
        dc = -dc;
        //将当前cell四周邻点中potarr的最小值赋给ta
        ta = tc;
      }

      //计算当前cell的新的Potential值

      //计算当前点Potential值时，有两种情况：
      //  需要对“左右邻点最小pot值与上下邻点最小pot值之差的绝对值”和“当前cell的cost值”比较
      //  决定采用直接相加的公式还是二次逼近后再相加的公式
      //  两个公式的区别是所谓的“圆形传播”和“菱形传播”，后者能够产生效果更好的菱形传播
      float pot;
      //为什么是这个判断条件决定更新Potential值的公式的选择？
      if (dc >= hf)
        //当前点Potential值=四周最小的Potential值+当前点cost值
        //这种传播过程中cost的累加造成Potential值的上升能够反映离目标点的远近
        //并且cost的大小与cell离障碍物的远近对应，更大的cost对应更大的Potential
        //并且障碍物点不更新Potential，使得其值停留在无限大
        //故Potential值的大小也能反映点与障碍物的接近程度
        pot = ta+hf;
      else{
        //二次逼近可以通过查找表来加快速度，但仍然需要做除法  
        float d = dc/hf;
        float v = -0.2301*d*d + 0.5307*d + 0.7040;
        pot = ta + hf*v;
      }

      // now add affected neighbors to priority blocks
      if (pot < potarr[n]){
        //INVSQRT2为1/根号2
        float le = INVSQRT2*(float)costarr[n-1];
        float re = INVSQRT2*(float)costarr[n+1];
        float ue = INVSQRT2*(float)costarr[n-nx];
        float de = INVSQRT2*(float)costarr[n+nx];
        //只有当前cell的Potential计算值<原本的Potential值，更新当前cell的Potential值
        potarr[n] = pot;

        //将临近cell放入nextP或overP，供下次迭代使用
        //如果当前cell的pot值小于传播阈值curT，则进入nextP，否则进入overP
        //curT是当前传播阈值（curT=COST_OBS=254）
        //这里区分进入nextP和overP的目的只是为了获得更好的传播效果：
        //    若不加区分都进入nextP则是菱形传播
        //    若如此区分分别进入nextP和overP则是圆形传播(效果更好)
        if (pot < curT){
          //如果四周的cell本身的pot值比经过当前cell后再进入对应cell生成的pot值大的话
          //就进入nextP或overP，说明对应的cell的pot值可以进行优化
          if (l > pot+le) push_next(n-1);
          if (r > pot+re) push_next(n+1);
          if (u > pot+ue) push_next(n-nx);
          if (d > pot+de) push_next(n+nx);
        }else{
          if (l > pot+le) push_over(n-1);
          if (r > pot+re) push_over(n+1);
          if (u > pot+ue) push_over(n-nx);
          if (d > pot+de) push_over(n+nx);
        }
      }
    }
  }

#define INVSQRT2 0.707106781
  //for A*-method
  inline void NavFn::updateCellAstar(int n){
    // get neighbors
    float u,d,l,r;
    l = potarr[n-1];
    r = potarr[n+1];		
    u = potarr[n-nx];
    d = potarr[n+nx];
    //ROS_INFO("[Update] c: %0.1f  l: %0.1f  r: %0.1f  u: %0.1f  d: %0.1f\n", 
    //	 potarr[n], l, r, u, d);
    // ROS_INFO("[Update] cost of %d: %d\n", n, costarr[n]);

    // find lowest, and its lowest neighbor
    float ta, tc;
    if (l<r) tc=l; else tc=r;
    if (u<d) ta=u; else ta=d;

    // do planar wave update
    if (costarr[n] < COST_OBS)	// don't propagate into obstacles
    {
      float hf = (float)costarr[n]; // traversability factor
      float dc = tc-ta;		// relative cost between ta,tc
      if (dc < 0) 		// ta is lowest
      {
        dc = -dc;
        ta = tc;
      }

      // calculate new potential
      float pot;
      if (dc >= hf)		// if too large, use ta-only update
        pot = ta+hf;
      else			// two-neighbor interpolation update
      {
        // use quadratic approximation
        // might speed this up through table lookup, but still have to 
        //   do the divide
        float d = dc/hf;
        float v = -0.2301*d*d + 0.5307*d + 0.7040;
        pot = ta + hf*v;
      }

      //ROS_INFO("[Update] new pot: %d\n", costarr[n]);

      // now add affected neighbors to priority blocks
      if (pot < potarr[n])
      {
        float le = INVSQRT2*(float)costarr[n-1];
        float re = INVSQRT2*(float)costarr[n+1];
        float ue = INVSQRT2*(float)costarr[n-nx];
        float de = INVSQRT2*(float)costarr[n+nx];

        // calculate distance
        int x = n%nx;
        int y = n/nx;
        float dist = hypot(x-start[0], y-start[1])*(float)COST_NEUTRAL;

        potarr[n] = pot;
        pot += dist;
        if (pot < curT)	// low-cost buffer block 
        {
          if (l > pot+le) push_next(n-1);
          if (r > pot+re) push_next(n+1);
          if (u > pot+ue) push_next(n-nx);
          if (d > pot+de) push_next(n+nx);
        }
        else
        {
          if (l > pot+le) push_over(n-1);
          if (r > pot+re) push_over(n+1);
          if (u > pot+ue) push_over(n-nx);
          if (d > pot+de) push_over(n+nx);
        }
      }
    }
  }

  //以目标点（Potential值已初始化为0）为起点，向整张地图的cell传播，填充potarr数组，直到找到起始点为止
  //使用dijkstra算法广度优先传播，更新potential数组，获得传播起点(goal)到传播过程中任意点的最优路径
  //结束条件：达到了最大循环次数cycles、跑完了所有可以更新的cell、找到了起始点(则atStart = true)
  bool NavFn::propNavFnDijkstra(int cycles, bool atStart)	{
    int nwv = 0;	//priority block的最大数量(就是curPe的最大值)(没有用)
    int nc = 0;		//priority blocks中的cell数(进行传播的cell的总数)(没有用)
    int cycle = 0; //当前迭代次数

    //记录起始位置(也就是算法寻找的终点)的索引
    int startCell = start[1]*nx + start[0];

    //循环迭代最大为cycles
    for (; cycle < cycles; cycle++){
      //如果当前正在传播和下一步传播的集都为空，那么说明已经无法继续传播
      //可能有无法越过的障碍或其他情况，退出
      if (curPe == 0 && nextPe == 0)
        break;

      //curPe是当前用于传播的cell的数量
      nc += curPe;
      if (curPe > nwv)
        nwv = curPe;

      //对pending数组进行设置(curP的cell设置为非待办状态)
      int *pb = curP;
      int i = curPe;			
      while (i-- > 0)
        pending[*(pb++)] = false;

      pb = curP; 
      i = curPe;
      while (i-- > 0)
        //传播当前节点，更新其pot值
        //并将其四周符合特定条件的点放入nextP或overP，用于下一步传播
        updateCell(*pb++);

      if (displayInt > 0 &&  (cycle % displayInt) == 0)
        displayFn(this);

      //将nextP数组中的cell传递给curP，继续上述传播
      curPe = nextPe;
      nextPe = 0;
      //navigation源码中很多时候感觉直接赋值就可以了，这种交换并没有什么意义
      pb = curP;		
      curP = nextP;
      nextP = pb;

      //若nextP没有cell可以用来传播，则引入overP中的cell
      if (curPe == 0){
        //增大传播阈值(初始是 COST_OBS = 254)(每次增加priInc默认是 2*COST_NEUTRAL = 2*50 = 100)
        curT += priInc;
        curPe = overPe;
        overPe = 0;
        pb = curP;
        curP = overP;
        overP = pb;
      }

      //检查我们是否到达了起始点
      if (atStart)
        //当[起点]的Pot值(可以认为每个cell的Pot值就是goal到该点的路径长度)不再是被初始化的无穷大
        //而是有一个实际的值时，说明到达了起点，传播停止
        if (potarr[startCell] < POT_HIGH)
          break;
    }

    ROS_DEBUG("[NavFn] Used %d cycles, %d cells visited (%d%%), priority buf max %d\n", 
        cycle,nc,(int)((nc*100.0)/(ns-nobs)),nwv);

    if (cycle < cycles) return true;
    else return false;
  }

  //for A*-method
  bool NavFn::propNavFnAstar(int cycles){
      int nwv = 0;			// max priority block size
      int nc = 0;			// number of cells put into priority blocks
      int cycle = 0;		// which cycle we're on

      // set initial threshold, based on distance
      float dist = hypot(goal[0]-start[0], goal[1]-start[1])*(float)COST_NEUTRAL;
      curT = dist + curT;

      // set up start cell
      int startCell = start[1]*nx + start[0];

      // do main cycle
      for (; cycle < cycles; cycle++) // go for this many cycles, unless interrupted
      {
        // 
        if (curPe == 0 && nextPe == 0) // priority blocks empty
          break;

        // stats
        nc += curPe;
        if (curPe > nwv)
          nwv = curPe;

        // reset pending flags on current priority buffer
        int *pb = curP;
        int i = curPe;			
        while (i-- > 0)		
          pending[*(pb++)] = false;

        // process current priority buffer
        pb = curP; 
        i = curPe;
        while (i-- > 0)		
          updateCellAstar(*pb++);

        if (displayInt > 0 &&  (cycle % displayInt) == 0)
          displayFn(this);

        // swap priority blocks curP <=> nextP
        curPe = nextPe;
        nextPe = 0;
        pb = curP;		// swap buffers
        curP = nextP;
        nextP = pb;

        // see if we're done with this priority level
        if (curPe == 0)
        {
          curT += priInc;	// increment priority threshold
          curPe = overPe;	// set current to overflow block
          overPe = 0;
          pb = curP;		// swap buffers
          curP = overP;
          overP = pb;
        }

        // check if we've hit the Start cell
        if (potarr[startCell] < POT_HIGH)
          break;

      }

      last_path_cost_ = potarr[startCell];

      ROS_DEBUG("[NavFn] Used %d cycles, %d cells visited (%d%%), priority buf max %d\n", 
          cycle,nc,(int)((nc*100.0)/(ns-nobs)),nwv);


      if (potarr[startCell] < POT_HIGH) return true; // finished up here
      else return false;
    }

  float NavFn::getLastPathCost(){
    return last_path_cost_;
  }

  //该函数负责在potarr数组的基础上选取一些cell点来生成最终的全局规划路径
  //从起点开始沿着最优行走代价值梯度下降的方向寻找到目标点的最优轨迹
  int NavFn::calcPath(int n, int *st){
    //savemap("test");

    //初始化路径数组(若其中有数据则重建)
    if (npathbuf < n){
      if (pathx) delete [] pathx;
      if (pathy) delete [] pathy;
      pathx = new float[n];
      pathy = new float[n];
      npathbuf = n;
    }

    //设置起始点，对于四点双线性插值 st总是在左上角
    if (st == NULL) st = start; //st指向起点(是指针)
    int stc = st[1]*nx + st[0]; //stc记录起点索引(是索引值)

    //设置偏移量(用于指示梯度下降的传播方向)
    float dx=0;
    float dy=0;
    //路径点索引
    npath = 0;

    //最多进行cycles次循环
    for (int i=0; i<n; i++){
      //计算下一个临近点的索引(根据dx、dy给出的方向)
      int nearest_point = std::max(0,std::min(nx*ny-1,stc+(int)round(dx)+(int)(nx*round(dy))));
      //如果下一个临近点的pot值小于COST_NEUTRAL(只有被初始化为0的目标点才有可能)则表示找到了目标点
      //并且我们发现，在用梯度下降法进行路径搜索时，是从起始点到目标点方向(不同于传播计算pot值时是从目标点到起始点的方向)
      if (potarr[nearest_point] < COST_NEUTRAL){
        pathx[npath] = (float)goal[0];
        pathy[npath] = (float)goal[1];
        return ++npath;
      }

      //如果到了第一行或最后一行，即超出边界
      if (stc < nx || stc > ns-nx){
        ROS_DEBUG("[PathCalc] Out of bounds");
        return 0;
      }

      //添加至路径点(dx、dy总是小于1的值，相当于就是指示方向)
      pathx[npath] = stc%nx + dx;
      pathy[npath] = stc/nx + dy;
      npath++;

      //震荡检测(若某一步和上上步的位置相同则认为在振荡)
      bool oscillation_detected = false;
      if( npath > 2 && pathx[npath-1] == pathx[npath-3] && pathy[npath-1] == pathy[npath-3] ){
        ROS_DEBUG("[PathCalc] oscillation detected, attempting fix.");
        oscillation_detected = true;
      }

      int stcnx = stc+nx; //当前点下方的点的索引
      int stcpx = stc-nx; //当前点上方的点的索引

      //检查当前到达节点及周边的8个节点是否有障碍物(或者上一步发现了震荡现象)
      //如果有的话，则直接将stc指向这8个节点中potential值最低的节点
      if (potarr[stc]     >= POT_HIGH ||  potarr[stc+1]   >= POT_HIGH ||
          potarr[stc-1]   >= POT_HIGH ||  potarr[stcnx]   >= POT_HIGH ||
          potarr[stcnx+1] >= POT_HIGH ||  potarr[stcnx-1] >= POT_HIGH ||
          potarr[stcpx]   >= POT_HIGH ||  potarr[stcpx+1] >= POT_HIGH ||
          potarr[stcpx-1] >= POT_HIGH ||  oscillation_detected){
        ROS_DEBUG("[Path] Pot fn boundary, following grid (%0.1f/%d)", potarr[stc], npath);
        int minc = stc;
        int minp = potarr[stc];
        int st = stcpx - 1;
        //寻找周围八个邻点的pot中的最小值
        if (potarr[st] < minp) {minp = potarr[st]; minc = st; }
        st++;
        if (potarr[st] < minp) {minp = potarr[st]; minc = st; }
        st++;
        if (potarr[st] < minp) {minp = potarr[st]; minc = st; }
        st = stc-1;
        if (potarr[st] < minp) {minp = potarr[st]; minc = st; }
        st = stc+1;
        if (potarr[st] < minp) {minp = potarr[st]; minc = st; }
        st = stcnx-1;
        if (potarr[st] < minp) {minp = potarr[st]; minc = st; }
        st++;
        if (potarr[st] < minp) {minp = potarr[st]; minc = st; }
        st++;
        if (potarr[st] < minp) {minp = potarr[st]; minc = st; }
        stc = minc;
        dx = 0;
        dy = 0;

        ROS_DEBUG("[Path] Pot: %0.1f  pos: %0.1f,%0.1f", potarr[stc], pathx[npath-1], pathy[npath-1]);

        if (potarr[stc] >= POT_HIGH){
          ROS_DEBUG("[PathCalc] No path found, high potential");
          //savemap("navfn_highpot");
          return 0;
        }
      }
      //当周围八个邻点没有障碍物时
      //如果有好的梯度，则直接计算梯度，并沿着梯度方向查找下一个节点
      else{
        //计算以下四个点的梯度值(用于插值得到当前的点的插值梯度)
        gradCell(stc);    //当前点
        gradCell(stc+1);  //当前点右侧点
        gradCell(stcnx);  //当前点下方点
        gradCell(stcnx+1);//当前点右下方点

        //获取插值梯度(不太理解这个计算方法的意义)
        float x1 = (1.0-dx)*gradx[stc] + dx*gradx[stc+1];
        float x2 = (1.0-dx)*gradx[stcnx] + dx*gradx[stcnx+1];
        float x = (1.0-dy)*x1 + dy*x2;
        float y1 = (1.0-dx)*grady[stc] + dx*grady[stc+1];
        float y2 = (1.0-dx)*grady[stcnx] + dx*grady[stcnx+1];
        float y = (1.0-dy)*y1 + dy*y2;

        //显示梯度
        ROS_DEBUG("[Path] %0.2f,%0.2f  %0.2f,%0.2f  %0.2f,%0.2f  %0.2f,%0.2f; final x=%.3f, y=%.3f\n", gradx[stc], grady[stc], gradx[stc+1], grady[stc+1], gradx[stcnx], grady[stcnx], gradx[stcnx+1], grady[stcnx+1], x, y);

        //检查梯度是否为0
        if (x == 0.0 && y == 0.0){
          ROS_DEBUG("[PathCalc] Zero gradient");	  
          return 0;
        }

        //向正确方向移动(pathStep设置移动的步长)
        //hypot函数返回给定数字的斜边(即sqrt(x^2+y^2))
        float ss = pathStep/hypot(x, y);
        dx += x*ss;
        dy += y*ss;

        //检查溢出
        if (dx > 1.0)   { stc++; dx -= 1.0; }
        if (dx < -1.0)  { stc--; dx += 1.0; }
        if (dy > 1.0)   { stc+=nx; dy -= 1.0; }
        if (dy < -1.0)  { stc-=nx; dy += 1.0; }
      }
    }
    ROS_DEBUG("[PathCalc] No path found, path too long");
    //超出了最大循环此时 返回失败
    return 0;
  }

  //计算一个cell处的梯度值 定义向右和向下为梯度的正方向
  float	NavFn::gradCell(int n){
    // check this cell？
    if (gradx[n]+grady[n] > 0.0)	
      return 1.0;			

    //如果到了第一行或最后一行，即超出边界
    if (n < nx || n > ns-nx)
      return 0.0;

    float cv = potarr[n];
    float dx = 0.0;
    float dy = 0.0;

    //如果当前cell是障碍区 
    //则如果四周点不是障碍区就直接给dx、dy赋最大值(COST_OBS=+-254)(右下为正方向)
    if (cv >= POT_HIGH){
      if (potarr[n-1] < POT_HIGH)
        dx = -COST_OBS;
      else if (potarr[n+1] < POT_HIGH)
        dx = COST_OBS;
      if (potarr[n-nx] < POT_HIGH)
        dy = -COST_OBS;
      else if (potarr[n+nx] < POT_HIGH)
        dy = COST_OBS;
    }
    //如果当前cell不是障碍区 
    //根据左右侧的梯度的平均值获得dx，根据上下方梯度平均值获得dy
    else{
      if (potarr[n-1] < POT_HIGH)
        dx += potarr[n-1]- cv;	
      if (potarr[n+1] < POT_HIGH)
        dx += cv - potarr[n+1]; 
      if (potarr[n-nx] < POT_HIGH)
        dy += potarr[n-nx]- cv;	
      if (potarr[n+nx] < POT_HIGH)
        dy += cv - potarr[n+nx]; 
    }

    //将dx、dy归一化后赋值给梯度数组
    float norm = hypot(dx, dy);
    if (norm > 0){
      norm = 1.0/norm;
      gradx[n] = norm*dx;
      grady[n] = norm*dy;
    }
    return norm;
  }

  //display function setup
  //<n> is the number of cycles to wait before displaying(use 0 to turn it off)
  void NavFn::display(void fn(NavFn *nav), int n){
    displayFn = fn;
    displayInt = n;
  }

  //debug writes(saves costmap and start/goal)
  void NavFn::savemap(const char *fname){
    char fn[4096];

    ROS_DEBUG("[NavFn] Saving costmap and start/goal points");
    // write start and goal points
    sprintf(fn,"%s.txt",fname);
    FILE *fp = fopen(fn,"w");
    if (!fp)
    {
      ROS_WARN("Can't open file %s", fn);
      return;
    }
    fprintf(fp,"Goal: %d %d\nStart: %d %d\n",goal[0],goal[1],start[0],start[1]);
    fclose(fp);

    // write cost array
    if (!costarr) return;
    sprintf(fn,"%s.pgm",fname);
    fp = fopen(fn,"wb");
    if (!fp)
    {
      ROS_WARN("Can't open file %s", fn);
      return;
    }
    fprintf(fp,"P5\n%d\n%d\n%d\n", nx, ny, 0xff);
    fwrite(costarr,1,nx*ny,fp);
    fclose(fp);
  }
};
