## 1. 预积分问题引出

如果不考虑IMU和相机的数据同步问题(时间不同步使用插值法添加IMU数据), 相机采集数据时由于Camera帧率和IMU帧率不同,  在相机的$i$时刻到$j$时刻中间有很多IMU数据,将这些数据累计起来, 可以计算IMU姿态递推公式如下:
$$
\begin{array}{c} 
\mathbf{R}_j = \mathbf{R}_i \displaystyle \prod_{k=i}^{j-1} \operatorname{Exp}\left((\hat{\mathbf{w}}_k - \mathbf{b}_{g,k} - \eta_{g,k})\Delta{t}\right) 
\\ 
\mathbf{v}_j = \mathbf{v}_i + \mathbf{g}\Delta{t}_{ij} + \displaystyle\sum_{k=i}^{j-1}\mathbf{R}_k(\hat{\mathbf{a}}_k - \mathbf{b}_{a,k} - \eta_{a,k})\Delta{t} 
\\ 
\mathbf{p}_j = \mathbf{p}_i + \frac{1}{2} \mathbf{g} \Delta{t}^{2}_{ij}+ \displaystyle\sum_{k=i}^{j-1}\left[ \mathbf{v}_k \Delta{t}  + \frac{1}{2}\mathbf{R}_k(\hat{a}_k - \mathbf{b}_{a,k} - \eta_{a,k})\Delta{t}^2 \right]  \tag{1}
\end{array}
$$
对于这样一个多变量($\mathbf{R}_i\space ...$)优化问题, 如果前一个时刻($i$ 时刻)的位姿发生变化,后面时刻($j$ 时刻)的位姿需要重新计算, 这不是我们这样一个优化问题所希望看到的, 为了解决这个问题, 预积分的概念产生,  作者将直接积分变成一种不依赖$i$ 时刻和$j$ 时刻位姿的表达式, 这种表达将在优化中作为一条二元边归结到图优化.但是归结到图优化的预积分,我们会产生几个任务,我们先将这几个任务列出来,在每一小节进行详细描述:
1. 推导出一个不依赖 $i$ 时刻和 $j$ 时刻位姿的表达式(预积分结果)
2. 图优化中的信息矩阵的推导
3. $\mathbf{b}_{a,k}, \mathbf{b}_{g,k}$ 更新导致的预积分结果的更新
4. 在图优化中两段位姿之间有多种观测(融合IMU,相机), 这条二元边对两个节点($i$和$j$时刻状态变量)的更新(求解预积分结果对优化变量的雅克比)

## 2. 推导预积分结果
<!-- 公式简化, 令:
$$
\hat{\omega}_{k} = \hat{\mathbf{w}}_k - \mathbf{b}_{g,k} - \eta_{g,k}
\\
\hat{\alpha}_{k} = \hat{\mathbf{a}}_k - \mathbf{b}_{a,k} - \eta_{a,k}
\\ 
$$ 

则(1)式子简化为
$$
\begin{array}{c} 
\mathbf{R}_j = \mathbf{R}_i \displaystyle \prod_{k=i}^{j-1} \operatorname{Exp}\left(\hat{\omega}_{k}\Delta{t}\right) 
\\ 
\mathbf{v}_j = \mathbf{v}_i + \mathbf{g}\Delta{t}_{ij} + \displaystyle\sum_{k=i}^{j-1}\mathbf{R}_k\hat{\alpha}_{k}\Delta{t} 
\\ 
\mathbf{p}_j = \mathbf{p}_i + \displaystyle\sum_{k=i}^{j-1}\left[ \mathbf{v}_k \Delta{t} + \frac{1}{2} \mathbf{g} \Delta{t}^{2} + \frac{1}{2}\mathbf{R}_k\hat{\alpha}_{k}\Delta{t}^2 \right]  \tag{2}
\end{array}
$$ -->
预积分是一个相对概念, 表达式(1)是在世界坐标系下的, 如果我们优化位姿的时候优化了 $i$ 时刻位姿, 我们就需要重新计算这个积分结果, 但是如果我们将(1)式转换到 $i$ 系,比如产生一个 $\Delta{\mathbf{X}}$, 那么当 $i$ 系更新时我们就可以不用更新积分结果, 而是直接加上这个 $\Delta{\mathbf{X}}$多好. 接下来就是这段尝试
### 2.1. 旋转的预积分

$$
\Delta{\mathbf{R}}_{ij} \triangleq \mathbf{R}_i^{T}\mathbf{R}_j
 = \displaystyle \prod_{k=i}^{j-1} \operatorname{Exp}\left((\hat{\mathbf{w}}_k - \mathbf{b}_{g,k} - \eta_{g,k})\Delta{t}\right) 
$$
这个结果很好理解, 所以在第一次积分的时候,我们可以使用这个量暂时代替积分结果,但是强调一下这个结果其实就是将积分变量转换到$i$系, 所以如果想求 $j$ 系 我们可以直接使用 $\mathbf{R}_j = \mathbf{R}_i \Delta \mathbf{R}_{ij}$

### 2.2 速度的预积分
$$
\begin{aligned}
\Delta{\mathbf{v}_{ij}}  
    & \triangleq \mathbf{R}_i^T(\mathbf{v}_j - \mathbf{v}_i)
    \\& = \mathbf{R}_i^T\mathbf{g}\Delta{t}_{ij} + \displaystyle\sum_{k=i}^{j-1} \mathbf{R}_i^T \mathbf{R}_k(\hat{\mathbf{a}}_k - \mathbf{b}_{a,k} - \eta_{a,k})\Delta{t} 
    \\& = \mathbf{R}_i^T\mathbf{g}\Delta{t}_{ij} + \displaystyle\sum_{k=i}^{j-1}\Delta{\mathbf{R}_{ik}}(\hat{\mathbf{a}}_k - \mathbf{b}_{a,k} - \eta_{a,k})\Delta{t} 
\end{aligned}
$$
我们对它做一些处理,因为我们不希望一个固定在世界坐标下的$\mathbf{g}$(也许更新)出现在一个$i$系下, 所以我们废弃上式, 使用如下表达:
$$
\begin{aligned}
\Delta{\mathbf{v}_{ij}}  
    & \triangleq \mathbf{R}_i^T(\mathbf{v}_j - \mathbf{v}_i - \mathbf{g}\Delta{t_{ij}})
    \\& = \displaystyle\sum_{k=i}^{j-1}\Delta{\mathbf{R}_{ik}}(\hat{\mathbf{a}}_k - \mathbf{b}_{a,k} - \eta_{a,k})\Delta{t} 
\end{aligned}
$$
那么注意,这里不能简单使用 $\mathbf{v}_j = \mathbf{v}_i + \Delta{\mathbf{v}_{ij}}$这个公式
### 2.3 位置的预积分
$$
\begin{aligned} \Delta \mathbf{p}_{i j} & \triangleq \mathbf{R}_{i}^{T}\left(\mathbf{p}_{j}-\mathbf{p}_{i}-\mathbf{v}_{i} \cdot \Delta t_{i j}-\frac{1}{2} \mathbf{g} \cdot \Delta t_{i j}^{2}\right) \\ &=\sum_{k=i}^{j-1}\left[\Delta \mathbf{v}_{i k} \cdot \Delta t+\frac{1}{2} \Delta \mathbf{R}_{i k} \cdot\left(\hat{\mathbf{a}}_{k}-\mathbf{b}_{k}^{a}-\boldsymbol{\eta}_{k}^{a d}\right) \cdot \Delta t^{2}\right] \end{aligned}
$$
这里的$\mathbf{v}_k$和上式处理一样, 值得注意的是这三个量的更新顺序不能改变, 因为可以看到递进的依赖.


## 3. 信息矩阵的推导





