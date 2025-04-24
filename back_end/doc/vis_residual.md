# 视觉残差的推导

$$
\begin{aligned}
\mathbf{r}_{\mathcal{C}}\left(\hat{\mathbf{z}}_{l}^{c_{j}}, \mathcal{X}\right) & = \left[\begin{array}{ll}
\mathbf{b}_{1} & \mathbf{b}_{2}
\end{array}\right]^{T} \cdot\left(\hat{\mathcal{P}}_{l}^{c_{j}}-\frac{\mathcal{P}_{l}^{c_{j}}}{\left\|\mathcal{P}_{l}^{c_{j}}\right\|}\right) \\
\hat{\mathcal{P}}_{l}^{c_{j}} & = \pi_{c}^{-1}\left(\left[\begin{array}{c}
\hat{u}_{l}^{c_{j}} \\
\hat{v}_{l}^{c_{j}}
\end{array}\right]\right) \\
\mathcal{P}_{l}^{c_{j}} & = \mathbf{R}_{b}^{c}\left(\mathbf { R } _ { w } ^ { b _ { j } } \left(\mathbf { R } _ { b _ { i } } ^ { w } \left(\mathbf{R}_{c}^{b} \frac{1}{\lambda_{l}} \pi_{c}^{-1}\left(\left[\begin{array}{c}
\hat{u}_{l}^{c_{i}} \\
\hat{v}_{l}^{c_{i}}
\end{array}\right]\right)
+\mathbf{p}_{c}^{b}\right)+\mathbf{p}_{b_{i}}^{w}-\mathbf{p}_{b_{j}}^{w}\right)-\mathbf{p}_{c}^{b}\right)
\end{aligned}
$$

其实主要是推导$\mathcal{P}_l^{c_j}$ 对各个变量的导数
为了好写便于计算，记各被求导变量为

$$
    R_1 \triangleq \mathbf{R}_{b_i}^{w}, p_1 \triangleq \mathbf{p}^{w}_{b_i} \\
    R_2 \triangleq \mathbf{R}_{b_j}^{w}, p_2 \triangleq \mathbf{p}^{w}_{b_j}\\
    R_3 \triangleq \mathbf{R}^{b}_{c}, p_3 \triangleq \mathbf{p}^{b}_{c}\\
    \pi =  \pi_{c}^{-1}\left(\left[\begin{array}{c}
                    \hat{u}_{l}^{c_{i}} \\
                    \hat{v}_{l}^{c_{i}}
                    \end{array}\right]\right)
$$

带入上式并展开，$\mathcal{P}_l^{c_j}$ 可以表示为：

$$
    \mathcal{P}_l^{c_j} = l^{-1} R_3^T R_2^T R_1 R_3 \pi + R_3^T R_2^T R_1 p_3 + R_3^T R_2^T p_1 - R_3^T R_2^T p_2 - R_3^T p_3 
$$

对$R_1$ 求导(右扰动)：

$$
\begin{aligned}
    右扰动 &= R_3^T R_2^T R_1 Exp(\delta{\theta}) (l^{-1} R_3 \pi + p_3 )\\
     &= R_3^T R_2^T R_1 (l^{-1} R_3 \pi + p_3 ) - R_3^T R_2^T R_1 (l^{-1} R_3 \pi + p_3)_{\times} \delta{\theta}\\
    \frac{\partial{\mathcal{P}_l^{c_j}}}{\partial{R_1}} &=  - R_3^T R_2^T R_1 (l^{-1} R_3 \pi + p_3)_{\times}
\end{aligned}
$$

对$p_1$ 求导：

$$
\frac{\partial{\mathcal{P}_l^{c_j}}}{\partial{p_1}} =  R_3^T R_2^T
$$

对$R_2$ 求导（右扰动）：

$$
\begin{aligned}
右扰动 &\stackrel{p=R_1(l_{-1} R_2 \pi + p_3) + p_ - p_2}{=} R_3^T (R_2 Exp(\delta{\theta}))^T  p \\
&= R_3^T Exp(-\delta{\theta}) R_2^T p \\
&= R_3^T R_2^T p + R_3^T [R_2^T p]_{\times} \delta{\theta}\\
 \frac{\partial{\mathcal{P}_l^{c_j}}}{\partial{R_2}} &= R_3^T [R_2^T (R_1(l_{-1} R_2 \pi + p_3) + p_1 - p_2)]_{\times}
\end{aligned}
$$

对$p_2$ 求导:

$$
  \frac{\partial{\mathcal{P}_l^{c_j}}}{\partial{p_2}}   = - R_3^T R_2^T
$$

对$R_3$ 求导：

$$
\mathcal{P}_l^{c_j} = l^{-1} R_3^T R_2^T R_1 R_3 \pi + R_3^T(R_2^T R_1 p_3 + R_2^T p_1 - R_2^T p_2 - p_3) \\ 
$$

对右侧部分求偏导很简单(右扰动)：

$$
右部分 = [R_3^T (R_2^T R_1 p_3 + R_2^T p_1 - R_2^T p_2 - p_3)]_{\times}
$$

对于左侧部分使用伴随公式 $ RExp(\theta)R^T = Exp(R\theta)$ 

$$
\begin{aligned}
左部分 &= l^{-1} Exp(R_3^T Log(R_2^T R_1)) \pi \\
右扰动 &= l^{-1} Exp(R_3^T Log(R_2^T R_1) + [R_3^T Log(R_2^T R_1)]_{\times}\delta{\theta}) \pi \\
&\overset{BCH}{=} l^{-1}Exp(R_3^T Log(R_2^T R_1)) Exp(J_r [R_3^T Log(R_2^T R_1)]_{\times}\delta{\theta}) \pi \\
&= l^{-1}Exp(R_3^T Log(R_2^T R_1)) \pi - l^{-1}Exp(R_3^T Log(R_2^T R_1)) \pi_{\times} (J_r R_3^T Log(R_2^T R_1)_{\times}\delta{\theta}) \\
 \frac{\partial{\mathcal{P}_l^{c_j}}}{\partial{R_3}} &= - l^{-1}Exp(R_3^T Log(R_2^T R_1)) \pi_{\times} J_r [R_3^T Log(R_2^T R_1)]_{\times} + [R_3^T (R_2^T R_1 p_3 + R_2^T p_2 - R_2^T p_1 - p_3)]_{\times}\\
\end{aligned}
$$

其中 $J_r \triangleq J_r(R_3^T Log(R_2^T R_1))$ 

对 $p_3$ 求导：

$$
\frac{\partial{\mathcal{P}_l^{c_j}}}{\partial{p_3}} =R_3^T R_2^T R_1 - R_3^T
$$

对 $l$ 求导：

$$
\frac{\partial{\mathcal{P}_l^{c_j}}}{\partial{l}} = - R_3^T R_2^T R_1 R_3 \pi \frac{1}{l^2}
$$

这里说一下为什么都用右扰动，由于我们是在一个流型空间上优化，ceres在优化流型的时候需要对Parameters进行定义在流型上的加法，如果不统一这种加法，就需要在ceres定义多个流型参数,因为之前在对IMU残差进行求导的时候用的是右扰动，所以这里也要用右扰动。
对于前面的部分求导结果是：

$$
\begin{equation}
  \begin{aligned}
    \frac{\partial{\mathbf{r}_{\mathcal{C}}}}{\partial{\mathcal{P}_l^{c_j}}} = -\left[\begin{array}{ll}\mathbf{b}_{1} & \mathbf{b}_{2}\end{array}\right]^{\rm T} \frac{1}{\|\mathcal{P}_l^{c_j}\|^2}(\|\mathcal{P}_l^{c_j}\| I  - \frac{\mathcal{P}_l^{c_j} (\mathcal{P}_l^{c_j})^T}{\|\mathcal{P}_l^{c_j}\|})
  \end{aligned}
\end{equation}
$$

## 关于 $\boldsymbol{\pi}_{-1}(\cdot)$：
$$
\begin{equation}
  \begin{aligned}
    
\frac{X}{Z} f_x + c_x = u \\
\frac{Y}{Z} f_y + c_y = v

  \end{aligned}
\end{equation}
$$

要求 $(\frac{X}{\sqrt{X^2 + Y^2 + Z^2}},\frac{Y}{\sqrt{X^2 + Y^2 + Z^2}}, \frac{Z}{\sqrt{X^2 + Y^2 + Z^2}})$, 由上式推出:
$$
\begin{equation}
  \begin{aligned}
A \triangleq \frac{X^2}{Z^2} = (\frac{u - c_x}{f_x})^2\\
B \triangleq \frac{Y^2}{Z^2} = (\frac{v-c_y}{f_y})^2
  \end{aligned}
\end{equation}
$$
因此所求变量为：
$$
\begin{equation}
  \begin{aligned}
\frac{X}{\sqrt{X^2 + Y^2 + Z^2}} = \pm \frac{1}{\sqrt{1 + \frac{B}{A} + \frac{1}{A}}} \\
\frac{Y}{\sqrt{X^2 + Y^2 + Z^2}} = \pm \frac{1}{\sqrt{1 + \frac{A}{B} + \frac{1}{B}}}\\
\frac{Z}{\sqrt{X^2 + Y^2 + Z^2}} = \frac{1}{\sqrt{1 + A + B}}
  \end{aligned}
\end{equation}
$$

其中当 $u-c_x > 0$ 时 第一个等式取正号，第二个等式同理


