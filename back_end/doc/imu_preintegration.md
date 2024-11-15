## 1. 预积分问题引出

IMU预积分由正常的位姿定积分引出,表示为:
$$
\begin{equation}
    \begin{aligned}
    {}^w_{b_j}\mathbf{p} &= {}^w_{b_i}\mathbf{p} + {}^{w}_{b_i}\mathbf{v}\Delta{t} - \frac{1}{2} {}^w\mathbf{g} + {}^w_{bi}\mathbf{q}\iint_{t\in[i,j]}{({}^{b_i}_{b_t}\mathbf{q} {}^{b_t}\mathbf{a})\delta{t^2}}
\\  {}^{w}_{b_j}\mathbf{v} &= {}^{w}_{b_i}\mathbf{v} - {}^{w}\mathbf{g}\Delta{t} + {}^{w}_{b_i}\mathbf{q} \int_{t\in[i,j]}{{}^{b_i}_{b_t}\mathbf{q} {}^{b_t}\mathbf{a}\delta{t}}
\\  {}^{w}_{b_j}\mathbf{q} &= {}^{w}_{b_i}\mathbf{q} \otimes \int_{t\in[i,j]}{{}^{b_i}_{b_t}\mathbf{q} \otimes \begin{bmatrix}0 \\ \frac{1}{2} {}^{b_t}\mathbf{w}\end{bmatrix}\delta{t}}
    \end{aligned}
\end{equation}
$$




