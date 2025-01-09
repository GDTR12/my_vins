# 关于Ceres里的流型

```cpp
class CERES_EXPORT Manifold {
 public:
  virtual ~Manifold();

  virtual int AmbientSize() const = 0;

  virtual int TangentSize() const = 0;

  virtual bool Plus(const double* x,
                    const double* delta,
                    double* x_plus_delta) const = 0;

  virtual bool PlusJacobian(const double* x, double* jacobian) const = 0;

  virtual bool RightMultiplyByPlusJacobian(const double* x,
                                           const int num_rows,
                                           const double* ambient_matrix,
                                           double* tangent_matrix) const;

  virtual bool Minus(const double* y,
                     const double* x,
                     double* y_minus_x) const = 0;

  virtual bool MinusJacobian(const double* x, double* jacobian) const = 0;
};
```

对于个人使用者，如果要使用流型，流型上的`Plus, Minus`是一定要定义的。 如果要使用自动求导，那么需要对 `PlusJacobian, MinuxJacobian`这些函数进行定义。

对于ceres，流型的加减法被定义为扰动，具体怎么扰动看个人习惯。我们先来分析一下ceres::QuaternionMainfold这个类的实现，可以定位到文件mainfold.cc：

```cpp
template <typename Order>
inline void QuaternionPlusImpl(const double* x,
                               const double* delta,
                               double* x_plus_delta) {
  // x_plus_delta = QuaternionProduct(q_delta, x), where q_delta is the
  // quaternion constructed from delta.
  const double norm_delta = std::hypot(delta[0], delta[1], delta[2]);

  if (std::fpclassify(norm_delta) == FP_ZERO) {
    // No change in rotation: return the quaternion as is.
    std::copy_n(x, 4, x_plus_delta);
    return;
  }
const double sin_delta_by_delta = (std::sin(norm_delta) / norm_delta);
  double q_delta[4];
  q_delta[Order::kW] = std::cos(norm_delta);
  q_delta[Order::kX] = sin_delta_by_delta * delta[0];
  q_delta[Order::kY] = sin_delta_by_delta * delta[1];
  q_delta[Order::kZ] = sin_delta_by_delta * delta[2];

  x_plus_delta[Order::kW] =
      q_delta[Order::kW] * x[Order::kW] - q_delta[Order::kX] * x[Order::kX] -
      q_delta[Order::kY] * x[Order::kY] - q_delta[Order::kZ] * x[Order::kZ];
  x_plus_delta[Order::kX] =
      q_delta[Order::kW] * x[Order::kX] + q_delta[Order::kX] * x[Order::kW] +
      q_delta[Order::kY] * x[Order::kZ] - q_delta[Order::kZ] * x[Order::kY];
  x_plus_delta[Order::kY] =
      q_delta[Order::kW] * x[Order::kY] - q_delta[Order::kX] * x[Order::kZ] +
      q_delta[Order::kY] * x[Order::kW] + q_delta[Order::kZ] * x[Order::kX];
  x_plus_delta[Order::kZ] =
      q_delta[Order::kW] * x[Order::kZ] + q_delta[Order::kX] * x[Order::kY] -
      q_delta[Order::kY] * x[Order::kX] + q_delta[Order::kZ] * x[Order::kW];
}

template <typename Order>
inline void QuaternionPlusJacobianImpl(const double* x, double* jacobian_ptr) {
  Eigen::Map<Eigen::Matrix<double, 4, 3, Eigen::RowMajor>> jacobian(
      jacobian_ptr);

  jacobian(Order::kW, 0) = -x[Order::kX];
  jacobian(Order::kW, 1) = -x[Order::kY];
  jacobian(Order::kW, 2) = -x[Order::kZ];
  jacobian(Order::kX, 0) = x[Order::kW];
  jacobian(Order::kX, 1) = x[Order::kZ];
  jacobian(Order::kX, 2) = -x[Order::kY];
  jacobian(Order::kY, 0) = -x[Order::kZ];
  jacobian(Order::kY, 1) = x[Order::kW];
  jacobian(Order::kY, 2) = x[Order::kX];
  jacobian(Order::kZ, 0) = x[Order::kY];
  jacobian(Order::kZ, 1) = -x[Order::kX];
  jacobian(Order::kZ, 2) = x[Order::kW];
}
```
可以分析这个函数`QuaternionPlusImpl`,这里对四元数的扰动为左扰动，这里的加法就是这个过程：
$$
左扰动 = \begin{bmatrix}1 \\ \delta{\boldsymbol{\theta}} \end{bmatrix} \otimes \mathbf{q}
$$

对于`QuaternionPlusJacobianImpl`这个函数，其实就是对这个四元数$\mathbf{q} = [s, \mathbf{v}^{T}]$的求导（使用上面说的左扰动），这里的求导过程是这样：
$$
\begin{aligned}
    \frac{\partial{\mathbf{q}}}{\partial{\delta{\boldsymbol{\theta}}}} &= \lim
    \frac{\begin{bmatrix}1 \\ \delta{\boldsymbol{\theta}} \end{bmatrix} \otimes \mathbf{q} - \begin{bmatrix}1 \\ \mathbf{0} \end{bmatrix} \otimes \mathbf{q}}{\delta{\boldsymbol{\theta}}} \\
    &= \lim{\frac{\mathcal{R}(\mathbf{q})\begin{bmatrix}0 \\ \delta{\boldsymbol{\theta}} \end{bmatrix}}{\delta{\boldsymbol{\theta}}}}\\
    &= \mathcal{R}(\mathbf{q})_{[:, 1:3]} \\
    &= s \mathbf{I} - [\mathbf{v}]_{\times}
\end{aligned}
$$


