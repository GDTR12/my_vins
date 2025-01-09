#pragma once

#include <deque>
#include <map>
#include <unordered_map>
#include <vector>

#include <math.h>
#include <Eigen/Dense>
#include <sophus/so3.hpp>
#include <ceres/ceres.h>

namespace MathUtils {

template <typename T>
using aligned_vector = std::vector<T, Eigen::aligned_allocator<T>>;

template <typename T>
using aligned_deque = std::deque<T, Eigen::aligned_allocator<T>>;

template <typename K, typename V>
using aligned_map = std::map<K, V, std::less<K>,
                             Eigen::aligned_allocator<std::pair<K const, V>>>;

template <typename K, typename V>
using aligned_unordered_map =
    std::unordered_map<K, V, std::hash<K>, std::equal_to<K>,
                       Eigen::aligned_allocator<std::pair<K const, V>>>;

/** sorts vectors from large to small
 * vec: vector to be sorted
 * sorted_vec: sorted results
 * ind: the position of each element in the sort result in the original vector
 * https://www.programmersought.com/article/343692646/
 */
inline void sort_vec(const Eigen::Vector3d& vec, Eigen::Vector3d& sorted_vec,
                     Eigen::Vector3i& ind) {
  ind = Eigen::Vector3i::LinSpaced(vec.size(), 0, vec.size() - 1);  //[0 1 2]
  auto rule = [vec](int i, int j) -> bool {
    return vec(i) > vec(j);
  };  // regular expression, as a predicate of sort

  std::sort(ind.data(), ind.data() + ind.size(), rule);

  // The data member function returns a pointer to the first element of
  // VectorXd, similar to begin()
  for (int i = 0; i < vec.size(); i++) {
    sorted_vec(i) = vec(ind(i));
  }
}

template<typename Scalar = float>
Scalar getSO3Distance(const Sophus::SO3<Scalar> p0, const Sophus::SO3<Scalar> p1){
    Eigen::Vector3f p0_qua = p0.unit_quaternion().vec();
    Eigen::Vector3f p1_qua = p1.unit_quaternion().vec();
    return (p0_qua - p1_qua).norm();
}


// template<typename Scalar = float>
// Scalar getRd3Distance(const So)
// {
// }
    // bool getCrossPoint(Eigen::Vector4f& plane, Eigen::)
template<typename Scalar>
bool calIntersection(const Eigen::Matrix<Scalar, 4, 1>& plane1_coeff, const Eigen::Matrix<Scalar, 4, 1>& plane2_coeff, Eigen::Matrix<Scalar, 3, 1>& line_dir, Eigen::Matrix<Scalar, 3, 1>& line_point) {
    Eigen::Matrix<Scalar, 3, 1> n1, n2, u;
    float d1, d2, d3;
    Eigen::Matrix<Scalar, 3, 3> A;
    n1 = plane1_coeff.block(0, 0, 3, 1);
    n2 = plane2_coeff.block(0, 0, 3, 1);
    if (n1.cross(n2).norm() < 1e-9)return false;

    u = n1.cross(n2);
    d1 = plane1_coeff[3];
    d2 = plane2_coeff[3];
    d3 = 0;
    A << n1, n2, u;
    line_dir = u;
    line_point = (-d1 * n2.cross(u) - d2 * u.cross(n1) - d3 * n1.cross(n2)) / A.determinant();
    return true;
}

template<typename Scalar>
bool LineRayToPlanePnt(Eigen::Matrix<Scalar, 3, 1>& o_orign, Eigen::Matrix<Scalar, 3, 1>& o_dir, Eigen::Matrix<Scalar, 4, 1>& fn, Eigen::Matrix<Scalar, 3, 1>& inter_pnt)
{
  Eigen::Matrix<Scalar, 3, 1> N = Eigen::Matrix<Scalar, 3, 1>(fn[0], fn[1], fn[2]);
  float D = fn[3];
  if (std::abs(o_dir.dot(N)) < 1e-8)
  {
    return false;
  }
  float t = -(o_orign.dot(N) + D) / (o_dir.dot(N));
  inter_pnt = o_orign + t*o_dir;
  return true;
}

template<typename Scalar>
Eigen::Matrix<Scalar, 4, 4> quaLeftMultiMat(
        const Eigen::Quaternion<Scalar>& qua){
    Eigen::Matrix<Scalar, 4, 4> mat = Eigen::Matrix<Scalar, 4, 4>::Zero();   
    // std::cout << qua.w() << " " << qua.vec().transpose() << std::endl;
    mat.block(0,1,1,3) = -qua.vec().transpose();
    mat.block(1,0,3,1) = qua.vec();
    mat.block(1,1,3,3) = (qua.w() * Eigen::Matrix<Scalar, 3, 3>::Identity()) + Sophus::SO3<Scalar>::hat(qua.vec()); 
    mat(0,0) = qua.w();
    return mat;
}

template<typename Scalar>
Eigen::Matrix<Scalar, 4, 4> quaRightMultiMat(
        const Eigen::Quaternion<Scalar>& qua){
    Eigen::Matrix<Scalar, 4, 4> mat = Eigen::Matrix<Scalar, 4, 4>::Zero();   
    mat.block(0,1,1,3) = -qua.vec().transpose();
    mat.block(1,0,3,1) = qua.vec();
    mat.block(1,1,3,3) = -Sophus::SO3<Scalar>::hat(qua.vec()) + qua.w() * Eigen::Matrix<Scalar,3,3>::Identity(); 
    mat(0,0) = qua.w();
    return mat;
}
template<typename Scalar>
Scalar rand_float(Scalar min, Scalar max){
    std::random_device rd;
    std::mt19937 gen(rd());  // Mersenne Twister 引擎
    std::uniform_real_distribution<> dis(min, max);  // 0.0到1.0之间的均匀分布
    return dis(gen);
}

template<typename Scalar>
Eigen::Matrix<Scalar, 4, 1> QuaSubtraction(const Eigen::Quaternion<Scalar>& qua0, const Eigen::Quaternion<Scalar>& qua1)
{
  Eigen::Matrix<Scalar, 4, 1> ret;
  ret << qua0.w() - qua1.w(),
         qua0.x() - qua1.x(),
         qua0.y() - qua1.y(),
         qua0.z() - qua1.z();
  return ret;
}

template<typename Scalar = double>
Eigen::Quaternion<Scalar> quaExp(Eigen::Quaternion<Scalar> q, Scalar t){

    Scalar theta = acos(q.w());
    Eigen::Matrix<Scalar, 3, 1> n = q.vec() / sin(theta);
    Scalar alpha = t * theta;
    return Eigen::Quaternion<Scalar>(cos(alpha), n.x() * sin(alpha), n.y() * sin(alpha), n.z() * sin(alpha));
}

Eigen::Quaterniond deltaQua(const Eigen::Vector3d& delata_theta)
{
  return Eigen::Quaterniond(1, delata_theta.x(), delata_theta.y(), delata_theta.z());
}

class PoseEigenQuaRightPerturbManifold : public ceres::Manifold
{
public:
    PoseEigenQuaRightPerturbManifold(){};
    ~PoseEigenQuaRightPerturbManifold(){};

    virtual bool Plus(const double* x, const double* delta, double* x_plus_delta) const override
    {
      Eigen::Map<const Eigen::Vector3d> p(x);
      Eigen::Map<const Eigen::Quaterniond> q(x + 3);
      Eigen::Map<const Eigen::Vector3d> dp(delta);
      Eigen::Map<const Eigen::Vector3d> dq(delta + 3);
      Eigen::Map<Eigen::Vector3d> rx(x_plus_delta);
      Eigen::Map<Eigen::Quaterniond> rq(x_plus_delta + 3);

      rx = p + dp;
      rq = (q * deltaQua(dq)).normalized();
      return true;
    }

    virtual bool Minus(const double* x, const double* y, double* delta) const  override
    {
      Eigen::Map<const Eigen::Vector3d> px(x);
      Eigen::Map<const Eigen::Quaterniond> qx(x+3);
      Eigen::Map<const Eigen::Vector3d> py(y);
      Eigen::Map<const Eigen::Quaterniond> qy(y+3);
      Eigen::Map<Eigen::Vector3d> dp(delta);
      Eigen::Map<Eigen::Vector3d> dq(delta);

      dp = px - py;      
      Eigen::Quaterniond dq_ = qy.inverse() * qx;
      dq_.normalize();
      dq = dq_.vec();
      return true;
    }

    virtual bool PlusJacobian(const double* x, double* jacobian) const 
    {
      Eigen::Map<Eigen::Matrix<double, 7, 6, Eigen::RowMajor>> jac(jacobian);
      Eigen::Map<const Eigen::Vector3d> p(x);
      Eigen::Map<const Eigen::Quaterniond> q(x+3);
      jac.setZero();
      jac.block<6,6>(0,0).setIdentity();
      return true;
    }

    virtual bool MinusJacobian(const double* x, double* jacobian) const
    {
      Eigen::Map<Eigen::Matrix<double, 6, 7, Eigen::RowMajor>> jac(jacobian);
      Eigen::Map<const Eigen::Vector3d> p(x);
      Eigen::Map<const Eigen::Quaterniond> q(x+3);
      jac.setZero();
      jac.block<6,6>(0,0).setIdentity();
      return true;
    }

    int AmbientSize() const override { return 7; }
    int TangentSize() const override { return 6; }

private:

};

} 
