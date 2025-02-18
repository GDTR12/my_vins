message(STATUS "Conan: Using CMakeDeps conandeps_legacy.cmake aggregator via include()")
message(STATUS "Conan: It is recommended to use explicit find_package() per dependency instead")

find_package(Ceres)
find_package(GTSAM)
find_package(Sophus)
find_package(fmt)

set(CONANDEPS_LEGACY  Ceres::ceres  gtsam::gtsam  Sophus::Sophus  fmt::fmt )