/*
 * @Description: ICP 匹配模块
 * @Author: Chaoyu Zhang
 * @Date: 2020-10-26
 */
#include "lidar_localization/models/registration/icp_registration.hpp"

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/icp.h>  //ICP(iterative closest point)配准

#include <cmath>

#include "glog/logging.h"

namespace lidar_localization {

bool ICPRegistration::SetInputTarget(const CloudData::CLOUD_PTR& input_target) {
  target_cloud_ = input_target;
  return true;
}

bool ICPRegistration::ScanMatch(const CloudData::CLOUD_PTR& input_source, const Eigen::Matrix4f& predict_pose,
                                CloudData::CLOUD_PTR& result_cloud_ptr, Eigen::Matrix4f& result_pose) {
  // lambda functions
  auto func_point_add = [](const auto& p0, auto& p1) {
    p1.x += p0.x;
    p1.y += p0.y;
    p1.z += p0.z;
  };

  auto func_point_divide = [](const double a, auto& p) {
    p.x /= a;
    p.y /= a;
    p.z /= a;
  };

  auto func_to_vector = [](const auto& point, auto& vec) {
    vec(0) = point.x;
    vec(1) = point.y;
    vec(2) = point.z;
  };
  auto func_to_point = [](const auto& vec, auto& point) {
    point.x = vec(0);
    point.y = vec(1);
    point.z = vec(2);
  };

  auto func_cloud_transform = [&func_to_vector, &func_to_point](const auto T, auto& points) {
    Eigen::Vector4f vec0;
    Eigen::Vector4f vec1;
    vec0(3) = 1.0f;
    for (auto& point : points) {
      func_to_vector(point, vec0);
      vec1 = T * vec0;
      func_to_point(vec1, point);
    }
  };

  auto func_pose_mat_transform = [](const Eigen::Matrix4f& T, auto& pose_mat) {
    Eigen::Matrix4f TR = Eigen::Matrix4f::Identity();
    TR.block<3, 3>(0, 0) = T.block<3, 3>(0, 0);
    Eigen::Matrix4f Tt = Eigen::Matrix4f::Identity();
    Tt.block<3, 1>(0, 3) = T.block<3, 1>(0, 3);
    pose_mat *= TR * Tt;
  };

  // set up kd binary tree
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud(target_cloud_);

  CloudData::CLOUD_PTR input_cloud(new CloudData::CLOUD());
  input_cloud->points.reserve(input_source->width * input_source->height);
  LOG(INFO) << "[ScanMatch] input size: " << input_source->width << " " << input_source->height << " "
            << input_source->points.size();

  // init result pose & step pose
  result_pose = predict_pose;
  Eigen::Matrix4f step_pose = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f step_pose_total = Eigen::Matrix4f::Identity();

  // main opt loop
  const int max_iters = 10;
  const float min_error = 10.0;
  float error = std::numeric_limits<float>::max();
  int iter = 0;
  while (iter < max_iters || error < min_error) {
    LOG(INFO) << "[ScanMatch] iter: " << iter;
    // restore input cloud to lidar frame
    input_cloud->points = input_source->points;

    // get correlated points
    CloudData::CLOUD_PTR correlated_cloud(new CloudData::CLOUD());
    correlated_cloud->points.reserve(input_source->width * input_source->height);
    CloudData::CLOUD_PTR valid_input_cloud(new CloudData::CLOUD());
    valid_input_cloud->points.reserve(input_source->width * input_source->height);

    // transform input cloud to world frame for first iter
    func_cloud_transform(result_pose, input_cloud->points);

    // radius search
    const double search_radius = 5.0;  // max deviation for 0.1s
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;
    CloudData::POINT mass_center_input, mass_center_correlated;
    for (const auto& point : input_cloud->points) {
      if (kdtree.radiusSearch(point, search_radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) {
        valid_input_cloud->points.emplace_back(point);
        correlated_cloud->points.emplace_back(target_cloud_->points[pointIdxRadiusSearch.front()]);
        // get mass center for input & correlated point cloud
        func_point_add(point, mass_center_input);
        func_point_add(target_cloud_->points[pointIdxRadiusSearch.front()], mass_center_correlated);
        pointIdxRadiusSearch.clear();  // maybe unecessary
      }
    }
    if (valid_input_cloud->points.empty()) {
      LOG(INFO) << "[ScanMatch] valid input cloud empty, ScanMatch failed !!!";
      return false;
    }
    LOG(INFO) << "[ScanMatch] valid input cloud size: " << valid_input_cloud->points.size();
    LOG(INFO) << "[ScanMatch] mass center input: " << mass_center_input.x << " " << mass_center_input.y << " "
              << mass_center_input.z;
    LOG(INFO) << "[ScanMatch] mass center correlated: " << mass_center_correlated.x << " " << mass_center_correlated.y
              << " " << mass_center_correlated.z;
    double valid_num = static_cast<double>(valid_input_cloud->points.size());
    func_point_divide(valid_num, mass_center_input);
    func_point_divide(valid_num, mass_center_correlated);
    LOG(INFO) << "[ScanMatch] mass center input: " << mass_center_input.x << " " << mass_center_input.y << " "
              << mass_center_input.z;
    LOG(INFO) << "[ScanMatch] mass center correlated: " << mass_center_correlated.x << " " << mass_center_correlated.y
              << " " << mass_center_correlated.z;

    // get rid of mean value
    auto func_remove_mean = [](const auto& mass_center, auto& points) {
      for (auto& point : points) {
        point.x -= mass_center.x;
        point.y -= mass_center.y;
        point.z -= mass_center.z;
      }
    };
    func_remove_mean(mass_center_input, valid_input_cloud->points);
    func_remove_mean(mass_center_correlated, correlated_cloud->points);

    // get H matrix (3 by 3)
    Eigen::Matrix3f H = Eigen::Matrix3f::Zero();
    for (size_t i = 0; i < valid_input_cloud->points.size(); ++i) {
      Eigen::Vector3f vec_input, vec_correlated;
      func_to_vector(valid_input_cloud->points[i], vec_input);
      func_to_vector(correlated_cloud->points[i], vec_correlated);
      H += vec_input * vec_correlated.transpose();
    }
    // LOG(INFO) << "[ScanMatch] H matrix: " << H;
    std::cout << "[ScanMatch] H matrix: " << std::endl << H << std::endl;

    // do SVD decomposition for H, R = V*U^T
    Eigen::JacobiSVD<Eigen::Matrix3f> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
    // deal with singularity
    Eigen::Matrix3f M = Eigen::Matrix3f::Identity();
    float det = svd.matrixV().determinant() * svd.matrixU().determinant();
    // if (det < 0) {
    M(2, 2) = det;
    // }
    Eigen::Matrix3f R = svd.matrixV() * M * svd.matrixU().transpose();
    std::cout << "[ScanMatch] R matrix: " << std::endl << R << std::endl;

    // get translation t = mean_corr - R * mean_input;
    Eigen::Vector3f vec_mc_input, vec_mc_correlated, vec_trans;
    func_to_vector(mass_center_input, vec_mc_input);
    func_to_vector(mass_center_correlated, vec_mc_correlated);
    vec_trans = vec_mc_correlated - R * vec_mc_input;
    std::cout << "[ScanMatch] vec trans: " << std::endl << vec_trans << std::endl;

    step_pose.block<3, 1>(0, 3) = vec_trans;
    step_pose.block<3, 3>(0, 0) = R;

    ++iter;

    // transform input cloud to better position estimate for other iters
    func_pose_mat_transform(step_pose, result_pose);

    // get total step pose
    func_pose_mat_transform(step_pose, step_pose_total);

    input_cloud->points.clear();
  }

  input_cloud->points = input_source->points;
  func_cloud_transform(result_pose, input_cloud->points);
  result_cloud_ptr = input_cloud;

  // for comparison
  // ScanMatchPCL(input_source, predict_pose, result_cloud_ptr, result_pose);

  return true;
}

bool ICPRegistration::ScanMatchPCL(const CloudData::CLOUD_PTR& input_source, const Eigen::Matrix4f& predict_pose,
                                   CloudData::CLOUD_PTR& result_cloud_ptr, Eigen::Matrix4f& result_pose) {
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>::Ptr icp_ptr(
      new pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>());
  icp_ptr->setInputCloud(input_source);
  icp_ptr->setInputTarget(target_cloud_);  // x_target = R * x_input + t
  // scan match and save result to result_cloud_ptr
  icp_ptr->align(*result_cloud_ptr, predict_pose);
  LOG(INFO) << "[ScanMatchPCL] icp has converged:" << icp_ptr->hasConverged()
            << " score: " << icp_ptr->getFitnessScore() << std::endl;
  result_pose = icp_ptr->getFinalTransformation();
  std::cout << result_pose << std::endl;
  // std::cout << "end of icp";
  return true;
}

}  // namespace lidar_localization