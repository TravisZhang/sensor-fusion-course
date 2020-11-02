/*
 * @Description: ICP 匹配模块
 * @Author: Chaoyu Zhang
 * @Date: 2020-10-26
 */
#ifndef LIDAR_LOCALIZATION_MODELS_REGISTRATION_ICP_REGISTRATION_HPP_
#define LIDAR_LOCALIZATION_MODELS_REGISTRATION_ICP_REGISTRATION_HPP_

#include "lidar_localization/models/registration/registration_interface.hpp"

namespace lidar_localization {
class ICPRegistration : public RegistrationInterface {
 public:
  ICPRegistration() = default;

  bool SetInputTarget(const CloudData::CLOUD_PTR& input_target) override;
  bool ScanMatch(const CloudData::CLOUD_PTR& input_source, const Eigen::Matrix4f& predict_pose,
                 CloudData::CLOUD_PTR& result_cloud_ptr, Eigen::Matrix4f& result_pose) override;
  bool ScanMatchPCL(const CloudData::CLOUD_PTR& input_source, const Eigen::Matrix4f& predict_pose,
                    CloudData::CLOUD_PTR& result_cloud_ptr, Eigen::Matrix4f& result_pose);

 private:
  CloudData::CLOUD_PTR target_cloud_;
};
}  // namespace lidar_localization

#endif