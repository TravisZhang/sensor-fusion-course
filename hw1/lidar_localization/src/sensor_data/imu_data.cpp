/*
 * @Description: imu data
 * @Author: Ren Qian
 * @Date: 2020-02-23 22:20:41
 */
#include "lidar_localization/sensor_data/imu_data.hpp"

#include <cmath>

#include "glog/logging.h"

namespace lidar_localization {
Eigen::Matrix3f IMUData::GetOrientationMatrix() {
  Eigen::Quaterniond q(orientation.w, orientation.x, orientation.y, orientation.z);
  Eigen::Matrix3f matrix = q.matrix().cast<float>();

  return matrix;
}

bool IMUData::SyncData(std::deque<IMUData>& UnsyncedData, std::deque<IMUData>& SyncedData, double sync_time) {
  // 传感器数据按时间序列排列，在传感器数据中为同步的时间点找到合适的时间位置
  // 即找到与同步时间相邻的左右两个数据
  // 需要注意的是，如果左右相邻数据有一个离同步时间差值比较大，则说明数据有丢失，时间离得太远不适合做差值
  LOG(INFO) << "[SyncData] imu unsync data size, " << UnsyncedData.size();
  while (UnsyncedData.size() >= 2) {
    if (UnsyncedData.front().time > sync_time) {
      LOG(INFO) << "[SyncData] data time too late, " << std::to_string(UnsyncedData.front().time).substr(0, 14);
      break;
    }
    if (UnsyncedData.at(1).time < sync_time) {
      LOG(INFO) << "[SyncData] 2nd data time too early, " << std::to_string(UnsyncedData.at(1).time).substr(0, 14);
      UnsyncedData.pop_front();
      if (UnsyncedData.size() > 1) {
        continue;
      } else {
        break;
      }
    }
    if (sync_time - UnsyncedData.front().time > 0.2) {
      LOG(INFO) << "[SyncData] front data time too early, " << std::to_string(UnsyncedData.front().time).substr(0, 14);
      UnsyncedData.pop_front();
      return false;
    }
    if (UnsyncedData.at(1).time - sync_time > 0.2) {
      LOG(INFO) << "[SyncData] 2nd data time too late, " << std::to_string(UnsyncedData.at(1).time).substr(0, 14);
      return false;
    }
    break;
  }
  if (UnsyncedData.size() < 1) {
    LOG(INFO) << "[SyncData] remain data not enough";
    return false;
  }

  IMUData front_data = UnsyncedData.at(0);
  IMUData back_data = UnsyncedData.back();
  if (UnsyncedData.size() > 1) {
    back_data = UnsyncedData.at(1);
  }
  IMUData synced_data;

  // LOG(INFO) << "[SyncData] imu front time, " << std::to_string(front_data.time).substr(0, 14) << " back time, "
  //           << std::to_string(back_data.time).substr(0, 14);

  double front_scale = (back_data.time - sync_time) / (back_data.time - front_data.time);
  double back_scale = (sync_time - front_data.time) / (back_data.time - front_data.time);
  front_scale = std::max(std::min(1.0, front_scale), 0.0);
  back_scale = std::max(std::min(1.0, back_scale), 0.0);

  synced_data.time = sync_time;
  synced_data.linear_acceleration.x =
      front_data.linear_acceleration.x * front_scale + back_data.linear_acceleration.x * back_scale;
  synced_data.linear_acceleration.y =
      front_data.linear_acceleration.y * front_scale + back_data.linear_acceleration.y * back_scale;
  synced_data.linear_acceleration.z =
      front_data.linear_acceleration.z * front_scale + back_data.linear_acceleration.z * back_scale;
  synced_data.angular_velocity.x =
      front_data.angular_velocity.x * front_scale + back_data.angular_velocity.x * back_scale;
  synced_data.angular_velocity.y =
      front_data.angular_velocity.y * front_scale + back_data.angular_velocity.y * back_scale;
  synced_data.angular_velocity.z =
      front_data.angular_velocity.z * front_scale + back_data.angular_velocity.z * back_scale;
  // 四元数插值有线性插值和球面插值，球面插值更准确，但是两个四元数差别不大是，二者精度相当
  // 由于是对相邻两时刻姿态插值，姿态差比较小，所以可以用线性插值
  synced_data.orientation.x = front_data.orientation.x * front_scale + back_data.orientation.x * back_scale;
  synced_data.orientation.y = front_data.orientation.y * front_scale + back_data.orientation.y * back_scale;
  synced_data.orientation.z = front_data.orientation.z * front_scale + back_data.orientation.z * back_scale;
  synced_data.orientation.w = front_data.orientation.w * front_scale + back_data.orientation.w * back_scale;
  // 线性插值之后要归一化
  synced_data.orientation.Normlize();

  SyncedData.push_back(synced_data);

  return true;
}
}  // namespace lidar_localization