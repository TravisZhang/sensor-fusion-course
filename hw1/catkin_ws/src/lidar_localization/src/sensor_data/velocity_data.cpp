/*
 * @Description: velocity data
 * @Author: Ren Qian
 * @Date: 2020-02-23 22:20:41
 */
#include "lidar_localization/sensor_data/velocity_data.hpp"

#include "glog/logging.h"

namespace lidar_localization {
bool VelocityData::SyncData(std::deque<VelocityData>& UnsyncedData, std::deque<VelocityData>& SyncedData,
                            double sync_time) {
  // 传感器数据按时间序列排列，在传感器数据中为同步的时间点找到合适的时间位置
  // 即找到与同步时间相邻的左右两个数据
  // 需要注意的是，如果左右相邻数据有一个离同步时间差值比较大，则说明数据有丢失，时间离得太远不适合做差值
  LOG(INFO) << "[SyncData] vel unsync data size, " << UnsyncedData.size();
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

  VelocityData front_data = UnsyncedData.at(0);
  VelocityData back_data = UnsyncedData.back();
  if (UnsyncedData.size() > 1) {
    back_data = UnsyncedData.at(1);
  }
  VelocityData synced_data;

  // LOG(INFO) << "[SyncData] vel front time, " << std::to_string(front_data.time).substr(0, 14) << " back time, "
  //           << std::to_string(back_data.time).substr(0, 14);

  double front_scale = (back_data.time - sync_time) / (back_data.time - front_data.time);
  double back_scale = (sync_time - front_data.time) / (back_data.time - front_data.time);
  front_scale = std::max(std::min(1.0, front_scale), 0.0);
  back_scale = std::max(std::min(1.0, back_scale), 0.0);

  synced_data.time = sync_time;
  synced_data.linear_velocity.x = front_data.linear_velocity.x * front_scale + back_data.linear_velocity.x * back_scale;
  synced_data.linear_velocity.y = front_data.linear_velocity.y * front_scale + back_data.linear_velocity.y * back_scale;
  synced_data.linear_velocity.z = front_data.linear_velocity.z * front_scale + back_data.linear_velocity.z * back_scale;
  synced_data.angular_velocity.x =
      front_data.angular_velocity.x * front_scale + back_data.angular_velocity.x * back_scale;
  synced_data.angular_velocity.y =
      front_data.angular_velocity.y * front_scale + back_data.angular_velocity.y * back_scale;
  synced_data.angular_velocity.z =
      front_data.angular_velocity.z * front_scale + back_data.angular_velocity.z * back_scale;

  SyncedData.push_back(synced_data);

  return true;
}
}  // namespace lidar_localization