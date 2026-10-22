// 다리 기구학 오도메트리 (KinematicsThread와 tools/fk_check가 공유).
#pragma once

#include <algorithm>
#include <array>

#include <Eigen/Dense>

#include "g1_joints.h"

namespace walking_kjw {

// 접촉 판정: 가장 낮은 발바닥 대비 이 높이 미만이면 접촉 (상세: 코드 해설 문서).
inline constexpr double kFootContactHeightM = 0.005;
// 오도메트리 XY 고정(anchor) 판정은 더 엄격하게: 발이 이 높이 안이면서 움직이는 구간(스윙 초·말)이
// 길면 anchor가 어긋난다 (threshold 2cm -> 3.1cm, 3mm -> 약 3mm 오차: tools/fk_check).
inline constexpr double kOdomPlantedHeightM = 0.003;

// 다리 기구학 오도메트리: 발-pelvis 상대 위치(월드 정렬)로 pelvis 월드 위치 추정.
// Z: 가장 낮은 발이 지면. XY: 접지 중인 발은 월드에서 고정이라는 가정 (초기 위치 대비 상대값,
// 미끄러짐/전환 오차는 누적 드리프트). 상세: 코드 해설 문서 01절.
class LegOdometry {
 public:
  void Reset() {
    pelvis_xy_.setZero();
    anchor_valid_ = {false, false};
  }

  // foot_rel: pelvis를 원점으로 둔 월드 정렬 발 위치 (좌, 우).
  Eigen::Vector3d Update(const std::array<Eigen::Vector3d, 2>& foot_rel,
                         std::array<bool, 2>* contact) {
    // 프레임 원점이 아니라 발바닥 높이 기준 (kFootSoleOffsetM).
    const double ground_z = std::min(foot_rel[0].z(), foot_rel[1].z()) - kFootSoleOffsetM;
    for (int i = 0; i < 2; ++i) {
      (*contact)[i] = foot_rel[i].z() - kFootSoleOffsetM - ground_z < kFootContactHeightM;
    }

    std::array<bool, 2> planted;
    for (int i = 0; i < 2; ++i) {
      planted[i] = foot_rel[i].z() - kFootSoleOffsetM - ground_z < kOdomPlantedHeightM;
    }

    Eigen::Vector2d sum = Eigen::Vector2d::Zero();
    int n = 0;
    for (int i = 0; i < 2; ++i) {
      if (planted[i] && anchor_valid_[i]) {
        sum += anchor_xy_[i] - foot_rel[i].head<2>();
        ++n;
      }
    }
    if (n > 0) pelvis_xy_ = sum / n;  // 지지발이 없으면 직전 추정 유지

    for (int i = 0; i < 2; ++i) {
      if (!planted[i]) {
        anchor_valid_[i] = false;
      } else if (!anchor_valid_[i]) {
        anchor_xy_[i] = pelvis_xy_ + foot_rel[i].head<2>();  // 새로 착지한 발을 현재 추정에 고정
        anchor_valid_[i] = true;
      }
    }
    return Eigen::Vector3d(pelvis_xy_.x(), pelvis_xy_.y(), -ground_z);
  }

 private:
  Eigen::Vector2d pelvis_xy_ = Eigen::Vector2d::Zero();
  std::array<Eigen::Vector2d, 2> anchor_xy_{Eigen::Vector2d::Zero(), Eigen::Vector2d::Zero()};
  std::array<bool, 2> anchor_valid_{false, false};
};

}  // namespace walking_kjw
