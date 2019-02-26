#include <glog/logging.h>

#include <aslam/calibration/SensorId.h>
#include <sm/kinematics/quaternion_algebra.hpp>

#include <aslam/calibration/test/MockMotionCaptureSource.h>

namespace aslam {
namespace calibration {
namespace test {

constexpr Timestamp MockMotionCaptureSource::StartTime;

MockMotionCaptureSource::~MockMotionCaptureSource() {
}

MotionCaptureSource::PoseStamped MockMotionCaptureSource::getPoseAt(Timestamp at) const {
  PoseStamped p;
  p.time = at;
  func(at, p);
  return p;
}

std::vector<MotionCaptureSource::PoseStamped> MockMotionCaptureSource::getPoses(Timestamp from, Timestamp till) const {
  Timestamp inc(1e-2);
  std::vector<PoseStamped> poses;
  for(auto t = from; t <= till + inc; t += inc){
    poses.emplace_back(getPoseAt(t));
  }
  return poses;
}

MockMotionCaptureSource MmcsStraightLine([](Timestamp now, MotionCaptureSource::PoseStamped & p){
  p.q = sm::kinematics::quatIdentity();
  p.p = Eigen::Vector3d::UnitX() * (now - MockMotionCaptureSource::StartTime);
});

MockMotionCaptureSource MmcsRotatingStraightLine([](Timestamp now, MotionCaptureSource::PoseStamped & p){
  const double deltaTime = now - MockMotionCaptureSource::StartTime;
  p.q = sm::kinematics::axisAngle2quat({-deltaTime, 0, 0}); // to passive quaternion yielding a positive rotation
  p.p = Eigen::Vector3d::UnitX() * deltaTime;
});

MockMotionCaptureSource MmcsCircle([](Timestamp now, MotionCaptureSource::PoseStamped & p){
  const double deltaTime = now - MockMotionCaptureSource::StartTime;
  const double angleRad = deltaTime;
  p.q = sm::kinematics::axisAngle2quat({0, 0, -(angleRad + M_PI / 2)}); // to passive quaternion yielding a positive rotation
  p.p = Eigen::Vector3d::UnitX() * cos(angleRad) + Eigen::Vector3d::UnitY() * sin(angleRad);
});

MockMotionCaptureSource MmcsEight([](Timestamp now, MotionCaptureSource::PoseStamped & p){
  const double deltaTime = now - MockMotionCaptureSource::StartTime;
  double angleRad = deltaTime;

  double xOffset;
  const int phase = int(floor(angleRad / M_PI)) % 4;

  if (phase == 1 || phase == 2) {  // left circle
    xOffset = -2;
    angleRad = - (angleRad - M_PI);
  } else { // right circle
    xOffset = 0;
  }
  //LOG(WARNING)<<"aabbcc"<<sm::kinematics::axisAngle2quat({0, 0, (angleRad + M_PI / 2)});
  p.q = sm::kinematics::axisAngle2quat({0, 0, (angleRad + M_PI / 2)}); // to passive quaternion yielding a positive rotation //Is this - correct here (I think should be +)
  p.p = Eigen::Vector3d::UnitX() * (cos(angleRad) + xOffset) + Eigen::Vector3d::UnitY() * sin(angleRad);
});

} /* namespace test */
} /* namespace calibration */
} /* namespace aslam */

