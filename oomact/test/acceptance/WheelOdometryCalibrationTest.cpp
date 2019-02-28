#include <memory>

#include <gtest/gtest.h>
#include <glog/logging.h>

#include <aslam/calibration/calibrator/CalibratorI.h>
#include <aslam/calibration/data/AccelerometerMeasurement.h>
#include <aslam/calibration/data/GyroscopeMeasurement.h>
#include <aslam/calibration/model/FrameGraphModel.h>
#include <aslam/calibration/model/PoseTrajectory.h>
#include <aslam/calibration/model/sensors/WheelOdometry.h>
#include <aslam/calibration/model/sensors/PoseSensor.h>
#include <aslam/calibration/test/MockMotionCaptureSource.h>
#include <aslam/calibration/tools/SmartPointerTools.h>
#include <sm/kinematics/Transformation.hpp>


using namespace aslam::calibration;
using namespace aslam::calibration::test;

/** This tests extrinsic calibration for wheel and pose sensor using a straight line
 * as trajectory
 * Note that right now initial and final values are identical (I'm not sure whats
 * observable from a straight line though either) otherwise fails test
 */
TEST(CalibrationTestSuite, testWheelOdometryCalibrationStaight) {
  auto vs = ValueStoreRef::fromFile("acceptance/wheelOdometry-pose.info");

  FrameGraphModel m(vs.getChild("model"));
  PoseSensor psA(m, "pose");
  WheelOdometry wheelOdometry(m, "wheelOdometry");
  PoseTrajectory traj(m, "traj");
  m.addModulesAndInit(psA, wheelOdometry, traj);

  //wheelOdometry.getTranslationVariable().set({0., 1., 0.});
  //const double rotUpdate[] = {0., 0.0, 0.1};
  //wheelOdometry.getRotationVariable().update(rotUpdate, 3);
  //I dont think this can be estimated of a straight line try for eight instead
  //diffdrive constraint als error term

  auto spModel = aslam::to_local_shared_ptr(m);
  auto c = createBatchCalibrator(vs.getChild("calibrator"), spModel);

  constexpr double l = 0.2; // base length (unused just for info, remove?)
  constexpr double d = 0.1; // wheel diameters //this is actually the radius

  for (auto& p : MmcsRotatingStraightLine.getPoses(4.0 * M_PI)) {
    psA.addMeasurement(p.time, p.q, p.p, c->getCurrentStorage());
    c->addMeasurementTimestamp(p.time, psA);

    // 1 m/s robot velocity (straight line)
    const double transVel = 1.0;
    const double rW = transVel / d;
    const double lW = transVel / d;
    wheelOdometry.addMeasurement(*c, p.time, { lW, rW });
    c->addMeasurementTimestamp(p.time, wheelOdometry);
  }

  //EXPECT_NEAR(1, wheelOdometry.getTranslationToParent()[1], 0.0001);
  //EXPECT_NEAR(0.05, wheelOdometry.getRotationQuaternionToParent()[1], 0.01);
  //I dont think this can be estimated of a straight line try for eight instead
  EXPECT_NEAR(0.13, wheelOdometry.getWheelRadiusL()->getValue(), 0.001);
  c->calibrate();
  EXPECT_NEAR(0, wheelOdometry.getTranslationToParent()[1], 0.0001);
  EXPECT_NEAR(0, wheelOdometry.getRotationQuaternionToParent()[2], 0.01);
  EXPECT_NEAR(d, wheelOdometry.getWheelRadiusL()->getValue(), 0.001);
}

/** This tests extrinsic calibration for wheel and pose sensor using an eight
 * as trajectory
 * Note that right now initial and final values are identical (I'm not sure whats
 * observable from an eight though either) otherwise fails test.
 */
TEST(CalibrationTestSuite, testWheelOdometryCalibrationEight) {
  auto vs = ValueStoreRef::fromFile("acceptance/wheelOdometry-pose.info");

  FrameGraphModel m(vs.getChild("model"));
  PoseSensor psA(m, "pose");
  WheelOdometry wheelOdometry(m, "wheelOdometry");
  PoseTrajectory traj(m, "traj");
  m.addModulesAndInit(psA, wheelOdometry, traj);

  //wheelOdometry.getTranslationVariable().set({0., 1., 0.});
  //const double rotUpdate[] = {0., 0.1, 0.};
  //wheelOdometry.getRotationVariable().update(rotUpdate, 3);

  auto spModel = aslam::to_local_shared_ptr(m);
  auto c = createBatchCalibrator(vs.getChild("calibrator"), spModel);

  constexpr double l = 0.2; // base length 
  constexpr double d = 0.13; // wheel diameters

  for (auto& p : MmcsEight.getPoses(4.0 * M_PI)) {
    psA.addMeasurement(p.time, p.q, p.p, c->getCurrentStorage());
    c->addMeasurementTimestamp(p.time, psA);

    // 1 rad/s rotation (right circle positively, left  (x < -1) circle negatively)
    const double omega = p.p[0] < -1.0 ? -1 : 1;
    const double transVel = 1.0; //speed is defined as 1, therefore we need measurements until 4*M_PI as this is length of eight
    const double omegaL = omega * l;
    const double lW = (transVel - omegaL) / d;
    const double rW = (transVel + omegaL) / d;
    wheelOdometry.addMeasurement(*c, p.time, { lW, rW });
    c->addMeasurementTimestamp(p.time, wheelOdometry);
  }

  //EXPECT_NEAR(1, wheelOdometry.getTranslationToParent()[1], 0.0001);
  //EXPECT_NEAR(0.05, wheelOdometry.getRotationQuaternionToParent()[1], 0.01);
  c->calibrate();
  EXPECT_NEAR(0, wheelOdometry.getTranslationToParent()[1], 0.0001);
  EXPECT_NEAR(0, wheelOdometry.getRotationQuaternionToParent()[1], 0.01);
}


TEST(CalibrationTestSuite, testWheelOdometryCalibrationEightNew) {
  auto vs = ValueStoreRef::fromFile("acceptance/wheelOdometry-pose.info");

  FrameGraphModel m(vs.getChild("model"));
  PoseSensor psA(m, "pose");
  WheelOdometry wheelOdometry(m, "wheelOdometry");
  PoseTrajectory traj(m, "traj");
  m.addModulesAndInit(psA, wheelOdometry, traj);

  //wheelOdometry.getTranslationVariable().set({0., 1., 0.});
  //const double rotUpdate[] = {0., 0.1, 0.};
  //wheelOdometry.getRotationVariable().update(rotUpdate, 3);
  

  //I think yaw should be 1.57079632679 for that case
  //LOG(WARNING)<<"aabbcc"<< wheelOdometry.getRotationQuaternionToParent();

  auto spModel = aslam::to_local_shared_ptr(m);
  auto c = createBatchCalibrator(vs.getChild("calibrator"), spModel);

  constexpr double l = 0.2; // base length
  constexpr double d = 0.13; // wheel diameters //actually the wheel radius

  for (auto& p : MmcsEight.getPoses(4.0 * M_PI)) {
    psA.addMeasurement(p.time, p.q, p.p, c->getCurrentStorage());
    c->addMeasurementTimestamp(p.time, psA);

    // 1 rad/s rotation (right circle positively, left  (x < -1) circle negatively)
    const double omega = p.p[0] < -1.0 ? -1 : 1; //-1 if in left circle, + 1 if in right
    LOG(WARNING)<<omega;
    const double transVel = 1;
    const double omegaL = omega * l;
    //V*(R+l/2)/d
    const double lW = (transVel*(1 - omegaL / 2)) / d;
    const double rW = (transVel*(1 + omegaL / 2)) / d;
    wheelOdometry.addMeasurement(*c, p.time, { lW, rW });
    c->addMeasurementTimestamp(p.time, wheelOdometry);
  }

  //EXPECT_NEAR(1, wheelOdometry.getTranslationToParent()[1], 0.0001);
  //EXPECT_NEAR(0.05, wheelOdometry.getRotationQuaternionToParent()[1], 0.01);
  c->calibrate();
  //EXPECT_NEAR(0, wheelOdometry.getTranslationToParent()[1], 0.0001);
  //EXPECT_NEAR(0, wheelOdometry.getRotationQuaternionToParent()[1], 0.01);
  EXPECT_NEAR(d, wheelOdometry.getWheelRadiusR()->getValue(), 0.001);
}