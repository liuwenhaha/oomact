//Here come all functions that export stuff, defined inside other oomact_python/src files
// It is extremely important to use this header
// if you are using the numpy_eigen interface
#include <numpy_eigen/boost_python_headers.hpp>

//models
void exportModel();
void exportFrameGraphModel();
void exportPoseTrajectory();
//measurements
void exportWheelSpeedsMeasurement();
void exportAccelerometerMeasurement();
void exportGyroscopeMeasurement();
//sensors
void exportPoseCv();
void exportDelayCv();
void exportModule();
void exportSensor();
void exportPoseSensorI();
void exportAbstractPoseSensor();
void exportPoseSensor();
void exportWheelOdometry();
void exportImu();
//calibrators
void exportAbstractCalibrator();
void exportBatchCalibratorI();
//void exportBatchCalibrator();
void exportCreateBatchCalibrator();
// The title of this library must match exactly
BOOST_PYTHON_MODULE(liboomact_python)
{
    //models
    exportModel();
    exportFrameGraphModel();
    //measurements
    exportWheelSpeedsMeasurement();
    exportAccelerometerMeasurement();
    exportGyroscopeMeasurement();
    //sensors
    exportPoseCv();
    exportDelayCv();
    exportModule();
    exportPoseTrajectory();
    exportSensor();
    exportPoseSensorI();
    exportAbstractPoseSensor();
    exportPoseSensor();
    exportWheelOdometry();
    exportImu();
    //calibrators
    exportAbstractCalibrator();
    exportBatchCalibratorI();
    //exportBatchCalibrator();
    exportCreateBatchCalibrator();
}