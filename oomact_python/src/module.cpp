//Here come all functions that export stuff, defined inside other oomact_python/src files
// It is extremely important to use this header
// if you are using the numpy_eigen interface
#include <numpy_eigen/boost_python_headers.hpp>

void exportFrameGraphModel();
//measurements
void exportWheelSpeedsMeasurement();
//sensors
void exportPoseCv();
void exportSensor();
void exportPoseSensor();
void exportWheelOdometry();
//calibrators
void exportAbstractCalibrator();
void exportBatchCalibrator();
void exportCreateBatchCalibrator();
// The title of this library must match exactly
BOOST_PYTHON_MODULE(liboomact_backend_python)
{
    exportFrameGraphModel();
    //measurements
    exportWheelSpeedsMeasurement();
    //sensors
    exportPoseCv();
    exportSensor();
    exportPoseSensor();
    exportWheelOdometry();
    //calibrators
    exportAbstractCalibrator();
    exportBatchCalibrator();
    exportCreateBatchCalibrator();
}