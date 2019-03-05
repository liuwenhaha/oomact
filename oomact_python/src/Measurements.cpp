#include <numpy_eigen/boost_python_headers.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/python.hpp>

#include <aslam/calibration/data/WheelSpeedsMeasurement.h>
#include <aslam/calibration/data/AccelerometerMeasurement.h>
#include <aslam/calibration/data/GyroscopeMeasurement.h>


using namespace boost::python;
using namespace aslam::calibration;



void exportWheelSpeedsMeasurement()
{
    class_<WheelSpeedsMeasurement>("WheelSpeedsMeasurement")
    ;
}

void exportAccelerometerMeasurement()
{
    class_<AccelerometerMeasurement>("AccelerometerMeasurement")
    ;
}

void exportGyroscopeMeasurement()
{
    class_<GyroscopeMeasurement>("GyroscopeMeasurement")
    ;
}