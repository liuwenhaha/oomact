#include <numpy_eigen/boost_python_headers.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/python.hpp>

#include <aslam/calibration/data/WheelSpeedsMeasurement.h>


using namespace boost::python;
using namespace aslam::calibration;



void exportWheeSpedsMeasurement()
{
    class_<WheelSpeedsMeasurement>("WheelSpeedsMeasurement")
    ;
}