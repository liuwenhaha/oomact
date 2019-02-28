#include <numpy_eigen/boost_python_headers.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/python.hpp>


#include <aslam/calibration/input/MotionCaptureSource.h>
#include <test/MockMotionCaptureSource.h>

using namespace boost::python;

//base class
void exportMotionCaptureSource()
{
    class_<MotionCaptureSource, boost::shared_ptr<MotionCaptureSource>>("MotionCaptureSource")
    .def("getPoses", &MotionCaptureSource::getPoses)
}

//derived MockCaptureSources for tests
void exportMockMotionCaptureSource()
{
    class_<MockMotionCaptureSource, boost::shared_ptr<MockMotionCaptureSource>, Base<MotionCaptureSource>>("MockMotionCaptureSource", init<std::function<void(Timestamp now, PoseStamped & p)>>) //I don't think that works
    .def("getPoses", &MockMotionCaptureSource::getPoses)
    .def("getPoseAt", &MockMotionCaptureSource::getPoseAt)
}

