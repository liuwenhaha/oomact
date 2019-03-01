#include <boost/python.hpp>

#include <aslam/calibration/model/Sensor.h>

#include <aslam/calibration/calibrator/AbstractCalibrator.h>
#include <aslam/calibration/calibrator/CalibratorI.h>



using namespace boost::python;
using namespace aslam::calibration;

void exportAbstractCalibrator()
{
    class_<AbstractCalibrator, boost::shared_ptr<AbstractCalibrator>, boost::noncopyable>("AbstractCalibrator", no_init)
    //void addMeasurementTimestamp(Timestamp t, const Sensor & sensor) override
    .def("addMeasurementTimestamp", static_cast<void(AbstractCalibrator::*)(Timestamp, const Sensor&)>(&AbstractCalibrator::addMeasurementTimestamp))
    ;
}
void exportBatchCalibratorI()
{
    class_<BatchCalibratorI, boost::shared_ptr<BatchCalibratorI>, bases<AbstractCalibrator>, boost::noncopyable>("BatchCalibratorI", no_init)
    //virtual void calibrate()
    .def("calibrate", &BatchCalibratorI::calibrate)
    ;
}
//I don't know if we need this class, and if we do how to do it as there is no header
/*void exportBatchCalibrator()
{
    class_<BatchCalibrator, boost::shared_ptr<BatchCalibrator>, bases<AbstractCalibrator>>("BatchCalibraotr", init<ValueStoreRef, std::shared_ptr<Model>>)
    // virtual void calibrate() override
    .def("calibrate", &BatchCalibrator::calibrate)
    ;
}*/
void exportCreateBatchCalibrator()
{
    //std::unique_ptr<BatchCalibratorI> createBatchCalibrator(ValueStoreRef vs, std::shared_ptr<Model> model)
    def("createBatchCalibrator", &createBatchCalibrator)
    ;
}