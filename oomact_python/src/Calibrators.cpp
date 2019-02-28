#include <boost/python.hpp>

#include <aslam/calibration/calibrator/AbstractCalibrator.h>
#include <aslam/calibration/calibrator/CalibratorI.h>



using namespace boost::python;
using namespace aslam::calibration;

/*void exportAbstractCalibrator()
{
    class_<AbstractCalibrator, boost::shared_ptr<AbstractCalibrator>>("AbstractCalibrator", init<ValueStoreRef, std::shared_ptr<Model>>())
    //void addMeasurementTimestamp(Timestamp t, const Sensor & sensor) override
    .def("addMeasurementTimestamp", static_cast<void(AbstractCalibrator::*)(Timestamp, const Sensor&)>(&AbstractCalibrator::addMeasurementTimestamp))
    ;
}*/
/*void exportBatchCalibratorI()
{
    class_<BatchCalibratorI, boost::shared_ptr<BatchCalibratorI>>("BatchCalibratorI")
    //virtual void calibrate()
    .def("calibrate", &BatchCalibratorI::calibrate)
    ;
}*/
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