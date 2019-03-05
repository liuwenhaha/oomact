#include <numpy_eigen/boost_python_headers.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/python.hpp>

#include <aslam/calibration/data/PoseMeasurement.h>
#include <aslam/calibration/data/GyroscopeMeasurement.h>
#include <aslam/calibration/data/AccelerometerMeasurement.h>

#include <aslam/calibration/calibrator/CalibratorI.h>
#include <aslam/backend/OptimizationProblemBase.hpp>

#include <aslam/calibration/model/fragments/PoseCv.h>
#include <aslam/calibration/model/fragments/DelayCv.h>
#include <aslam/calibration/model/Sensor.h>
#include <aslam/calibration/model/sensors/PoseSensorI.h>
#include <aslam/calibration/model/sensors/AbstractPoseSensor.h>
#include <aslam/calibration/model/sensors/PoseSensor.h>
#include <aslam/calibration/model/sensors/WheelOdometry.h>
#include <aslam/calibration/model/sensors/Imu.h>



using namespace boost::python;
using namespace aslam::calibration;
void exportPoseCv()
{
    class_<PoseCv, boost::shared_ptr<PoseCv>>("PoseCv", init<Module *, boost::optional<std::string>>()) //do we even need the constructor, what to do about boost::optional
    //      const Eigen::Vector3d & getTranslationToParent() const
    .def("getTranslationToParent", &PoseCv::getTranslationToParent, return_value_policy<copy_const_reference>())
    //      const Eigen::Vector4d & getRotationQuaternionToParent() const
    .def("getRotationQuaternionToParent", &PoseCv::getRotationQuaternionToParent, return_value_policy<copy_const_reference>())
    //      Put more as needed
    ;
}

void exportDelayCv()
{
    class_<DelayCv, boost::shared_ptr<DelayCv>>("DelayCv", init<const Module*>())
    //bool hasDelay() const
    .def("hasDelay", &DelayCv::hasDelay)
    ;
}

//This is the abstract parent class for sensors (does indirect inheritance work like that?, what about multiple inheritance)
void exportSensor()
{
    //bases makes problems, I think because PoseCb has no constructor exposed (see above)
    class_<Sensor, boost::shared_ptr<Sensor>, bases<PoseCv, DelayCv>>("Sensor", init<Model&, std::string, sm::value_store::ValueStoreRef>())
    //      BoundedTimeExpression getBoundedTimestampExpression(const CalibratorI& calib, Timestamp t) const
    .def("getBoundedTimestampExpression", &Sensor::getBoundedTimestampExpression)
    //TODO C: complete exports for all functions
    //      aslam::backend::TransformationExpression getTransformationExpressionToAtMeasurementTimestamp(const CalibratorI & calib, Timestamp t, const Frame & to, bool ignoreBounds = false) const
    //.def("getTransformationExpressionToAtMeasurementTimestamp"), &Sensor::getTransformationExpressionToAtMeasurementTimestamp)
    //      virtual aslam::backend::TransformationExpression getTransformationExpressionTo(const ModelAtTime & robotModel, const Frame & to) const;
    //.def("getTransformationExpressionTo", &Sensor::getTransformationExpressionTo)
    //      sm::kinematics::Transformation getTransformationTo(const ModelAtTime & robotModel, const Frame & to) const;
    //.def("getTransformationTo", &Sensor::getTransformationTo) //what about multiple implementations with different arguments?
    ;
}

void exportPoseSensorI()
{
    class_<PoseSensorI, boost::shared_ptr<PoseSensorI>, boost::noncopyable>("PoseSensorI", no_init)
    ;
}

void exportAbstractPoseSensor()
{   //can not give this init because of pure virtual functions
    class_<AbstractPoseSensor, boost::shared_ptr<AbstractPoseSensor>, bases<PoseSensorI, Sensor>, boost::noncopyable>("AbstractPoseSensor", no_init)
    ;
}

void exportPoseSensor()
{
    class_<PoseSensor, boost::shared_ptr<PoseSensor>, bases<AbstractPoseSensor>>("PoseSensor", init<Model&, std::string, sm::value_store::ValueStoreRef>())
    .def(init<Model&, std::string>())
    //      virtual void addMeasurementErrorTerms(CalibratorI & calib, const CalibrationConfI & ec, ErrorTermReceiver & problem, bool observeOnly) const override;
    .def("addMeasurementErrorTerms", &PoseSensor::addMeasurementErrorTerms)
    //      void addInputTo(Timestamp t, const PoseMeasurement& pose, ModuleStorage & storage) const override;
    .def("addInputTo", &PoseSensor::addInputTo)
    //      void addMeasurement(Timestamp t, const PoseMeasurement& pose, ModuleStorage & storage) const;
    .def("addMeasurement", static_cast<void(PoseSensor::*)(Timestamp,const PoseMeasurement&, ModuleStorage&)const>(&PoseSensor::addMeasurement))
    //      void addMeasurement(Timestamp t, const Eigen::Vector4d & quat, const Eigen::Vector3d & trans, ModuleStorage & storage) const;
    .def("addMeasurementOrientationTranslation", static_cast<void(PoseSensor::*)(Timestamp, const Eigen::Vector4d&, const Eigen::Vector3d&, ModuleStorage&)const>(&PoseSensor::addMeasurement))
    //      const Frame& getTargetFrame() const override
    .def("getTargetFrame", &PoseSensor::getTargetFrame, return_value_policy<copy_const_reference>())
    ;
}

void exportWheelOdometry()
{
    class_<WheelOdometry, boost::shared_ptr<WheelOdometry>, bases<Sensor>>("WheelOdometry", init<Model&, std::string, sm::value_store::ValueStoreRef>())
    .def(init<Model&, std::string>())
    //      void addMeasurementErrorTerms(CalibratorI & calib, const CalibrationConfI & ec, ErrorTermReceiver & problem, bool observeOnly) const override;
    .def("addMeasurementErrorTerms", &WheelOdometry::addMeasurementErrorTerms)
    //      void addMeasurement(CalibratorI & calib, Timestamp t, const WheelSpeedsMeasurement & m) const;
    .def("addMeasurement", &WheelOdometry::addMeasurement)
    //      ScalarCvSp& getL()
    .def("getL", static_cast<ScalarCvSp&(WheelOdometry::*)()>(&WheelOdometry::getL), return_value_policy<copy_non_const_reference>())
    //      ScalarCvSp& getWheelRadiusL()
    .def("getWheelRadiusL", static_cast<ScalarCvSp&(WheelOdometry::*)()>(&WheelOdometry::getWheelRadiusL), return_value_policy<copy_non_const_reference>())
    //      ScalarCvSp& getWheelRadiusR()
    .def("getWheelRadiusR", static_cast<ScalarCvSp&(WheelOdometry::*)()>(&WheelOdometry::getWheelRadiusR), return_value_policy<copy_non_const_reference>())
    //      const MeasurementsContainer<WheelSpeedsMeasurement> & getMeasurements() const
    .def("getMeasurements", &WheelOdometry::getMeasurements, return_value_policy<copy_const_reference>())
    ;
}

void exportImu()
{
    class_<Imu, boost::shared_ptr<Imu>, bases<Sensor>>("Imu", init<Model&, std::string, sm::value_store::ValueStoreRef>())
    .def(init<Model&, std::string>())
    //      void addAccelerometerMeasurement(CalibratorI & calib, const AccelerometerMeasurement& data, Timestamp timestamp) const
    .def("addAccelerometerMeasurement", &Imu::addAccelerometerMeasurement)
    //      void addGyroscopeMeasurement(CalibratorI & calib, const GyroscopeMeasurement& data, Timestamp timestamp) const
    .def("addGyroscopeMeasurement", &Imu::addGyroscopeMeasurement)
    ;
}