#include <numpy_eigen/boost_python_headers.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/python.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>

#include <aslam/calibration/model/FrameGraphModel.h>
#include <aslam/calibration/model/Model.h>


using namespace boost::python;
using namespace aslam::calibration;

void exportModel()
{
    typedef std::vector<const Frame*> FramePointers;

    class_<FramePointers>("FramePointerVector")
    .def(boost::python::vector_indexing_suite<FramePointers>() )
    .def("__iter__", boost::python::iterator<FramePointers>())
    ;

    //Model(ValueStoreRef config, std::shared_ptr<ConfigPathResolver> configPathResolver, const std::vector<const Frame *> frames = {})
    class_<Model, boost::shared_ptr<Model>, boost::noncopyable>("Model", no_init)
    .def(init<ValueStoreRef, std::shared_ptr<ConfigPathResolver>, const FramePointers>())
    //TODO
    //void addModulesAndInit(Modules_ & ... modules)
    ;
}

void exportFrameGraphModel()
{
    typedef std::vector<const Frame*> FramePointers;

    class_<FramePointers>("FramePointerVector")
    .def(boost::python::vector_indexing_suite<FramePointers>() )
    .def("__iter__", boost::python::iterator<FramePointers>())
    ;

    //FrameGraphModel(ValueStoreRef config, std::shared_ptr<ConfigPathResolver> configPathResolver = nullptr, const std::vector<const Frame *> frames = {})
    class_<FrameGraphModel, boost::shared_ptr<FrameGraphModel>, bases<Model>, boost::noncopyable>("FrameGraphModel", no_init)
    .def(init<ValueStoreRef, std::shared_ptr<ConfigPathResolver>, const FramePointers>()) //how to define default values
    //ModelAtTime getAtTime(Timestamp timestamp, int maximalDerivativeOrder, const ModelSimplification & simplification) const override
    /*.def("getAtTimeTimestamp", static_cast(<ModelAtTime(FrameGraphModel::*)(Timestamp, int, const ModelSimplification&)const>(&FrameGraphModel::getAtTime))*/
    //void registerModule(Module & m) override
    /*.def("registerModule", &FrameGraphModel::registerModule)*/
    ;
}
