#include <numpy_eigen/boost_python_headers.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/python.hpp>

#include <aslam/calibration/model/FrameGraphModel.h>


using namespace boost::python;
using namespace aslam::calibration;
void exportFrameGraphModel()
{
    //FrameGraphModel(ValueStoreRef config, std::shared_ptr<ConfigPathResolver> configPathResolver = nullptr, const std::vector<const Frame *> frames = {})
    //class_<FrameGraphModel, boost::shared_ptr<FrameGraphModel>>("FrameGraphModel", init<ValueStoreRef, std::shared_ptr<ConfigPathResolver>, std::vector<Frame *>()) //how to define default values
    //.def("getAtTime", &FrameGraphModel::getAtTime)
    //void registerModule(Module & m) override
    //.def("registerModule", &FrameGraphModel::registerModule)
}
