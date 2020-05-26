
#include <nimbus_fh_detector/recognition.hpp>

#include <pcl/io/pcd_io.h>

nimbus::Recognition::Recognition(ros::NodeHandle nh, const std::string path): _path(path), _features(nh, 0.01, 0.01, 0.01)
{

}
nimbus::Recognition::~Recognition(){}

void nimbus::Recognition::constructModelParam()
{
    _model_normals.resize(10);
    _model_keypoints.resize(10);
    _model_description.resize(10);
    // ToDo: Change the fixed number of *.pcd file to for loop
    std::stringstream model_path;
    model_path << "/box_";
    for(int i = 0; i < 10; i++)
    {
        _model_normals[i].reset(new pcl::PointCloud<pcl::Normal>());
        _model_keypoints[i].reset(new pcl::PointCloud<pcl::PointXYZI>());
        _model_description[i].reset(new pcl::PointCloud<pcl::SHOT352>());
        typename pcl::PointCloud<pcl::PointXYZI>::Ptr blob (new pcl::PointCloud<pcl::PointXYZI>());
        model_path << std::to_string(i) << ".pcd";
        pcl::io::loadPCDFile(model_path.str(), *blob);
        _features.extraction(blob);

        _model_keypoints[i] = _features.keypoints;
        _model_normals[i] = _features.normals;
        _model_description[i] = _features.descriptor;
    }
}