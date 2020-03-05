#include <nimbus_cloud/cloud_recognition.h>

nimbus::cloudRecognition::cloudRecognition(ros::NodeHandle nh) : _nh(nh){}
nimbus::cloudRecognition::~cloudRecognition(){}

void nimbus::cloudRecognition::cloudCorrespondence(const pcl::PointCloud<pcl::SHOT352>::ConstPtr model_descriptor, 
                                            const pcl::PointCloud<pcl::SHOT352>::ConstPtr scene_descriptor, 
                                            correspondencesPtr model_scene_corr)
{
    //pcl::CorrespondencesPtr model_scene_corr (new pcl::Correspondences());
    pcl::KdTreeFLANN<pcl::SHOT352> match_search;
    match_search.setInputCloud(model_descriptor);
    for(std::size_t i = 0; i < scene_descriptor->size(); ++i){
        std::vector<int> neigh_indices(1);
        std::vector<float> neigh_sqrt_distance(1);

        if(! std::isfinite(scene_descriptor->at (i).descriptor[0])){
            continue;
        }
        
        int found_neighs = match_search.nearestKSearch(scene_descriptor->at (i), 1, neigh_indices, neigh_sqrt_distance);
        if(found_neighs == 1 && neigh_sqrt_distance[0] < 0.25f){
            pcl::Correspondence corr(neigh_indices[0], static_cast<int> (i), neigh_sqrt_distance[0]);
            model_scene_corr->push_back(corr);
        }
    }
}