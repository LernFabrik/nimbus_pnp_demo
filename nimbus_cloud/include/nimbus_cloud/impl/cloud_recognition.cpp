#include <nimbus_cloud/cloud_recognition.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/common/transforms.h>
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/recognition/cg/geometric_consistency.h>

template <class PointType, class NormalType>
nimbus::cloudRecognition<PointType, NormalType>::cloudRecognition(ros::NodeHandle nh) : 
                                                 cloudFeatures<PointType, NormalType>(nh), _nh(nh)
{}

template <class PointType, class NormalType>
nimbus::cloudRecognition<PointType, NormalType>::~cloudRecognition(){}

template <class PointType, class NormalType>
void nimbus::cloudRecognition<PointType, NormalType>::updateParm(double ns, double ks, double ds, double rs, double cs, double ct)
{
    this->normal_sr =  ns;
    this->keypoint_sr = ks;
    this->shot_sr = ds;
    this->reference_sr = rs;
    this->cg_size_ = cs;
    this->cg_thresh_ = ct;
}

template <class PointType, class NormalType>
void nimbus::cloudRecognition<PointType, NormalType>::modelConstruct(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr blob)
{
    this->cloudNormalEstimationOMP(blob);
    this->cloudUniformSampling(blob);
    this->cloudSHOTEstimationOMP(blob);
    this->cloudBoardLocalRefeFrame(blob);
    mData.normal = this->normalOut;
    mData.keypoint = this->keypointOut;
    mData.descriptor = this->shotOut;
    ROS_ERROR("Model decriptor size %d", (int)mData.descriptor->points.size());
    //ros::Duration(5).sleep();
    mData.blRefence = this->referenceOut;
}

template <class PointType, class NormalType>
void nimbus::cloudRecognition<PointType, NormalType>::cloudCorrespondence(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr blob)
{
    //pcl::CorrespondencesPtr model_scene_corr (new pcl::Correspondences());
    this->cloudNormalEstimationOMP(blob);
    this->cloudUniformSampling(blob);
    this->cloudSHOTEstimationOMP(blob);
    this->cloudBoardLocalRefeFrame(blob);
    model_scene_corr.reset(new pcl::Correspondences());
    pcl::KdTreeFLANN<pcl::SHOT352> match_search;
    match_search.setInputCloud(mData.descriptor);
    for(std::size_t i = 0; i < this->shotOut->size(); ++i){
        std::vector<int> neigh_indices(1);
        std::vector<float> neigh_sqrt_distance(1);

        if(! std::isfinite(this->shotOut->at (i).descriptor[0])){
            continue;
        }
        
        int found_neighs = match_search.nearestKSearch(this->shotOut->at (i), 1, neigh_indices, neigh_sqrt_distance);
        if(found_neighs == 1 && neigh_sqrt_distance[0] < 0.25f){
            pcl::Correspondence corr(neigh_indices[0], static_cast<int> (i), neigh_sqrt_distance[0]);
            model_scene_corr->push_back(corr);
        }
    }
    ROS_INFO("Correspondence found: %d", (int)model_scene_corr->size());
}

template <class PointType, class NormalType>
void nimbus::cloudRecognition<PointType, NormalType>::cloudHough3D(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr blob,
                              std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > &rototranslations, 
                              std::vector<pcl::Correspondences> &clustered_corrs)
{
    cloudCorrespondence(blob);
    pcl::Hough3DGrouping<PointType, PointType, pcl::ReferenceFrame, pcl::ReferenceFrame> clusterer;
    clusterer.setHoughBinSize (cg_size_);
    clusterer.setHoughThreshold (cg_thresh_);
    clusterer.setUseInterpolation (true);
    clusterer.setUseDistanceWeight (false);

    clusterer.setInputCloud (mData.keypoint);
    clusterer.setInputRf (mData.blRefence);
    clusterer.setSceneCloud (this->keypointOut);
    clusterer.setSceneRf (this->referenceOut);
    clusterer.setModelSceneCorrespondences (model_scene_corr);

    clusterer.recognize (rototranslations, clustered_corrs);
}