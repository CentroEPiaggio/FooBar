/*
 * Software License Agreement (BSD License)
 *
 *   Copyright (c) 2016, Federico Spinelli (fspinelli@gmail.com)
 *   All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of copyright holder(s) nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <foobar/foobar.h>

namespace foobar
{

bool enforceCurvature (const pcl::PointXYZRGBNormal& a, const pcl::PointXYZRGBNormal& b, float s_dist)
{
    Eigen::Map<const Eigen::Vector3f> a_normal = a.normal, b_normal = b.normal;
    if (std::fabs(a_normal.dot(b_normal))> 0.9995){
        return (true);
    }
    return (false);
}

FooBar::FooBar(const std::string name_space): processing(false)
{
    nh_ = boost::make_shared<ros::NodeHandle>(name_space);
    /* nh_->param<double>("cluster_tolerance", clus_tol_, 0.005); */
    /* nh_->param<bool>("invert_z_projection", invert_, false); */
    /* nh_->param<int>("cluster_min_size", min_size_, 1000); */
    /* nh_->param<int>("cluster_max_size", max_size_, 10000); */
    nh_->param<std::string>("input_topic", topic_, "/pacman_vision/processed_scene");   //TODO change accordingly
    nh_->param<std::string>("reference_frame", frame_, "/camera_rgb_optical_frame");
    nh_->param<double>("tolerance", tolerance_, 0.05);
    nh_->param<double>("cluster_tolerance", clus_tol_, 0.02);
    nh_->param<double>("bar_width", width_, 0.048);
    nh_->param<double>("bar_length", length_, 1.08);
    sub_ = nh_->subscribe(nh_->resolveName(topic_), 0, &FooBar::cbCloud, this);
    transf_.setIdentity();
}

void FooBar::spinOnce()
{
    ros::spinOnce();
    find_it();
    broadcast();
    publishMarkers();
}

void FooBar::broadcast()
{
    brcaster_.sendTransform(tf::StampedTransform(
            transf_, ros::Time::now(),
            frame_, "bar"));
}

void FooBar::publishMarkers()
{
    //TODO
}

void FooBar::cbCloud(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    if (processing)
        return;
    try
    {
        cloud_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
        pcl::fromROSMsg (*msg, *cloud_);
    }
    catch (...)
    {
        ROS_ERROR("[FooBar::%s]Error recieving cloud from topic %s",__func__, topic_.c_str());
        return;
    }
}

void FooBar::find_it()
{
    if(!cloud_){
        ROS_ERROR_THROTTLE(10,"[FooBar::%s]No point cloud to process, aborting.",__func__);
        return;
    }
    if(cloud_->empty()){
        ROS_ERROR_THROTTLE(10,"[FooBar::%s]Point cloud is empty, aborting.",__func__);
        return;
    }
    ne.setInputCloud(cloud_);
    ne.setRadiusSearch(0.01);
    ne.useSensorOriginAsViewPoint();
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_and_normals = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGBNormal>>();
    ne.compute(*cloud_and_normals);
    cec.setClusterTolerance(tolerance_);
    cec.setMinClusterSize(50);
    cec.setMaxClusterSize(cloud_and_normals->size());
    cec.setInputCloud(cloud_and_normals);
    cec.setConditionFunction(enforceCurvature);
    std::vector<pcl::PointIndices> cluster_indices;
    cec.segment(cluster_indices);
        
    /* Eigen::Vector3f dirZ(coefficients->values[3], coefficients->values[4], coefficients->values[5]); */
    /*     Eigen::Vector3f Tx,Ty; */
    /*     dirZ.normalize(); */
    /*     if ( !dirZ.isApprox(Eigen::Vector3f::UnitX(), 1e-3)){ */
    /*         Tx = Eigen::Vector3f::UnitX() - (dirZ*(dirZ.dot(Eigen::Vector3f::UnitX()))); */
    /*         Tx.normalize(); */
    /*         Ty = dirZ.cross(Tx); */
    /*         Ty.normalize(); */
    /*     } */
    /*     else{ */
    /*         Tx = Eigen::Vector3f::UnitY() - (dirZ*(dirZ.dot(Eigen::Vector3f::UnitY()))); */
    /*         Tx.normalize(); */
    /*         Ty = dirZ.cross(Tx); */
    /*         Ty.normalize(); */
    /*     } */
    /*     tf::Matrix3x3 rot(Tx[0],Ty[0],dirZ[0], */
    /*             Tx[1],Ty[1],dirZ[1], */
    /*             Tx[2],Ty[2],dirZ[2]); */
    /*     tf::Vector3 trals(coefficients->values[0],coefficients->values[1],coefficients->values[2]); */
    /*     transf_.setBasis(rot); */
    /*     transf_.setOrigin(trals); */
    
   
        ROS_WARN("[FooBar::%s]No bar found...",__func__);
    ROS_WARN("[FooBar::%s]End",__func__);
    processing = false;
}

} //End namespace
