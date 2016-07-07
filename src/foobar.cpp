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

FooBar::FooBar(const std::string name_space): processing(false)
{
    nh_ = boost::make_shared<ros::NodeHandle>(name_space);
    /* nh_->param<double>("cluster_tolerance", clus_tol_, 0.005); */
    /* nh_->param<bool>("invert_z_projection", invert_, false); */
    /* nh_->param<int>("cluster_min_size", min_size_, 1000); */
    /* nh_->param<int>("cluster_max_size", max_size_, 10000); */
    nh_->param<std::string>("input_topic", topic_, "/pacman_vision/processed_scene");   //TODO change accordingly
    nh_->param<std::string>("reference_frame", frame_, "/camera_rgb_optical_frame");
    nh_->param<double>("bar_tolerance", tolerance_, 0.05);
    nh_->param<double>("plane_tolerance", plane_tol_, 0.02);
    nh_->param<double>("bar_width", width_, 0.048);
    nh_->param<double>("bar_length", length_, 1.08);
    nh_->param<double>("normals_radius", normals_rad_, 0.01); //Should be approx 2.5~3 times point density
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
        cloud_ = boost::make_shared<PtC>();
        pcl::fromROSMsg (*msg, *cloud_);
    }
    catch (...)
    {
        ROS_ERROR("[FooBar::%s]Error recieving cloud from topic %s",__func__, topic_.c_str());
        return;
    }
}

void FooBar::removeIndices(const PtC::ConstPtr& source, PtC::Ptr dest,
        pcl::PointIndices::Ptr ind)
{
    pcl::ExtractIndices<Pt> ex;
    if (!dest)
        dest = boost::make_shared<PtC>();
    ex.setInputCloud(source);
    ex.setIndices(ind);
    ex.setNegative(true);
    ex.filter(*dest);
}

void FooBar::find_it()
{
    if(!cloud_){
        ROS_ERROR_DELAYED_THROTTLE(2,"[FooBar::%s]No point cloud to process, aborting.",__func__);
        return;
    }
    if(cloud_->empty()){
        ROS_ERROR_DELAYED_THROTTLE(2,"[FooBar::%s]Point cloud is empty, aborting.",__func__);
        return;
    }
    processing = true;
    while (cloud_->size()>50)
    {
        //Find and segment a plane
        sac.setInputCloud(cloud_);
        pcl::ModelCoefficients::Ptr coeff =
            boost::make_shared<pcl::ModelCoefficients>();
        pcl::PointIndices::Ptr inliers =
            boost::make_shared<pcl::PointIndices>();
        sac.setModelType (pcl::SACMODEL_PLANE);
        sac.setMethodType (pcl::SAC_RANSAC);
        sac.setDistanceThreshold(plane_tol_);
        sac.segment(*inliers, *coeff);
        //oneliner overkill pointcloud initialization with plane inliers
        PtC::Ptr plane = boost::make_shared<PtC> (*cloud_, inliers->indices);
        //find plane centroid
        Eigen::Vector4f centroid;
        if (!pcl::compute3DCentroid(*plane, centroid)){
            ROS_WARN("[FooBar::%s]Cannot calculate centroid for plane, discaring it.",__func__);
            PtC::Ptr leftover;
            removeIndices(cloud_, leftover, inliers);
            cloud_ = leftover;
            continue;
        }
        //Find the nearest plane point to the centroid
        pcl::search::KdTree<Pt> tree;
        tree.setInputCloud(plane);
        Pt pt_cent, center;
        pt_cent.x = centroid[0];
        pt_cent.y = centroid[1];
        pt_cent.z = centroid[2];
        std::vector<int> k_ind(1);
        std::vector<float> k_dist(1);
        tree.nearestKSearch(pt_cent, 1, k_ind, k_dist);
        center = plane->points.at(k_ind[0]);
        //Get principal components of the cluster
        pcl::PCA<Pt> pca(true);
        pca.setInputCloud(plane);
        Eigen::Matrix3f pcaXYZ = pca.getEigenVectors();
        //First eigenvector should be along the bar length, lets put X axis there
        Eigen::Vector3f pcaX = pcaXYZ.block<3,1>(0,0);
        //Put Z axis as the normal of the plane (already computed)
        Eigen::Vector3f Z (coeff->values[0], coeff->values[1], coeff->values[2]);
        //Now find an X axis as close as possible to pcaX but normal to Z,
        //then find also Y as cross prod
        Eigen::Vector3f X,Y;
        Z.normalize();
        pcaX.normalize();
        if ( Z.isApprox(pcaX, 1e-4)){ //should never happen that Z==pcaX
            ROS_WARN("[FooBar::%s]Cannot find a suitable basis for plane, discaring it.",__func__);
            PtC::Ptr leftover;
            removeIndices(cloud_, leftover, inliers);
            cloud_ = leftover;
            continue;
        }
        X = pcaX - Z*(Z.dot(pcaX));
        X.normalize();
        Y = Z.cross(X);
        Y.normalize();

        //TODO just show the first cluster transform
        tf::Matrix3x3 rot(X[0],Y[0],Z[0],
                X[1],Y[1],Z[1],
                X[2],Y[2],Z[2]);
        tf::Vector3 trals(center.x,center.y,center.z);
        transf_.setBasis(rot);
        transf_.setOrigin(trals);
        break;
        boost::this_thread::sleep(boost::posix_time::milliseconds(10));
    }
    processing = false;
}

} //End namespace
