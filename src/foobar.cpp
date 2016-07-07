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

FooBar::FooBar(const std::string name_space): processing(false), found_len(.0),
    found_wid(.0)
{
    nh_ = boost::make_shared<ros::NodeHandle>(name_space);
    /* nh_->param<double>("cluster_tolerance", clus_tol_, 0.005); */
    /* nh_->param<bool>("invert_z_projection", invert_, false); */
    /* nh_->param<int>("cluster_min_size", min_size_, 1000); */
    /* nh_->param<int>("cluster_max_size", max_size_, 10000); */
    nh_->param<std::string>("input_topic", topic_, "/pacman_vision/processed_scene");   //TODO change accordingly
    nh_->param<std::string>("reference_frame", frame_, "/camera_rgb_optical_frame");
    nh_->param<double>("bar_tolerance", tolerance_, 0.05);
    nh_->param<double>("plane_tolerance", plane_tol_, 0.03);
    nh_->param<double>("bar_width", width_, 0.048);
    nh_->param<double>("bar_length", length_, 1.08);
    sub_ = nh_->subscribe(nh_->resolveName(topic_), 1, &FooBar::cbCloud, this);
    pub_marks_ = nh_->advertise<visualization_msgs::MarkerArray>("markers",1);
    transf_.setIdentity();
}

void FooBar::spinOnce()
{
    find_it();
    broadcast();
    publishMarkers();
    ros::spinOnce();
}

void FooBar::broadcast()
{
    brcaster_.sendTransform(tf::StampedTransform(
            transf_, ros::Time::now(),
            frame_, "bar"));
}

void FooBar::publishMarkers()
{
    if (!marks)
        return;
    pub_marks_.publish(marks);
    marks.reset();
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

void FooBar::removeIndices(PtC::Ptr& source, pcl::PointIndices::Ptr ind)
{
    pcl::ExtractIndices<Pt> ex;
    PtC::Ptr leftover = boost::make_shared<PtC>();
    ex.setInputCloud(source);
    ex.setIndices(ind);
    ex.setNegative(true);
    ex.filter(*leftover);
    source = leftover;
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
            removeIndices(cloud_, inliers);
            boost::this_thread::sleep(boost::posix_time::milliseconds(10));
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
        Z.normalize();
        //See if we need to flip normal towards the camera, assuming viewpoint is (0,0,0)
        Eigen::Vector3f cen(center.x,center.y,center.z);
        float cos_theta = (-cen).dot(Z);
        if (cos_theta < 0)
            //Normal has to be flipped
            Z *= -1;
        //Now find an X axis as close as possible to pcaX but normal to Z,
        //then find also Y as cross prod
        Eigen::Vector3f X,Y;
        pcaX.normalize();
        if ( Z.isApprox(pcaX, 1e-4)){ //should never happen that Z==pcaX
            ROS_WARN("[FooBar::%s]Cannot find a suitable basis for plane, discaring it.",__func__);
            removeIndices(cloud_, inliers);
            boost::this_thread::sleep(boost::posix_time::milliseconds(10));
            continue;
        }
        X = pcaX - Z*(Z.dot(pcaX));
        X.normalize();
        Y = Z.cross(X);
        Y.normalize();
        //Homo-transform (bar frame"b" with respect to the camera"k")
        Eigen::Matrix4f Tkb;
        Tkb <<  X[0], Y[0], Z[0], center.x,
                X[1], Y[1], Z[1], center.y,
                X[2], Y[2], Z[2], center.z,
                0,     0,    0,      1;
        PtC::Ptr plane_transformed = boost::make_shared<PtC>();
        Eigen::Matrix4f Tbk = Tkb.inverse();
        //transform the plane cloud into its reference frame
        pcl::transformPointCloud(*plane, *plane_transformed, Tbk);
        //now the plane dimensions can be extracted easily
        Pt min,max;
        pcl::getMinMax3D(*plane_transformed, min,max);
        found_len = max.x - min.x;
        found_wid = max.y - min.y;
        //The Check for Bar identification
        if (found_len < (length_ - tolerance_) || found_len > (length_ + tolerance_) ||
            found_wid < (width_ - tolerance_) || found_wid > (width_ + tolerance_)){
            //This is not our bar, discard it
            ROS_WARN("[FooBar::%s]Found a plane, but it is not our bar (%g x %g), discaring it.",__func__,
                    found_len, found_wid);
            removeIndices(cloud_, inliers);
            createMarker(found_len,found_wid,Tkb); //tmp visualization
            boost::this_thread::sleep(boost::posix_time::milliseconds(10));
            continue;
        }
        //This plane is our bar!!
        tf::Matrix3x3 rot(  Tkb(0,0),Tkb(0,1),Tkb(0,2),
                            Tkb(1,0),Tkb(1,1),Tkb(1,2),
                            Tkb(2,0),Tkb(2,1),Tkb(2,2));
        tf::Vector3 trals(Tkb(0,3),Tkb(1,3),Tkb(2,3));
        transf_.setBasis(rot);
        transf_.setOrigin(trals);
        break;
    }
    processing = false;
    ROS_INFO("[FooBar::%s]DONE",__func__);
}

void FooBar::createMarker(double dimx, double dimy, const Eigen::Matrix4f &trans)
{
    if (!marks)
        marks = boost::make_shared<visualization_msgs::MarkerArray>();
    visualization_msgs::Marker m;
    m.header.frame_id = frame_;
    m.header.stamp = ros::Time();
    m.ns = "plane";
    m.id = std::round(0.5*(dimx+dimy)*(dimx+dimy+1)+dimy);
    m.type = visualization_msgs::Marker::CUBE;
    m.action = visualization_msgs::Marker::ADD;
    m.pose.position.x = trans(0,3);
    m.pose.position.y = trans(1,3);
    m.pose.position.z = trans(2,3);
    Eigen::Quaternionf q(trans.block<3,3>(0,0));
    m.pose.orientation.x = q.x();
    m.pose.orientation.y = q.y();
    m.pose.orientation.z = q.z();
    m.pose.orientation.w = q.w();
    m.scale.x = dimx;
    m.scale.y = dimy;
    m.scale.z = plane_tol_;
    m.color.a=0.7;
    m.color.r=0.5;
    m.color.b=1.0;
    m.color.g=0.1;
    m.lifetime = ros::Duration(1);
    marks->markers.push_back(m);
}

} //End namespace
