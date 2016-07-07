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
 * * Neither the name of the copyright holder(s) nor the names of its
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

#ifndef _INCL_SACBAR_H_
#define _INCL_SACBAR_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/impl/common.hpp>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/centroid.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <ros/console.h>
#include <ros/common.h>
#include <ros/package.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/MarkerArray.h>

namespace sacbar
{
    bool enforceCurvature (const pcl::PointXYZRGBNormal& a, const pcl::PointXYZRGBNormal b, float s_dist)
    {
        Eigen::Map<const Eigen::Vector3f> a_normal = a.normal, b_normal = b.normal;
        if (std::fabs(a_normal.dot(b_normal))> 0.9995){
            return (true);
        }
        return (false);
    }

    class SacBar
    {
        public:
        SacBar()=delete;
        SacBar(const std::string name_space);

        ///Get NodeHandle
        inline ros::NodeHandle getNodeHandle() const
        {
            return *nh_;
        }
        ///Main spin method
        void spinOnce();

        private:
        ///Callback to get input point cloud
        void cbCloud(const sensor_msgs::PointCloud2::ConstPtr &msg);
        boost::shared_ptr<ros::NodeHandle> nh_;
        ros::Subscriber sub_;
        ros::Publisher pub_marks_;
        tf::TransformBroadcaster brcaster_;

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_;
        tf::Transform transf_;
        bool processing;
        pcl::ConditionalEuclideanClustering<pcl::PointXYZRGBNormal> cec;
        pcl::NormalEstimationOMP<pcl::PointXYZRGB, pcl::PointXYZRGBNormal> ne;
        //params
        std::string topic_, frame_;
        double tolerance_;
        double width_, length_;
        ///Worker functions
        void broadcast(); //Tf
        void publishMarkers(); //rviz marker
        void sac_it(); //actual computation
    };
}
#endif //_INCL_SACBAR_H_
