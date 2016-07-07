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

#ifndef _INCL_FOOBAR_H_
#define _INCL_FOOBAR_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/impl/common.hpp>
#include <pcl/common/transforms.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/centroid.h>
#include <pcl/common/pca.h>
#include <pcl/search/kdtree.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <ros/console.h>
#include <ros/common.h>
#include <ros/package.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/MarkerArray.h>

namespace foobar
{
    typedef pcl::PointXYZRGB Pt; //default point type
    typedef pcl::PointCloud<Pt> PtC; //default point cloud

    class FooBar
    {
        public:
        FooBar()=delete;
        FooBar(const std::string name_space);

        ///Get NodeHandle
        inline ros::NodeHandle getNodeHandle() const
        {
            return *nh_;
        }
        ///Main spin method
        void spinOnce();

        private:
        //ROS
        ///Callback to get input point cloud
        void cbCloud(const sensor_msgs::PointCloud2::ConstPtr &msg);
        boost::shared_ptr<ros::NodeHandle> nh_;
        ros::Subscriber sub_;
        ros::Publisher pub_marks_;
        tf::TransformBroadcaster brcaster_;
        tf::Transform transf_;
        bool processing;
        double found_len, found_wid;
        //PCL
        PtC::Ptr cloud_;
        pcl::SACSegmentation<Pt> sac;
        //params
        std::string topic_, frame_;
        double tolerance_, plane_tol_;
        double width_, length_;
        ///Worker functions
        void broadcast(); //Tf
        void publishMarkers(); //rviz marker
        void find_it(); //actual computation
        void removeIndices(PtC::Ptr& source, pcl::PointIndices::Ptr ind); //helper to remove a set of indices from source
    };
}
#endif //_INCL_FOOBAR_H_
