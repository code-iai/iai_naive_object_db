/*
 * Copyright (c) 2016, Georg Bartels, <georg.bartels@cs.uni-bremen.de>
 * Copyright (c) 2016, Cristi Iacob, <il.cristian@yahoo.com>
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the name of the Institute of Artificial Intelligence, 
 *     University of Bremen nor the names of its contributors may be used 
 *     to endorse or promote products derived from this software without 
 *     specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef IAI_NAIVE_OBJECT_DB_IAI_NAIVE_OBJECT_DB_CLIENT_HPP
#define IAI_NAIVE_OBJECT_DB_IAI_NAIVE_OBJECT_DB_CLIENT_HPP

#include <map>
#include <string>
#include <exception>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <iai_naive_object_db/Object.h>
#include <iai_naive_object_db/ObjectArray.h>

namespace iai_naive_object_db_client 
{
  class ObjectDBClient
  {
    public:
      ObjectDBClient(const ros::NodeHandle& nh): nh_(nh)
      {}
  
      ~ObjectDBClient() {}

      std::vector<iai_naive_object_db::Object> createObjects()
      {
	std::vector<iai_naive_object_db::Object> objects;
	std::string name = "first_object";
	iai_naive_object_db::Object object;
        visualization_msgs::Marker marker;

        marker.header.frame_id = "/base_footprint";
        marker.header.stamp = ros::Time();
        marker.ns = "test_namespace";
        marker.id = 1;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = 0.8;
        marker.pose.position.y = 0;
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 0.0;
        marker.scale.x = 1;
        marker.scale.y = 1;
        marker.scale.z = 1;
        marker.color.r = 0.38f;
        marker.color.g = 0.48f;
        marker.color.b = 0.55f;
        marker.color.a = 1.0;
        marker.lifetime = ros::Duration();
        marker.frame_locked = false;
	
	object.name = name;
        object.markers.push_back(marker);

	object.frames.resize(1);
	object.frames[0].transform = createTransform();
	object.frames[0].child_frame_id = "/cube_frame";
	object.frames[0].header.stamp  = ros::Time::now();
	object.frames[0].header.frame_id = "/base_footprint";
 
	objects.push_back(object);

        return objects;	
      }

      geometry_msgs::Transform createTransform()
      {
	geometry_msgs::Transform transform;
        transform.translation.x = 0.0;
	transform.translation.y = 0.0;
	transform.translation.z = 0.0;
	transform.rotation.x = 0;
	transform.rotation.y = 0;
	transform.rotation.z = 0;
	transform.rotation.w = 1;

	return transform;
      }

      void start()
      {
        client_ = nh_.serviceClient<iai_naive_object_db::ObjectArray>("/iai_naive_object_db/add_object_service");	
	object_array_.request.objects = createObjects();

        try
	{
	  ros::service::waitForService("/iai_naive_object_db/add_object_service");
	  client_.call(object_array_);
	  ROS_INFO("Request sent!");
	}
	catch(const std::exception& e)
	{
	  ROS_ERROR("%s", e.what());
	}

      }

    private:
      ros::NodeHandle nh_;
      ros::ServiceClient client_;
      iai_naive_object_db::ObjectArray object_array_;	
  };
}
#endif
