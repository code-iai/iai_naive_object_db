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

      iai_naive_object_db::Object createTable()
      {
	std::string name = "table_marker";
	iai_naive_object_db::Object object;
    //    visualization_msgs::Marker marker;
	iai_naive_object_db::Visual visual;
	visual.mesh_resource = "package://iai_naive_object_db/meshes/chemlab_table/chemlab_table.dae";

        visual.header.frame_id = "/base_footprint";
        visual.header.stamp = ros::Time::now();
        visual.type = iai_naive_object_db::Visual::MESH_RESOURCE;
        visual.scale.x = 1;
        visual.scale.y = 1;
        visual.scale.z = 1;
        visual.color.r = 0.38f;
        visual.color.g = 0.48f;
        visual.color.b = 0.55f;
        visual.color.a = 1.0;
	
	object.name = name;
        object.visuals.push_back(visual);

	object.frames.resize(2);
	object.frames[0].transform = createTransform(0.8, 0.0, 0.0);
	object.frames[0].child_frame_id = "/table_frame_base";
	object.frames[0].header.stamp  = ros::Time::now();
	object.frames[0].header.frame_id = "/base_footprint";
	object.frames[1].transform = createTransform(0.0, 0.0, 0.75);
	object.frames[1].child_frame_id = "/table_frame_high";
	object.frames[1].header.stamp  = ros::Time::now();
	object.frames[1].header.frame_id = "/table_frame_base";

        return object;	
      }

      iai_naive_object_db::Object createBottle(std::string color, float x, float y, float z)
      {
	std::string name = color + "_bottle_marker";
	iai_naive_object_db::Object object;
        //    visualization_msgs::Marker marker;
	iai_naive_object_db::Visual visual;
	visual.mesh_resource = "package://iai_naive_object_db/meshes/bottle/bottle.dae";

        visual.header.frame_id = "/l_gripper_tool_frame";
        visual.header.stamp = ros::Time::now();
        visual.type = visualization_msgs::Marker::MESH_RESOURCE;
        visual.scale.x = 1;
        visual.scale.y = 1;
        visual.scale.z = 1;
      
	if(color == "red")
	{	
	  visual.color.r = 0.9f;
          visual.color.g = 0.2f;
          visual.color.b = 0.3f;
	}
	else if(color == "blue")
	{	
	  visual.color.r = 0.3f;
          visual.color.g = 0.2f;
          visual.color.b = 0.9f;
	}
	
        visual.color.a = 1.0;
	object.name = name;
        object.visuals.push_back(visual);

	object.frames.resize(1);
	object.frames[0].transform = createTransform(x, y, z);
	object.frames[0].child_frame_id = "/" + color + "_bottle_frame";
	object.frames[0].header.stamp  = ros::Time::now();
	object.frames[0].header.frame_id = "/l_gripper_tool_frame";

        return object;	
      }
/*
      iai_naive_object_db::Object createBottleGripped(std::string color, float x, float y, float z)
      {
	std::string name = color + "_bottle_marker";
	iai_naive_object_db::Object object;
        visualization_msgs::Marker marker;
	marker.mesh_resource = "package://iai_naive_object_db/meshes/bottle/bottle.dae";

 //     marker.ns = "test_namespace";
 //     marker.id = 1;
 //     marker.action = visualization_msgs::Marker::ADD;
 //     marker.lifetime = ros::Duration();
        marker.header.frame_id = "/r_gripper_tool_frame";
        marker.header.stamp = ros::Time::now();
        marker.type = visualization_msgs::Marker::MESH_RESOURCE;
        marker.pose.position.x = x;
        marker.pose.position.y = y;
        marker.pose.position.z = z;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0; 
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        marker.scale.x = 1;
        marker.scale.y = 1;
        marker.scale.z = 1;
      
	if(color == "red")
	{	
	  marker.color.r = 0.9f;
          marker.color.g = 0.2f;
          marker.color.b = 0.3f;
	}
	else if(color == "blue")
	{	
	  marker.color.r = 0.3f;
          marker.color.g = 0.2f;
          marker.color.b = 0.9f;
	}
	else if(color == "green")
	{	
	  marker.color.r = 0.3f;
          marker.color.g = 0.9f;
          marker.color.b = 0.2f;
	}
	
        marker.color.a = 1.0;
        marker.frame_locked = true;
 // 	add mesh
	
	object.name = name;
        object.markers.push_back(marker);

	object.frames.resize(2);
	object.frames[0].transform = createTransform(x, y, z);
	object.frames[0].child_frame_id = "/" + color + "_bottle_frame";
	object.frames[0].header.stamp  = ros::Time::now();
	object.frames[0].header.frame_id = "/r_gripper_tool_frame";
	object.frames[1].transform = createTransform(x, y, z+0.2);
	object.frames[1].child_frame_id = "/" + color + "_low_handle_frame";
	object.frames[1].header.stamp  = ros::Time::now();
	object.frames[1].header.frame_id = "/" + color + "_bottle_frame";

        return object;	
      }
*/
      geometry_msgs::Transform createTransform(float x, float y, float z)
      {
	geometry_msgs::Transform transform;	
        transform.translation.x = x;
	transform.translation.y = y;
	transform.translation.z = z;
	transform.rotation.x = 0;
	transform.rotation.y = 0;
	transform.rotation.z = 0;
	transform.rotation.w = 1;

	return transform;
      }

      void start()
      {
        client_ = nh_.serviceClient<iai_naive_object_db::ObjectArray>("/iai_naive_object_db/add_object_service");	
	object_array_.request.objects.resize(4);
	object_array_.request.objects[0] = createTable();
	object_array_.request.objects[1] = createBottle("red", 0.0, 0.0, 0.0);
//	object_array_.request.objects[2] = createBottle("blue", 0.1, 0.4, 0.0);
//	object_array_.request.objects[3] = createBottleGripped("green", 0.1, 0.1, 0.1);

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
