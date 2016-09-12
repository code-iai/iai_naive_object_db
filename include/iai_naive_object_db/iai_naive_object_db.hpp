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

#ifndef IAI_NAIVE_OBJECT_DB_IAI_NAIVE_OBJECT_DB_HPP
#define IAI_NAIVE_OBJECT_DB_IAI_NAIVE_OBJECT_DB_HPP

#include <map>
#include <string>
#include <exception>
#include <algorithm>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <tf2_msgs/TFMessage.h>
#include <iai_naive_object_db/Object.h>
#include <iai_naive_object_db/ObjectArray.h>

namespace iai_naive_object_db
{
  // TODO: add comments
  class ObjectDB
  {
    public:
      ObjectDB(const ros::NodeHandle& nh): nh_(nh)
      {}

      ~ObjectDB() {}

      bool add_object(iai_naive_object_db::ObjectArray::Request &req,
		      iai_naive_object_db::ObjectArray::Response &res)
      {
	ROS_INFO("Doing adding stuff...");
	
	for(size_t i = 0; i < req.objects.size(); ++i)
	{
		set_object(req.objects[i].name, req.objects[i]);
		ROS_INFO("One object set");
	}

	return true;	
      }

      bool remove_object(iai_naive_object_db::ObjectArray::Request &req,
		      iai_naive_object_db::ObjectArray::Response &res)
      {
	ROS_INFO("Doing removing stuff...");
	
	for(size_t i = 0; i < req.objects.size(); ++i)
	{
		remove_object(req.objects[i].name, req.objects[i]);
		ROS_INFO("One object removed");	
	}

	return true;	
      }
	
      void set_object(const std::string& name, const iai_naive_object_db::Object& object)
      {
        map_.insert(std::pair<std::string, iai_naive_object_db::Object>(name, object));
	update_markers(object, "add");	
        update_transforms(object);
      }

      void remove_object(const std::string& name, const iai_naive_object_db::Object& object)
      {
        map_.erase(name);
        update_markers(object, "remove");
        update_transforms(object);
      }

      void set_transform_msg(const std::vector<geometry_msgs::TransformStamped>& transforms)
      {
	transform_msg_.transforms.clear();  // reinitialize the list
	transform_msg_.transforms = transforms;
      }

      void set_marker_array(const std::vector<visualization_msgs::Marker>& markers)
      {
	marker_array_.markers.clear();  // reinitialize the list
	marker_array_.markers = markers;
      }

      const std::vector<geometry_msgs::TransformStamped>& get_transforms() const
      {
        return transforms_;
      }

      const std::vector<visualization_msgs::Marker>& get_markers() const
      {
        return markers_;
      }

      const std::map<std::string, iai_naive_object_db::Object>& get_map() const
      {
        return map_;
      }	

      const visualization_msgs::MarkerArray& get_marker_array() const
      {
	return marker_array_;
      }

      const tf2_msgs::TFMessage& get_transform_msg() const
      {
	return transform_msg_;
      }

      void start(const ros::Duration& period)
      {
	pub_markers_ = nh_.advertise<visualization_msgs::MarkerArray>("/visualization_marker_array", 1);
	pub_transforms_ = nh_.advertise<tf2_msgs::TFMessage>("/tf", 1);
	timer_ = nh_.createTimer(period, &ObjectDB::callback, this);
	srv_add_ = nh_.advertiseService("/iai_naive_object_db/add_object_service", &ObjectDB::add_object, this);
	srv_remove_ = nh_.advertiseService("/iai_naive_object_db/remove_object_service", &ObjectDB::remove_object, this);
      }

    private:
      std::map<std::string, iai_naive_object_db::Object> map_;
      std::vector<geometry_msgs::TransformStamped> transforms_;
      tf2_msgs::TFMessage transform_msg_;
      std::vector<visualization_msgs::Marker> markers_;
      visualization_msgs::MarkerArray marker_array_;
      ros::NodeHandle nh_;
      ros::Timer timer_;
      ros::Publisher pub_markers_;
      ros::Publisher pub_transforms_;
      ros::ServiceServer srv_add_;
      ros::ServiceServer srv_remove_;

      void update_markers(const iai_naive_object_db::Object& object, const std::string& intent)
      {
	markers_.clear();  // reinitialize the list
	std::map<std::string, iai_naive_object_db::Object>::iterator it;
	
	for(it = map_.begin(); it != map_.end(); ++it)
	{
	  for(size_t i = 0; i < it->second.markers.size(); ++i)
	  {
	    
	    markers_.push_back(it->second.markers[i]);
	  }
	}

	set_marker_array(markers_);	
      }

      void update_transforms(const iai_naive_object_db::Object& object)
      {
	transforms_.clear();  // reinitialize the list
	std::map<std::string, iai_naive_object_db::Object>::iterator it;

	for(it = map_.begin(); it != map_.end(); ++it)
	{
	  for(size_t i = 0; i < object.frames.size(); ++i)
	  {
	    transforms_.push_back(it->second.frames[i]);
	  }
	}
	set_transform_msg(transforms_);	
      }

      void callback(const ros::TimerEvent& e)
      {
        while (pub_markers_.getNumSubscribers() < 1)
	{
	  ros::Duration(1).sleep();
	  if (!ros::ok())
	  {
	    throw std::runtime_error("Problem");
	  }
	  ROS_WARN_ONCE("Please create a subscriber to the marker");
	  ros::Duration(1).sleep();
	}

        while (pub_transforms_.getNumSubscribers() < 1)
	{
	  ros::Duration(1).sleep();
	  if (!ros::ok())
	  {
	    throw std::runtime_error("Problem");
	  }
	  ROS_WARN_ONCE("Please create a subscriber to frames");
	  ros::Duration(1).sleep();
	}

	pub_markers_.publish(get_marker_array());
	ROS_INFO("Published markers!");
	pub_transforms_.publish(get_transform_msg());
	ROS_INFO("Published tfs!");
      }
  };
}
#endif
