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

#ifndef IAI_NAIVE_OBJECT_DB_IAI_NAIVE_OBJECT_DB_NODE_HPP
#define IAI_NAIVE_OBJECT_DB_IAI_NAIVE_OBJECT_DB_NODE_HPP

#include <iai_naive_object_db/iai_naive_object_db.hpp>
#include <ros/ros.h>

namespace iai_naive_object_db
{
  class ObjectDBNode
  {
    public:
      ObjectDBNode(const ros::NodeHandle& nh): nh_(nh)
      {}

      ~ObjectDBNode() {}

      bool add_object(iai_naive_object_db::ObjectArray::Request &req,
		      iai_naive_object_db::ObjectArray::Response &res)
      {
	ROS_INFO("Doing adding stuff...");
	// TODO: catch exceptions
        database_.set_objects(req.objects);

	return true;	
      }

      bool remove_object(iai_naive_object_db::ObjectArray::Request &req,
		         iai_naive_object_db::ObjectArray::Response &res)
      {
	ROS_INFO("Doing removing stuff...");
	// TODO: catch exceptions
	database_.remove_objects(req.objects);

      	return true;	
      }

      void start(const ros::Duration& period)
      {
	pub_markers_ = nh_.advertise<visualization_msgs::MarkerArray>("/visualization_marker_array", 1);
	pub_transforms_ = nh_.advertise<tf2_msgs::TFMessage>("/tf", 1);
	timer_ = nh_.createTimer(period, &ObjectDBNode::callback, this);
	srv_add_ = nh_.advertiseService("/iai_naive_object_db/add_object_service", &ObjectDBNode::add_object, this);
	srv_remove_ = nh_.advertiseService("/iai_naive_object_db/remove_object_service", &ObjectDBNode::remove_object, this);
      }

    private:
      ros::NodeHandle nh_;
      ros::Timer timer_;
      ros::Publisher pub_markers_;
      ros::Publisher pub_transforms_;
      ros::ServiceServer srv_add_;
      ros::ServiceServer srv_remove_;
      ObjectDB database_;

      void callback(const ros::TimerEvent& e)
      {
        database_.update_timestamps(ros::Time::now());
	pub_markers_.publish(database_.get_marker_array());
	ROS_INFO("Published markers!");
	pub_transforms_.publish(database_.get_transform_msg());
	ROS_INFO("Published tfs!");
      }
  };
}
#endif
