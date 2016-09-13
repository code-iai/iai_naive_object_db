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
#include <tf2_msgs/TFMessage.h>
#include <iai_naive_object_db/Object.h>
#include <iai_naive_object_db/ObjectArray.h>

namespace iai_naive_object_db
{
  // TODO: add comments
  class ObjectDB
  {
    public:
      ObjectDB()
      {
	map_.clear();
      }

      ~ObjectDB() {}
	
      void set_objects(const std::vector<iai_naive_object_db::Object>& objects)
      {
        for (size_t i=0; i<objects.size(); ++i)
          map_.insert(std::pair<std::string, iai_naive_object_db::Object>(objects[i].name, objects[i]));
	update_markers();	
        update_transforms();
      }

      void remove_objects(const std::vector<iai_naive_object_db::Object>& objects)
      {
        for (size_t i=0; i<objects.size(); ++i)
          map_.erase(objects[i].name);
        update_markers();
        update_transforms();
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

    private:
      std::map<std::string, iai_naive_object_db::Object> map_;
      tf2_msgs::TFMessage transform_msg_;
      visualization_msgs::MarkerArray marker_array_;

      void update_markers()
      {
	marker_array_.markers.clear();  // reinitialize the list
	std::map<std::string, iai_naive_object_db::Object>::iterator it;
	
	for(it = map_.begin(); it != map_.end(); ++it)
	{
	  for(size_t i = 0; i < it->second.markers.size(); ++i)
	  {
	    
	    marker_array_.markers.push_back(it->second.markers[i]);
	  }
	}
      }

      void update_transforms()
      {
	transform_msg_.transforms.clear();  // reinitialize the list
	std::map<std::string, iai_naive_object_db::Object>::iterator it;

	for(it = map_.begin(); it != map_.end(); ++it)
	{
	  for(size_t i = 0; i < it->second.frames.size(); ++i)
	  {
	    transform_msg_.transforms.push_back(it->second.frames[i]);
	  }
	}
      }
  };
}
#endif
