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
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/TransformStamped.h>

namespace iai_naive_object_db
{
  // TODO: add comments
  class ObjectDB
  {
    public:
      void set_object(const std::string& name, const visualization_msgs::Marker& object)
      {
        map_[name] = object;
        update_markers();
        update_transforms();
      }

      void remove_object(const std::string& name)
      {
        map_.erase(name);
        update_markers();
        update_transforms();
      }

      const std::vector<geometry_msgs::TransformStamped>& get_transforms() const
      {
        return transforms_;
      }

      const std::vector<visualization_msgs::Marker>& get_markers() const
      {
        return markers_;
      }

      const std::map<std::string, visualization_msgs::Marker>& get_map() const
      {
        return map_;
      }

    private:
      std::map<std::string, visualization_msgs::Marker> map_;
      std::vector<geometry_msgs::TransformStamped> transforms_;
      std::vector<visualization_msgs::Marker> markers_;

      void update_markers()
      {
        // TODO: implemente me
      }

      void update_transforms()
      {
        // TODO: implemente me
      }
  };
}
#endif
