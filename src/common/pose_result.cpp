/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <map>

#include <object_recognition/common/pose_result.h>

namespace object_recognition
{
  namespace common
  {
    /** Define the static member */
    std::map<std::string, PoseResult::DbInfo> PoseResult::cached_name_mesh_id_ = std::map<std::string, DbInfo>();

    /** Read the name_ and mesh_id_ from the DB and store it */
    void
    PoseResult::check_db() const
    {
      if (is_db_checked_)
        return;
      is_db_checked_ = true;

      // Check if the data is already cached
      {
        std::map<std::string, DbInfo>::const_iterator iter = cached_name_mesh_id_.find(cache_key());
        if (iter != cached_name_mesh_id_.end())
        {
          db_info_ = iter->second;
          return;
        }
      }

      // Find all the models of that type for that object
      db::View view(db::View::VIEW_MODEL_WHERE_OBJECT_ID_AND_MODEL_TYPE);
      view.Initialize("mesh");
      view.set_key(object_id_);
      db::ObjectDb db(db_params_);
      db::ViewIterator view_iterator(view, db);

      db::ViewIterator iter = view_iterator.begin(), end = view_iterator.end();
      for (; iter != end; ++iter)
      {
        // Get the mesh_id_
        db_info_.mesh_id_ = (*iter).value_.get_obj().find("_id")->second.get_str();
        // Get the object name
        db::Document doc(db, object_id_);
        db_info_.name_ = doc.get_value("object_name").get_str();
      }

      // Cache all the results
      cached_name_mesh_id_[cache_key()] = db_info_;
    }
  }
}
