/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
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

#include <iostream>
#include <map>

#include <object_recognition_core/db/prototypes/object_info.h>
#include <object_recognition_core/db/view.h>

namespace object_recognition_core
{
  namespace prototypes
  {
    /** Define the static member */
    std::map<std::string, ObjectInfo> ObjectInfo::cached_name_mesh_id_ =
        std::map<std::string, ObjectInfo>();

    /** Read the name_ and mesh_id_ from the DB and store it */
    void
    ObjectInfo::load_fields_and_attachments()
    {
      // Check if the data is already cached
      {
        std::map<std::string, ObjectInfo>::const_iterator iter = cached_name_mesh_id_.find(cache_key());
        if (iter != cached_name_mesh_id_.end())
        {
          *this = iter->second;
          return;
        }
      }

      if (db_->parameters().type() == db::ObjectDbParameters::EMPTY)
        // nothing to do
        return;

  // Get information about the object
  or_json::mObject fields;
  db_->load_fields(object_id_, fields);

  // Get the object name
  std::string name;
  if (fields.find("object_name") != fields.end())
    name = fields.find("object_name")->second.get_str();

  // If no name, set one
  if (name.empty())
    set_field("name", object_id_);
  else
    set_field("name", name);

  // Get the mesh_id
  if (fields.find("mesh_uri") != fields.end())
    set_field("mesh_uri", fields.find("mesh_uri")->second.get_str());

  // Get the mesh id
  std::string mesh_id;
  db::View view = db::View(db::View::VIEW_MODEL_WHERE_OBJECT_ID_AND_MODEL_TYPE);
  db::ViewIterator view_iterator(view, db_);
  view.Initialize("mesh");
  view.set_key(object_id_);
  view_iterator = db::ViewIterator(view, db_);
  db::ViewIterator iter = view_iterator.begin(), end = view_iterator.end();

  // Get the mesh_URI if not set
  switch (db_->parameters().type()) {
    case db::ObjectDbParameters::COUCHDB:
      for (; iter != end; ++iter) {
        if (((*iter).has_field("_id")) && ((*iter).has_field("_attachments"))) {
          mesh_id = (*iter).get_field<std::string>("_id");
          break;
        }
      }
      // E.g. http://localhost:5984/object_recognition/_design/models/_view/by_object_id_and_mesh?key=%2212a1e6eb663a41f8a4fb9baa060f191c%22
      if (!mesh_id.empty()) {
        // Figure out the name of the mesh
        std::vector<std::string> attachments_names = (*iter).attachment_names();
        std::string mesh_name;
        BOOST_FOREACH(const std::string& attachment_name, attachments_names) {
          // Check that the end of the mesh is proper for display
          if ((attachment_name.find(".stl") != std::string::npos) || (attachment_name.find(".obj") != std::string::npos)) {
            mesh_name = attachment_name;
            break;
          }
        }
        if (!mesh_name.empty())
          set_field("mesh_uri", db_->parameters().at("root").get_str() + std::string("/")
                + db_->parameters().at("collection").get_str() + "/" + mesh_id
                + "/" + mesh_name);
      }
      break;
    default:
      for (; iter != end; ++iter) {
        if ((*iter).has_field("mesh_uri")) {
          set_field("mesh_uri", (*iter).get_field<std::string>("mesh_uri"));
          break;
        } else if ((*iter).has_attachment("mesh")) {
          std::stringstream stream;
          (*iter).get_attachment_stream("mesh", stream);
          set_attachment_stream("mesh", stream);
          break;
        }
      }
      break;
  }

      // Cache all the results
      cached_name_mesh_id_[cache_key()] = *this;
    }
  }
}
