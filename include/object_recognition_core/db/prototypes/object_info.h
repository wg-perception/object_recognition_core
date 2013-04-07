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

#ifndef OBJECT_INFO_H_
#define OBJECT_INFO_H_

#include <object_recognition_core/common/types.h>
#include <object_recognition_core/db/db.h>
#include <object_recognition_core/db/view.h>

namespace object_recognition_core
{
  namespace prototypes
  {
    /** Class storing information about an object. This info comes from the DB
     * The possible attributes are as follows:
     * - std::string name: the name of the object, some string you can understand: "Can of Coke"
     * - std::string mesh_uri: the full URI of where the mesh can be retrieved (this can be useful for RViz)
     * - stream mesh: the mesh as a file
     */
    class ObjectInfo : public object_recognition_core::db::DummyDocument
    {
    public:
      ObjectInfo()
      {
      }

      ObjectInfo(const db::ObjectId &object_id, const db::ObjectDbPtr &db)
          :
            object_id_(object_id),
            db_(db)
      {
      }

      // Setter functions
      /** An object id only makes sense with respect to a DB so you have to set the two together
       * @param db the DB where the object is stored
       * @param object_id the id of the found object
       */
      void
      set_object_id(const db::ObjectDbPtr & db, const db::ObjectId &object_id)
      {
        db_ = db;
        object_id_ = object_id;
      }

      // Getter functions
      inline const db::ObjectId &
      object_id() const
      {
        return object_id_;
      }

      /** Read the name_ and mesh_id_ from the DB and store it */
      void
      load_fields_and_attachments();
    private:
      inline std::string
      cache_key() const
      {
        std::string parameter_hash = or_json::write(or_json::mValue(db_->parameters().raw()));
        return parameter_hash + object_id_;
      }

      /** The object id of the found object */
      db::ObjectId object_id_;
      /** The db in which the object_id is */
      db::ObjectDbPtr db_;

      static std::map<std::string, ObjectInfo> cached_name_mesh_id_;
    };
  }
}

#endif /* OBJECT_INFO_H_ */
