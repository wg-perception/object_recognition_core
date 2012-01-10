/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
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

#ifndef IO_H_
#define IO_H_

#include "types.h"
#include <object_recognition/db/db.h>
#include <object_recognition/db/view.h>

namespace object_recognition
{
  namespace common
  {
    /** Class storing an object recognition result: an object_id, with its pose and confidence
     */
    class PoseResult
    {
    public:
      PoseResult()
          :
            is_db_checked_(false)
      {
      }

      PoseResult(const PoseResult &pose_result)
          :
            R_(pose_result.R_),
            T_(pose_result.T_),
            object_id_(pose_result.object_id_),
            db_params_(pose_result.db_params_),
            is_db_checked_(false)
      {
      }

      void
      set_object_id(const db::ObjectDbParameters & db_params, const db::ObjectId &object_id)
      {
        db_params_ = db_params;
        object_id_ = object_id;
        is_db_checked_ = false;
      }

      inline void
      set_R(const cv::Mat & R)
      {
        R.copyTo(R_);
      }

      inline void
      set_T(const cv::Mat & T)
      {
        T.copyTo(T_);
      }

      inline const db::ObjectId &
      object_id() const
      {
        return object_id_;
      }

      inline const cv::Mat &
      R() const
      {
        return R_;
      }

      inline const cv::Mat &
      T() const
      {
        return T_;
      }

      inline const std::string &
      name() const
      {
        if (!is_db_checked_)
          check_db();
        return db_info_.name_;
      }

      inline const std::string &
      mesh_id() const
      {
        if (!is_db_checked_)
          check_db();
        return db_info_.mesh_id_;
      }

      /** Returns the URL where the mesh of the obejct is stored
       * @return
       */
      inline std::string
      mesh_resource() const
      {
        switch (db_params_.type_)
        {
          case db::ObjectDbParameters::COUCHDB:
            // E.g. http://localhost:5984/object_recognition/_design/models/_view/by_object_id_and_mesh?key=%2212a1e6eb663a41f8a4fb9baa060f191c%22
            return db_params_.root_ + std::string("/") + db_params_.collection_ + "/" + mesh_id() + "/mesh.stl";
          default:
            return "";
        }
      }

    private:
      struct DbInfo
      {
        /** Object name as queried from the db */
        mutable std::string name_;
        /** Mesh id as queried from the db */
        mutable std::string mesh_id_;
      };

      inline std::string
      cache_key() const
      {
        return db_params_.TypeToString(db_params_.type_) + db_params_.root_ + db_params_.collection_;
      }

      /** Read the name_ and mesh_id_ from the DB and store it */
      void
      check_db() const;

      /** The rotation matrix of the estimated pose */
      cv::Mat R_;
      /** The translation matrix of the estimated pose */
      cv::Mat T_;
      /** The object id of the found object */
      db::ObjectId object_id_;
      /** The parameters that define the db in which the object_id is */
      db::ObjectDbParameters db_params_;

      /** True if the name_ and mesh_id_ have been read from the DB */
      mutable bool is_db_checked_;
      /** DB info */
      mutable DbInfo db_info_;

      static std::map<std::string, DbInfo> cached_name_mesh_id_;
    };
  }
}

#endif /* IO_H_ */
