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
#include <object_recognition_core/db/db.h>
#include <object_recognition_core/db/view.h>

#ifdef CV_MAJOR_VERSION
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#endif

#ifdef EIGEN_CORE_H
#include <Eigen/Eigen>
#endif

namespace object_recognition_core
{
  namespace common
  {
    /** Class storing an object recognition result: an object_id, with its pose and confidence
     * It is not meant to be fast, but it is meant to be flexible (uses Eigen, OpenCV ...)
     */
    class PoseResult
    {
    public:
      PoseResult()
          :
            is_db_checked_(false)
      {
        R_.resize(9);
        T_.resize(3);

      }

      PoseResult(const PoseResult &pose_result)
          :
            R_(pose_result.R_),
            T_(pose_result.T_),
            object_id_(pose_result.object_id_),
            db_(pose_result.db_),
            is_db_checked_(false)
      {
      }

      void
      set_confidence(float confidence)
      {
        confidence_ = confidence;
      }

      void
      set_object_id(const db::ObjectDb & db, const db::ObjectId &object_id)
      {
        db_ = db;
        object_id_ = object_id;
        is_db_checked_ = false;
      }

      template<typename Type>
      void
      set_R(const Type & R);

      template<typename Type>
      void
      set_T(const Type & T);

      // Getter functions
      float
      confidence() const
      {
        return confidence_;
      }

      inline const db::ObjectId &
      object_id() const
      {
        return object_id_;
      }

      template<typename Type>
      Type
      R() const;

      template<typename Type>
      Type
      T() const;

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
        switch (db_.parameters().type_)
        {
          case db::ObjectDbParameters::COUCHDB:
            // E.g. http://localhost:5984/object_recognition/_design/models/_view/by_object_id_and_mesh?key=%2212a1e6eb663a41f8a4fb9baa060f191c%22
            return db_.parameters().root_ + std::string("/") + db_.parameters().collection_ + "/" + mesh_id()
                   + "/mesh.stl";
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
        return db_.parameters().TypeToString(db_.parameters().type_) + db_.parameters().root_
               + db_.parameters().collection_ + object_id_;
      }

      /** Read the name_ and mesh_id_ from the DB and store it */
      void
      check_db() const;

      /** The rotation matrix of the estimated pose, stored row by row */
      std::vector<float> R_;
      /** The translation vector of the estimated pose */
      std::vector<float> T_;
      /** The absolute confidence, between 0 and 1 */
      float confidence_;
      /** The object id of the found object */
      db::ObjectId object_id_;
      /** The db in which the object_id is */
      db::ObjectDb db_;

      /** True if the name_ and mesh_id_ have been read from the DB */
      mutable bool is_db_checked_;
      /** DB info */
      mutable DbInfo db_info_;

      static std::map<std::string, DbInfo> cached_name_mesh_id_;
    };

#ifdef CV_MAJOR_VERSION
// OpenCV specializations
  template<>
  inline void
  PoseResult::set_R(const cv::Mat_<float> & R_float)
  {
    cv::Mat R_full;
    if (R_float.cols * R_float.rows == 3)
    cv::Rodrigues(R_float, R_full);
    else
    R_full = R_float;

    // TODO, speedup that process
    std::vector<float>::iterator iter = R_.begin();
    for (unsigned int j = 0; j < 3; ++j, iter += 3)
    {
      float * data = R_full.ptr<float>(j);
      std::copy(data, data + 3, iter);
    }
  }

  template<>
  inline void
  PoseResult::set_R(const cv::Mat & R)
  {
    cv::Mat_<float> R_float;
    R.convertTo(R_float, CV_32F);

    set_R<cv::Mat_<float> >(R_float);
  }

  template<>
  inline void
  PoseResult::set_T(const cv::Mat_<float> & T)
  {
    T_[0] = T(0);
    T_[1] = T(1);
    T_[2] = T(2);
  }

  template<>
  inline void
  PoseResult::set_T(const cv::Mat & T)
  {
    cv::Mat_<float> T_float;
    T.convertTo(T_float, CV_32F);

    set_T<cv::Mat_<float> >(T_float);
  }

  template<>
  inline cv::Mat_<float>
  PoseResult::R() const
  {
    return (cv::Mat_<float>(3, 3) << R_[0], R_[1], R_[2], R_[3], R_[4], R_[5], R_[6], R_[7], R_[8]);
  }

  template<>
  inline cv::Mat_<float>
  PoseResult::T() const
  {
    return (cv::Mat_<float>(3, 1) << T_[0], T_[1], T_[2]);
  }
#endif

#ifdef EIGEN_CORE_H
// Eigen specializations
  template<>
  inline void
  PoseResult::set_R(const Eigen::Matrix3f & R_float)
  {
    std::copy(R_float.data(),R_float.data()+R_float.size(),R_.begin());
  }

  template<>
  inline void
  PoseResult::set_R(const Eigen::Quaternionf & quaternion)
  {
    Eigen::Matrix3f m(quaternion);
    set_R(m);
  }

  template<>
  inline void
  PoseResult::set_T(const Eigen::Vector3f & T)
  {
    std::copy(T.data(),T.data()+T.size(),T_.begin());
  }

  template<>
  inline Eigen::Matrix3f
  PoseResult::R() const
  {
    Eigen::Matrix3f R;
    std::copy(R_.begin(),R_.end(),R.data());
    return R;
  }

  template<>
  inline Eigen::Vector3f
  PoseResult::T() const
  {
    Eigen::Vector3f T;
    std::copy(T_.begin(),T_.end(),T.data());
    return T;
  }
#endif
  }
}

#endif /* IO_H_ */
