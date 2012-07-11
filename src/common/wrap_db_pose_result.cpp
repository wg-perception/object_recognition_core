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
#include <string>

#include <boost/function.hpp>
#include <boost/python.hpp>
#include <boost/python/stl_iterator.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>
#include <boost/shared_ptr.hpp>

#include <object_recognition_core/common/pose_result.h>

namespace bp = boost::python;

namespace
{
  class PoseResultSimple
  {
  public:
    PoseResultSimple()
        :
          confidence_(0)
    {
      R_.resize(9);
      T_.resize(3);
    }

    PoseResultSimple(const object_recognition_core::common::PoseResult &pose_result)
        :
          R_(pose_result.R()),
          T_(pose_result.T()),
          confidence_(pose_result.confidence()),
          object_id_(pose_result.object_id()),
          db_(pose_result.db())
    {
    }

    bool
    operator==(const PoseResultSimple &pose)
    {
      return object_id_ == pose.object_id_;
    }

    ~PoseResultSimple()
    {
    }

    /** The rotation matrix of the estimated pose, stored row by row */
    std::vector<float> R_;
    /** The translation vector of the estimated pose */
    std::vector<float> T_;
    /** The absolute confidence, between 0 and 1 */
    float confidence_;
    /** The object id of the found object */
    object_recognition_core::db::ObjectId object_id_;
    /** The db in which the object_id is */
    object_recognition_core::db::ObjectDb db_;
  };
}

namespace object_recognition_core
{
  namespace common
  {
    typedef std::vector<PoseResultSimple> PoseResults;
    typedef boost::shared_ptr<PoseResults> PoseResultsPtr;

    /** Function used to create a vector of db PoseResult's from Python
     * @param db_params
     * @param python_document_ids
     * @return
     */
    PoseResultsPtr
    PoseResultsConstructor(const std::vector<PoseResult> & pose_results)
    {
      // Create the PoseResults from the ids
      PoseResultsPtr p(new PoseResults());
      p->resize(pose_results.size());
      for (size_t i = 0; i < pose_results.size(); ++i)
        (*p)[i] = pose_results[i];

      return p;
    }

    // Define the pickling of the object
    struct pose_results_pickle_suite: boost::python::pickle_suite
    {
      static boost::python::tuple
      getinitargs(const PoseResults& db_params)
      {
        return boost::python::make_tuple();
      }

      static boost::python::tuple
      getstate(const PoseResults& db_params)
      {
        // TODO
        return boost::python::make_tuple();
      }

      static
      void
      setstate(PoseResults& db_params, boost::python::tuple state)
      {
        using namespace boost::python;
        // TODO
      }
    };

    std::string
    object_id(boost::shared_ptr<const PoseResultSimple> &p)
    {
      return p->object_id_;
    }

    void
    wrap_db_pose_result()
    {
      bp::class_<PoseResultSimple> PoseResultClass("PoseResult");
      PoseResultClass.def(bp::init<>()).def(bp::init<PoseResultSimple>());
      PoseResultClass.def("object_id", object_id);

      bp::class_<PoseResults, PoseResultsPtr> PoseResultsClass("PoseResults");
      PoseResultsClass.def("__init__", bp::make_constructor(PoseResultsConstructor));
      PoseResultsClass.def(boost::python::vector_indexing_suite<PoseResults>());
      PoseResultsClass.def("size", &PoseResults::size);
      PoseResultsClass.def_pickle(pose_results_pickle_suite());
    }
  }
}

