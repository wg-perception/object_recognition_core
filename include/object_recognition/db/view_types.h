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

#ifndef VIEW_TYPES_H_
#define VIEW_TYPES_H_

#include "object_recognition/common/types.h"

namespace object_recognition
{
  namespace db
  {
    // Forward declare the base class
    class ObjectDbBase;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    /** A view can be of different type, and for each, the Initialize function needs to be called with the right
     * arguments
     * VIEW_MODEL_WHERE_OBJECT_ID_AND_MODEL_TYPE: Initialize(std::string object_id, st::string model_type)
     */
    class View
    {
    public:
      enum ViewType
      {
        VIEW_MODEL_WHERE_OBJECT_ID_AND_MODEL_TYPE
      };

      View(ViewType type)
      {
        type_ = type;
      }

      void
      Initialize(const std::string & arg1, const std::string &arg2)
      {
        switch (type_)
        {
          case VIEW_MODEL_WHERE_OBJECT_ID_AND_MODEL_TYPE:
            //TODO
            parameters_["object_id"] = json_spirit::mValue(arg1);
            parameters_["model_type"] = json_spirit::mValue(arg2);
            break;
          default:
            throw std::runtime_error("Not a valid View type for initialization arguments: std::string, std::string");
            break;
        }
      }
      friend class ObjectDbBase;
    protected:
      ViewType type_;
      json_spirit::mObject parameters_;
    };
  }
}
#endif /* VIEW_TYPES_H_ */
