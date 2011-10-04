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

#include <vector>
#include <iostream>

#include <cxxabi.h>
#include <string>
#include <cstdlib>

#include <boost/format.hpp>

#include "object_recognition/db/utils.h"

namespace object_recognition
{
  std::string
  name_of(const std::type_info &ti)
  {
    const static std::string typename_notavailable = "N/A";

    const char* mangled = ti.name();

    if (!mangled)
    {
      return typename_notavailable;
    }

    int status;

    char* demangled = abi::__cxa_demangle(mangled, 0, 0, &status);

    std::string rv;

    if (status != 0)
      rv = mangled;
    else
      rv = demangled ? demangled : typename_notavailable;

    std::free(demangled);
    return rv;
  }
}

namespace object_recognition
{
  namespace db_future
  {
    namespace couch
    {
      // Utilities for creating queries for CouchDB
#define STRINGYFY(A) #A
      std::string
      WhereDocId(const std::string & object_id)
      {
        return boost::str(boost::format(STRINGYFY(
            function(doc)
            {
              if(doc._id == "%s")
                emit(null,doc);
            }
        ))
                % object_id);
      }

      std::string
      WhereObjectId(const std::string & object_id)
      {
        return boost::str(boost::format(STRINGYFY(
            function(doc)
            {
              if(doc.object_id == "%s")
                emit(null,doc);
            }
        ))
                % object_id);
      }

      std::string
      WhereSessionId(const std::string& session_id)
      {
        return boost::str(boost::format(STRINGYFY(
            function(doc)
            {
              if(doc.session_id == "%s")
              emit("frame_number",doc.frame_number);
            }
        ))
                % session_id);
      }

    }
  }
}
