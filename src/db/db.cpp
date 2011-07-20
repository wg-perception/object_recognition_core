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

#include "db_base.h"
#include "db_couch.h"
#include "objcog/db/db.h"

namespace db_future
{

ObjectDb::ObjectDb(const std::string & json_params)
{
  boost::property_tree::ptree params;
  std::stringstream ssparams;
  ssparams << json_params;
  boost::property_tree::read_json(ssparams, params);

  std::string db_type = params.get < std::string > ("type");
  if (db_type == "couch")
  {
    db_ = boost::shared_ptr < ObjectDbBase > (new ObjectDbCouch(params.get < std::string > ("url")));
  }
}

void ObjectDb::set_attachment(ObjectId & object_id, RevisionId & revision_id, const CollectionName &collection,
                              const std::string& attachment_name, std::istream& stream, const std::string& content_type)
{
}

void ObjectDb::get_attachment(const std::string& attachment_name, std::ostream& stream)
{
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Document::clear_all_fields()
{
  fields_.clear();
}

void Document::clear_field(const std::string& key)
{
  fields_.erase(key);
}

void Document::SetIdRev(const std::string& id, const std::string& rev)
{
  object_id_ = id;
  revision_id_ = rev;
  set_value < std::string > ("_id", id);
  set_value < std::string > ("_rev", rev);
}

// Implementation of the class functions
template<>
  bool Document::get_value<bool>(const std::string& key)
  {
    return fields_.get<bool>(key);
  }
template<>
  int Document::get_value<int>(const std::string& key)
  {
    return fields_.get<int>(key);
  }
template<>
  double Document::get_value<double>(const std::string& key)
  {
    return fields_.get<double>(key);
  }
template<>
  std::string Document::get_value<std::string>(const std::string& key)
  {
    return fields_.get < std::string > (key);
  }

template<>
  void Document::set_value<bool>(const std::string& key, const bool& val)
  {
    fields_.put<bool>(key, val);
  }
template<>
  void Document::set_value<int>(const std::string& key, const int& val)
  {
    fields_.put<int>(key, val);
  }
template<>
  void Document::set_value<double>(const std::string& key, const double& val)
  {
    fields_.put<double>(key, val);
  }
/*template<>
 void Document::set_value<std::string>(const std::string& key, const std::string& val)
 {
 fields_.put<std::string>(key, val);
 }*/
}
