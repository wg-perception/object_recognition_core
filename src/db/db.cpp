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
#include "object_recognition/db/db.h"

namespace db_future
{

ObjectDb::ObjectDb(const std::string & json_params)
{
  boost::property_tree::ptree params;
  std::stringstream ssparams;
  ssparams << json_params;
  boost::property_tree::read_json(ssparams, params);

  std::string db_type = params.get<std::string>("type");
  if (db_type == "couch")
  {
    db_ = boost::shared_ptr<ObjectDbBase>(new ObjectDbCouch(params.get<std::string>("url")));
  }
}

void ObjectDb::set_attachment(ObjectId & object_id, RevisionId & revision_id, const CollectionName &collection,
                              const std::string& attachment_name, std::istream& stream, const std::string& content_type)
{
}

void ObjectDb::get_attachment(const std::string& attachment_name, std::ostream& stream)
{
}

void ObjectDb::insert_object(const CollectionName &collection, const boost::property_tree::ptree &fields,
                             ObjectId & object_id, RevisionId & revision_id)
{
  db_->insert_object(collection, fields, object_id, revision_id);
}

void ObjectDb::load_fields(const ObjectId & object_id, const CollectionName &collection,
                           boost::property_tree::ptree &fields)
{
  //TODO
}

void ObjectDb::persist_fields(ObjectId & object_id, RevisionId & revision_id, const CollectionName &collection,
                              const boost::property_tree::ptree &fields)
{ //TODO
}

void ObjectDb::query(const CollectionName &collection, const std::map<FieldName, std::string> &regexps
                     , std::vector<ObjectId> & object_ids)
{ //TODO
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Document::ClearAllFields()
{
  fields_.clear();
}

void Document::ClearField(const std::string& key)
{
  fields_.erase(key);
}

void Document::SetIdRev(const std::string& id, const std::string& rev)
{
  object_id_ = id;
  revision_id_ = rev;
  set_value<std::string>("_id", id);
  set_value<std::string>("_rev", rev);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

Query::Query()
{
}

/** Add requirements for the documents to retrieve
 * @param field a field to match. Only one regex per field will be accepted
 * @param regex the regular expression the field verifies, in TODO format
 */
void Query::add_where(const FieldName & field, const std::string & regex)
{
  regexes_[field] = regex;
}

/** Add collections that should be checked for specific fields
 * @param collection
 */
void Query::set_collection(CollectionName & collection)
{
  collection_ = collection;
}

/** Perform the query itself
 * @param db The db on which the query is performed
 * @return an Iterator that will iterate over each result
 */
QueryIterator Query::query(ObjectDb &db)
{
  // Process the query and get the ids of several objects
  std::vector<ObjectId> object_ids;
  db.query(collection_, regexes_, object_ids);
  return QueryIterator(db, collection_, object_ids);
}
}
