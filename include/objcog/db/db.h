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

#ifndef DB_H_
#define DB_H_

#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>

namespace db_future
{
//Forward declare some classes
class ObjectDbBase;

typedef std::string CollectionName;
typedef std::string Field;
typedef std::string FieldName;
typedef std::string ObjectId;
typedef std::string RevisionId;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

class ObjectDb
{
public:
  /** Constructor
   * @param params a JSON string containing the parameters for the DB
   */
  ObjectDb(const std::string & json_params);

  void set_attachment(ObjectId & object_id, RevisionId & revision_id, const CollectionName &collection,
                      const std::string& attachment_name, std::istream& stream, const std::string& content_type);

  void get_attachment(const std::string& attachment_name, std::ostream& stream);

  void insert_object(const CollectionName &collection, const boost::property_tree::ptree &fields, ObjectId & object_id,
                     RevisionId & revision_id);

  void load_fields(const ObjectId & object_id, const CollectionName &collection, boost::property_tree::ptree &fields);

  void persist_fields(ObjectId & object_id, RevisionId & revision_id, const CollectionName &collection,
                      const boost::property_tree::ptree &fields);

  void query(const CollectionName &collection, const std::map<FieldName, std::string> &regexps
             , std::vector<ObjectId> & object_ids);

private:
  /** The DB from which we'll get all the info */
  boost::shared_ptr<ObjectDbBase> db_;
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/** A Document holds fields (in the CouchDB sense) which are strings that are queryable, and attachments (that are
 * un-queryable binary blobs)
 */
class Document
{
public:
  Document(ObjectDb & db) :
      db_(db)
  {
  }

  Document(ObjectDb & db, const CollectionName & collection, const ObjectId &object_id) :
      collection_(collection), object_id_(object_id), db_(db)
  {
    // Load all fields from the DB (not the attachments)
    db_.load_fields(object_id_, collection_, fields_);
  }

  virtual ~Document()
  {
  }
  ;

  virtual void Persist()
  {
    // Persist the object if it does not exist in the DB
    if (object_id_.empty())
      db_.insert_object(collection_, fields_, object_id_, revision_id_);
    else
      db_.persist_fields(object_id_, revision_id_, collection_, fields_);

    // Persist the attachments
    boost::any nothing_any;
    for (std::map<FieldName, boost::any>::const_iterator attachment = attachments_.begin(), attachment_end =
                                                             attachments_.end(); attachment != attachment_end;
        ++attachment)
        {
      if (attachment->second.empty())
        continue;
      // Persist the attachment
      // TODO
      //persist_attachment(db_, object_id_, collection_, attachment->first, attachment->second);
    }
  }

  /** Extract a specific field from the pre-loaded Document
   * @param field
   * @param t
   */
  template<typename T>
    void get_attachment(const FieldName &field, T & t) const
    {
      // check if it is loaded
      std::map<FieldName, boost::any>::const_iterator val = attachments_.find(field);
      if ((val != attachments_.end()) && (!val->second.empty()))
      {
        t = *val;
        return;
      }
      else
      {
        // Otherwise, load it from the DB
        // TODO
        //load_attachment(db_, object_id_, collection_, field, t);
        attachments_[field] = t;
      }
    }

  /** Add a specific field to a Document (that has been pre-loaded or not)
   * @param field
   * @param t
   */
  template<typename T>
    void set_attachment(const FieldName &field, const T & t)
    {
      attachments_[field] = t;
    }

  /** Get a specific value */
  template<typename T>
    T get_value(const std::string& key)
    {
      std::cerr << "Document::get_value<T> not implemented for that type";
      throw;
    }

  /** Set a specific value */
  template<typename T>
    void set_value(const std::string& key, const T& val)
    {
      std::cerr << "Document::set_value<T> not implemented for that type";
      throw;
    }

  /** Clear all the fields, there are no fields left after */
  void ClearAllFields();

  /** Remove a specific field */
  void ClearField(const std::string& key);

  /** Set the id and the revision number */
  void SetIdRev(const std::string& id, const std::string& rev);

private:
  bool is_loaded_;
  mutable CollectionName collection_;
  mutable ObjectId object_id_;
  RevisionId revision_id_;
  ObjectDb db_;
  /** contains the attachments: binary blobs */
  std::map<FieldName, boost::any> attachments_;
  /** contains the fields: they are of integral types */
  boost::property_tree::ptree fields_;
};

// Implementation of some specializations
template<>
  inline bool Document::get_value<bool>(const std::string& key)
  {
    return fields_.get<bool>(key);
  }
template<>
  inline int Document::get_value<int>(const std::string& key)
  {
    return fields_.get<int>(key);
  }
template<>
  inline double Document::get_value<double>(const std::string& key)
  {
    return fields_.get<double>(key);
  }
template<>
  inline std::string Document::get_value<std::string>(const std::string& key)
  {
    return fields_.get<std::string>(key);
  }

template<>
  inline void Document::set_value<bool>(const std::string& key, const bool& val)
  {
    fields_.put<bool>(key, val);
  }
template<>
  inline void Document::set_value<int>(const std::string& key, const int& val)
  {
    fields_.put<int>(key, val);
  }
template<>
  inline void Document::set_value<double>(const std::string& key, const double& val)
  {
    fields_.put<double>(key, val);
  }
template<>
  inline void Document::set_value<std::string>(const std::string& key, const std::string& val)
  {
    fields_.put<std::string>(key, val);
  }

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

class QueryIterator : public std::iterator<std::forward_iterator_tag, int>
{
public:
  QueryIterator(ObjectDb& db) :
      db_(db)
  {
  }

  QueryIterator(ObjectDb& db, const CollectionName &collection, const std::vector<std::string> & object_ids) :
      db_(db), collection_(collection), object_ids_(object_ids)
  {
    // Load the first element in the db
    if (object_ids_.empty())
      return;
    object_ = boost::shared_ptr<Document>(new Document(db_, collection_, object_ids_.back()));
    object_ids_.pop_back();
  }

  QueryIterator & operator++()
  {
    // Move forward in the list of Objects to check
    object_ids_.pop_back();
    // Return the end iterator if we are done
    if (object_ids_.empty())
    {
      object_ = boost::shared_ptr<Document>();
    }
    else
    {
      // Fill the current object
      object_ = boost::shared_ptr<Document>(new Document(db_, collection_, object_ids_.back()));
      object_ids_.pop_back();
    }
    return *this;
  }

  bool operator!=(const QueryIterator & query_iterator) const
  {
    if (query_iterator.object_ids_.empty())
      return (!object_ids_.empty());
    if (object_ids_.size() >= query_iterator.object_ids_.size())
      return std::equal(object_ids_.begin(), object_ids_.end(), query_iterator.object_ids_.begin());
    else
      return std::equal(query_iterator.object_ids_.begin(), query_iterator.object_ids_.end(), object_ids_.begin());
  }

  QueryIterator end()
  {
    return QueryIterator(db_);
  }
  friend class Query;
private:
  ObjectDb db_;
  CollectionName collection_;
  boost::shared_ptr<Document> object_;
  std::vector<ObjectId> object_ids_;
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

class Query
{
public:
  Query();

  /** Add requirements for the documents to retrieve
   * @param field a field to match. Only one regex per field will be accepted
   * @param regex the regular expression the field verifies, in TODO format
   */
  void add_where(const FieldName & field, const std::string & regex);

  /** Add collections that should be checked for specific fields
   * @param collection
   */
  void set_collection(CollectionName & collection);

  QueryIterator query(ObjectDb &db)
  {
    // Process the query and get the ids of several objects
    std::vector<ObjectId> object_ids;
    db.query(collection_, regexes_, object_ids);
    return QueryIterator(db, collection_, object_ids);
  }
private:
  CollectionName collection_;
  /** the list of regexes to use */
  std::map<FieldName, std::string> regexes_;
};
}

#endif /* DB_H_ */
