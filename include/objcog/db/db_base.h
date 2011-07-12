/*
 * query.h
 *
 *  Created on: Jun 3, 2011
 *      Author: vrabaud
 */

#include <algorithm>
#include <iterator>
#include <map>
#include <vector>

#include <boost/foreach.hpp>
#include <boost/any.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>

#include <boost/shared_ptr.hpp>
#include "opencv2/core/core.hpp"

typedef std::string CollectionName;
typedef std::string Field;
typedef std::string FieldName;
typedef std::string ObjectId;
typedef std::string RevisionId;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

namespace db_serialization
{
template<class Archive, typename T>
  void save(Archive & ar, const T & m);

template<class Archive, typename T>
  void load(const Archive & ar, T & m);
} // namespace serialization

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/** The main class that interact with the db
 * A collection is similar to the term used in CouchDB. It could be a schema/table in SQL
 */
class ObjectDb
{
public:
  virtual ~ObjectDb()
  {
  }
  virtual ObjectId insert_object(const CollectionName &collection, const std::map<FieldName, Field> &fields) const = 0;

  virtual void persist_fields(const ObjectId & object_id, const CollectionName &collection,
                              const std::map<FieldName, Field> &fields) const = 0;

  virtual void load_fields(const ObjectId & object_id, const CollectionName &collection,
                           std::map<FieldName, Field> &fields) const = 0;

  virtual void query(const CollectionName &collection, const std::map<FieldName, std::string> &regexps
                     , std::vector<ObjectId> & object_ids) const = 0;
};

template<typename DbType, typename Attachment>
  void load_attachment(const DbType&db, const ObjectId & object_id, const CollectionName &collection,
                       const FieldName &field_name, Attachment &attachment);

template<typename DbType, typename Attachment>
  void persist_attachment(const DbType&db, const ObjectId & object_id, const CollectionName &collection,
                          const FieldName &field, Attachment &attachment);

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/** A Document holds fields (in the CouchDB sense) which are strings that are queryable, and attachments (that are
 * un-queryable binary blobs)
 */
class Document
{
public:
  Document(const ObjectDb & db) :
      db_(&db)
  {
  }

  Document(const ObjectDb & db, const CollectionName & collection, const ObjectId &object_id) :
      object_id_(object_id), collection_(collection), db_(&db)
  {
    // Load all fields from the DB (not the attachments)
    db_->load_fields(object_id_, collection_, fields_);
  }

  virtual ~Document();

  virtual void persist() const
  {
    // Persist the object if it does not exist in the DB
    if (object_id_.empty())
      object_id_ = db_->insert_object(collection_, fields_);
    else
      db_->persist_fields(object_id_, collection_, fields_);

    // Persist the attachments
    boost::any nothing_any;
    for (std::map<FieldName, boost::any>::const_iterator attachment = attachments_.begin(), attachment_end =
                                                             attachments_.end(); attachment != attachment_end;
        ++attachment)
        {
      if (attachment->second.empty())
        continue;
      // Persist the attachment
      persist_attachment(db_, object_id_, collection_, attachment->first, attachment->second);
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
        // TODO : maybe we want to load all attachments first
        load_attachment(db_, object_id_, collection_, field, t);
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
private:
  bool is_loaded_;
  mutable ObjectId object_id_;
  mutable CollectionName collection_;
  RevisionId revision_id;
  boost::shared_ptr<ObjectDb> db_;
  std::map<FieldName, boost::any> attachments_;
  std::map<FieldName, Field> fields_;
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

class QueryIterator : public std::iterator<std::forward_iterator_tag, int>
{
public:
  QueryIterator()
  {
  }

  QueryIterator(const ObjectDb& db, const CollectionName &collection, const std::vector<std::string> & object_ids) :
      db_(&db), collection_(collection), object_ids_(object_ids)
  {
    // Load the first element in the db
    if (object_ids_.empty())
      return;
    object_ = boost::shared_ptr < Document > (new Document(*db_, collection_, object_ids_.back()));
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
      object_ = boost::shared_ptr < Document > (new Document(*db_, collection_, object_ids_.back()));
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
    return QueryIterator();
  }
  friend class Query;
private:
  boost::shared_ptr<ObjectDb> db_;
  CollectionName collection_;
  boost::shared_ptr<Document> object_;
  std::vector<ObjectId> object_ids_;
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

class Query
{
  Query();

  /** Add requirements for the documents to retrieve
   * @param field a field to match
   * @param regex the regular expression the field verifies, in TODO format
   */
  void add_where(FieldName & field, std::string & regex);

  /** Add collections that should be checked for specific fields
   * @param collection
   */
  void set_collection(CollectionName & collection);

  QueryIterator query(const ObjectDb &db)
  {
    // Process the query and get the ids of several objects
    std::vector<ObjectId> object_ids;
    db.query(collection_, regexes_, object_ids);
    return QueryIterator(db, collection_, object_ids);
  }
private:
  CollectionName collection_;
  std::map<FieldName, std::string> regexes_;
};

