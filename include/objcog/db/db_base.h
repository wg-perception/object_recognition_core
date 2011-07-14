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
class ObjectDbBase
{
public:
  virtual ~ObjectDbBase()
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
