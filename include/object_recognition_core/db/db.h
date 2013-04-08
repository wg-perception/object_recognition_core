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
/** This file defines a base to inherit from for creating a database
 * The databases inheriting from this class will be able to 
 * store/manage/retrieve Documents that are very generic: they are defined
 * by a JSON string and several binary blobs
 */
 
#ifndef ORK_CORE_DB_DB_BASE_H_
#define ORK_CORE_DB_DB_BASE_H_

#include <algorithm>
#include <iterator>
#include <map>
#include <vector>

#include <boost/foreach.hpp>
#include <boost/shared_ptr.hpp>

#include <object_recognition_core/common/types.h>
#include <object_recognition_core/common/json_spirit/json_spirit.h>

#include <object_recognition_core/db/view.h>
#include <object_recognition_core/db/parameters.h>

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

namespace object_recognition_core
{
  namespace db
  {
    /** The main class that interact with the db
     * A collection is similar to the term used in CouchDB. It could be a schema/table in SQL
     * Each inheriting class must have an extra static class with the following signature:
     *   static object_recognition_core::db::ObjectDbParameters default_parameters()
     * The databases that this interface handles are very generic: they store documents
     * that are only defined by a JSON string and several binary blobs. That's it.
     * All relationships between documents therefore have to happen in the JSON.
     * This paradigm is common to non-relational databases (CouchDB, MongoDB)
     * An object is defined by a unique id and a revision number (not all
     * databases support this version number)
     */
    class ObjectDb
    {
    public:
      /** Default constructor
       * Make your children classes have the default parameter: ObjectDbParameters(default_parameters())
       */
      ObjectDb()
      {
      }

      virtual
      ~ObjectDb()
      {
      }

      /** Get the parameters of the database
       * @return an ObjectDbParameters object as stored internally
       */
      const ObjectDbParameters &
      parameters() const
      {
        return parameters_;
      }

      /** @return the raw parameters as a JSON object
       */
      virtual ObjectDbParametersRaw
      default_raw_parameters() const = 0;

      /** Set the parameters of a database: this can only be done from the ObjectDBParameters
       * This is also where internals can be set. The parameter is not const as it can be modified to show what
       * was actually used.
       * @param parameters the parameters to impose to the database
       */
      virtual void
      set_parameters(ObjectDbParameters & parameters)
      {
        parameters_ = parameters;
      }

      /** Insert a document in the database
       * @param fields the JSON description of the document to insert
       * @param document_id the returned id of the document that was inserted
       * @param revision_id the revision number of the inserted object (some 
       * DB provide it)
       */
      virtual void
      insert_object(const or_json::mObject &fields, DocumentId & document_id, RevisionId & revision_id) = 0;

      /** When a document already belongs to a database, its fields can be updated
       * through this function
       * @param document_id the id (unique identifier) of the document to update
       * @param fields the new JSON description of the object to upload
       * @param revision_id the resulting new revision of the object
       */
      virtual void
      persist_fields(const DocumentId & document_id, const or_json::mObject &fields, RevisionId & revision_id) = 0;

      /** Load the JSON fields of an object from the database
       * @param document_id the id (unique identifier) of the document to update
       * @param fields the returned fields as a JSON object
       */
      virtual void
      load_fields(const DocumentId & document_id, or_json::mObject &fields) = 0;

      /** Delete a document of a given id in the database
       * @param id the id of the document to delete
       */
      virtual void
      Delete(const ObjectId & id) = 0;

      /** Execute a given View on the database to find some documents
       * @param view a view object defining a query
       * @param limit_rows a maximum number of queries to return (0 for infinite)
       * @param start_offset the offset at which to return the found documents
       * @param total_rows the total number of elements found
       * @param offset the offset at which the results start_offset
       * @param view_elements a vector of the found elements
       */
      virtual
      void
      QueryView(const View & view, int limit_rows, int start_offset, int& total_rows, int& offset,
                std::vector<Document> & view_elements) = 0;

      /** A generic function to make any query on the database
       * @param queries a set of queries to perform. The strings can mean anything
       * to the database and are therefore specific to a type of database
       * @param limit_rows a maximum number of queries to return (0 for infinite)
       * @param start_offset the offset at which to return the found documents
       * @param total_rows the total number of elements found
       * @param offset the offset at which the results start_offset
       * @param view_elements a vector of the found elements
       */
      virtual void
      QueryGeneric(const std::vector<std::string> & queries, int limit_rows, int start_offset, int& total_rows,
                   int& offset, std::vector<Document> & view_elements) = 0;

      /** Given a Document, set a binary blobs
       * @param document_id the id (unique identifier) of the document to update
       * @param attachment_name the name/key of the binary blob to add
       * @param mime_type the MIME type of the binary blob to add
       * @param stream the binary blob itself
       * @param revision_id the new revision id of the object after insertion/update
       */
      virtual void
      set_attachment_stream(const DocumentId & document_id, const AttachmentName& attachment_name,
                            const MimeType& mime_type, const std::istream& stream, RevisionId & revision_id)=0;

      /** Given a Document, get one of its binary blobs
       * @param document_id the id (unique identifier) of the document to update
       * @param revision_id the revision id of the object
       * @param attachment_name the name/key of the binary blob to add
       * @param mime_type the MIME type of the binary blob to add
       * @param stream the binary blob itself
       */
      virtual void
      get_attachment_stream(const DocumentId & document_id, const RevisionId & revision_id,
                            const AttachmentName& attachment_name,
                            const MimeType& mime_type, std::ostream& stream)=0;

      /** @return a string that defines the status of the database
       */
      virtual std::string
      Status() const = 0;

      /** @return a string that defines the status of the database for a given collection
       */
      virtual std::string
      Status(const CollectionName& collection) const = 0;

      /** Create a new collection in the database
       * @param collection the name of the collection to create
       */
      virtual void
      CreateCollection(const CollectionName &collection) = 0;

      /** Delete a new collection in the database
       * @param collection the name of the collection to delete
       */
      virtual void
      DeleteCollection(const CollectionName &collection) = 0;

      /** The type of the DB : e.g. 'CouchDB' ...
       * @return one of the enum defining the possible types
       */
      virtual DbType
      type() const = 0;

    protected:
      /** The parameters of the current DB */
      ObjectDbParameters parameters_;
    };

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    typedef boost::shared_ptr<ObjectDb> ObjectDbPtr;
    typedef boost::shared_ptr<const ObjectDb> ObjectDbConstPtr;
  }
}

#endif // ORK_CORE_DB_DB_BASE_H_
