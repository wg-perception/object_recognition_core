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

#ifndef DB_H_
#define DB_H_

#include <sstream>

#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include "utils.h"

namespace object_recognition
{
  namespace db_future
  {
    //Forward declare some classes
    class ObjectDbBase;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    class ObjectDb
    {
    public:
      static const std::string JSON_PARAMS_EMPTY_DB;

      /** Constructor
       * @param params a JSON string containing the parameters for the DB. Depending on the type of DB, it should have the
       * following formatting:
       *    - empty DB: {"type": "empty"}
       *    - CouchDB: {"type": "CouchDB", "url": "whatever_url_you_want:whatever_port"}
       */
      explicit
      ObjectDb(const std::string & json_params = JSON_PARAMS_EMPTY_DB);
      explicit
      ObjectDb(const boost::property_tree::ptree& params);

      /** Set the parameters of the DB.
       * @param json_params string that follows the conventions of the constructor
       */
      void
      set_params(const std::string & json_params = JSON_PARAMS_EMPTY_DB);

      void
      set_params(const boost::property_tree::ptree& pt);

      void
      get_attachment_stream(const DocumentId & document_id, const CollectionName &collection,
                            const AttachmentName& attachment_name, MimeType& content_type, std::ostream& stream,
                            RevisionId & revision_id) const;

      void
      set_attachment_stream(const DocumentId & document_id, const CollectionName &collection,
                            const AttachmentName& attachment_name, const MimeType& content_type,
                            const std::istream& stream, RevisionId & revision_id) const;

      void
      insert_object(const CollectionName &collection, const boost::property_tree::ptree &fields,
                    DocumentId & document_id, RevisionId & revision_id) const;

      void
      load_fields(const DocumentId & document_id, const CollectionName &collection,
                  boost::property_tree::ptree &fields) const;

      void
      persist_fields(const DocumentId & document_id, const CollectionName &collection,
                     const boost::property_tree::ptree &fields, RevisionId & revision_id) const;

      void
      Query(const std::vector<std::string> & queries, const CollectionName & collection_name, int limit_rows,
            int start_offset, int& total_rows, int& offset, std::vector<DocumentId> & document_ids) const;

      /** The type of the DB
       * @return The type of the DB as a string
       */
      DbType
      type();
    private:
      /** Set the db_ using a property tree
       * @params the boost property tree containing the different parameters
       */
      void
      set_db(const boost::property_tree::ptree& params);

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
      Document()
      {
      }

      Document(const ObjectDb & db, const CollectionName & collection, const DocumentId &document_id)
          :
            collection_(collection),
            document_id_(document_id)
      {
        // Load all fields from the DB (not the attachments)
        db.load_fields(document_id_, collection_, fields_);
      }

      virtual
      ~Document()
      {
      }

      /** Persist your object to a given DB
       * @param db the DB to persist to
       * @param collection the collection/schema where it should be saved
       */
      virtual void
      Persist(ObjectDb & db, const CollectionName & collection);

      /** Extract a specific field from the pre-loaded Document
       * @param attachment_name
       * @param value
       */
      template<typename T>
      void
      get_attachment(const AttachmentName &attachment_name, T & value) const
      {
        typedef boost::archive::binary_iarchive InputArchive;
        std::stringstream stream;
        get_attachment_stream(attachment_name, stream);
        stream.seekg(0);
        InputArchive ar(stream);
        ar & value;
      }

      /** Extract the stream of a specific attachment from the pre-loaded Document
       * @param attachment_name the name of the attachment
       * @param stream the string of data to write to
       * @param mime_type the MIME type as stoerd in the DB
       */
      void
      get_attachment_stream(const AttachmentName &attachment_name, std::ostream& stream, MimeType mime_type =
          MIME_TYPE_DEFAULT) const;

      /** Extract a specific attachment from a document in the DB
       * @param db the db to read from
       * @param attachment_name
       * @param value
       * @param do_use_cache if true, try to load and store data in the object itself
       */
      template<typename T>
      void
      get_attachment(ObjectDb & db, const AttachmentName &attachment_name, T & value, bool do_use_cache = true) const
      {
        typedef boost::archive::binary_iarchive InputArchive;
        std::stringstream stream;
        std::string tmp_mime_type;
        get_attachment_stream(db, attachment_name, stream, tmp_mime_type, do_use_cache);
        stream.seekg(0);
        InputArchive ar(stream);
        ar & value;
      }

      /** Extract the stream of a specific attachment for a Document from the DB
       * @param db the db to read from
       * @param attachment_name the name of the attachment
       * @param stream the string of data to write to
       * @param mime_type the MIME type as stored in the DB
       * @param do_use_cache if true, try to load and store data in the object itself
       */
      void
      get_attachment_stream(ObjectDb & db, const AttachmentName &attachment_name, std::ostream& stream,
                            MimeType mime_type = MIME_TYPE_DEFAULT, bool do_use_cache = true) const;

      /** Add a specific field to a Document (that has been pre-loaded or not)
       * @param attachment_name the name of the attachment
       * @param value the attachment itself, that needs to be boost serializable
       */
      template<typename T>
      void
      set_attachment(const AttachmentName &attachment_name, const T & value)
      {
        typedef boost::archive::binary_oarchive OutputArchive;
        std::stringstream ss;
        OutputArchive ar(ss);
        ar & value;
        set_attachment_stream(attachment_name, ss);
      }

      /** Add a stream attachment to a a Document
       * @param attachment_name the name of the stream
       * @param stream the stream itself
       * @param content_type the MIME type of the stream
       */
      void
      set_attachment_stream(const AttachmentName &attachment_name, const std::istream& stream,
                            const MimeType& mime_type = MIME_TYPE_DEFAULT);

      /** Get a specific value */
      template<typename T>
      T
      get_value(const std::string& key) const
      {
        std::cerr << "Document::get_value<T> not implemented for that type";
        throw;
      }

      /** Set a specific value */
      template<typename T>
      void
      set_value(const std::string& key, const T& val)
      {
        std::cerr << "Document::set_value<T> not implemented for that type";
        throw;
      }

      /** Clear all the fields, there are no fields left after */
      void
      ClearAllFields();

      /** Remove a specific field */
      void
      ClearField(const std::string& key);

      /** Set the id and the revision number */
      void
      SetIdRev(const std::string& id, const std::string& rev);

    private:
      /** contains the attachments: binary blobs */
      struct StreamAttachment: boost::noncopyable
      {
        StreamAttachment()
        {
        }

        StreamAttachment(const MimeType &type)
            :
              type_(type)
        {
        }

        StreamAttachment(const MimeType &type, const std::istream &stream)
            :
              type_(type)
        {
          copy_from(stream);
        }
        void
        copy_from(const std::istream& stream)
        {
          stream_ << stream.rdbuf();
          stream_.seekg(0);
        }
        MimeType type_;
        std::stringstream stream_;
        typedef boost::shared_ptr<StreamAttachment> ptr;
      };

      CollectionName collection_;
      DocumentId document_id_;
      mutable RevisionId revision_id_;
      typedef std::map<AttachmentName, StreamAttachment::ptr> AttachmentMap;
      mutable AttachmentMap attachments_;
      /** contains the fields: they are of integral types */
      boost::property_tree::ptree fields_;
    };

    // Implementation of some specializations
    template<>
    inline bool
    Document::get_value<bool>(const std::string& key) const
    {
      return fields_.get<bool>(key);
    }
    template<>
    inline int
    Document::get_value<int>(const std::string& key) const
    {
      return fields_.get<int>(key);
    }
    template<>
    inline double
    Document::get_value<double>(const std::string& key) const
    {
      return fields_.get<double>(key);
    }
    template<>
    inline std::string
    Document::get_value<std::string>(const std::string& key) const
    {
      return fields_.get<std::string>(key);
    }

    template<>
    inline void
    Document::set_value<bool>(const std::string& key, const bool& val)
    {
      fields_.put<bool>(key, val);
    }
    template<>
    inline void
    Document::set_value<int>(const std::string& key, const int& val)
    {
      fields_.put<int>(key, val);
    }
    template<>
    inline void
    Document::set_value<double>(const std::string& key, const double& val)
    {
      fields_.put<double>(key, val);
    }
    template<>
    inline void
    Document::set_value<std::string>(const std::string& key, const std::string& val)
    {
      fields_.put<std::string>(key, val);
    }

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    class DocumentView
    {
    public:
      static const unsigned int BATCH_SIZE;

      DocumentView();

      /** Add collections that should be checked for specific fields
       * @param collection
       */
      void
      set_db(const ObjectDb & db);

      /** Set the collection on which to perform the Query. This might be part of the views_
       * and unnecessary for certain DB's
       * @param collection The collection on which the query is performed
       */
      void
      set_collection(const CollectionName & collection);

      /** Add requirements for the documents to retrieve
       * @param db_type the type of the db that will be used
       * @param view a View that will filter the Documents. The format depends on your ObjectDb
       */
      void
      AddView(const DbType &db_type, const View & view);

      /** Perform the query itself
       * @param db The db on which the query is performed
       * @return an Iterator that will iterate over each result
       */
      DocumentView &
      begin();

      /** Perform the query itself
       * @param db The db on which the query is performed
       * @return an Iterator that will iterate over each result
       */
      static DocumentView
      end();

      DocumentView &
      operator++();

      bool
      operator!=(const DocumentView & document_view) const;

      Document
      operator*() const;
    private:
      ObjectDb db_;
      CollectionName collection_;
      std::vector<DocumentId> document_ids_;
      int start_offset_;
      int total_rows_;
      /** The strings to send to the db_ to perform the query, as well as for which db they are meant */
      struct PodView
      {
        PodView(const DbType &db_type, const View &view)
            :
              view_(view),
              db_type_(db_type)
        {
        }
        View view_;
        DbType db_type_;
      };
      std::vector<PodView> pod_views_;
      /** The strings to send to the db_ to perform the query */
      std::vector<View> views_;
    };
  }
}

#endif /* DB_H_ */
