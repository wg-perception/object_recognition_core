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
#include <map>

#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/foreach.hpp>
#include <boost/function.hpp>
#include <boost/property_tree/ptree.hpp>

#include <opencv2/core/core.hpp>

#include "object_recognition/common/types.h"
#include "object_recognition/db/view_types.h"

namespace object_recognition
{
  namespace db
  {
    //Forward declare some classes
    class ObjectDbBase;
    class ViewIterator;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    /** A class that stores the common parameters for the object DB */
    class ObjectDbParameters
    {
    public:
      enum ObjectDbType
      {
        EMPTY, COUCHDB
      };
      ObjectDbParameters();

      /** Default constructor for certain types
       * @param type Default type
       */
      explicit
      ObjectDbParameters(const std::string& type);

      /**
       * @param params A map between some db parameters and their value
       */
      explicit
      ObjectDbParameters(const std::map<std::string, std::string>& params);

      static ObjectDbType
      StringToType(const std::string & type);

      static std::string
      TypeToString(const ObjectDbParameters::ObjectDbType & type);

      /** The collection where the data is stored (or schema in certain naming conventions) */
      std::string collection_;
      /** The base url/path of where the DB is located */
      std::string root_;
      /** The type of the collection 'CouchDB' ... */
      ObjectDbType type_;
      /** All the raw parameters */
      std::map<std::string, std::string> all_parameters_;
    protected:
      void
      FillParameters(const std::map<std::string, std::string>& json_params);
    };

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    class ObjectDb
    {
    public:
      typedef boost::function<void
      (int limit_rows, int start_offset, int& total_rows, int& offset, std::vector<DocumentId> &)> QueryFunction;

      ObjectDb()
      {
      }

      /** Constructor
       * @param in_params any class that inherits from ObjectDbBaseParameters
       */
      ObjectDb(const std::string& json_params);
      ObjectDb(const ObjectDbParameters &in_params);

      void
      set_params(const std::string& json_params);
      void
      set_params(const ObjectDbParameters &in_params);

      /*** Get the parameters */
      const ObjectDbParameters &
      get_params() const;

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
      Delete(const ObjectId & id, const CollectionName & collection_name) const;

      QueryFunction
      Query(const View &view, const CollectionName & collection_name) const;

      void
      Status(std::string& status);

      void
      Status(const CollectionName& collection, std::string& status);

      void
      CreateCollection(const CollectionName &collection);

      void
      DeleteCollection(const CollectionName &collection);

      /** The type of the DB
       * @return The type of the DB as a string
       */
      DbType
      type();
    private:
      void
      Query_(const View &view, const CollectionName & collection_name, int limit_rows, int start_offset,
             int& total_rows, int& offset, std::vector<DocumentId> & document_ids);

      /** The DB from which we'll get all the info */
      boost::shared_ptr<ObjectDbBase> db_;
      /** The parameters of the current DB */
      ObjectDbParameters db_parameters_;
    };

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    /** A Document holds fields (in the CouchDB sense) which are strings that are queryable, and attachments (that are
     * un-queryable binary blobs)
     */
    class Document
    {
    public:
      Document();
      ~Document();
      Document(const ObjectDb & db, const CollectionName & collection);
      Document(const ObjectDb & db, const CollectionName & collection, const DocumentId &document_id);

      /** Persist your object to a given DB
       */
      void
      Persist();

      /** Extract a specific attachment from a document in the DB
       * @param db the db to read from
       * @param attachment_name
       * @param value
       * @param do_use_cache if true, try to load and store data in the object itself
       */
      template<typename T>
      void
      get_attachment(const AttachmentName &attachment_name, T & value) const
      {
        typedef boost::archive::binary_iarchive InputArchive;
        std::stringstream stream;
        std::string tmp_mime_type;
        get_attachment_stream(attachment_name, stream, tmp_mime_type);
        stream.seekg(0);
        InputArchive ar(stream);
        ar & value;
      }

      /** Extract a specific attachment from a document in the DB
       * @param db the db to read from
       * @param attachment_name
       * @param value
       * @param do_use_cache if true, try to load and store data in the object itself
       */
      template<typename T>
      void
      get_attachment_and_cache(const AttachmentName &attachment_name, T & value)
      {
        typedef boost::archive::binary_iarchive InputArchive;
        std::stringstream stream;
        std::string tmp_mime_type;
        get_attachment_stream_and_cache(attachment_name, stream, tmp_mime_type);
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
      get_attachment_stream(const AttachmentName &attachment_name, std::ostream& stream, MimeType mime_type =
          MIME_TYPE_DEFAULT) const;

      /** Extract the stream of a specific attachment for a Document from the DB
       * @param db the db to read from
       * @param attachment_name the name of the attachment
       * @param stream the string of data to write to
       * @param mime_type the MIME type as stored in the DB
       * @param do_use_cache if true, try to load and store data in the object itself
       */
      void
      get_attachment_stream_and_cache(const AttachmentName &attachment_name, std::ostream& stream, MimeType mime_type =
          MIME_TYPE_DEFAULT);

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
        return fields_.get<T>(key);
      }

      /** Set a specific value */
      template<typename T>
      void
      set_value(const std::string& key, const T& val)
      {
        fields_.put<T>(key, val);
      }

      /** Set several values by inserting a property tree */
      void
      set_values(const boost::property_tree::ptree & property_tree)
      {
        fields_.insert(fields_.begin(), property_tree.begin(), property_tree.end());
      }

      /** Set several values by inserting a property tree at a specific key*/
      void
      set_values(const std::string& key, const boost::property_tree::ptree & property_tree)
      {
        if (fields_.find(key) == fields_.not_found())
          fields_.insert(fields_.begin(), std::make_pair(key, property_tree));
        else
          fields_.insert(fields_.get_child(key).begin(), property_tree.begin(), property_tree.end());
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

      const std::string &
      id() const
      {
        return document_id_;
      }

      const std::string &
      rev() const
      {
        return revision_id_;
      }
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

      ObjectDb db_;
      CollectionName collection_;
      DocumentId document_id_;
      RevisionId revision_id_;
      typedef std::map<AttachmentName, StreamAttachment::ptr> AttachmentMap;
      AttachmentMap attachments_;
      /** contains the fields: they are of integral types */
      boost::property_tree::ptree fields_;
    };

    // Specializations for cv::Mat
    template<>
    void
    Document::get_attachment<cv::Mat>(const AttachmentName &attachment_name, cv::Mat & value) const;

    template<>
    void
    Document::get_attachment_and_cache<cv::Mat>(const AttachmentName &attachment_name, cv::Mat & value);

    template<>
    void
    Document::set_attachment<cv::Mat>(const AttachmentName &attachment_name, const cv::Mat & value);

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    typedef std::vector<Document> Documents;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    class ViewIterator
    {
    public:
      static const unsigned int BATCH_SIZE;
      ViewIterator();

      ViewIterator(const View &view, ObjectDb& db, const CollectionName & collection_name)
          :
            query_(db.Query(view, collection_name)),
            collection_(collection_name),
            db_(db)
      {
      }

      /** Perform the query itself
       * @param db The db on which the query is performed
       * @return an Iterator that will iterate over each result
       */
      ViewIterator &
      begin();

      /** Perform the query itself
       * @param db The db on which the query is performed
       * @return an Iterator that will iterate over each result
       */
      static ViewIterator
      end();

      ViewIterator &
      operator++();

      /** Set the db on which to perform the Query
       * @param db The db on which the query is performed
       */
      void
      set_db(const ObjectDb & db);

      /** Set the collection on which to perform the Query. This might be part of the views_
       * and unnecessary for certain DB's
       * @param collection The collection on which the query is performed
       */
      void
      set_collection(const CollectionName & collection);

      bool
      operator!=(const ViewIterator & document_view) const;

      Document
      operator*() const;
    private:
      std::vector<DocumentId> document_ids_;
      int start_offset_;
      int total_rows_;
      /** The strings to send to the db_ to perform the query */
      ObjectDb::QueryFunction query_;
      CollectionName collection_;
      ObjectDb db_;
    };

  }
}

#endif /* DB_H_ */
