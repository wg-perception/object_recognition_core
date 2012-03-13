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

#include <boost/foreach.hpp>
#include <boost/function.hpp>

#include <object_recognition_core/common/types.h>
#include <object_recognition_core/common/json_spirit/json_spirit.h>
#include <object_recognition_core/db/view.h>

namespace object_recognition_core
{
  namespace db
  {
    //Forward declare some classes
    class ObjectDbBase;
    class View;
    class ViewElement;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    /** A class that stores the common parameters for the object DB
     * If it is not from a type provided by object_recognition_core, it is of type NONCORE
     */
    class ObjectDbParameters
    {
    public:
      enum ObjectDbType
      {
        EMPTY, COUCHDB, FILESYSTEM, NONCORE
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
      ObjectDbParameters(const or_json::mObject& params);

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
      /** All the raw parameters: they are of integral types */
      or_json::mObject raw_;
    protected:
      void
      FillParameters(const or_json::mObject& json_params);
    };

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    class ObjectDb
    {
    public:
      typedef boost::function<void
      (int limit_rows, int start_offset, int& total_rows, int& offset, std::vector<ViewElement> &)> QueryFunction;

      ObjectDb()
      {
      }

      ObjectDb(const ObjectDb &db)
          :
            db_(db.db_),
            parameters_(db.parameters_)
      {
      }

      /** Constructor
       * @param in_params any class that inherits from ObjectDbBaseParameters
       */
      explicit
      ObjectDb(const ObjectDbParameters &in_params);

      ObjectDb&
      operator=(const ObjectDb &db)
      {
        db_ = db.db_;
        parameters_ = db.parameters_;
        return *this;
      }

      void
      set_parameters(const ObjectDbParameters &in_params);
      void
      set_db_and_parameters(const boost::shared_ptr<object_recognition_core::db::ObjectDbBase>&,
                            const object_recognition_core::db::ObjectDbParameters&);

      /*** Get the parameters */
      const ObjectDbParameters &
      parameters() const
      {
        return parameters_;
      }

      void
      get_attachment_stream(const DocumentId & document_id, const AttachmentName& attachment_name,
                            MimeType& content_type, std::ostream& stream, RevisionId & revision_id) const;

      void
      set_attachment_stream(const DocumentId & document_id, const AttachmentName& attachment_name,
                            const MimeType& content_type, const std::istream& stream, RevisionId & revision_id) const;

      void
      insert_object(const or_json::mObject &fields, DocumentId & document_id, RevisionId & revision_id) const;

      void
      load_fields(const DocumentId & document_id, or_json::mObject &fields) const;

      void
      persist_fields(const DocumentId & document_id, const or_json::mObject &fields, RevisionId & revision_id) const;

      void
      Delete(const ObjectId & id) const;

      QueryFunction
      Query(const View &view) const;

      std::string
      Status();

      std::string
      Status(const CollectionName& collection);

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
      Query_(const View &view, int limit_rows, int start_offset, int& total_rows, int& offset,
             std::vector<ViewElement> & view_elements);

      /** The DB from which we'll get all the info */
      boost::shared_ptr<ObjectDbBase> db_;
      /** The parameters of the current DB */
      ObjectDbParameters parameters_;
    };

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    /** A Document holds fields (in the CouchDB sense) which are strings that are queryable, and attachments (that are
     * un-queryable binary blobs)
     */
    class Document: public DummyDocument
    {
    public:
      Document();
      ~Document();
      Document(const ObjectDb & db);
      Document(const ObjectDb & db, const DocumentId &document_id);

      bool
      operator==(const Document & document) const
      {
        return document_id_ == document.document_id_;
      }

      /**
       * Update the db that this document should be associated with.
       * @param db
       */
      void
      update_db(const ObjectDb& db);

      /** Persist your object to a given DB
       */
      void
      Persist();

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

      /** Extract a specific attachment from a document in the DB
       * @param db the db to read from
       * @param attachment_name
       * @param value
       * @param do_use_cache if true, try to load and store data in the object itself
       */
      template<typename T>
      void
      get_attachment(const AttachmentName &attachment_name, T & value) const;

      /** Extract the stream of a specific attachment for a Document from the DB
       * @param db the db to read from
       * @param attachment_name the name of the attachment
       * @param stream the string of data to write to
       * @param mime_type the MIME type as stored in the DB
       * @param do_use_cache if true, try to load and store data in the object itself
       */
      virtual void
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
      get_attachment_and_cache(const AttachmentName &attachment_name, T & value);

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
      set_attachment(const AttachmentName &attachment_name, const T & value);
    private:
      ObjectDb db_;
      DocumentId document_id_;
      RevisionId revision_id_;
    };

#ifdef CV_MAJOR_VERSION
    // Specializations for cv::Mat
    template<>
    void
    DummyDocument::get_attachment<cv::Mat>(const AttachmentName &attachment_name, cv::Mat & value) const;

    template<>
    void
    DummyDocument::set_attachment<cv::Mat>(const AttachmentName &attachment_name, const cv::Mat & value);

    template<>
    void
    Document::get_attachment_and_cache<cv::Mat>(const AttachmentName &attachment_name, cv::Mat & value);
#endif

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    typedef std::vector<Document> Documents;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    class ViewIterator
    {
    public:
      static const unsigned int BATCH_SIZE;
      ViewIterator();

      ViewIterator(const View &view, const ObjectDb& db);

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

      bool
      operator!=(const ViewIterator & document_view) const;

      ViewElement
      operator*() const;
    private:
      std::vector<ViewElement> view_elements_;
      int start_offset_;
      int total_rows_;
      /** The strings to send to the db_ to perform the query */
      ObjectDb::QueryFunction query_;
      ObjectDb db_;
    };
  }
}

#endif /* DB_H_ */
