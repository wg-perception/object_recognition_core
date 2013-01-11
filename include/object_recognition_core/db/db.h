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

#ifndef ORK_CORE_DB_DB_H_
#define ORK_CORE_DB_DB_H_

#include <sstream>
#include <map>

#include <boost/bind.hpp>
#include <boost/foreach.hpp>
#include <boost/function.hpp>

#include <object_recognition_core/common/types.h>
#include <object_recognition_core/common/json_spirit/json_spirit.h>
#include <object_recognition_core/db/db_base.h>
#include <object_recognition_core/db/view.h>

namespace object_recognition_core
{
  namespace db
  {
    //Forward declare some classes
    class View;
    class ViewElement;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    /** A Document holds fields (in the CouchDB sense) which are strings that are queryable, and attachments (that are
     * un-queryable binary blobs)
     */
    class Document: public DummyDocument
    {
    public:
      Document();
      ~Document();
      Document(const ObjectDbPtr & db);
      Document(const ObjectDbPtr & db, const DocumentId &document_id);

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
      update_db(const ObjectDbPtr& db);

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
       * @param attachment_name
       * @param value
       */
      template<typename T>
      void
      get_attachment(const AttachmentName &attachment_name, T & value) const;

      /** Extract the stream of a specific attachment for a Document from the DB
       * @param attachment_name the name of the attachment
       * @param stream the string of data to write to
       * @param mime_type the MIME type as stored in the DB
       */
      virtual void
      get_attachment_stream(const AttachmentName &attachment_name, std::ostream& stream, MimeType mime_type =
          MIME_TYPE_DEFAULT) const;

      /** Extract a specific attachment from a document in the DB
       * @param attachment_name
       * @param value
       */
      template<typename T>
      void
      get_attachment_and_cache(const AttachmentName &attachment_name, T & value);

      /** Extract the stream of a specific attachment for a Document from the DB
       * @param attachment_name the name of the attachment
       * @param stream the string of data to write to
       * @param mime_type the MIME type as stored in the DB
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
      ObjectDbPtr db_;
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

      ViewIterator(const View &view, const ObjectDbPtr& db);

      /** Perform the query itself
       * @return an Iterator that will iterate over each result
       */
      ViewIterator &
      begin();

      /** Perform the query itself
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
      set_db(const ObjectDbPtr & db);

      bool
      operator!=(const ViewIterator & document_view) const;

      ViewElement
      operator*() const;
    private:
      std::vector<ViewElement> view_elements_;
      int start_offset_;
      int total_rows_;
      /** The strings to send to the db_ to perform the query */
      boost::function<void
      (int limit_rows, int start_offset, int& total_rows, int& offset, std::vector<ViewElement> &)> query_;
      ObjectDbPtr db_;
    };
  }
}

#endif /* ORK_CORE_DB_DB_H_ */
