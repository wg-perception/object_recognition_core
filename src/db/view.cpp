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

#include <object_recognition_core/db/db.h>

namespace object_recognition_core
{
  namespace db
  {
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    /** Given a document, returns whether it is in the view, and if so, returns the key and value
     * @param document
     * @param key
     * @param value
     * @return
     */
    bool
    View::GetKey(const or_json::mObject & document, or_json::mValue & key, or_json::mValue & value)
    {
      switch (type_)
      {
        case VIEW_MODEL_WHERE_OBJECT_ID_AND_MODEL_TYPE:
        {
          if (document.find("method")->second == parameters_.find("model_type")->second)
          {
            key = document.find("_id")->second.get_str();
            value = or_json::mValue(document);
            return true;
          }
          break;
        }
        case VIEW_OBSERVATION_WHERE_OBJECT_ID:
        {
          // It is a dummy document so it never belong to the db
          return false;
          break;
        }
      }
      return false;
    }

    /** Set the results to have one specific key
     * @param key
     */
    void
    View::set_key(const Key & key)
    {
      is_key_set_ = true;
      key_ = key;
    }

    /** Set the view to return all results, no matter the key
     */
    void
    View::unset_key()
    {
      is_key_set_ = false;
    }

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

ViewIterator::ViewIterator(const View &view, const ObjectDbPtr& db) :
    start_offset_(0), query_(
        boost::bind(&ObjectDb::QueryView, db, view, _1, _2, _3, _4, _5)), db_(
        db) {
}

const unsigned int ViewIterator::BATCH_SIZE = 100;

ViewIterator::ViewIterator() :
    start_offset_(0) {
}

/** Set the db on which to perform the Query
 * @param db The db on which the query is performed
 */
void ViewIterator::set_db(const ObjectDbPtr & db) {
  db_ = db;
}

/** Perform the query itself
 * @return an Iterator that will iterate over each result
 */
ViewIterator &
ViewIterator::begin() {
  // Process the query and get the ids of several objects
  query_(BATCH_SIZE, start_offset_, total_rows_, start_offset_, view_elements_);
  BOOST_FOREACH(Document & doc, view_elements_)
    doc.set_db(db_);
  return *this;
}

ViewIterator ViewIterator::end() {
  return ViewIterator();
}

ViewIterator &
ViewIterator::operator++() {
  // If we have nothing else to pop, try to get more from the DB
  if (view_elements_.empty()) {
    // Figure out if we need to query for more document ids
    if (start_offset_ < total_rows_) {
      query_(BATCH_SIZE, start_offset_, total_rows_, start_offset_,
          view_elements_);
      BOOST_FOREACH(Document & doc, view_elements_)
        doc.set_db(db_);
    }
  } else if (!view_elements_.empty())
    view_elements_.pop_back();
  return *this;
}

bool ViewIterator::operator==(const ViewIterator & document_view) const {
  return !this->operator !=(document_view);
}

bool ViewIterator::operator!=(const ViewIterator & document_view) const {
  if (document_view.view_elements_.empty())
    return (!view_elements_.empty());
  if (view_elements_.size() >= document_view.view_elements_.size())
    return std::equal(view_elements_.begin(), view_elements_.end(),
        document_view.view_elements_.begin());
  else
    return std::equal(document_view.view_elements_.begin(),
        document_view.view_elements_.end(), view_elements_.begin());
}

Document ViewIterator::operator*() const {
  return view_elements_.back();
}
  }
}
