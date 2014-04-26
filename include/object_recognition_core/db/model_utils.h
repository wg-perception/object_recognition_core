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

#ifndef ORK_CORE_DB_UTILS_H_
#define ORK_CORE_DB_UTILS_H_

#include <object_recognition_core/db/document.h>

namespace object_recognition_core
{
  namespace db
  {
    /** Function that compares the intersection of two JSON trees
     * @param obj1
     * @param obj2
     * @return true if the intersection between the keys have the same values
     */
    bool
    CompareJsonIntersection(const or_json::mValue &obj1, const or_json::mValue &obj2);

    /** Function filling a DB document for a model with the common attributes
     * @param db the DB where the model will be saved
     * @param object_id the id of the object for that model
     * @param method the method used to compute the models (e.g. 'TOD')
     * @param parameters_str a JSON string detailing the non-discriminative parameters used in the method
     * @param doc the document to fill
     * @return the Document to update
     */
    void
    PopulateModel(const ObjectDbPtr& db, const ObjectId& object_id, const std::string& method,
                    const std::string& parameters_str, Document& doc);

    /** Given some parameters, retrieve Documents that are models with an object_id
     * that is in object_ids and with parameters matching model_json_params
     * @param db
     * @param object_ids
     * @param method
     * @return
     */
    Documents
    ModelDocuments(ObjectDbPtr &db, const std::vector<ObjectId> & object_ids, const std::string & method);

    /** Given some parameters, retrieve Documents that are models with parameters matching model_json_params
     * @param db
     * @param method
     * @return
     */
    Documents
    ModelDocuments(ObjectDbPtr &db, const std::string & method);
}
}

#endif /* ORK_CORE_DB_UTILS_H_ */
