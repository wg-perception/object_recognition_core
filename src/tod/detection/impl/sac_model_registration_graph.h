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

#ifndef SAC_MODEL_REGISTRATION_GRAPH_H_
#define SAC_MODEL_REGISTRATION_GRAPH_H_

#include "pcl/sample_consensus/sac_model.h"
#include "pcl/sample_consensus/model_types.h"

#include "maximum_clique.h"

namespace object_recognition
{
  namespace tod
  {
    /**
     * Class that computes the registration between two point clouds in the specific case where we have an adjacency graph
     * (and some points cannot be connected together)
     */
    template<typename PointT>
    class SampleConsensusModelRegistrationGraph: public pcl::SampleConsensusModelRegistration<PointT>
    {
    public:
      typedef typename pcl::SampleConsensusModelRegistration<PointT>::PointCloudConstPtr PointCloudConstPtr;

      /** \brief Constructor for base SampleConsensusModelRegistration.
       * \param cloud the input point cloud dataset
       */
      SampleConsensusModelRegistrationGraph(const PointCloudConstPtr &cloud,
                                            const object_recognition::maximum_clique::Graph & graph)
          :
            pcl::SampleConsensusModelRegistration<PointT>(cloud),
            adjacency_(graph.adjacency())
      {
        BuildNeighbors();
      }

      /** \brief Constructor for base SampleConsensusModelRegistration.
       * \param cloud the input point cloud dataset
       * \param indices a vector of point indices to be used from \a cloud
       */
      SampleConsensusModelRegistrationGraph(const PointCloudConstPtr &cloud, const std::vector<int> &indices,
                                            const object_recognition::maximum_clique::Graph & graph)
          :
            pcl::SampleConsensusModelRegistration<PointT>(cloud, indices),
            adjacency_(graph.adjacency()),
            indices_(indices),
            shuffled_indices_(indices)
      {
        BuildNeighbors();
      }

      inline void
      drawIndexSample(std::vector<int> & sample)
      {
        size_t sample_size = sample.size();
        sample.clear();
        size_t index_size = shuffled_indices_.size();
        std::vector<unsigned int> valid_samples = shuffled_indices_;
        drawIndexSample(valid_samples, sample);
        while (true)
        {
          // Decide on the first sample
          unsigned int first_sample;
          while (true)
          {
            first_sample = rand() % index_size;
            if (neighbors_[first_sample].size() >= sample_size)
              break;
          }
          sample.push_back(first_sample);

          // Then, draw from its neighbors
          std::vector<unsigned int> valid_samples = neighbors_[first_sample];
          while ((sample.size() < sample_size) && (!valid_samples.empty()))
          {
            unsigned int new_sample = rand() % valid_samples.size();
            sample.push_back(first_sample);
          }
          for (unsigned int i = 0; i < sample_size; ++i)
            // The 1/(RAND_MAX+1.0) trick is when the random numbers are not uniformly distributed and for small modulo
            // elements, that does not matter (and nowadays, random number generators are good)
            std::swap(shuffled_indices_[i], shuffled_indices_[i + (rand() % (index_size - i))]);
        }
        std::copy(shuffled_indices_.begin(), shuffled_indices_.begin() + sample_size, sample.begin());
      }
      bool
      drawIndexSampleHelper(const std::vector<unsigned int> & valid_samples, unsigned int n_samples,
                            std::vector<unsigned int> & samples)
      {
        if (n_samples == 0)
          return true;
        unsigned int sample = rand() % valid_samples.size();
        std::vector<unsigned int> new_valid_samples = valid_samples;
        std::vector<unsigned int>::iterator end = std::set_intersection(valid_samples.begin(), valid_samples.end(),
                                                                        neighbors_[sample].begin(),
                                                                        neighbors_[sample].end(),
                                                                        new_valid_samples.begin());
        new_valid_samples.resize(end - new_valid_samples.begin());
        if (new_valid_samples.empty())
          return false;
        std::vector<unsigned int> new_samples;
        if (drawIndexSampleHelper(new_valid_samples, n_samples - 1, new_samples))
        {
          samples = new_samples;
          samples.push_back(sample);
          return true;
        }
        else
          return false;
      }

      /*      bool
       isSampleGood(const std::vector<int> &samples) const
       {
       bool is_good = true;
       // First make sure all the samples can belong to one common clique
       std::vector<int>::const_iterator iter1 = samples.begin(), iter2, end = samples.end();
       for (; iter1 != end; ++iter1)
       {
       const uchar* row = adjacency_.ptr(*iter1);
       for (iter2 = iter1 + 1; iter2 != end; ++iter2)
       if (!row[*iter2])
       {
       is_good = false;
       break;
       }
       }
       if (is_good)
       {
       is_good = pcl::SampleConsensusModelRegistration<PointT>::isSampleGood(samples);
       if (is_good)
       samples_ = samples;
       }

       return is_good;
       }*/
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      void
      getDistancesToModel(const Eigen::VectorXf &model_coefficients, std::vector<double> &distances)
      {
        pcl::SampleConsensusModelRegistration<PointT>::getDistancesToModel(model_coefficients, distances);

        // Assign a maximum distance for all the points that cannot belong to a clique including the sample
        std::vector<int> final_inliers;
        for (size_t i = 0; i < indices_.size(); ++i)
        {
          const uchar* row = adjacency_.ptr(indices_[i]);
          BOOST_FOREACH(int sample, samples_)
                if (!row[sample])
                {
                  distances[i] = std::numeric_limits<double>::max();
                  break;
                }
        }
      }

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      void
      selectWithinDistance(const Eigen::VectorXf &model_coefficients, double threshold, std::vector<int> &inliers)
      {
        pcl::SampleConsensusModelRegistration<PointT>::selectWithinDistance(model_coefficients, threshold, inliers);

        // Remove all the points that cannot belong to a clique including the samples
        std::vector<int> final_inliers;
        BOOST_FOREACH(int inlier, inliers)
            {
              bool is_good = true;
              const uchar* row = adjacency_.ptr(inlier);
              BOOST_FOREACH(int sample, samples_)
                    if (!row[sample])
                    {
                      is_good = false;
                      break;
                    }
              if (is_good)
                final_inliers.push_back(inlier);
            }
        inliers = final_inliers;
      }
    private:
      void
      BuildNeighbors()
      {
        neighbors_.resize(adjacency_.rows);
        for (int j = 0; j < adjacency_.rows; ++j)
        {
          const uchar * row = adjacency_.ptr<uchar>(j);
          for (int i = 0; i < adjacency_.cols; ++i)
            if (row[i])
              neighbors_[j].push_back(i);
          std::sort(neighbors_[j].begin(), neighbors_[j].end());
        }
      }

      mutable std::vector<int> samples_;
      const cv::Mat_<uchar> adjacency_;
      std::vector<int> indices_;
      std::vector<int> shuffled_indices_;
      std::vector<std::vector<unsigned int> > neighbors_;
    };
  }
}
#endif
