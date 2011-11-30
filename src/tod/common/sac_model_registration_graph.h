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
#include "pcl/sample_consensus/sac_model_registration.h"
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
      typedef boost::shared_ptr<SampleConsensusModelRegistrationGraph> Ptr;

      using pcl::SampleConsensusModel<PointT>::drawIndexSample;

      /** \brief Constructor for base SampleConsensusModelRegistration.
       * \param cloud the input point cloud dataset
       */
      SampleConsensusModelRegistrationGraph(const PointCloudConstPtr &cloud,
                                            const object_recognition::maximum_clique::Graph & graph, float threshold)
          :
            pcl::SampleConsensusModelRegistration<PointT>(cloud),
            physical_adjacency_(graph.adjacency()),
            best_inlier_number_(0),
            input_(cloud),
            threshold_(threshold)
      {
        BuildNeighbors();
      }

      /** \brief Constructor for base SampleConsensusModelRegistration.
       * \param cloud the input point cloud dataset
       * \param indices a vector of point indices to be used from \a cloud
       */
      SampleConsensusModelRegistrationGraph(
          const PointCloudConstPtr &cloud, const std::vector<int> &indices, float threshold,
          const object_recognition::maximum_clique::AdjacencyMatrix & physical_adjacency,
          const object_recognition::maximum_clique::AdjacencyMatrix &sample_adjacency)
          :
            pcl::SampleConsensusModelRegistration<PointT>(cloud, indices),
            physical_adjacency_(physical_adjacency),
            sample_adjacency_(sample_adjacency),
            indices_(indices),
            best_inlier_number_(0),
            input_(cloud),
            threshold_(threshold)
      {
        BuildNeighbors();
      }

      bool
      drawIndexSampleHelper(std::vector<int> & valid_samples, unsigned int n_samples, std::vector<int> & samples) const
      {
        if (n_samples == 0)
          return true;
        if (valid_samples.empty())
          return false;
        while (true)
        {
          int sample = valid_samples[rand() % valid_samples.size()];
          std::vector<int> new_valid_samples(valid_samples.size());
          std::vector<int>::iterator end = std::set_intersection(valid_samples.begin(), valid_samples.end(),
                                                                 neighbors_[sample].begin(), neighbors_[sample].end(),
                                                                 new_valid_samples.begin());
          new_valid_samples.resize(end - new_valid_samples.begin());
          std::vector<int> new_samples;
          if (drawIndexSampleHelper(new_valid_samples, n_samples - 1, new_samples))
          {
            samples = new_samples;
            valid_samples = new_valid_samples;
            samples.push_back(sample);
            return true;
          }
          else
          {
            std::vector<int>::iterator end = std::remove(valid_samples.begin(), valid_samples.end(), sample);
            valid_samples.resize(end - valid_samples.begin());
            if (valid_samples.empty())
              return false;
          }
        }
        return false;
      }

      bool
      isSampleGood(const std::vector<int> &samples) const
      {
        std::vector<int> valid_samples = sample_pool_;
        std::vector<int> &new_samples = const_cast<std::vector<int> &>(samples);
        size_t sample_size = new_samples.size();
        bool is_good = drawIndexSampleHelper(valid_samples, sample_size, new_samples);

        if (is_good)
          samples_ = new_samples;

        return is_good;
      }
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      void
      getDistancesToModel(const Eigen::VectorXf &model_coefficients, std::vector<double> &distances)
      {
        pcl::SampleConsensusModelRegistration<PointT>::getDistancesToModel(model_coefficients, distances);

        // Assign a maximum distance for all the points that cannot belong to a clique including the sample
        for (size_t i = 0; i < indices_.size(); ++i)
        {
          BOOST_FOREACH(int sample, samples_)
              {
                if (sample == indices_[i])
                  continue;
                if (!physical_adjacency_.test(indices_[i], sample))
                {
                  distances[i] = std::numeric_limits<double>::max();
                  break;
                }
              }
        }
      }

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      void
      selectWithinDistance(const Eigen::VectorXf &model_coefficients, double threshold, std::vector<int> &in_inliers)
      {
        std::vector<int> possible_inliers;
        pcl::SampleConsensusModelRegistration<PointT>::selectWithinDistance(model_coefficients, threshold,
                                                                            possible_inliers);

        in_inliers.clear();
        // Make sure the sample belongs to the inliers
        BOOST_FOREACH(int sample, samples_)
              if (std::find(possible_inliers.begin(), possible_inliers.end(), sample) == possible_inliers.end())
                return;

        // Remove all the points that cannot belong to a clique including the samples
        BOOST_FOREACH(int inlier, possible_inliers)
            {
              bool is_good = true;
              BOOST_FOREACH(int sample, samples_)
                  {
                    if (sample == inlier)
                      break;
                    if (!physical_adjacency_.test(inlier, sample))
                    {
                      is_good = false;
                      break;
                    }
                  }
              if (is_good)
                in_inliers.push_back(inlier);
            }

        // If that set is not bigger than the best so far, no need to refine it
        if (in_inliers.size() < best_inlier_number_)
          return;

        object_recognition::maximum_clique::Graph graph(in_inliers.size());
        for (unsigned int j = 0; j < in_inliers.size(); ++j)
          for (unsigned int i = j + 1; i < in_inliers.size(); ++i)
            if (sample_adjacency_.test(in_inliers[j], in_inliers[i]))
              graph.AddEdge(j, i);

        // If we cannot even find enough points well distributed in the sample, stop here
        unsigned int minimal_size = 8;
        std::vector<unsigned int> vertices;
        graph.FindClique(vertices, minimal_size);
        if (vertices.size() < minimal_size)
        {
          in_inliers.clear();
          return;
        }

        best_inlier_number_ = std::max(in_inliers.size(), best_inlier_number_);
      }

      void
      optimizeModelCoefficients(const PointCloudConstPtr &target, const std::vector<int> &inliers,
                                const Eigen::VectorXf &model_coefficients, Eigen::VectorXf &optimized_coefficients)
      {
        Eigen::Matrix4f transform;
        pcl::estimateRigidTransformationSVD<PointT, PointT>(*input_, inliers, *target, inliers, transform);
        optimized_coefficients.resize(16);
        optimized_coefficients.segment<4>(0) = transform.row(0);
        optimized_coefficients.segment<4>(4) = transform.row(1);
        optimized_coefficients.segment<4>(8) = transform.row(2);
        optimized_coefficients.segment<4>(12) = transform.row(3);
      }

      mutable std::vector<int> samples_;
    private:
      void
      BuildNeighbors()
      {
        neighbors_.resize(sample_adjacency_.size());
        size_t max_neighbors_size = 10;
        for (unsigned int j = 0; j < sample_adjacency_.size(); ++j)
        {
          neighbors_[j] = sample_adjacency_.neighbors(j);
          max_neighbors_size = std::max(max_neighbors_size, neighbors_[j].size());
          if (neighbors_[j].size() >= 3)
            sample_pool_.push_back(j);
        }
        if (!indices_.empty())
        {
          std::vector<int>::iterator end = std::set_intersection(sample_pool_.begin(), sample_pool_.end(),
                                                                 indices_.begin(), indices_.end(),
                                                                 sample_pool_.begin());
          sample_pool_.resize(end - sample_pool_.begin());
        }
      }

      const object_recognition::maximum_clique::AdjacencyMatrix physical_adjacency_;
      const object_recognition::maximum_clique::AdjacencyMatrix sample_adjacency_;
      std::vector<int> indices_;
      std::vector<int> sample_pool_;
      std::vector<std::vector<unsigned int> > neighbors_;
      size_t best_inlier_number_;
      PointCloudConstPtr input_;
      float threshold_;
    };
  }
}
#endif
