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

// inspired by KONC, Janez, JANEŽIČ, Dušanka. An improved branch and bound algorithm for the maximum clique problem

#include <opencv2/core/core.hpp>

namespace object_recognition
{
  namespace maximum_clique
  {
    class Graph
    {
    public:
      typedef unsigned int Vertex;
      typedef std::vector<Vertex> Vertices;

      /** Constructor */
      Graph(unsigned int vertex_number);
      /** Add an edge to the graph */
      void
      addEdge(Vertex vertex_1, Vertex vertex_2);
      /** Given a vertex, delete all the edges containing it */
      void
      deleteEdges(unsigned int vertex);
      void
      deleteEdge(Vertex vertex_1, Vertex vertex_2);
      void
      findMaximumClique(Vertices &max_clique);
    private:
      typedef std::vector<unsigned int> Neighbors;
      typedef unsigned int Color;
      typedef std::vector<Color> Colors;

      struct RCCompare
      {
        RCCompare(const std::vector<Neighbors> &E)
            :
              E_(E)
        {
        }

        bool
        operator()(const Vertex & vertex_1, const Vertex & vertex_2) const
        {
          return (E_[vertex_1].size() > E_[vertex_2].size());
        }
        const std::vector<Neighbors> &E_;
      };

      struct PairSorter
      {
        bool
        operator()(const std::pair<Vertex, unsigned int> &left, const std::pair<Vertex, unsigned int> &right)
        {
          return left.second < right.second;
        }
      };

      bool
      IsIntersecting(Vertex p, const Vertices &B);
      bool
      Intersection(Vertex p, const Vertices & B, Vertices &C);
      void
      ColorSort(Vertices &R, Colors &C, Vertices &QMax, Vertices &Q);
      void
      DegreeSort(Vertices & R);
      void
      MaxCliqueDyn(Vertices & R, Colors &C, int level, Vertices &QMax, Vertices &Q,
             std::vector<unsigned int> &S, std::vector<unsigned int> &SOld);

      /** The list of edges per vertex */
      std::vector<Neighbors> E_;
      /** Mask for the edges */
      cv::Mat_<uchar> e_;
      /** The number of vertices in the graph */
      unsigned int n_vertices_;

      int all_steps_;
      double t_limit_;
    };
  }
}
