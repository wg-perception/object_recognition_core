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

#ifndef MAXIMUM_CLIQUE_H_
#define MAXIMUM_CLIQUE_H_
// inspired by KONC, Janez, JANEŽIČ, Dušanka. An improved branch and bound algorithm for the maximum clique problem
#include <set>

#include <boost/foreach.hpp>
#include <boost/dynamic_bitset.hpp>

#include <opencv2/core/core.hpp>

namespace object_recognition
{
  namespace maximum_clique
  {
    /** Simple (and hopefully) fast implementation of an adjacency matrix
     */
    class AdjacencyMatrix
    {
    public:
      typedef unsigned int Index;

      AdjacencyMatrix();

      AdjacencyMatrix(Index size);

      void
      clear();

      void
      InvalidateCluster(const std::vector<Index> &indices);

      void
      invalidate(const std::vector<Index> &indices);
      void
      invalidate(Index index);
      void
      invalidate(Index index1, Index index2);

      bool
      test(Index i, Index j) const;

      void
      set(Index i, Index j);

      /** Nothing set later for i will be <= j
       * Nothing set later for j will be <= i
       * @param i
       * @param j
       */
      void
      set_sorted(Index i, Index j)
      {
        adjacency_[i].push_back(j);
        adjacency_[j].push_back(i);
      }

      inline bool
      empty() const
      {
        return adjacency_.empty();
      }

      inline size_t
      size() const
      {
        return adjacency_.size();
      }

      size_t
      count(Index index) const;

      ///////// Non standard functions
      std::vector<Index>
      neighbors(Index i) const;
    private:
      inline void
      InvalidateOneWay(Index index1, Index index2)
      {
        std::vector<Index> & row = adjacency_[index1];
        std::vector<Index>::iterator iter = std::lower_bound(row.begin(), row.end(), index2);
        std::copy(iter + 1, row.end(), iter);
        row.resize(row.size() - 1);
      }

      inline void
      SetOneWay(Index index1, Index index2)
      {
        std::vector<Index> & row = adjacency_[index1];
        if (row.empty())
        {
          //row.reserve(16);
          row.push_back(index2);
          return;
        }
        std::vector<Index>::iterator end = row.end();
        std::vector<Index>::iterator iter = std::lower_bound(row.begin(), end, index2);
        if (iter == end)
        {
          row.push_back(index2);
          return;
        }
        if ((*iter) == index2)
          return;
        else
        {
          row.push_back(index2);
          std::copy_backward(iter, row.end() - 1, row.end());
          *iter = index2;
        }
      }

      //std::vector<boost::dynamic_bitset<> > adjacency_;
      std::vector<std::vector<Index> > adjacency_;
    };

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    class Graph
    {
    public:
      typedef unsigned int Vertex;
      typedef std::vector<Vertex> Vertices;

      /** Constructor */
      Graph()
      {
      }

      /** Constructor */
      Graph(unsigned int vertex_number)
      {
        set_vertex_number(vertex_number);
      }

      /** Construct froma dimacs file */
      Graph(const std::string & name);

      void
      clear()
      {
        adjacency_.clear();
      }

      void
      set_vertex_number(unsigned int vertex_number)
      {
        adjacency_ = AdjacencyMatrix(vertex_number);
      }

      /** Add an edge to the graph */
      void
      AddEdge(Vertex vertex_1, Vertex vertex_2);

      /** Add an edge to the graph
       * But later on, nothing will be smaller than vertex_2 for vertex_1 and reciprocally
       */
      void
      AddEdgeSorted(Vertex vertex_1, Vertex vertex_2);

      /** Given a vertex, delete all the edges containing it */
      void
      DeleteEdges(unsigned int vertex);

      void
      DeleteEdge(Vertex vertex_1, Vertex vertex_2);

      void
      FindMaximumClique(Vertices &max_clique);

      /** Tries to find a clique of a size at least min_size. If it does nto find it, it returns the biggest clique
       * it can
       * @param QMax The final clique
       * @param min_size The minimal size the desired clique should have
       */
      void
      FindClique(Vertices & QMax, unsigned int minimal_size);

      inline const AdjacencyMatrix &
      adjacency() const
      {
        return adjacency_;
      }

      inline void
      set_adjacency(const AdjacencyMatrix & adjacency)
      {
        adjacency_ = adjacency;
      }

    private:
      typedef std::set<Vertex> Neighbors;
      typedef unsigned int Color;
      typedef std::vector<Color> Colors;

      /** Returns true if any element in B is a neighbor of in_vertex
       * @param in_vertex
       * @param vertices
       * @return
       */
      inline bool
      IsIntersecting(Vertex in_vertex, const Vertices &vertices)
      {
#if 0
        const AdjacencyMatrix::Neighbors & neighbors = ajacency_.neighbors(in_Vertex);
        AdjacencyMatrix::Neighbors::const_iterator first1 = neighbors.begin(), last1 = neighbors.end();
        Vertices::const_iterator first2 = vertices.begin(), last2 = vertices.end();

        while (first1 != last1 && first2 != last2)
        {
          if (*first1 < *first2)
          ++first1;
          else if (*first2 < *first1)
          ++first2;
          else
          return true;
        }
#else
        BOOST_FOREACH(Vertex vertex, vertices)
              if (adjacency_.test(in_vertex, vertex))
                return true;
#endif
        return false;
      }

      bool
      Intersection(Vertex p, const Vertices & B, Vertices &C);
      void
      ColorSort(Vertices &R, Colors &C, Vertices &QMax, Vertices &Q);
      void
      DegreeSort(Vertices & R);
      void
      MaxCliqueDyn(Vertices & R, Colors &C, unsigned int level, unsigned int minimal_size, Vertices &QMax, Vertices &Q,
                   std::vector<unsigned int> &S, std::vector<unsigned int> &SOld);

      /** Mask for the edges */
      AdjacencyMatrix adjacency_;

      int all_steps_;
      double t_limit_;
    };
  }
}
#endif
