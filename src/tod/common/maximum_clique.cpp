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

#include<fstream>
#include <iostream>
#include <sstream>

#include <boost/foreach.hpp>

#include "maximum_clique.h"

namespace object_recognition
{
  namespace maximum_clique
  {
    AdjacencyMatrix::AdjacencyMatrix()
    {
    }
    /*
     AdjacencyMatrix::AdjacencyMatrix(unsigned int size)
     :
     adjacency_(size)
     {
     for (size_t i = 0; i < size; ++i)
     adjacency_[i].resize(size, false);
     }
     */
    void
    AdjacencyMatrix::clear()
    {
      adjacency_.clear();
    }

    void
    AdjacencyMatrix::invalidate(const std::vector<Index> &indices)
    {
      BOOST_FOREACH(Index index, indices)
          {
            invalidate(index);
          }
    }

    /*
     void
     AdjacencyMatrix::invalidate(Index index)
     {
     // Invalidate the column
     boost::dynamic_bitset<> & row = adjacency_[index];
     for (Index i = 0; i < adjacency_.size(); ++i)
     if (row.test(i))
     {
     adjacency_[i].reset(index);
     // Invalidate the row
     row.reset(i);
     }
     }

     void
     AdjacencyMatrix::invalidate(Index index1, Index index2)
     {
     adjacency_[index1].reset(index2);
     adjacency_[index2].reset(index1);
     }

     bool
     AdjacencyMatrix::test(Index i, Index j) const
     {
     return adjacency_[i].test(j);
     }

     void
     AdjacencyMatrix::set(Index i, Index j)
     {
     adjacency_[i].set(j);
     adjacency_[j].set(i);
     }

     size_t
     AdjacencyMatrix::count(Index index) const
     {
     return adjacency_[index].count();
     }

     std::vector<Index>
     AdjacencyMatrix::neighbors(Index i) const
     {
     std::vector<AdjacencyMatrix::Index> neighbors;
     AdjacencyMatrix::neighbors.reserve(adjacency_.size());
     const boost::dynamic_bitset<> & row = adjacency_[i];

     for (Index i = 0; i < adjacency_.size(); ++i)
     if (row.test(i))
     neighbors.push_back(i);
     return neighbors;
     }
     */

    AdjacencyMatrix::AdjacencyMatrix(unsigned int size)
        :
          adjacency_(size)
    {
    }

    void
    AdjacencyMatrix::invalidate(Index index)
    {
      // Invalidate the column
      BOOST_FOREACH(Index sub_index, adjacency_[index])
          {
            InvalidateOneWay(sub_index, index);
          }
      // Invalidate the row
      adjacency_[index].clear();
    }

    void
    AdjacencyMatrix::invalidate(Index index1, Index index2)
    {
      InvalidateOneWay(index1, index2);
      InvalidateOneWay(index2, index1);
    }

    bool
    AdjacencyMatrix::test(Index i, Index j) const
    {
      return std::binary_search(adjacency_[i].begin(), adjacency_[i].end(), j);
    }

    void
    AdjacencyMatrix::set(Index i, Index j)
    {
      SetOneWay(i, j);
      SetOneWay(j, i);
    }

    size_t
    AdjacencyMatrix::count(Index index) const
    {
      return adjacency_[index].size();
    }

    std::vector<AdjacencyMatrix::Index>
    AdjacencyMatrix::neighbors(Index i) const
    {
      return adjacency_[i];
    }
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    /** Construct from a dimacs file */
    Graph::Graph(const std::string & name)
    {
      std::ifstream f(name.c_str());
      char buffer[256], token[20];
      int i, j;
      int vi, vj;
      int num_edges = 0;

      if (!f.is_open())
      {
        std::cout << "Error opening file!" << std::endl;
        exit(1);
      }

      while (!f.eof())
      {
        f.getline(buffer, 250);
        if (buffer[0] == 'p')
        {
          unsigned int vertex_number;
          sscanf(&buffer[7], "%d", &vertex_number);
          set_vertex_number(vertex_number);
        }
        if (buffer[0] == 'e')
        {
          num_edges++;
          i = 2;
          j = 0;
          while (buffer[i] != ' ')
          {
            token[j++] = buffer[i];
            i++;
          }
          token[j] = '\0';
          vi = atoi(token);
          i++;
          j = 0;
          while (buffer[i] != ' ')
          {
            token[j++] = buffer[i];
            i++;
          }
          token[j] = '\0';
          vj = atoi(token);
          vi--;
          vj--;
          AddEdge(vi, vj);
        }
      }
      std::cout << "|E| = " << num_edges << "  |V| = " << adjacency_.size() << std::endl;
      f.close();
    }

    /** Returns true if any element in B is a neighbor of in_vertex
     * but also the set of corresponding neighbors
     * @param in_vertex
     * @param vertices
     * @param intersection
     * @return
     */
    bool
    Graph::Intersection(Vertex p, const Vertices & vertices, Vertices &intersection)
    {
      intersection.clear();
      BOOST_FOREACH(Vertex vertex, vertices)
            if (adjacency_.test(p, vertex))
              intersection.push_back(vertex);
      return !intersection.empty();
    }

    void
    Graph::ColorSort(Vertices &R, Colors &C, Vertices &QMax, Vertices &Q)
    {
      unsigned int min_k = std::max(1, int(QMax.size()) - int(Q.size()) + 1);

      std::vector<std::vector<unsigned int> > Ck(2);
      Ck.reserve(R.size());

      unsigned int j = 0;
      unsigned int maxno = Ck.size();
      BOOST_FOREACH(Vertex p, R)
          {
            unsigned int k = 1;
            while (IsIntersecting(p, Ck[k]))
            {
              ++k;
              if (k >= maxno)
              {
                ++maxno;
                Ck.resize(maxno);
                break;
              }
            }
            if (k < min_k)
              R[j++] = p;
            else
              Ck[k].push_back(p);
          }
      if (j > 0)
        C[j - 1] = 0;
      if (min_k <= 0)
        min_k = 1;
      Vertices::iterator R_iter = R.begin() + j;
      Colors::iterator C_iter = C.begin() + j;
      for (unsigned int k = min_k; k < maxno; ++k)
      {
        unsigned int Ck_size = Ck[k].size();
        std::copy(Ck[k].begin(), Ck[k].end(), R_iter);
        R_iter += Ck_size;
        std::fill(C_iter, C_iter + Ck_size, k);
        C_iter += Ck_size;
      }
    }

    void
    Graph::DegreeSort(Vertices &R)
    {
      unsigned int R_size = R.size();
      std::vector<std::pair<unsigned int, Vertex> > degrees(R_size);
      for (unsigned int i = 0; i < R_size; ++i)
      {
        degrees[i] = std::make_pair(0, R[i]);
        for (unsigned int j = 0; j < i; ++j)
          if (adjacency_.test(R[i], R[j]))
          {
            ++degrees[i].first;
            ++degrees[j].first;
          }
      }
      // Sort the vertices according to their degree
      std::sort(degrees.begin(), degrees.end());

      // Copy back to R in reverse
      for (unsigned int i = 0; i < R_size; ++i)
        R[i] = degrees[R_size - 1 - i].second;
    }

    void
    Graph::MaxCliqueDyn(Vertices & R, Colors &C, unsigned int level, unsigned int minimal_size, Vertices &QMax,
                        Vertices &Q, std::vector<unsigned int> &S, std::vector<unsigned int> &SOld)
    {
      if (QMax.size() >= minimal_size)
        return;
      if (level >= S.size())
      {
        S.reserve(2 * S.size());
        S.resize(S.size() + 1);
        SOld.reserve(2 * SOld.size());
        SOld.resize(SOld.size() + 1);
      }

      S[level] = S[level] + S[level - 1] - SOld[level];
      SOld[level] = S[level - 1];

      while (!R.empty())
      {
        Vertex p = R.back();
        Color c = C.back();
        if (Q.size() + c > QMax.size())
        {
          Q.push_back(p);
          Vertices Rp;
          if (Intersection(p, R, Rp))
          {
            if ((double) S[level] / all_steps_ < t_limit_)
              DegreeSort(Rp);
            ColorSort(Rp, C, QMax, Q);
            ++S[level];
            ++all_steps_;
            if (all_steps_ > 100000)
              return;
            MaxCliqueDyn(Rp, C, level + 1, minimal_size, QMax, Q, S, SOld);
          }
          else if (Q.size() > QMax.size())
          {
            QMax = Q;
            if (QMax.size() >= minimal_size)
              return;
            //std::cout << "step: " << all_steps_ << " with size " << QMax.size() << std::endl;
          }
          Q.pop_back();
        }
        else
          return;
        R.pop_back();
        C.pop_back();
      }
    }

    /** Tries to find a clique of a size at least min_size. If it does nto find it, it returns the biggest clique
     * it can
     * @param QMax The final clique
     * @param min_size The minimal size the desired clique should have
     */
    void
    Graph::FindClique(Vertices & QMax, unsigned int minimal_size)
    {
      if (adjacency_.empty())
        return;
      all_steps_ = 1;
      t_limit_ = 0.025;

      Vertices R(adjacency_.size());
      for (unsigned int i = 0; i < adjacency_.size(); ++i)
        R[i] = i;
      DegreeSort(R);

      unsigned int max_degree = adjacency_.count(R[0]);
      Colors C(adjacency_.size());
      for (unsigned int i = 0; i < max_degree; i++)
        C[i] = i + 1;
      for (unsigned int i = max_degree; i < adjacency_.size(); i++)
        C[i] = max_degree + 1;

      Vertices Q;
      QMax.clear();
      // +1 as we start at level 1
      std::vector<unsigned int> S(adjacency_.size() + 1, 0), SOld(adjacency_.size() + 1, 0);

      MaxCliqueDyn(R, C, 1, minimal_size, QMax, Q, S, SOld);
    }

    void
    Graph::FindMaximumClique(Vertices &QMax)
    {
      FindClique(QMax, std::numeric_limits<unsigned int>::max());
    }

    void
    Graph::AddEdge(unsigned int vertex_1, unsigned int vertex_2)
    {
      adjacency_.set(vertex_1, vertex_2);
    }

    /** Given a vertex, delete all the edges containing it */
    void
    Graph::DeleteEdges(unsigned int in_vertex)
    {
      adjacency_.invalidate(in_vertex);
    }
    /** Given a vertex, delete all the edges containing it */
    void
    Graph::DeleteEdge(Vertex vertex_1, Vertex vertex_2)
    {
      adjacency_.invalidate(vertex_1, vertex_2);
    }
  }
}
