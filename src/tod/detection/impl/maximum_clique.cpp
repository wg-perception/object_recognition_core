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

#include <boost/foreach.hpp>

#include "maximum_clique.h"

namespace object_recognition
{
  namespace maximum_clique
  {
    /** Returns true if any element in B is a neighbor of in_vertex
     * @param in_vertex
     * @param vertices
     * @return
     */
    bool
    Graph::IsIntersecting(Vertex in_vertex, const Vertices &vertices)
    {
      BOOST_FOREACH(Vertex vertex, vertices)
            if (e_(in_vertex, vertex))
              return true;
      return false;
    }

    /** Returns true if any element in B is a neighbor of in_vertex
     * but also the set of corresponding enighbors
     * @param in_vertex
     * @param vertices
     * @param intersection
     * @return
     */
    bool
    Graph::Intersection(Vertex p, const Vertices & vertices, Vertices &intersection)
    {
      BOOST_FOREACH(Vertex vertex, vertices)
            if (e_(p, vertex))
              intersection.push_back(vertex);
      return !intersection.empty();
    }

    void
    Graph::ColorSort(Vertices &R, Colors &C, Vertices &QMax, Vertices &Q)
    {
      unsigned int min_k = std::max(1, int(QMax.size()) - int(Q.size()) + 1);

      std::vector<std::vector<unsigned int> > Ck(2);

      unsigned int j = 0;
      BOOST_FOREACH(Vertex p, R)
          {
            unsigned int k = 1;
            while (IsIntersecting(p, Ck[k]))
            {
              ++k;
              if (k >= Ck.size())
              {
                Ck.resize(Ck.size() + 1);
                break;
              }
            }
            Ck[k].push_back(p);
            if (k < min_k)
              R[j++] = p;
          }
      C.resize(R.size());
      if (j > 0)
        C[j - 1] = 0;
      for (unsigned int k = min_k; k < Ck.size(); k++)
        for (unsigned int i = 0; i < Ck[k].size(); i++)
        {
          R[j] = Ck[k][i];
          C[j++] = k;
        }
    }

    void
    Graph::DegreeSort(Vertices &R)
    {
      unsigned int R_size = R.size();
      std::vector<std::pair<Vertex, unsigned int> > degrees(R.size());
      for (unsigned int i = 0; i < R_size; ++i)
      {
        degrees[i].first = R[i];
        degrees[i].second = 0;
        for (unsigned int j = 0; j < i; ++j)
          if (e_(R[i], R[j]))
          {
            degrees[i].second++;
            degrees[j].second++;
          }
      }
      // Sort the vertices according to their degree
      std::sort(degrees.begin(), degrees.end(), PairSorter());

      // Copy back to R in reverse
      for (unsigned int i = 0; i < R_size; i++)
        R[i] = degrees[R_size - 1 - i].first;
    }

    void
    Graph::MaxCliqueDyn(Vertices & R, Colors &C, int level, Vertices &QMax, Vertices &Q, std::vector<unsigned int> &S,
                        std::vector<unsigned int> &SOld)
    {
      S[level] = S[level] + S[level - 1] - SOld[level];
      SOld[level] = S[level - 1];

      while (!R.empty())
      {
        Vertex p = R.back();
        Color c = C.back();
        R.pop_back();
        C.pop_back();
        if (Q.size() + c > QMax.size())
        {
          Q.push_back(p);
          Vertices Rp;
          if (Intersection(p, R, Rp))
          {
            if ((double) S[level] / all_steps_ < t_limit_)
              DegreeSort(Rp);
            Colors Cp;
            ColorSort(Rp, Cp, QMax, Q);
            ++S[level];
            ++all_steps_;
            MaxCliqueDyn(Rp, Cp, level + 1, QMax, Q, S, SOld);
          }
          else if (Q.size() > QMax.size())
            QMax = Q;
          Q.pop_back();
        }
        else
          return;
      }
    }

    void
    Graph::findMaximumClique(std::vector<unsigned int> &max_clique)
    {
      all_steps_ = 0;
      t_limit_ = 0.025;

      Vertices R(n_vertices_);
      for (unsigned int i = 0; i < n_vertices_; ++i)
        R[i] = i;
      std::sort(R.begin(), R.end(), RCCompare(E_));

      unsigned int max_degree = E_[R[0]].size();
      Colors C(n_vertices_);
      for (unsigned int i = 0; i < max_degree; i++)
        C[i] = i + 1;
      for (unsigned int i = max_degree; i < n_vertices_; i++)
        C[i] = max_degree + 1;

      Vertices QMax, Q;
      std::vector<unsigned int> S(n_vertices_, 0), SOld(n_vertices_, 0);
      MaxCliqueDyn(R, C, 1, QMax, Q, S, SOld);

      BOOST_FOREACH(Vertex vertex, QMax)
            max_clique.push_back(vertex);

      // Check that the clique is valid
      /*int count = 0;
       for (unsigned int i = 0; i < max_clique.size(); ++i)
       for (unsigned int j = i + 1; j < max_clique.size(); ++j)
       if (!e_(max_clique[i], max_clique[j]))
       {
       ++count;
       std::cout << max_clique[i] << " " << max_clique[j] << std::endl;
       }
       if (count > 0)
       std::cerr << count << " are bad out of " << max_clique.size() << std::endl;*/
    }

    Graph::Graph(unsigned int vertex_number)
    {
      E_.resize(vertex_number);
      e_ = cv::Mat_<uchar>::zeros(vertex_number, vertex_number);
      n_vertices_ = vertex_number;
    }
    void
    Graph::addEdge(unsigned int vertex_1, unsigned int vertex_2)
    {
      E_[vertex_1].push_back(vertex_2);
      E_[vertex_2].push_back(vertex_1);
      e_(vertex_1, vertex_2) = 1;
      e_(vertex_2, vertex_1) = 1;
    }
    /** Given a vertex, delete all the edges containing it */
    void
    Graph::deleteEdges(unsigned int in_vertex)
    {
      BOOST_FOREACH(int vertex, E_[in_vertex])
          {
            e_(vertex, in_vertex) = 0;
            e_(in_vertex, vertex) = 0;
            std::remove(E_[vertex].begin(), E_[vertex].end(), in_vertex);
          }
      E_[in_vertex].clear();
    }
    /** Given a vertex, delete all the edges containing it */
    void
    Graph::deleteEdge(Vertex vertex_1, Vertex vertex_2)
    {
      e_(vertex_1, vertex_2) = 0;
      e_(vertex_2, vertex_1) = 0;
      std::remove(E_[vertex_1].begin(), E_[vertex_1].end(), vertex_2);
      std::remove(E_[vertex_2].begin(), E_[vertex_2].end(), vertex_1);
    }
  }
}
