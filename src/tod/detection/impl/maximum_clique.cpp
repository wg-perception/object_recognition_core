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

#include <iostream>

#include <boost/foreach.hpp>

#include "maximum_clique.h"

#ifdef DEBUG
#include <valgrind/callgrind.h>
#endif

namespace object_recognition
{
  namespace maximum_clique
  {
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
      const uchar * row = adjacency_.ptr(p);
      BOOST_FOREACH(Vertex vertex, vertices)
            if (row[vertex])
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
      C.resize(R.size());
      if (j > 0)
        C[j - 1] = 0;
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
      Vertices R_tmp = R;
      std::sort(R_tmp.begin(), R_tmp.end());
      for (unsigned int i = 0; i < R_size; ++i)
      {
        // Compute the members of R that are also neighbors of R[i]
        size_t intersection_size = 0;

        Vertices::const_iterator first1 = R_tmp.begin(), last1 = R_tmp.end(), first2 = neighbors_[R_tmp[i]].begin(),
            last2 = neighbors_[R_tmp[i]].end();
        while (first1 != last1 && first2 != last2)
        {
          if (*first1 < *first2)
            ++first1;
          else if (*first2 < *first1)
            ++first2;
          else
          {
            ++first1;
            ++first2;
            ++intersection_size;
          }
        }
        degrees[i] = std::make_pair(intersection_size, R_tmp[i]);
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
            //if (all_steps_ > 100000)
            //return;
            MaxCliqueDyn(Rp, Cp, level + 1, minimal_size, QMax, Q, S, SOld);
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
      if (n_vertices_ == 0)
        return;
      all_steps_ = 1;
      t_limit_ = 0.025;

      Vertices R(n_vertices_);
      for (unsigned int i = 0; i < n_vertices_; ++i)
        R[i] = i;
      DegreeSort(R);

      unsigned int max_degree = cv::countNonZero(adjacency_.row(R[0]));
      Colors C(n_vertices_);
      for (unsigned int i = 0; i < max_degree; i++)
        C[i] = i + 1;
      for (unsigned int i = max_degree; i < n_vertices_; i++)
        C[i] = max_degree + 1;

      Vertices Q;
      QMax.clear();
      // +1 as we start at level 1
      std::vector<unsigned int> S(n_vertices_ + 1, 0), SOld(n_vertices_ + 1, 0);

#ifdef DEBUG
      CALLGRIND_START_INSTRUMENTATION;
#endif
      MaxCliqueDyn(R, C, 1, minimal_size, QMax, Q, S, SOld);
#ifdef DEBUG
      CALLGRIND_STOP_INSTRUMENTATION;
#endif
    }

    void
    Graph::FindMaximumClique(Vertices &QMax)
    {
      FindClique(QMax, std::numeric_limits<unsigned int>::max());
    }

    void
    Graph::AddEdge(unsigned int vertex_1, unsigned int vertex_2)
    {
      adjacency_(vertex_1, vertex_2) = 1;
      adjacency_(vertex_2, vertex_1) = 1;

      std::vector<Vertex> vec_tmp_1(neighbors_[vertex_1].size() + 1);
      std::vector<Vertex> vec_tmp_2(neighbors_[vertex_2].size() + 1);
      std::vector<Vertex> vec_1(1, vertex_1), vec_2(1, vertex_2);
      vec_tmp_1.resize(
          std::set_union(neighbors_[vertex_1].begin(), neighbors_[vertex_1].end(), vec_2.begin(), vec_2.end(),
                         vec_tmp_1.begin())
          - vec_tmp_1.begin());
      vec_tmp_2.resize(
          std::set_union(neighbors_[vertex_2].begin(), neighbors_[vertex_2].end(), vec_1.begin(), vec_1.end(),
                         vec_tmp_2.begin())
          - vec_tmp_2.begin());
      neighbors_[vertex_1] = vec_tmp_1;
      neighbors_[vertex_2] = vec_tmp_2;
    }

    /** Given a vertex, delete all the edges containing it */
    void
    Graph::DeleteEdges(unsigned int in_vertex)
    {
      adjacency_.col(in_vertex).setTo(cv::Scalar(0));
      adjacency_.row(in_vertex).setTo(cv::Scalar(0));

      BOOST_FOREACH(Vertex vertex, neighbors_[in_vertex])
          {
            neighbors_[vertex].resize(
                std::remove(neighbors_[vertex].begin(), neighbors_[vertex].end(), in_vertex)
                - neighbors_[vertex].begin());
          }
      neighbors_[in_vertex].clear();
    }
    /** Given a vertex, delete all the edges containing it */
    void
    Graph::DeleteEdge(Vertex vertex_1, Vertex vertex_2)
    {
      adjacency_(vertex_1, vertex_2) = 0;
      adjacency_(vertex_2, vertex_1) = 0;

      neighbors_[vertex_1].resize(
          std::remove(neighbors_[vertex_1].begin(), neighbors_[vertex_1].end(), vertex_2)
          - neighbors_[vertex_1].begin());
      neighbors_[vertex_2].resize(
          std::remove(neighbors_[vertex_2].begin(), neighbors_[vertex_2].end(), vertex_1)
          - neighbors_[vertex_2].begin());
    }
  }
}
