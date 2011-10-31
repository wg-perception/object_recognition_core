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

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
namespace{
struct HammingOperator
{
  unsigned int
  operator()(const unsigned char* a, const unsigned char* b, int size)
  {
#if __GNUC__
    unsigned int result = 0;
    {
      //for portability just use unsigned long -- and use the __builtin_popcountll (see docs for __builtin_popcountll)
      typedef unsigned long long pop_t;
      const size_t modulo = size % sizeof(pop_t);
      const pop_t * a2 = reinterpret_cast<const pop_t*>(a);
      const pop_t * b2 = reinterpret_cast<const pop_t*>(b);

      if (size == sizeof(pop_t))
        return __builtin_popcountll((*a2) ^ (*b2));
      if (size == 2 * sizeof(pop_t))
        return __builtin_popcountll((*a2) ^ (*b2)) + __builtin_popcountll((*(a2 + 1)) ^ (*(b2 + 1)));
      if (size == 3 * sizeof(pop_t))
        return __builtin_popcountll((*a2) ^ (*b2)) + __builtin_popcountll((*(a2 + 1)) ^ (*(b2 + 1)))
               + __builtin_popcountll((*(a2 + 2)) ^ (*(b2 + 2)));
      if (size == 4 * sizeof(pop_t))
        return __builtin_popcountll((*a2) ^ (*b2)) + __builtin_popcountll((*(a2 + 1)) ^ (*(b2 + 1)))
               + __builtin_popcountll((*(a2 + 2)) ^ (*(b2 + 2)))
               + __builtin_popcountll((*(a2 + 3)) ^ (*(b2 + 3)));

      const pop_t * a2_end = a2 + (size / sizeof(pop_t));

      for (; a2 != a2_end; ++a2, ++b2)
        result += __builtin_popcountll((*a2) ^ (*b2));

      if (modulo)
      {
        //in the case where size is not divisible by sizeof(size_t)
        //need to mask off the bits at the end
        pop_t a_final = 0, b_final = 0;
        memcpy(&a_final, a2, modulo);
        memcpy(&b_final, b2, modulo);
        result += __builtin_popcountll(a_final ^ b_final);
      }
    }
    return result;
#else
    return cv::HammingLUT()(a,b,size);
#endif
  }
};
}
