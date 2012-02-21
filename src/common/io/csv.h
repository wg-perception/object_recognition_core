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
 */
#pragma once
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <boost/shared_ptr.hpp>

namespace object_recognition_core
{
  namespace io
  {
    //! @brief Timestamp descriptor object
    //!
    struct TimeStamp
    {
      int year;
      int month;
      int day;

      int hour;
      int min;
      int sec;
      int msec;

      //! @brief captures the current time.
      void
      set();
    };

    //! @brief Information regarding the current execution of the detection program
    //!
    struct RunInfo
    {
      //! @brief 4-digit run number
      int runID;

      //! @brief Team name
      std::string name;

      //! @brief Current timestamp
      TimeStamp ts;
    };

    //! @brief Information regarding the pose of a detected object
    //!
    struct PoseInfo
    {
      //! @brief Current timestamp
      TimeStamp ts;

      //! @brief 4-digit run number
      int run;

      //! @brief 3-digit frame number
      int frame;

      //! @brief 3-digit detection number
      int dID;

      //! @brief Object identification
      std::string oID;

      //! @brief Rotation matix
      double Rot[9];

      //! @brief Translation along the X axis;
      double Tx;

      //! @brief Translation along the Y axis
      double Ty;

      //! @brief Translation along the Z axis
      double Tz;

      //! @brief Access an element in row major form.
      double&
      R(int i, int j)
      {
        int offset = j * 3 + i;
        if (offset >= 9 || offset < 0)
          throw std::logic_error("i and j not in bounds.");
        return Rot[offset]; //row major
      }
      //! @brief Access an element in row major form. const overload
      double
      R(int i, int j) const
      {
        int offset = j * 3 + i;
        if (offset >= 9 || offset < 0)
          throw std::logic_error("i and j not in bounds.");
        return Rot[offset]; //row major
      }

    };

    typedef boost::shared_ptr<std::ofstream> CSVOutput;

    //! @brief Get a handle to the CSV output file
    //!
    //! @param rn   Information about the current run (run #, team name, etc.)
    //! @param ps   Information of a detected object, including the frame #
    //!
    CSVOutput
    openCSV(const RunInfo &rn);

    //! @brief Output the detected object information in a formatted CSV file
    //!
    //! @param rn   Information about the current run (run #, team name, etc.)
    //! @param ps   Information of a detected object, including the frame #
    //!
    void
    writeCSV(CSVOutput out, const PoseInfo &ps);

  }
}
