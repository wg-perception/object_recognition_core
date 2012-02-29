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

#include <boost/format.hpp>
#include <boost/filesystem.hpp>

#include "csv.h"

namespace object_recognition_core
{
  namespace io
  {

    void
    TimeStamp::set()
    {
      std::time_t local = std::time(NULL);
      std::tm *Tm = localtime(&local);
      year = Tm->tm_year;
      month = Tm->tm_mon;
      day = Tm->tm_mday;
      hour = Tm->tm_hour;
      min = Tm->tm_min;
      sec = Tm->tm_sec;
      msec = 0; //FIXME get better resolution
    }

    CSVOutput
    openCSV(const RunInfo &rn)
    {
      boost::shared_ptr<std::ofstream> out(new std::ofstream());

      //! Format the name of the output file
      std::string name = str(
          boost::format("RUN%.4d_%s_%d%d%d_%.2d.%.2d.%.2d.csv") % rn.runID % rn.name % rn.ts.year % rn.ts.month
          % rn.ts.day
          % rn.ts.hour
          % rn.ts.min
          % rn.ts.sec);
      out->open(name.c_str());
      //! Print the key at the top of the file for data interpretation
      *out << "ts,Run,Frame,dID,oID,R11,R12,R13,R21,R22,R23,R31,R32,R33,Tx,Ty,Tz" << std::endl;
      return out;
    }
    void
    writeCSV(CSVOutput out, const PoseInfo &ps)
    {

      //! Write the pose information
      //! JAM: Note, given the CSV format specified in the documentation, I
      //!      assumed the leading zeros were important, so I put everything
      //!      in character buffers before writing to disk.

      //! Write the timestamp
      //sprintf(buffer, "%.2d.%.2d.%.2d.%.3d,", ps.ts.hour, ps.ts.min, ps.ts.sec, ps.ts.msec);
      *out << boost::format("%.2d.%.2d.%.2d.%.3d,") % ps.ts.hour % ps.ts.min % ps.ts.sec % ps.ts.msec;
      //! Write the 4-digit run number
      //sprintf(buffer, "%.4d,", ps.run);
      *out << boost::format("%.4d,") % ps.run;
      //! Write the 3-digit frame number
      //sprintf(buffer, "%.3d,", ps.frame);
      *out << boost::format("%.3d,") % ps.frame;
      //! Write the 3-digit detection number
      //sprintf(buffer, "%.3d,", ps.dID);
      *out << boost::format("%.3d,") % ps.dID;
      //! Write the object name string
      *out << ps.oID << ",";

      //! Write the object transformation
      *out << ps.R(0, 0) << "," << ps.R(0, 1) << "," << ps.R(0, 2) //first row
           << "," << ps.R(1, 0) << "," << ps.R(1, 1) << "," << ps.R(1, 2) //second row
           << "," << ps.R(2, 0) << "," << ps.R(2, 1) << "," << ps.R(2, 2) << ","; //third row
      *out << ps.Tx << "," << ps.Ty << "," << ps.Tz << std::endl;
    }
  }
}
