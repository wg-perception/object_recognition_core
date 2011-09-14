#pragma once

#include <fstream>
#include <string>
#include <map>

#include <opencv2/core/core.hpp>
#include <boost/serialization/binary_object.hpp>
#include <boost/serialization/split_free.hpp>

#include <object_recognition/db/db.h>
namespace object_recognition
{
  namespace db
  {
    void
    mats2yaml(const std::map<std::string, cv::Mat>& mm,std::ostream& out);

    void
    yaml2mats(std::map<std::string, cv::Mat>& mm,std::istream& in);

    void
    png_attach(cv::Mat image, db_future::Document& doc, const std::string& name);

    void
    get_png_attachment(cv::Mat& image, db_future::Document& doc, const std::string& name);
  }
}

namespace boost
{
  namespace serialization
  {
    template<class Archive>
    void
    save(Archive & ar, const cv::Mat & m, const unsigned int version)
    {
      int type = m.type();
      ar & m.rows;
      ar & m.cols;
      ar & type;
      const uchar * data = m.data, *end = m.dataend;
      ar & boost::serialization::make_binary_object(const_cast<uchar*>(data), size_t(end - data));
    }

    template<class Archive>
    void
    load(Archive & ar, cv::Mat & m, const unsigned int version)
    {
      int rows, cols, type;
      ar & rows;
      ar & cols;
      ar & type;

      if (rows > 0 && cols > 0)
      {
        m.create(rows, cols, type);
        uchar * data = m.data, *end = m.dataend;
        ar & boost::serialization::make_binary_object(data, end - data);
      }
      else
      {
        std::cout << "bad matrix rows: " << rows << " cols: " << cols << " type: " << type << std::endl;
      }
    }
  } // namespace serialization
} // namespace boost

BOOST_SERIALIZATION_SPLIT_FREE(cv::Mat);

