#pragma once

#include <fstream>
#include <string>
#include <map>

#include <opencv2/core/core.hpp>
#include <boost/serialization/binary_object.hpp>
#include <boost/serialization/split_free.hpp>

namespace couch {
  class Document;
}
namespace object_recognition
{
  namespace db
  {
    void
    mats2yaml(const std::map<std::string, cv::Mat>& mm,std::ostream& out);

    void
    yaml2mats(std::map<std::string, cv::Mat>& mm,std::istream& in);

    void
    png_attach(cv::Mat image, couch::Document& doc, const std::string& name);

    void
    get_png_attachment(cv::Mat& image, couch::Document& doc, const std::string& name);
  }
}

BOOST_SERIALIZATION_SPLIT_FREE(cv::Mat);

