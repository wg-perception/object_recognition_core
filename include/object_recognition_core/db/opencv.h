#pragma once

#include <fstream>
#include <string>
#include <map>

#include <opencv2/core/core.hpp>
#include <boost/serialization/binary_object.hpp>
#include <boost/serialization/split_free.hpp>

#include <object_recognition_core/db/document.h>

namespace object_recognition_core
{
  namespace db
  {
    std::string
    temporary_yml_file_name(bool do_gzip);

    void
    mats2yaml(const std::map<std::string, cv::Mat>& mm,std::ostream& out, bool do_gzip = false);

    void
    yaml2mats(std::map<std::string, cv::Mat>& mm,std::istream& in, bool do_gzip = false);

    void
    png_attach(cv::Mat image, db::DummyDocument& doc, const std::string& name);

    void
    get_png_attachment(cv::Mat& image, const db::DummyDocument& doc, const std::string& name);
  }
}

