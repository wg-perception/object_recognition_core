#include <cstdio>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>

//#include "object_recognition/db/couch.hpp"
#include "object_recognition/db/opencv.h"

namespace fs = boost::filesystem;
namespace
{
  std::string
  gen_temp_yml()
  {
    std::string fname;
    {
      char buffer[L_tmpnam];
      char* p = std::tmpnam(buffer);
      if (p != NULL
      )
        fname = std::string(buffer) + ".yml";
      else
        throw std::runtime_error("Could not create temporary filename!");
    }
    return fname;
  }
}
namespace object_recognition
{
  namespace db
  {
    void
    mats2yaml(const std::map<std::string, cv::Mat>& mm,std::ostream& out)
    {
      std::string fname = gen_temp_yml();
      {
        cv::FileStorage fs(fname, cv::FileStorage::WRITE);
        typedef std::pair<std::string, cv::Mat> pair_t;
        BOOST_FOREACH(const pair_t& x, mm)
            {
              fs << x.first << x.second;
            }
      }
      {
        std::ifstream reader(fname.c_str());
        out << reader.rdbuf();
      }
      fs::remove(fname.c_str());
    }

    void
    yaml2mats(std::map<std::string, cv::Mat>& mm,std::istream& in)
    {
      std::string fname = gen_temp_yml();
      {
        std::ofstream writer(fname.c_str());
        writer << in.rdbuf();
      }
      {
        cv::FileStorage fs(fname, cv::FileStorage::READ);

        typedef std::pair<std::string, cv::Mat> pair_t;
        BOOST_FOREACH(const pair_t& x, mm)
            {
              fs[x.first] >> mm[x.first];
            }
      }
      fs::remove(fname.c_str());
    }

    void
    png_attach(cv::Mat image, db_future::Document& doc, const std::string& name)
    {
      std::vector<uint8_t> buffer;
      std::stringstream ss;
      cv::imencode(".png", image, buffer);
      std::copy(buffer.begin(), buffer.end(), std::ostream_iterator<uint8_t>(ss));
      doc.set_attachment_stream(name,ss,"image/png");
    }

    void
    get_png_attachment(cv::Mat& image, db_future::Document& doc, const std::string& name)
    {
      std::stringstream ss;
      doc.get_attachment_stream(name, ss);
      std::streampos length = ss.tellp();
      std::vector<uint8_t> buffer(length);
      ss.read((char*) buffer.data(), length);
      image = cv::imdecode(buffer, CV_LOAD_IMAGE_ANYDEPTH | CV_LOAD_IMAGE_ANYCOLOR);
    }
  }

}
