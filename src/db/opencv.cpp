#include <cstdio>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>

#include <object_recognition_core/db/opencv.h>

namespace fs = boost::filesystem;

namespace object_recognition_core
{
  namespace db
  {
    std::string
    temporary_yml_file_name(bool do_gzip)
    {
      std::string fname;
      {
        char buffer[L_tmpnam];
        char* p = std::tmpnam(buffer);
        if (p != NULL)
        {
          fname = std::string(buffer) + ".yml";
          if (do_gzip)
            fname += ".gz";
        }
        else
          throw std::runtime_error("Could not create temporary filename!");
      }
      return fname;
    }

    void
    mats2yaml(const std::map<std::string, cv::Mat>& mm,std::ostream& out, bool do_gzip)
    {
      std::string fname = temporary_yml_file_name(do_gzip);
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
    yaml2mats(std::map<std::string, cv::Mat>& mm,std::istream& in, bool do_gzip)
    {
      std::string fname = temporary_yml_file_name(do_gzip);
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
    png_attach(cv::Mat image, db::DummyDocument& doc, const std::string& name)
    {
      std::vector<uint8_t> buffer;
      std::stringstream ss;
      cv::imencode(".png", image, buffer);
      std::copy(buffer.begin(), buffer.end(), std::ostream_iterator<uint8_t>(ss));
      doc.set_attachment_stream(name, ss, "image/png");
    }

    void
    get_png_attachment(cv::Mat& image, const db::DummyDocument& doc, const std::string& name)
    {
      std::stringstream ss;
      doc.get_attachment_stream(name, ss);
      std::streampos length = ss.tellp();
      std::vector<uint8_t> buffer(length);
      ss.read((char*) buffer.data(), length);
      image = cv::imdecode(buffer, cv::IMREAD_ANYDEPTH | cv::IMREAD_ANYCOLOR);
    }
  }

}
