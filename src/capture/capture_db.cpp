#include <ecto/ecto.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <boost/python/stl_iterator.hpp>
#include <boost/format.hpp>
#include <string>

#include "object_recognition/capture/capture.hpp"
#include "object_recognition/db/couch.hpp"
#include "object_recognition/db/opencv.h"

using ecto::tendrils;

namespace object_recognition
{
  namespace capture
  {

    void
    Observation::operator>>(couch::Document& doc)
    {
      std::map<std::string, cv::Mat> intrinsics, extrinsics;
      intrinsics["K"] = K;
      extrinsics["R"] = R;
      extrinsics["T"] = T;
      std::stringstream intr_ss, extr_ss;
      object_recognition::db::mats2yaml(intrinsics, intr_ss);
      object_recognition::db::mats2yaml(extrinsics, extr_ss);

      doc.update();
      object_recognition::db::png_attach(image, doc, "image");
      object_recognition::db::png_attach(depth, doc, "depth");
      object_recognition::db::png_attach(mask, doc, "mask");
      doc.attach("intrinsics.yml", intr_ss, "text/x-yaml");
      doc.attach("extrinsics.yml", extr_ss, "text/x-yaml");
      doc.update();
      doc.set_value("object_id", object_id);
      doc.set_value("session_id", session_id);
      doc.set_value("frame_number", frame_number);
      doc.commit();
    }
    void
    Observation::operator<<(couch::Document& doc)
    {
      doc.update();
      object_id = doc.get_value<std::string>("object_id");
      session_id = doc.get_value<std::string>("session_id");
      frame_number = doc.get_value<int>("frame_number");
      object_recognition::db::get_png_attachment(image, doc, "image");
      object_recognition::db::get_png_attachment(depth, doc, "depth");
      object_recognition::db::get_png_attachment(mask, doc, "mask");
      std::stringstream intr_ss, extr_ss;
      doc.get_attachment_stream("intrinsics.yml", intr_ss);
      doc.get_attachment_stream("extrinsics.yml", extr_ss);
      std::map<std::string, cv::Mat> intrinsics, extrinsics;
      intrinsics["K"] = cv::Mat();
      extrinsics["R"] = cv::Mat();
      extrinsics["T"] = cv::Mat();
      object_recognition::db::yaml2mats(intrinsics, intr_ss);
      object_recognition::db::yaml2mats(extrinsics, extr_ss);
      K = intrinsics["K"];
      R = extrinsics["R"];
      T = extrinsics["T"];
    }
    void
    Observation::operator>>(const ecto::tendrils& o)
    {
      o.get<cv::Mat>("image") = image;
      o.get<cv::Mat>("mask") = mask;
      o.get<cv::Mat>("depth") = depth;
      o.get<cv::Mat>("R") = R;
      o.get<cv::Mat>("T") = T;
      o.get<cv::Mat>("K") = K;
      o.get<int>("frame_number") = frame_number;
    }
    void
    Observation::operator<<(const ecto::tendrils& i)
    {
      i["image"] >> image;
      i["mask"] >> mask;
      i["depth"] >> depth;
      i["R"] >> R;
      i["T"] >> T;
      i["K"] >> K;
      i["frame_number"] >> frame_number;
    }

  }
}
