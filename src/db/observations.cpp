#include <ecto/ecto.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <boost/python/stl_iterator.hpp>
#include <boost/format.hpp>
#include <string>

#include <object_recognition_core/db/document.h>
#include <object_recognition_core/db/opencv.h>
#include <object_recognition_core/db/prototypes/observations.hpp>

using ecto::tendrils;

namespace object_recognition_core
{
  namespace prototypes
  {
    void
    Observation::declare(ecto::tendrils& t, bool required)
    {
      t.declare<cv::Mat>("image", "An rgb full frame image.").required(required);
      t.declare<cv::Mat>("depth", "The 16bit depth image.").required(required);
      t.declare<cv::Mat>("mask", "The mask.").required(required);
      t.declare<cv::Mat>("R", "The orientation.").required(required);
      t.declare<cv::Mat>("T", "The translation.").required(required);
      t.declare<cv::Mat>("K", "The camera intrinsic matrix").required(required);
      t.declare<int>("frame_number", "The frame number");
    }
    void
    operator>>(Observation& o, db::DummyDocument* doc)
    {
      std::map<std::string, cv::Mat> intrinsics, extrinsics;
      intrinsics["K"] = o.K;
      extrinsics["R"] = o.R;
      extrinsics["T"] = o.T;
      std::stringstream intr_ss, extr_ss;
      object_recognition_core::db::mats2yaml(intrinsics, intr_ss);
      object_recognition_core::db::mats2yaml(extrinsics, extr_ss);

      object_recognition_core::db::png_attach(o.image, *doc, "image");
      object_recognition_core::db::png_attach(o.depth, *doc, "depth");
      object_recognition_core::db::png_attach(o.mask, *doc, "mask");
      doc->set_attachment_stream("intrinsics.yml", intr_ss, "text/x-yaml");
      doc->set_attachment_stream("extrinsics.yml", extr_ss, "text/x-yaml");
      doc->set_field("Type", "Observation");
      doc->set_field("object_id", o.object_id);
      doc->set_field("session_id", o.session_id);
      doc->set_field("frame_number", o.frame_number);
    }

    void
    operator<<(Observation& o, const db::DummyDocument* doc)
    {
      o.object_id = doc->get_field<std::string>("object_id");
      o.session_id = doc->get_field<std::string>("session_id");
      o.frame_number = doc->get_field<int>("frame_number");
      object_recognition_core::db::get_png_attachment(o.image, *doc, "image");
      object_recognition_core::db::get_png_attachment(o.depth, *doc, "depth");
      object_recognition_core::db::get_png_attachment(o.mask, *doc, "mask");
      std::stringstream intr_ss, extr_ss;
      doc->get_attachment_stream("intrinsics.yml", intr_ss);
      doc->get_attachment_stream("extrinsics.yml", extr_ss);
      std::map<std::string, cv::Mat> intrinsics, extrinsics;
      intrinsics["K"] = cv::Mat();
      extrinsics["R"] = cv::Mat();
      extrinsics["T"] = cv::Mat();
      object_recognition_core::db::yaml2mats(intrinsics, intr_ss);
      object_recognition_core::db::yaml2mats(extrinsics, extr_ss);
      o.K = intrinsics["K"];
      o.R = extrinsics["R"];
      o.T = extrinsics["T"];
    }
    void
    operator>>(Observation& obs, const ecto::tendrils& o)
    {
      o["image"] << obs.image;
      o["depth"] << obs.depth;
      o["mask"] << obs.mask;
      o["R"] << obs.R;
      o["T"] << obs.T;
      o["K"] << obs.K;
      o["frame_number"] << obs.frame_number;
    }

    void
    operator<<(Observation& obs, const ecto::tendrils& i)
    {
      i["image"] >> obs.image;
      i["mask"] >> obs.mask;
      i["depth"] >> obs.depth;
      if (obs.depth.depth() == CV_32F)
      {
        obs.depth.clone().convertTo(obs.depth, CV_16UC1, 1000);
      }
      i["R"] >> obs.R;
      i["T"] >> obs.T;
      i["K"] >> obs.K;
    }

  }
}
