#include <ecto/ecto.hpp>
#include <opencv2/core/core.hpp>

#include <object_recognition/db/db.h>

namespace object_recognition
{
  namespace tod
  {
    struct ModelFiller
    {
    public:
      static void
      declare_io(const ecto::tendrils& params, ecto::tendrils& inputs, ecto::tendrils& outputs)
      {
        typedef ModelFiller C;
        inputs.declare(&C::points_, "points", "The 3d position of the points.");
        inputs.declare(&C::descriptors_, "descriptors", "The descriptors.");
        outputs.declare(&C::db_document_, "db_document", "The filled document.");
      }

      int
      process(const ecto::tendrils& inputs, const ecto::tendrils& outputs)
      {
        db_document_->set_attachment<cv::Mat>("descriptors", *descriptors_);
        db_document_->set_attachment<cv::Mat>("points", *points_);
        return ecto::OK;
      }

    private:
      ecto::spore<cv::Mat> points_;
      ecto::spore<cv::Mat> descriptors_;
      ecto::spore<db::Document> db_document_;
    };

  }
}

ECTO_CELL(tod_training, object_recognition::tod::ModelFiller, "ModelFiller",
          "Populates a db document with a TOD model for persisting a later date.")
