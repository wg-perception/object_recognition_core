#include <ecto/ecto.hpp>
#include <opencv2/core/core.hpp>

#include <object_recognition/db/ModelInserter.hpp>

namespace object_recognition
{
  namespace tod
  {
    /** Class inserting the TOD models in the DB
     */
    struct ModelInserter_
    {
      static void
      declare_params(ecto::tendrils& params)
      {
      }

      static void
      declare_io(const ecto::tendrils& params, ecto::tendrils& inputs, ecto::tendrils& outputs)
      {
        typedef ModelInserter_ C;
        inputs.declare(&C::points_, "points", "The 3d position of the points.");
        inputs.declare(&C::descriptors_, "descriptors", "The descriptors.");
      }

      void
      configure(const ecto::tendrils& params, const ecto::tendrils& inputs, const ecto::tendrils& outputs)
      {

      }

      int
      process(const ecto::tendrils& inputs, const ecto::tendrils& outputs, db::Document& doc)
      {
        doc.set_attachment<cv::Mat>("descriptors", *descriptors_);
        doc.set_attachment<cv::Mat>("points", *points_);
        return ecto::OK;
      }

      const std::string& model_type() const
      {
        static std::string s = "TOD";
        return s;
      }

    private:
      ecto::spore<cv::Mat> points_;
      ecto::spore<cv::Mat> descriptors_;
    };

    //for type prettiness
    struct ModelInserter: db::bases::ModelInserter<ModelInserter_>
    {
    };
  }
}

ECTO_CELL(tod_training, object_recognition::tod::ModelInserter, "ModelInserter", "Insert a TOD model into the db")
