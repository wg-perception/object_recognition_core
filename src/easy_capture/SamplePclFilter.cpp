/* 
 * Example of how to create a cell with PclCell
 */

#include <ecto_pcl/ecto_pcl.hpp>
#include <ecto_pcl/pcl_cell.hpp>
// other pcl includes!
namespace object_recognition
{
  struct ExampleFilter
  {
    static void declare_params(ecto::tendrils& params)
    {
      // put declarations here as usual!
      params.declare<int> ("a_param", "Description of params.", 0);
    }

    static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
    {
      /* 
       * A single "ecto::pcl::PointCloud" input is already defined... any others should be here
       * If you need 2 input PointClouds, use the ecto::pcl::PclCellDualInputs
       */

      // Most cells will output a ecto::pcl::PointCloud
      outputs.declare<ecto::pcl::PointCloud> ("output", "Cloud after my stufz has run.");
    }

    void configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
    {
      // Store params in spores
      my_param_ = params["a_param"];

      // Same for inputs/outputs
      output_ = outputs["output"];
    }
    
    template <typename Point>
    int process(const tendrils& inputs, const tendrils& outputs, 
                boost::shared_ptr<const pcl::PointCloud<Point> >& input)
    {
      typedef pcl::PointCloud<Point> PointCloudT;
      // cloud to store our output in
      typename PointCloudT::Ptr cloud(new PointCloudT);

      // do something with our params/clouds
      *cloud = *input;

      // We have to use this variant to create an ecto::pcl::PointCloud
      *output_ = ecto::pcl::xyz_cloud_variant_t(cloud);
      return ecto::OK;
    }

    // Store params/inputs/outputs in spores
    ecto::spore<int> my_param_;
    ecto::spore<ecto::pcl::PointCloud> output_;
  };
}
using namespace object_recognition;
ECTO_CELL(easy_capture, ecto::pcl::PclCell<ExampleFilter>, "ExampleFilter",
  "Example of creating a custom filter.");
