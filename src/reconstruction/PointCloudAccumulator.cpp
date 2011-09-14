/*
 * Reconstruction.cpp
 *
 *  Created on: Jun 30, 2011
 *      Author: mkrainin
 */

#include <ecto/ecto.hpp>
#include "surfels.h"
#include "surfel_conversion.h"

#include "boost/foreach.hpp"
#include "pcl/filters/passthrough.h"
#include "pcl/filters/impl/passthrough.hpp"
#include "pcl/filters/statistical_outlier_removal.h"
#include "pcl/filters/impl/statistical_outlier_removal.hpp"
#include "pcl/kdtree/impl/kdtree_flann.hpp"
#include <pcl/features/integral_image_normal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <fstream>

#include <object_recognition/common/conversions.hpp>
#include <ecto_pcl/ecto_pcl.hpp>

using ecto::pcl::xyz_cloud_variant_t;
using ecto::tendrils;
namespace object_recognition
{
  namespace reconstruction
  {
    struct PointCloudAccumulator
    {
      typedef ecto::pcl::PointCloud CloudOutT;

      static void
      declare_params(tendrils& p)
      {

      }

      static void
      declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
      {
        inputs.declare<CloudOutT>("view", "The current 3d view, masked. and transformed into object coordinates");
        inputs.declare<CloudOutT>("previous", "The previous accumulated value.");
        outputs.declare<CloudOutT>("accumulation", "The accumulation of all views");
      }

      void
      configure(const tendrils&p, const tendrils&i, const tendrils&o)
      {
        view = i["view"];
        previous = i["previous"];
        accumulation = o["accumulation"];
      }

      struct accum_dispatch: boost::static_visitor<CloudOutT>
      {
        accum_dispatch(CloudOutT& accum)
            :
              previous(accum)
        {
        }

        template<typename Point>
        CloudOutT
        operator()(boost::shared_ptr<const ::pcl::PointCloud<Point> >& cloud) const
        {

          typedef ::pcl::PointCloud<Point> Cloud;
          typedef boost::shared_ptr<Cloud> Ptr;
          typedef boost::shared_ptr<const Cloud> CloudT;
          CloudOutT result;
          if (!previous.held)
          {
            result = cloud->makeShared();
          }
          else
          {
            CloudT accum_typed(boost::get<CloudT>(previous.held->make_variant()));
            Ptr mcloud(new Cloud(*accum_typed));
            *mcloud += *cloud;
            result = mcloud;
          }
          return result;
        }
        CloudOutT& previous;

      };

      int
      process(const tendrils& /*inputs*/, const tendrils& outputs)
      {
        accum_dispatch dispatch(*previous);
        xyz_cloud_variant_t varient = view->held->make_variant();
        *accumulation = boost::apply_visitor(dispatch, varient);
        return ecto::OK;
      }
      ecto::spore<CloudOutT> view, previous, accumulation;
    };
  }
}

using namespace object_recognition::reconstruction;

ECTO_CELL( reconstruction, PointCloudAccumulator, "PointCloudAccumulator",
          "Accumulate a set of XYZ style point clouds into a single cloud.");
