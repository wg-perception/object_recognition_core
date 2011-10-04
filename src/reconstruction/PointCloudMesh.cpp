#include <ecto/ecto.hpp>

#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/grid_projection.h>
#include <pcl/surface/marching_cubes.h>
#include <pcl/surface/organized_fast_mesh.h>
#include <pcl/surface/surfel_smoothing.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/reconstruction.h>
#include <ecto_pcl/ecto_pcl.hpp>

using ecto::tendrils;
using ecto::pcl::xyz_cloud_variant_t;
using ecto::tendrils;
namespace object_recognition
{
  namespace reconstruction
  {

    //////////////////////////////////////////////////////////////////////////////////////////////
    void
    saveTriangleMeshPly(const pcl::PolygonMesh &triangles, const std::string& mesh_file_name)
    {
      if (triangles.cloud.data.empty())
      {
        return;
      }
      int nr_points = triangles.cloud.width * triangles.cloud.height;
      int point_size = triangles.cloud.data.size() / nr_points;

      std::ofstream mesh_file(std::string(mesh_file_name).c_str());
      mesh_file << "ply\n"
                "format ascii 1.0\n"
                << "element vertex " << nr_points << "\n" << "property float x\n"
                "property float y\n"
                "property float z\n"
                "end_header\n";
      //Write the header information
      //fs << "# vtk DataFile Version 3.0\nvtk output\nASCII\nDATASET POLYDATA\nPOINTS " << nr_points << " float"
      //        << std::endl;

      //<x> <y> <z> <r> <g> <b>
      //for (size_t i = 0; i < cloud_m.points.size(); i++)
      //{
      //  const CloudPOINTXYZRGBNORMAL::PointType& p = cloud_m.points[i];
      //  mesh_file << p.x << " " << p.y << " " << p.z << " " << int(p.r) << " " << int(p.g) << " " << int(p.b) << " "
      //            << p.normal_x << " " << p.normal_y << " " << p.normal_z << "\n";
      //}
      const unsigned field_size = triangles.cloud.fields.size();
      // Iterate through the points
      for (int i = 0; i < nr_points; ++i)
      {
        float values[3];
        int c = 0;
        int xyz = 0;
        for (size_t d = 0; d < field_size; ++d)
        {
          int count = triangles.cloud.fields[d].count;
          if (count == 0)
            count = 1; // we simply cannot tolerate 0 counts (coming from older converter code)
          if ((triangles.cloud.fields[d].datatype == sensor_msgs::PointField::FLOAT32)
              && (triangles.cloud.fields[d].name == "x" || triangles.cloud.fields[d].name == "y"
                  || triangles.cloud.fields[d].name == "z"))
          {
            memcpy(&values[xyz],
                   &triangles.cloud.data[i * point_size + triangles.cloud.fields[d].offset + c * sizeof(float)],
                   sizeof(float));
            if (++xyz == 3)
              break;
          }

        }
        mesh_file << boost::str(boost::format("%0.10f %0.10f %0.10f\n") % values[0] % values[1] % values[2]);

      }

//      // Write vertices
//      fs << "\nVERTICES " << nr_points << " " << 2 * nr_points << std::endl;
//      for (int i = 0; i < nr_points; ++i)
//        fs << "1 " << i << std::endl;
//
//      // Write polygons
//      // compute the correct number of values:
//      size_t triangle_size = triangles.polygons.size();
//      int correct_number = triangle_size;
//      for (size_t i = 0; i < triangle_size; ++i)
//        correct_number += triangles.polygons[i].vertices.size();
//      fs << "\nPOLYGONS " << triangle_size << " " << correct_number << std::endl;
//      for (size_t i = 0; i < triangle_size; ++i)
//      {
//        fs << triangles.polygons[i].vertices.size() << " ";
//        size_t j = 0;
//        for (j = 0; j < triangles.polygons[i].vertices.size() - 1; ++j)
//          fs << triangles.polygons[i].vertices[j] << " ";
//        fs << triangles.polygons[i].vertices[j] << std::endl;
//      }
//
//      // Write RGB values
//      int field_index = pcl::getFieldIndex(triangles.cloud, "rgb");
//      if (field_index != -1)
//      {
//        fs << "\nPOINT_DATA " << nr_points << "\nCOLOR_SCALARS scalars 3\n";
//        for (int i = 0; i < nr_points; ++i)
//        {
//          int count = triangles.cloud.fields[field_index].count;
//          if (count == 0)
//            count = 1; // we simply cannot tolerate 0 counts (coming from older converter code)
//          int c = 0;
//          if (triangles.cloud.fields[field_index].datatype == sensor_msgs::PointField::FLOAT32)
//          {
//            float value;
//            memcpy(
//                &value,
//                &triangles.cloud.data[i * point_size + triangles.cloud.fields[field_index].offset + c * sizeof(float)],
//                sizeof(float));
//            int color = *reinterpret_cast<const int*>(&(value));
//            int r = (0xff0000 & color) >> 16;
//            int g = (0x00ff00 & color) >> 8;
//            int b = 0x0000ff & color;
//            fs << (float) r / 255.0 << " " << (float) g / 255.0 << " " << (float) b / 255.0;
//          }
//          fs << std::endl;
//        }
//      }
//
//      // Close file
//      fs.close();
    }

    struct PointCloudMesh
    {
      typedef ecto::pcl::PointCloud CloudOutT;

      static void
      declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
      {
        inputs.declare(&PointCloudMesh::cloud_input, "input",
                       "The current 3d view, masked. and transformed into object coordinates");
      }
      /* dispatch to handle process */
      struct meshit_: boost::static_visitor<void>
      {
        typedef pcl::PointXYZRGBNormal Point;
        typedef pcl::PointCloud<Point> Cloud;
        typedef boost::shared_ptr<const Cloud> CloudT;
        typedef pcl::KdTreeFLANN<Point> KdTree;
        typedef KdTree::Ptr KdTreePtr;

        void
        operator()(const CloudT& i) const
        {
          // Initialize objects
          pcl::GreedyProjectionTriangulation<Point> gp3;
          pcl::PolygonMesh triangles;

          // Set the maximum distance between connected points (maximum edge length)
          gp3.setSearchRadius(0.04);

          // Set typical values for the parameters
          gp3.setMu(2.5);
          gp3.setMaximumNearestNeighbors(100);
          //        gp3.setMaxi(M_PI / 4); // 45 degrees
          gp3.setMinimumAngle(M_PI / 18); // 10 degrees
          gp3.setMaximumAngle(2 * M_PI / 3); // 120 degrees
          gp3.setNormalConsistency(true);

          // Create search tree*
          KdTreePtr tree(new KdTree);
          tree->setInputCloud(i);

          // Get result
          gp3.setInputCloud(i);
          gp3.setSearchMethod(tree);
          gp3.reconstruct(triangles);
          saveTriangleMeshPly(triangles, "mesh.ply");
        }
        template<typename CloudType>
        void
        operator()(const CloudType& i) const
        {
          throw std::runtime_error("Not implemented for this type!");
        }
      };

      int
      process(const tendrils& /*inputs*/, const tendrils& outputs)
      {
        xyz_cloud_variant_t variant = cloud_input->make_variant();
        boost::apply_visitor(meshit_(), variant);
        return ecto::OK;
      }
      ecto::spore<CloudOutT> cloud_input;
    };
  }
}

using namespace object_recognition::reconstruction;

ECTO_CELL( reconstruction, PointCloudMesh, "PointCloudMesh", "Construct a mesh from a point cloud.");
