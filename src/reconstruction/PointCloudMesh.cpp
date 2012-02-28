#include <ecto/ecto.hpp>

#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/grid_projection.h>
#include <pcl/surface/marching_cubes.h>
#include <pcl/surface/organized_fast_mesh.h>
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
      size_t number_of_faces = triangles.polygons.size();

      std::ofstream mesh_file(std::string(mesh_file_name).c_str());
      mesh_file << "ply\n"
                "format ascii 1.0\n"
                << "element vertex " << nr_points << "\n" << "property float x\n"
                "property float y\n"
                "property float z\n"
                << "element face " << number_of_faces << '\n' << "property list uchar int vertex_indices\n"
                << "end_header\n";

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
      //3 2852 2853 2835
      for (size_t i = 0; i < number_of_faces; ++i)
      {
        mesh_file << triangles.polygons[i].vertices.size() << " ";
        size_t j = 0, end = triangles.polygons[i].vertices.size() - 1;
        for (j = 0; j < end; ++j)
          mesh_file << triangles.polygons[i].vertices[j] << " ";
        mesh_file << triangles.polygons[i].vertices[j] << '\n';
      }

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
#if PCL_VERSION_GE_151
        typedef pcl::search::KdTree<Point> KdTree;
#else
        typedef pcl::KdTreeFLANN<Point> KdTree;
#endif
        typedef KdTree::Ptr KdTreePtr;

        void
        operator()(const CloudT& i) const
        {
          // Initialize objects
          pcl::GreedyProjectionTriangulation<Point> gp3;
          pcl::PolygonMesh triangles;

          // Set the maximum distance between connected points (maximum edge length)
          gp3.setSearchRadius(0.1);

          // Set typical values for the parameters
          gp3.setMu(2.5);
          gp3.setMaximumNearestNeighbors(100);
          gp3.setNormalConsistency(true);
          //        gp3.setMaxi(M_PI / 4); // 45 degrees
          gp3.setMinimumAngle(M_PI / 30); // 10 degrees
          gp3.setMaximumAngle(M_PI); // 120 degrees
          // Create search tree*
          KdTreePtr tree(new KdTree);
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
