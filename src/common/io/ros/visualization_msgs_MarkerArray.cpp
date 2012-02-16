/* DO NOT EDIT or check into source control
 * Generated code for wrapping a ros message Pub/Sub in ecto
 *
 * package : visualization_msgs
 * msg : MarkerArray
 */
#include <ecto_ros/wrap_sub.hpp>
#include <ecto_ros/wrap_pub.hpp>
#include <ecto_ros/wrap_bag.hpp>


#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

namespace io_ros
{
    struct Subscriber_Marker : ecto_ros::Subscriber<visualization_msgs::Marker> {};
    struct Publisher_Marker : ecto_ros::Publisher<visualization_msgs::Marker> {};
    struct Bagger_Marker : ecto_ros::Bagger<visualization_msgs::Marker> {};
}

ECTO_CELL(io_ros, io_ros::Subscriber_Marker,"Subscriber_Marker", "Subscribes to a visualization_msgs::Marker.");
ECTO_CELL(io_ros, io_ros::Publisher_Marker,"Publisher_Marker", "Publishes a visualization_msgs::Marker.");
ECTO_CELL(io_ros, io_ros::Bagger_Marker,"Bagger_Marker", "A bagger for messages of a given type. Can enable read/write to ros bags.");

namespace io_ros
{
    struct Subscriber_MarkerArray : ecto_ros::Subscriber<visualization_msgs::MarkerArray> {};
    struct Publisher_MarkerArray : ecto_ros::Publisher<visualization_msgs::MarkerArray> {};
    struct Bagger_MarkerArray : ecto_ros::Bagger<visualization_msgs::MarkerArray> {};
}

ECTO_CELL(io_ros, io_ros::Subscriber_MarkerArray,"Subscriber_MarkerArray", "Subscribes to a visualization_msgs::MarkerArray.");
ECTO_CELL(io_ros, io_ros::Publisher_MarkerArray,"Publisher_MarkerArray", "Publishes a visualization_msgs::MarkerArray.");
ECTO_CELL(io_ros, io_ros::Bagger_MarkerArray,"Bagger_MarkerArray", "A bagger for messages of a given type. Can enable read/write to ros bags.");
