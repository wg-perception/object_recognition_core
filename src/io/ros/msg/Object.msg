################################################## OBJECT INFO #########################################################

# Contains information about the identity and the position of a found object
# Some of those fields might not be filled because the used techniques do not fill them or because the user does not
# request them

# The id of the found object
string id

# The db parameters stored as a JSON/compressed YAML string. An object id does not make sense without the corresponding
# database. E.g., in object_recognition, it can look like: "{'type':'CouchDB', 'root':'http://localhost'}"
# There is no conventional format for those parameters and it's nice to keep that flexibility.
string db

############################################## EXTRA OBJECT INFO #######################################################

# The human readable name of the object
string name

# The full mesh of the object: this can be useful for display purposes, augmented reality ... but it can be big
# Make sure the type is MESH_RESOURCE
visualization_msgs/Marker ground_truth_mesh

# Sometimes, you only have a cloud in the DB
sensor_msgs/PointCloud2 ground_truth_point_cloud

################################################ OBJECT CLUSTERS #######################################################

# Sometimes you can extract the 3d points that belong to the object
sensor_msgs/PointCloud2 point_cloud

# Sometimes, you can only provide a bounding box/shape, even in 3d
visualization_msgs/Marker bounding_mesh

# Sometimes, you only have 2d input so you can't really give a pose, you just get a contour, or a box
# The last point will be linked to the first one automatically
geometry_msgs/Point[] bounding_contour

##################################################### SEARCH ###########################################################

# The confidence is independent of the pose and is between 0 and 1: e.g. you have a technique that can find cans of soda
# but cannot differentiate between Coke and Sprite. If it only has those two in the DB, it will give a confidence of 0.5
# for each
float32 confidence

#################################################### POSE INFO #########################################################

# This is the result that everybody expects : the pose in some frame given with the input. The units are radian/meters
# as usual
geometry_msgs/PoseStamped pose

# The covariance matrix for the rotation angles in Rodrigues notation, in row major notation
float32[] covariance_rotation

# The covariance matrix for the translation vector, in row major notation
float32[] covariance_translation
