Training
==================================
At the highest level, textured object training boils down to the following
inputs and outputs.

inputs
	Consider a set of 2D/3D observations of an object situated in a known environment.
	
	 - 2D image
	 - 3D points per pixel (point cloud)
	 - Camera intrinsics
	 - fiducial pattern description
	 
outputs
	After all images have been observed of an object we may generate the following output.
	
	Set of N views
		- ORB descriptors
		- 3D points per descriptor
		- camera pose [R|T]
		
	Unified Points
		- ORB descriptors
		- 3D points all transformed into the same frame


modules
================================

Fiducial Detector
	Finds 2D points in an image that are correlated easily with a known 3D point.
	Think checkerboard.
	
	parameters
		- fiducial description (rows, cols, dimensions)
		
	inputs
		- 2D image
		
	outputs
		- 2D observation points
		- corresponding 3D ideal points

Pose Estimation (3D to 3D)
	The Pose Estimation is for finding the relative pose between two sets of points.
	
	inputs
		- 3D observation points
		- 3D model points
		
	outputs
		- Pose ( [R | T] ) that takes the model points to the observation points

Pose Estimation (2D to 3D)
	The Pose Estimation is for finding the relative pose between two sets of points.
	
	inputs
		- 2D observation points
		- 3D model points
		- camera model - calibrated
		
	outputs
		- Pose ( [R | T] ) that takes the model points to the observation points
		
ORB Detector
	Given a 2D frame, and an optional mask, computes ORB features.
	
	inputs
		- 2D image
		- 2D mask (optional)
	
	outputs
		- 2D keypoints
		- descriptors

Planar Segmentation (3D point cloud method)
	Given the pose, assuming it describes the center of the object coordinate system and lies on a plane,
	segment the object from the plane
	
	inputs
		- Pose of the plane in the scene
		- 3D point cloud
	
	outputs
		- 3D points that lie above the plane (this may then be projected into a 2D image to form a mask)

Planar Segmentation (Depth image only)
	Given the pose, assuming it describes the center of the object coordinate system and lies on a plane,
	segment the object from the plane
	
	inputs
		- Pose of the plane in the scene
		- 2D depth map
		- focal length
	
	outputs
		- 2D mask ( 255 for valid, 0 for invalid )

View Accumulator
	Accumulate all the views of the object. Internally this will either cache to some storage.
	
	inputs
		- 2D image
		- 2D mask
		- 2D depth map
		- Pose
		- Keypoints
		- Descriptors
	
	output
		- unified model
	
	parameters
		- model id
		- storage type
	
