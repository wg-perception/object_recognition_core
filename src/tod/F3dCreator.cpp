struct OpenNICapture
{
  static void declare_params(tendrils& params)
  {
    params.declare<int> ("video_mode", "Video size mode", CV_CAP_OPENNI_VGA_30HZ);

  }

  static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
  {
    //set outputs
    outputs.declare<cv::Mat> ("depth", "The output depth map", cv::Mat());
    outputs.declare<cv::Mat> ("valid", "The output valid mask", cv::Mat());

    declare_video_device_outputs(outputs);
    outputs.declare<cv::Mat>("K","The camera intrinsic matrix.");
  }

  void configure(tendrils& params)
  {
    int mode = params.get<int> ("video_mode");
    capture = cv::VideoCapture(CV_CAP_OPENNI);
    capture.set(CV_CAP_OPENNI_IMAGE_GENERATOR_OUTPUT_MODE, mode);
    capture.set(CV_CAP_PROP_OPENNI_REGISTRATION_ON, true);
    // Print some avalible Kinect settings.
    std::cout << "\nDepth generator output mode:" << std::endl << "FRAME_WIDTH    "
        << capture.get(CV_CAP_PROP_FRAME_WIDTH) << std::endl << "FRAME_HEIGHT   "
        << capture.get(CV_CAP_PROP_FRAME_HEIGHT) << std::endl << "FRAME_MAX_DEPTH    "
        << capture.get(CV_CAP_PROP_OPENNI_FRAME_MAX_DEPTH) << " mm" << std::endl << "FPS    "
        << capture.get(CV_CAP_PROP_FPS) << std::endl;

    double frame_width = capture.get(CV_CAP_OPENNI_IMAGE_GENERATOR + CV_CAP_PROP_FRAME_WIDTH);
    double frame_height = capture.get(CV_CAP_OPENNI_IMAGE_GENERATOR + CV_CAP_PROP_FRAME_HEIGHT);
    std::cout << "\nImage generator output mode:" << std::endl << "FRAME_WIDTH    "
        << frame_width << std::endl << "FRAME_HEIGHT   "
        << frame_height << std::endl << "FPS    "
        << capture.get(CV_CAP_OPENNI_IMAGE_GENERATOR + CV_CAP_PROP_FPS) << std::endl;
    double focal_length = capture.get(CV_CAP_OPENNI_DEPTH_GENERATOR_FOCAL_LENGTH);
    // Simple camera matrix: square pixels, principal point at center
    K.create(3,3,CV_64F);
    K = cv::Scalar::all(0);
    K.at<double>(0,0) =  K.at<double>(1,1)  = focal_length;
    K.at<double>(0,2) = (frame_width / 2) - 0.5;
    K.at<double>(1,2) = (frame_width * 3./8.) - 0.5; //aspect ratio for the camera center on kinect and presumably other devices is 4/3
    K.at<double>(2,2) = 1.0;
    std::cout << K << std::endl;
  }

  int process(const tendrils& inputs, tendrils& outputs)
  {
    if( !capture.grab() )
    {
        std::cout << "Can not grab images." << std::endl;
        return 1;
    }
    //outputs.get is a reference;
    capture.retrieve(outputs.get<cv::Mat>("depth"), CV_CAP_OPENNI_DEPTH_MAP);
    capture.retrieve(outputs.get<cv::Mat>("image"), CV_CAP_OPENNI_BGR_IMAGE);
    capture.retrieve(outputs.get<cv::Mat>("valid"), CV_CAP_OPENNI_VALID_DEPTH_MASK);
    outputs.get<cv::Mat>("K") = K;
    //increment our frame number.
    ++(outputs.get<int> ("frame_number"));
    return 0;
  }

  cv::VideoCapture capture;
  cv::Mat K;
};
