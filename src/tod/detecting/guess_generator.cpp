/*
 * twoDToThreeD.cpp
 *
 *  Created on: Jun 16, 2011
 *      Author: vrabaud
 */

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>

#include <ecto/ecto.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <iostream>

#include "tod/detecting/GuessGenerator.h"
#include "tod/detecting/Loader.h"
#include "tod/detecting/Recognizer.h"
#include "tod_stub/tod_stub.h"
#include "tod_stub/tod_stub_impl.h"
#include "tod_stub/csv.h"

namespace po = boost::program_options;

using ecto::tendrils;

struct DetectorOptions
{
  /** The path to a ROS bag file
   */
  std::string bag_file_;
  std::string imageFile;
  std::string baseDirectory;
  std::string config;
  tod::TODParameters params;
  int verbose;
  int mode;
};

/** Ecto implementation of a module that takes
 *
 */
struct GuessGenerator
{
  static void declare_params(tendrils& p)
  {
    p.declare<int>("n_features", "The number of desired features", 1000);
    p.declare<int>("n_levels", "The number of scales", 3);
    p.declare<float>("scale_factor", "The factor between scales", 1.2);
    p.declare<std::string>("base_directory", "Base directory");
    p.declare<std::string>("config_file", "Configuration file");
  }

  static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
  {
    inputs.declare<pcl::PointCloud<pcl::PointXYZRGB> >("point_cloud", "The point cloud");
    inputs.declare<std::vector<cv::KeyPoint> >("keypoints", "The depth image");
    inputs.declare<cv::Mat>("descriptors", "The depth image");
    outputs.declare<std::vector<tod::Guess> >("guesses", "The output 3d points");
  }

  void configure(tendrils& params, tendrils& inputs, tendrils& outputs)
  {
    std::string config_file = params.get<std::string>("config_file");
    DetectorOptions opts;

    po::variables_map vm;

    po::options_description desc("Allowed options");
    desc.add_options()("base,B", po::value<std::string>(&opts.baseDirectory)->default_value("./"),
                       "The directory that the training base is in.");
    desc.add_options()("verbose,V", po::value<int>(&opts.verbose)->default_value(0), "Verbosity level.");

    boost::filesystem::path cf(config_file);
    if (boost::filesystem::exists(cf))
    {
      std::ifstream cf_is(cf.string().c_str());

      po::store(po::parse_config_file(cf_is, desc, false), vm);
      po::notify(vm);
    }
    else
    {
      std::cerr << cf << " does not exist!" << std::endl;
      desc.print(std::cout);
      throw std::runtime_error("Bad options");
    }

    if (!vm.count("base"))
    {
      std::cout << "Must supply training base directory." << "\n";
      std::cout << desc << std::endl;
      throw std::runtime_error("Bad options");
    }

    // Create the base of training data
    tod::Loader loader(opts.baseDirectory);
    std::vector<cv::Ptr<tod::TexturedObject> > objects;
    loader.readTexturedObjects(objects);

    std::cout << "loaded " << objects.size() << " objects from training base." << std::endl;
    if (!objects.size())
    {
      std::cout << "Empty base\n" << std::endl;
      throw tod_stub::ERROR;
    }

    training_base_.reset(new tod::TrainingBase(objects));

    // Create the matcher
    cv::Ptr<tod::Matcher> rtMatcher = tod::Matcher::create(opts.params.matcherParams);
    rtMatcher->add(*training_base_);

    // Create the recognizer
    recognizer_.reset(
        new tod::KinectRecognizer(training_base_.get(), rtMatcher, &opts.params.guessParams, opts.verbose,
                                  opts.baseDirectory));
  }

  /** Get the 2d keypoints and figure out their 3D position from the depth map
   * @param inputs
   * @param outputs
   * @return
   */
  int process(const tendrils& inputs, tendrils& outputs)
  {
    const std::vector<cv::KeyPoint> &keypoints = inputs.get<std::vector<cv::KeyPoint> >("keypoints");
    const cv::Mat & descriptors = inputs.get<cv::Mat>("descriptors");
    const pcl::PointCloud<pcl::PointXYZRGB> & point_cloud = inputs.get<pcl::PointCloud<pcl::PointXYZRGB> >(
        "point_cloud");

    std::vector<tod_stub::Result> results;
    // match to our objects
    std::vector<tod::Guess> found_objects = outputs.get<std::vector<tod::Guess> >("guesses");
    tod::Features2d test;
    test.keypoints = keypoints;
    test.descriptors = descriptors;
    recognizer_->match(test, point_cloud, found_objects);

    return 0;
  }
private:
  boost::shared_ptr<tod::Recognizer> recognizer_;
  boost::shared_ptr<tod::TrainingBase> training_base_;
};

void wrap_GuessGenerator()
{
  ecto::wrap<GuessGenerator>("GuessGenerator", "Given ORB descriptors and 3D positions, compute object guesses.");
}
