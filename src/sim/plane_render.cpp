#if DO_ECTO
#include <ecto/ecto.hpp>
#endif

#include <vtkImageData.h>
#include <vtkPNGReader.h>
#include <vtkPolyDataMapper.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>
#include <vtkTextureMapToPlane.h>
#include <vtkPlaneSource.h>
#include <vtkTexture.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkTextureMapToSphere.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkCommand.h>
#include <vtkCallbackCommand.h>
#include <vtkCamera.h>
#include <vtkRendererCollection.h>
#include <vtkMatrix4x4.h>
#include <vtkPerspectiveTransform.h>

#include <boost/format.hpp>
#include <boost/thread.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace object_recognition
{
  void
  vtk_to_K(cv::Size sz, vtkCamera* cam, cv::Mat& K, cv::Mat& R, cv::Mat& T);

  void
  grab_frame(vtkRenderWindow* renderWindow, cv::Mat& image, cv::Mat& depth, cv::Mat& mask, cv::Mat& K, cv::Mat& R,
             cv::Mat& T);
  void
  grab_frame_callback(vtkObject* caller, long unsigned int eventId, void* clientData, void* callData);

  struct SimRunner
  {

    SimRunner(const std::string image_name, double width, double height, int window_width, int window_height)
        :
          data_ready(false),
          has_quit(false)
    {
      // Read the image which will be the texture
      vtkSmartPointer<vtkPNGReader> pngReader = vtkSmartPointer<vtkPNGReader>::New();
      pngReader->SetFileName(image_name.c_str());

      // Create a plane
      vtkSmartPointer<vtkPlaneSource> plane = vtkSmartPointer<vtkPlaneSource>::New();
      plane->SetCenter(0, 0, 0);
      plane->SetOrigin(-width / 2, -height / 2, 0.000000);
      plane->SetPoint1(width / 2, -height / 2, 0.000000);
      plane->SetPoint2(-width / 2, height / 2, 0.000000);
      plane->SetNormal(0, 0, 1);
      // Apply the texture
      vtkSmartPointer<vtkTexture> texture = vtkSmartPointer<vtkTexture>::New();
      texture->SetInput(pngReader->GetOutput());

      vtkSmartPointer<vtkTextureMapToPlane> texturePlane = vtkSmartPointer<vtkTextureMapToPlane>::New();
      texturePlane->SetInput(plane->GetOutput());

      vtkSmartPointer<vtkPolyDataMapper> planeMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
      planeMapper->SetInputConnection(texturePlane->GetOutputPort());

      vtkSmartPointer<vtkActor> texturedPlane = vtkSmartPointer<vtkActor>::New();
      texturedPlane->SetMapper(planeMapper);
      texturedPlane->SetTexture(texture);

      // Visualize the textured plane
      renderer = vtkSmartPointer<vtkRenderer>::New();
      renderer->AddActor(texturedPlane);
      renderer->SetBackground(0.2, 0.2, 0.2); // Background color
      renderer->ResetCamera();

      renderWindow = vtkSmartPointer<vtkRenderWindow>::New();
      renderWindow->SetSize(window_width, window_height);
      renderWindow->AddRenderer(renderer);

      renderWindowInteractor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
      renderWindowInteractor->SetRenderWindow(renderWindow);

      cb = vtkSmartPointer<vtkCallbackCommand>::New();
      cb->SetCallback(grab_frame_callback);
      cb->SetClientData(this);
      renderWindowInteractor->AddObserver(vtkCommand::RenderEvent, cb);

      //track ball interaction
      style = vtkSmartPointer<vtkInteractorStyleTrackballCamera>::New();
      renderWindowInteractor->SetInteractorStyle(style);
      renderer->GetActiveCamera()->SetViewAngle(45); //45 degree viewing angle.
    }

    void
    operator()()
    {
      renderWindowInteractor->Start();
      has_quit = true;
    }

    void
    put_data(const cv::Mat& image, const cv::Mat& depth, const cv::Mat& mask, const cv::Mat&K, const cv::Mat& R,
             const cv::Mat& T)
    {
      {
        boost::lock_guard<boost::mutex> lock(mtx);
        this->image = image;
        this->depth = depth;
        this->mask = mask;
        this->K = K;
        this->R = R;
        this->T = T;
        data_ready = true;
      }
      condition.notify_one();
    }
    bool
    get_data(cv::Mat& image, cv::Mat& depth, cv::Mat& mask, cv::Mat&K, cv::Mat& R, cv::Mat& T)
    {
      boost::unique_lock<boost::mutex> lock(mtx);
      while (!data_ready)
      {
        condition.timed_wait(lock, boost::posix_time::milliseconds(10));
        if (has_quit)
          break;
      }
      image = this->image;
      depth = this->depth;
      mask = this->mask;
      K = this->K;
      R = this->R;
      T = this->T;
      data_ready = false;
      return has_quit;
    }
    cv::Mat image, depth, mask, R, T, K;
    bool data_ready, has_quit;

    boost::condition_variable condition;
    boost::mutex mtx;

    // Visualize the textured plane
    vtkSmartPointer<vtkRenderer> renderer;
    vtkSmartPointer<vtkRenderWindow> renderWindow;
    vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor;

    //key callbacks
    vtkSmartPointer<vtkCallbackCommand> cb;
    //track ball interaction
    vtkSmartPointer<vtkInteractorStyleTrackballCamera> style;
  };
  void
  grab_frame_callback(vtkObject* caller, long unsigned int eventId, void* clientData, void* callData)
  {
    vtkRenderWindowInteractor *iren = static_cast<vtkRenderWindowInteractor*>(caller);
    cv::Mat image, depth, mask, R, T, K;
    grab_frame(iren->GetRenderWindow(), image, depth, mask, K, R, T);
    static_cast<SimRunner*>(clientData)->put_data(image, depth, mask, K, R, T);
  }

  void
  vtk_to_K(cv::Size sz, vtkCamera* cam, cv::Mat& K, cv::Mat& R, cv::Mat& T)
  {

    K = cv::Mat::eye(3, 3, CV_64F);
    double imgH = sz.width;
    double imgW = sz.height;
    double fovx, fovy;
    if (cam->GetUseHorizontalViewAngle())
    {
      fovx = cam->GetViewAngle();
      fovy = fovx * imgH / imgW;
    }
    else
    {
      fovy = cam->GetViewAngle();
      fovx = fovy * imgW / imgH;
    }
    double alphax = imgW / (2 * tan((fovx / 2.0) * CV_PI / 180.0)); // (imgW / 2.0) / tan(fovx / 2.0 * CV_PI / 180.0);
    double alphay = imgH / (2 * tan((fovy / 2.0) * CV_PI / 180.0)); //(imgH / 2.0) / tan(fovy / 2.0 * CV_PI / 180.0);

    K.at<double>(0, 0) = alphax;
    K.at<double>(0, 2) = (imgW - 1.0) / 2.0;
    K.at<double>(1, 1) = alphay;
    K.at<double>(1, 2) = (imgH - 1.0) / 2.0;

    double x, y, z;
    cam->GetPosition(x, y, z);
    vtkMatrix4x4* vm = cam->GetViewTransformMatrix();
    cv::Mat tR(4, 4, CV_64F, vm->Element);
    {
      cv::Mat sub(tR(cv::Range(0, 3), cv::Range(0, 3)));
      sub.copyTo(R);
    }
    {
      cv::Mat sub(tR(cv::Range(0, 3), cv::Range(3, 4)));
      sub.copyTo(T);
    }
  }

  void
  grab_frame(vtkRenderWindow* renderWindow, cv::Mat& image, cv::Mat& depth, cv::Mat& mask, cv::Mat& K, cv::Mat& R,
             cv::Mat& T)
  {
    int * ws = renderWindow->GetSize();
    cv::Size sz(ws[0], ws[1]);
    vtkCamera* camera = renderWindow->GetRenderers()->GetFirstRenderer()->GetActiveCamera();
    vtk_to_K(sz, camera, K, R, T);
    {
      unsigned char* pixel_data = renderWindow->GetRGBACharPixelData(0, 0, sz.width - 1, sz.height - 1, 1);
      cv::cvtColor(cv::Mat(sz, CV_8UC4, pixel_data), image, CV_RGBA2BGR);
      delete[] pixel_data; //need to delete this buffer
    }

    {
      double near, far;
      camera->GetClippingRange(near, far);
      float * zbuffer = renderWindow->GetZbufferData(0, 0, sz.width - 1, sz.height - 1);
      cv::Mat zt(sz, CV_32F, zbuffer);
      zt.convertTo(depth, CV_32F, far, near); //rescale , 0 is near, 1 is far
      mask = zt == 1.0;
      depth.setTo(cv::Scalar::all(0), mask); //mask off and set all far to some magic number, for now 0
      mask = mask != 0;
      delete[] zbuffer; //need to delete this buffer
    }
    cv::flip(image, image, 0); //vertical flip.
    cv::flip(depth, depth, 0); //vertical flip.
  }
}
#if DO_ECTO
namespace object_recognition
{
  using ecto::tendrils;
  struct PlanarSim
  {
    static
    void
    declare_params(tendrils& p)
    {
      p.declare<std::string>("image_name", "Image file name, should be a 3 channel png.").required(true);
      p.declare<double>("width", "width in meters.").required(true);
      p.declare<double>("height", "height in meters.").required(true);
      p.declare<int>("window_width", "Window width in pixels", 640);
      p.declare<int>("window_height", "Window height in pixels", 480);
    }

    static
    void
    declare_io(const tendrils& p, tendrils& i, tendrils& o)
    {
      o.declare<cv::Mat>("image", "Image generated");
      o.declare<cv::Mat>("depth", "Depth generated");
      o.declare<cv::Mat>("mask", "Valid depth points");

      o.declare<cv::Mat>("R", "Rotation matrix, double, 3x3");
      o.declare<cv::Mat>("T", "Translation matrix, double, 3x1");
      o.declare<cv::Mat>("K", "Camera Intrisics matrix, double, 3x3");
    }

    void
    configure(const tendrils& p, const tendrils& i, const tendrils& o)
    {
      double width, height;
      int window_width, window_height;
      std::string image_name;
      p["image_name"] >> image_name;
      p["window_width"] >> window_width;
      p["window_height"] >> window_height;
      p["width"] >> width;
      p["height"] >> height;
      sm.reset(new SimRunner(image_name, width, height, window_width, window_height));
      t.reset(new boost::thread(boost::ref(*sm)));
      image = o["image"];
      depth = o["depth"];
      mask = o["mask"];

      R = o["R"];
      K = o["K"];
      T = o["T"];
    }

    int
    process(const tendrils& i, const tendrils& o)
    {
      if (sm->get_data(*image, *depth, *mask, *K, *R, *T))
      {
        t->join();
        return ecto::QUIT;
      }
      return ecto::OK;
    }
    ~PlanarSim()
    {
      if (sm)
      {
        sm->renderWindowInteractor->TerminateApp();
      }
      if (t)
      {
        t->interrupt();
        t->join();
      }
    }
    boost::shared_ptr<SimRunner> sm;
    boost::shared_ptr<boost::thread> t;
    ecto::spore<cv::Mat> image, depth, mask, R, T, K;
  };
}

ECTO_CELL(sim, object_recognition::PlanarSim, "PlanarSim", "Simulates a view of a planar object.");
#else
int
main(int argc, char *argv[])
{
  using namespace object_recognition;
  // Parse command line arguments
  if (argc != 2)
  {
    std::cerr << "Usage: " << argv[0] << " Filename.png" << std::endl;
    return EXIT_FAILURE;
  }

  std::string inputFilename = argv[1];

  boost::shared_ptr<SimRunner> sm(new SimRunner(inputFilename, 0.4, 0.8, 640, 480));
  boost::thread t(boost::ref(*sm));
  cv::Mat image, depth, mask, R, T, K;
  while (!sm->get_data(image, depth, mask, K, R, T))
  {
    cv::imshow("Render", image);
    cv::waitKey(10);
    std::cout << "K = " << K << "\nR = " << R << "\nT = " << T << std::endl;

    boost::this_thread::sleep(boost::posix_time::milliseconds(10));
  }
  t.join();
  std::cout << "exiting" << std::endl;
  return EXIT_SUCCESS;
}
#endif
