#include <ecto/ecto.hpp>
#include <ecto/spore.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <vector>
#include <deque>
#include <numeric> //for inner_product
#include <functional> //for inner product
#include <opencv2/flann/flann.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/serialization/vector.hpp>
#include <fstream>


using ecto::tendrils;
using namespace cv;
using namespace std;

namespace boost {
namespace serialization {

/* BOILER_PLATE_MODULE
 struct MyModule
 {
 static void declare_params(tendrils& params);
 static void declare_io(const tendrils& params, tendrils& in, tendrils& out);
 void configure(tendrils& params, tendrils& in, tendrils& out);
 int process(const tendrils& in, tendrils& out);
 };
 */
namespace line_mod
{
  /////////////////////////////////////////////////////////////////////////
   /**
    * Compute the color order image:
    *
    * computeColorOrder which maps color ratios into 2 bits where
    * 0 => ratio is lower than 0.75, 1=> ratio is (0.75, 1], 2=> ratio is (1, 1.25], 3=> ratio is > 1.25 for:
    * B/G (bits 0,1),
    * R/B (bits 2,3)
    * G/R (bits 4,5)
    * Black (R,G,B all <25% of max), Whte (R,G,B all > 75% of max values) bits 6,7
    * If leading bits, 0xC0, are set => pixel is invalid (masked etc)
    * 0 if black, 2 if white, 1 otherwise.
    */
   struct ColorMod
   {
     /**
      * void computeColorOrder(const cv::Mat &Iina, cv::Mat &Icolorord, const cv::Mat &Mask)
      *
      * Code a color input image into a single channel (one byte) coded color image using an optional mask
      *
      * @param Iina        Input BGR image of uchars
      * @param Icolorord   Return coded image here
      * @param Mask        (optional) skip masked pixels (0's) if Mask is supplied.
      */
     void computeColorOrder(const cv::Mat &Iina, cv::Mat &Icolorord, cv::Mat Mask)
     {
       //    Mat Iin;
       //    resize(Iin,Iin,Size(Iin.cols/8,Iin.rows/8));
       Mat Iin;
       GaussianBlur(Iina, Iin, Size(gsize, gsize), gsig, gsig);

       //    Mat Ip2,Ip4,Iin;
       //    pyrDown(Iina,Ip2);
       //    pyrDown(Ip2,Ip4);
       //    pyrDown(Ip4,Iin);
       if (Iin.size() != Icolorord.size() || Icolorord.type() != CV_8UC1) //Make sure Icolorord is the right size
         {
           Icolorord.create(Iin.size(), CV_8UC1);
         }
       if(!Mask.empty()) //We have a mask
         {
           if(Iina.size() != Mask.size())
             {
               throw std::runtime_error("ERROR: Mask in computeColorOrder size != Iina");
             }
           if (Mask.type() == CV_8UC3)
             {
               //don't write into the Mask, as its supposed to be const.
               cv::Mat temp;
               cv::cvtColor(Mask,temp,CV_RGB2GRAY);
               Mask = temp;
             }
           if (Mask.type() != CV_8UC1)
             {
               throw std::runtime_error("ERROR: Mask is not of type CV_8UC1 in computeColorOrder");
             }
         }
       //float Bu = 0, Bs = 0, Gu = 0, Gs = 0, Ru = 0, Rs = 0, BM = 0, Bm = 1000, GM = 0, Gm = 1000, RM = 0, Rm = 1000;
       float cratio;
       uchar oresult;
       //   int foo = 1;
       if(Mask.empty()) //No mask
         {
         for (int y = 0; y < Iin.rows; y++)
           {
             const uchar *b = Iin.ptr<uchar> (y);
             const uchar *g = b + 1;
             const uchar *r = b + 2;
             uchar *o = Icolorord.ptr<uchar> (y);
             for (int x = 0; x < Iin.cols; x++, b += 3, g += 3, r += 3, o++)
               {
                 /**
                  * 2 bits each: 0=><64, 1=><128, 2=><192 3 >= 192
                  * B/G R/B G/R
                  */
                 float B = float(unsigned(*b)) + 1.0, R = float(unsigned(*r))
                     + 1.0, G = float(unsigned(*g)) + 1.0; //protect from 0

                 oresult = 0;
                 // B/G
                 cratio = B / G;
                 //        Bu += cratio;
                 //        Bs += cratio*cratio;
                 //        if(Bm > cratio) Bm = cratio;
                 //        if(BM < cratio) BM = cratio;
                 if (cratio > 1.25)
                   {
                     oresult |= 3;
                   }
                 else if (cratio > 1.0)
                   oresult |= 2;
                 else if (cratio > 0.75)
                   oresult |= 1;
                 // R/B
                 cratio = R / B;
                 //        Ru += cratio;
                 //        Rs += cratio*cratio;
                 //        if(Rm > cratio) Rm = cratio;
                 //        if(RM < cratio) RM = cratio;
                 if (cratio > 1.25)
                   {
                     oresult |= 12;
                   }
                 else if (cratio > 1.0)
                   oresult |= 8;
                 else if (cratio > 0.75)
                   oresult |= 4;
                 // G/R
                 cratio = G / R;
                 //        Gu += cratio;
                 //        Gs += cratio*cratio;
                 //        if(Gm > cratio) Gm = cratio;
                 //        if(GM < cratio) GM = cratio;
                 if (cratio > 1.25)
                   {
                     oresult |= 48;
                   }
                 else if (cratio > 1.0)
                   oresult |= 32;
                 else if (cratio > 0.75)
                   oresult |= 16;
                 //Determine black/white status
                 if ((R > 192) && (G > 192) && (B > 192))
                   oresult |= 128; //10______ means white
                 else if ((64 > R) && (64 > G) && (64 > B))
                   ; //00______ means dark
                 else
                   oresult |= 64; //01______ means in between,
                 *o = oresult;
               }
           }
         }
       else //Use mask
         {
           for (int y = 0; y < Iin.rows; y++)
              {
                const uchar *b = Iin.ptr<uchar> (y);
                const uchar *g = b + 1;
                const uchar *r = b + 2;
                const uchar *m = Mask.ptr<uchar> (y);
                uchar *o = Icolorord.ptr<uchar> (y);
                for (int x = 0; x < Iin.cols; x++, b += 3, g += 3, r += 3, o++)
                  {
                    if(!(*m))
                      {
                        *o = 0xC0; //This combination indicates that the bit is invalid
                        continue;
                      }
                    /**
                     * 2 bits each: 0=><64, 1=><128, 2=><192 3 >= 192
                     * B/G R/B G/R
                     */
                    float B = float(unsigned(*b)) + 1.0, R = float(unsigned(*r))
                        + 1.0, G = float(unsigned(*g)) + 1.0; //protect from 0

                    oresult = 0;
                    // B/G
                    cratio = B / G;
                    //        Bu += cratio;
                    //        Bs += cratio*cratio;
                    //        if(Bm > cratio) Bm = cratio;
                    //        if(BM < cratio) BM = cratio;
                    if (cratio > 1.25)
                      {
                        oresult |= 3;
                      }
                    else if (cratio > 1.0)
                      oresult |= 2;
                    else if (cratio > 0.75)
                      oresult |= 1;
                    // R/B
                    cratio = R / B;
                    //        Ru += cratio;
                    //        Rs += cratio*cratio;
                    //        if(Rm > cratio) Rm = cratio;
                    //        if(RM < cratio) RM = cratio;
                    if (cratio > 1.25)
                      {
                        oresult |= 12;
                      }
                    else if (cratio > 1.0)
                      oresult |= 8;
                    else if (cratio > 0.75)
                      oresult |= 4;
                    // G/R
                    cratio = G / R;
                    //        Gu += cratio;
                    //        Gs += cratio*cratio;
                    //        if(Gm > cratio) Gm = cratio;
                    //        if(GM < cratio) GM = cratio;
                    if (cratio > 1.25)
                      {
                        oresult |= 48;
                      }
                    else if (cratio > 1.0)
                      oresult |= 32;
                    else if (cratio > 0.75)
                      oresult |= 16;
                    //Determine black/white status
                    if ((R > 192) && (G > 192) && (B > 192))
                      oresult |= 128; //10______ means white
                    else if ((64 > R) && (64 > G) && (64 > B))
                      ; //00______ means dark
                    else
                      oresult |= 64; //01______ means in between,
                    *o = oresult;
                  }
              }
         } //end else use mask
     }

     static void declare_params(tendrils& p)
     {
       p.declare<int> ("gsize", "Size of NxN Gaussian blur template.", 5);
       p.declare<double> ("gsig", "Sigma of Gaussian blur template", 2.0);
     }

     static void declare_io(const tendrils& params, tendrils& inputs,
                            tendrils& outputs)
     {
       inputs.declare<cv::Mat> ("image", "An input image. RGB");
       inputs.declare<cv::Mat> ("mask", "An input mask. Default is Mat(), meaning the mask is optional.");

       outputs.declare<cv::Mat> ("output", "A binarized color image.");
     }

     void configure(const tendrils& params, const tendrils& in, const tendrils& out)
     {
       gsize = params.get<int> ("gsize");
       gsig = params.get<double> ("gsig");
     }

     int process(const tendrils& inputs, const tendrils& outputs)
     {
       cv::Mat image = inputs.get<cv::Mat> ("image");
       cv::Mat mask = inputs.get<cv::Mat> ("mask");
       cv::Mat& colorOrd = outputs.get<cv::Mat> ("output");
       computeColorOrder(image, colorOrd, mask);
       return 0;
     }

     int gsize; //size of NxN Gaussian blur template
     double gsig; //Size of Gaussian blur Sigma

   };

  /////////////////////////////////////////////////////////////
  /**
   * Color Template which computes a response to an image
   */
//  static int response_table[4][4] =
//    {
//       { 3, 2, 1, 0 },
//       { 2, 3, 2, 1 },
//       { 1, 2, 3, 2 },
//       { 0, 1, 2, 3 } };
//  struct ColorTempl
//  {
//    //Storage
//    std::vector<Point> offsets; //The template starts from the upper left corner of the bounding box of its mask
//    std::vector<uchar> coded_color;
//    Size template_size;
//    /**
//     * double response(const Mat &image, const Point &offset);
//     * @param image
//     * @param offset
//     * @return 1.0 = perfect match, 0 not.
//     */
//    double response(const Mat image, const Point offset)
//    {
//      if (image.type() != CV_8UC1)
//        {
//          throw std::runtime_error(
//                                   "ERROR: Image is not of type CV_8UC1 in ColorTempl");
//        }
//      if (image.channels() != 1)
//        {
//          throw std::runtime_error(
//                                   "ERROR, image in ColorTempl is not one channel");
//        }
//      int len = (int) offsets.size();
//      int step = image.step;
//      const uchar *base = image.ptr<uchar> (offset.y);
//      base += offset.x;
//      int score = 0,masked = 0;
//      int t, m; //test and model
//      for (int i = 0; i < len; ++i)
//        {
//          uchar test = *(base + (step * offsets[i].y) + offsets[i].x);
//          if(test & 0xC0) //masked
//            {
//              masked++;
//              continue;
//            }
//          //B/G
//          t = (int) (test & 0x3);
//          m = (int) (coded_color[i] & 0x3);
//          score += response_table[t][m];
//          //R/B
//          t = (int) ((test >> 2) & 0x3);
//          m = (int) ((coded_color[i] >> 2) & 0x3);
//          score += response_table[t][m];
//          //G/R
//          t = (int) ((test >> 4) & 0x3);
//          m = (int) ((coded_color[i] >> 4) & 0x3);
//          score += response_table[t][m];
//          //Black and white
//          t = (int) ((test >> 6) & 0x3);
//          m = (int) ((coded_color[i] >> 6) & 0x3);
//          score += response_table[t][m] - 1; // minus one because black and white can differ only by 2 not 3
//        }
//      return (double) score / (double) ((len-masked) * 11);
//    }
//
//    void clear()
//    {
//      offsets.clear();
//      coded_color.clear();
//      template_size = Size(0, 0);
//    }
//
//  };

   /**
    * Below defines the histogram regions per template.
    *
    * The start[x,y] and the end[x,y] each row denotes a different type of sub histogram pattern for the object
    * There can be up to 16 histogram regions, but -1 => invalid if there are less than 16.
    * Histogram "type" indexes start and stop histogram regions these, so
    * type 0 => invalid
    * type 1 => 4 overlapping histograms on a 3x3 grid
    * type 2 => 5 slightly overlapping left to right bands from top to bottom
    */
   static int max_hist_types = 2;
static float startx [3][16] =
    {
     { -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},
       { 0.0, 0.3333, 0.0, 0.3333,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1 },
       { 0.0,0.0,0.0,0.0,0.0,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1 }
    };
static float endx [3][16] =
    {
     { -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},
       { 0.6666, 1.0, 0.6666,1.0,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1 },
       { 1,1,1,1,1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1 }
    };

static float starty [3][16] =
    {
     { -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},
       { 0.0, 0.0, 0.3333,0.3333,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1 },
       { 0.0,0.18,0.38,0.58,0.78,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1 }
    };
static float endy [3][16] =
    {
     { -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},
       { 0.6666,0.6666,1,1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1 },
       { 0.22,0.42,0.62,0.82,1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1 }
    };
  struct ColorTempl
  {
    //Storage
    int type;  //0=>not allocated; 1 => 4 overlapping segments on a 3x3 grid, 2 => 5 horiz. bands
    std::vector<vector<float> > hists; //Histograms (each of size 16), L2 normed
    std::vector<float> weights;  //This weights the histograms according to their area in the mask. Sum to 1
    Size template_size;

    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        ar & type;
        ar & template_size.width;
        ar & template_size.height;
        ar & hists;
        ar & weights;
    }


    ColorTempl(): type(-1){};
    ColorTempl(int t) { type = t; template_size = Size(0,0);};
    ColorTempl(int t, Size s) { type = t; template_size = s;};
    ColorTempl(int t, Size s, int len)
    {
      type = t; template_size = s; hists.reserve(len); weights.reserve(len);
    };

    /**
     * int calc_hist(const Mat &colorOrd, const Mat &Mask, Point start,  Point stop, std::vector<float> hist, int skipx = 1, int skipy = 1 )
     *
     * Calculate the color ratio histogram of colorOrd in the region start to stop. All values must be in range!
     *
     * @param colorOrd Coded color image output from ColorMod
     * @param Mask    (optional) If mask, use it
     * @param start   Start point of histogram region
     * @param stop    Stop point of histogram region
     * @param hist    Histogram to be filled out
     * @param skipx, skipy Number of points to skip when computing hist. DEFAULT is 1 (no skip)
     * @return Number of points in the histogram
     */
    int calc_hist(const Mat &colorOrd, const Mat &Mask, Point start,  Point stop, std::vector<float> &hist,  int skipx = 1, int skipy = 1)
    {
      if(0 >= skipx) skipx = 1; if(0 >= skipy) skipy = 1;
      if((int)hist.size() != 16)
        {
          hist.resize(16,0.0);
        }
      else
        fill(hist.begin(),hist.end(),0.0);
      int num_pts = 0;
      if(!Mask.empty()) //We have a mask
        {
          for (int y = start.y; y <= stop.y; y+=skipy)
            {
              const uchar *o = colorOrd.ptr<uchar> (y);
              const uchar *m = Mask.ptr<uchar> (y);
              m += (int)start.x;
              o += (int)start.x;
              for (int x = start.x; x <= stop.x; x+=skipx, o+=skipx, m+=skipx)
                {
                  if(!(*m)) continue; //Ignore masked pixels
                  if(!((*o)^0xC0)) //Invalid color, don't count
                    {
                      continue;
                    }
                  num_pts++; //count this point
                  uchar c = *o; //Put it into the histogram
                  hist[c&0x3] += 1.0; //B/G
                  hist[((c>>2)&0x3)+4] += 1.0; //R/B
                  hist[((c>>4)&0x3)+8] += 1.0; //G/R
                  hist[(c>>6)+12] += 1.0; //B/W
                }
            }
        }
      else //Not masked
        {
          for (int y = start.y; y <= stop.y; y+=skipy)
            {
              const uchar *o = colorOrd.ptr<uchar> (y);
              o += start.x;
              for (int x = start.x; x <= stop.x; x+=skipx, o+=skipx)
                {
                  if(!((*o)^0xC0))
                    {
                      continue; //Invalid color, don't count
                    }
                  num_pts++; //count this point
                  uchar c = *o; //Put it into the histogram
                  hist[c&0x3] += 1.0; //B/G
                  hist[((c>>2)&0x3)+4] += 1.0; //R/B
                  hist[((c>>4)&0x3)+8] += 1.0; //G/R
                  hist[(c>>6)+12] += 1.0; //B/W
                }
            }
        }
      //L2 norm
      float L2 = 0.0;
      std::vector<float>::iterator it;
      for(it = hist.begin(); it != hist.end(); ++it)
        L2 += (*it)*(*it);
//      cout << "L2_pre = " << L2;
      L2 = (float)sqrt(L2);
//      cout << ", L2_post = " << L2 << ". num_pts " << num_pts << "\nL2:\n" << endl;
      if(num_pts == 0.0) L2 = 1.0;
//      float ssum = 0.0;
      int i = 0;
      for(it = hist.begin(); it != hist.end(); ++i, ++it)
        {
        *it /= L2; //Normalize it
//        ssum += (*it)*(*it);
//        cout << "[" << i << "] " << *it << ", ";
        }
//      cout << "L2 squared sum = " << ssum << endl;
      //get out
      return (num_pts);
    }

    /**
     * float match(const Mat colorOrd, const Point offset, const Mat &Mask, int skipx = 1, int skipy = 1)
     *
     * @brief Match a patch of an image using the classe's template starting at point "offset"
     *
     * @param offset    Point to start match at
     * @param Mask      If mask not empty, use it
     * @param skipx, skipy Number of points to skip when computing hist. DEFAULT is 1 (no skip)
     * @return 1.0 = perfect match, 0 = total miss.
     */
    float match(const Mat &colorOrd, const Point &offset, const Mat &Mask, int skipx = 1, int skipy = 1)
    {
      if(0 >= skipx) skipx = 1; if(0 >= skipy) skipy = 1;
      if ((type < 1)||(type > max_hist_types))
          {
            throw std::runtime_error("ERROR: bad hist type ColorTempl::match");
          }
      if (colorOrd.channels() != 1)
        {
          throw std::runtime_error("ERROR: channels != 1 in ColorTempl::match");
        }
      if (colorOrd.type() != CV_8UC1)
        {
          throw std::runtime_error("ERROR: colorOrd is not of type CV_8UC1 in ColorTempl::match");
        }
      Point start, stop;
      //Verify/set bounds, keep them in range
      int rows = colorOrd.rows, cols = colorOrd.cols;
      Point start_pt = offset;
      if((start_pt.x < 0)||(start_pt.x >= cols)) start_pt.x = 0;
      if((start_pt.y < 0)||(start_pt.y >= rows)) start_pt.y = 0;
      Point stop_pt = Point(start_pt.x + template_size.width,start_pt.y + template_size.height);
      if(stop_pt.x >= cols) stop_pt.x = cols - 1;
      if(stop_pt.y >= rows) stop_pt.y = rows - 1;
      float w = (float)(stop_pt.x - start_pt.x); //Adjusted width and height of our hist roi
      float h = (float)(stop_pt.y - start_pt.y);
      //Calculate histogram in roi of colorOrd

      int hlen = 0;
      for(;startx[type][hlen] >= 0.0;hlen++); //find the number of sub-histograms that compose this object
      ColorTempl ct(type,Size(w,h),hlen);  //Instantiate a color template
      vector<float> test_hist;  //This will store one of "hlen" sub-histograms
      for(int i = 0; i<hlen; ++i)
        {
          Point hstart = Point(start_pt.x + (int)(w*startx[type][i] + 0.5),
                               (start_pt.y + (int)(h*starty[type][i]+ 0.5)));
          Point hend = Point(start_pt.x + (int)(w*endx[type][i] + 0.5),
                             (start_pt.y + (int)(h*endy[type][i]+ 0.5)));
          calc_hist(colorOrd, Mask,  hstart, hend, test_hist, skipx, skipy);
          ct.hists.push_back(test_hist);
        }
      //Score the collected histograms against our internal template
      return( match_templates(ct));
    }

    /**
     * void learn(const Mat &colorOrd, const Mat &Mask, Point start_pt = Point(0,0), Point stop_pt = Point(0,0),  int skipx = 1, int skipy = 1)
     *
     * Learn (fill) this object's hists, weights, template_size according to type.
     * You can use a Mask or an ROI or both but at least one of Mask or ROI must be set.
     * This method will clear out any old values in the ColorTempl and fill in new ones.
     *
     * @param colorOrd  Coded color image output from ColorMod
     * @param Mask      If mask not empty, use it
     * @param start_pt, stop_pt If these are set, use them, else calculate from the mask
     * @param skipx, skipy Number of points to skip when computing hist. DEFAULT is 1 (no skip)
     */
    void learn(const Mat &colorOrd, const Mat &Mask,
               Point start_pt = Point(0,0), Point stop_pt = Point(0,0),
               int skipx = 1, int skipy = 1)
    {
      cout << "hists.size = " << hists.size() << " weights.size = " << weights.size() << " type = " << type << endl;
      //INPUT CHECKS
      if(0 >= skipx) skipx = 1; if(0 >= skipy) skipy = 1;
      if ((type < 1)||(type > max_hist_types))
          {
            throw std::runtime_error("ERROR: bad hist type ColorTempl::learn");
          }
      if (colorOrd.channels() != 1)
        {
          throw std::runtime_error("ERROR: colorOrd channels != 1 in ColorTempl::learn");
        }
      if (colorOrd.type() != CV_8UC1)
        {
          throw std::runtime_error("ERROR: colorOrd is not of type CV_8UC1 in ColorTempl::learn");
        }
      if (!Mask.empty())
        {
          if(colorOrd.size() != Mask.size())
            {
              throw std::runtime_error("ERROR colorOrd and Mask were not the same size in ColorTempl::learn");
            }
          if(Mask.channels() != 1)
            {
              throw std::runtime_error("ERROR: Mask channels != 1 in ColorTempl::learn");
            }
        }
     //COMPUTE ROI FROM MASK IF NOT GIVEN
      if(0 == stop_pt.y) //Need to compute roi from mask
        {
          if(Mask.empty())
            throw std::runtime_error("ERROR: neither Mask nore start_pt, stop_pt are set in ColorTempl::learn");
          //Find the bounding box of the template
        int cY = -1, cYend = 0, minx = Mask.cols, maxx = 0;
          for (int y = 0; y < Mask.rows; y++)
            {
              const uchar *m = Mask.ptr<uchar> (y);
              for (int x = 0; x < Mask.cols; x++, m++)
                {
                  if (*m)
                    {
                      if (cY < 0)
                        cY = y; //Mark the upper left corner, we'll need it for offsets below
                      cYend = y;
                      if (minx > x)
                        minx = x;
                      if(maxx < x) maxx = x;
                      for(;x < Mask.cols; x++, m++)
                        {
                          if(*m)
                            {
                              if(maxx < x)
                                maxx = x;
                            }
                        }
                      break;
                    }
                }
            }
          start_pt = Point(minx,cY);
          stop_pt = Point(maxx,cYend);
        }
      else //ADJUST ROI TO BE INBOUNDS
        {
          Point start, stop;
          //Verify/set bounds, keep them in range
          int rows = colorOrd.rows, cols = colorOrd.cols;
          if((start_pt.x < 0)||(start_pt.x >= cols)) start_pt.x = 0;
          if((start_pt.y < 0)||(start_pt.y >= rows)) start_pt.y = 0;
          if(stop_pt.x >= cols) stop_pt.x = cols - 1;
          if(stop_pt.y >= rows) stop_pt.y = rows - 1;
        }
      //COMPUTE HISTOGRAMS
      float w = (float)(stop_pt.x - start_pt.x); //Adjusted width and height of our hist roi
      float h = (float)(stop_pt.y - start_pt.y);
      int hlen = 0;
      for(;startx[type][hlen] >= 0.0;hlen++); //find the number of sub-histograms that compose this object

      cout << "w= " << w << " h= " << h << "start_pt(" << start_pt.x << ", " << start_pt.y << "), stop_pt(" << stop_pt.x << ", " << stop_pt.y << "), hlen = " << hlen << endl;



      clear(type);  //Reset all values except for type
      hists.reserve(hlen); //Make room for hlen hists
      template_size = Size((int)w, (int)h);
      vector<float> test_hist;  //This will store one of "hlen" sub-histograms
      vector<int> num_pts;
      float N,Ntot = 0;

      for(int i = 0; i<hlen; ++i)
        {
         Point hstart = Point(start_pt.x + (int)(w*startx[type][i] + 0.5),
                               (start_pt.y + (int)(h*starty[type][i]+ 0.5)));
          Point hend = Point(start_pt.x + (int)(w*endx[type][i] + 0.5),
                             (start_pt.y + (int)(h*endy[type][i]+ 0.5)));
         N = (float)calc_hist(colorOrd, Mask, hstart, hend, test_hist, skipx, skipy);
          Ntot += N;
          weights.push_back(N);
          hists.push_back(test_hist);
        }

      if(Ntot == 0.0) Ntot = 1.0;
      else Ntot = 1.0/Ntot;
      std::transform(weights.begin(), weights.end(), weights.begin(),
                     std::bind1st(std::multiplies<float>(),Ntot)); //normalize weights
    }


    /**
     * float match_templates(const ColorTempl &ct, int equal_wtd = 0)
     *
     * @brief match a template against an existing template. Use this to see if we should accept it for learning
     *
     * @param ct  Template to be matched against this one
     * @param equal_wtd  If set, average the ct weight with this template's weight. Default: 0
     * @return matching score.  1.0 is perfect, 0 is total miss
     */
    float match_templates(const ColorTempl &ct, int equal_wtd = 0 )
    {
      if(ct.type != type)
        {
          throw std::runtime_error("ERROR: In ColorTempl, ct.type != type");
        }
      int len = (int)hists.size();
      float score = 0.0;
      vector<float>::iterator it;
      vector<float>::const_iterator cit;

      if(equal_wtd) //Do equal weighting of ct and this template (for template to template compare)
        {
        for(int i = 0; i<len; ++i)
          {
            float tmpdot = 0.0;
            for(it=hists[i].begin(), cit = ct.hists[i].begin(); it < hists[i].end(); ++it,++cit)
              {
//                  cout << "("<< *it << ")*(" << *cit << ") = " << (*it)*(*cit) ;
                tmpdot += (*it)*(*cit);
//                  cout << " accum: " << tmpdot;
              }
            score += (weights[i]+ct.weights[i]*0.5)*tmpdot;
          }
        }
      else //Use this template's weights (for comparing a new image patch to this template)
        {
          cout << "in match_templates, 'else' statement. Len = " << len << endl;
//          float tmptotal = 0.0;
          for(int i = 0; i<len; ++i)
            {
              float tmpdot = 0.0;
//              cout << "weights[" << i << "] = " << weights[i] << ". Then hists: " << endl;
              for(it=hists[i].begin(), cit = ct.hists[i].begin(); it < hists[i].end(); ++it,++cit)
                {
//                  cout << "("<< *it << ")*(" << *cit << ") = " << (*it)*(*cit) ;
                  tmpdot += (*it)*(*cit);
//                  cout << " accum: " << tmpdot;
                }
              score += weights[i]*tmpdot;
              cout << "score(" << score << ") += weights[" << i << "](" << weights[i] << ")*tmpdot(" << tmpdot << ") w[i]*tmpdot == " <<  weights[i]*tmpdot << endl;
            }
        }
      return score;
    }

    void clear(int type_set = 0)
    {
      hists.clear();
      weights.clear();
      template_size = Size(0,0);
      type = type_set;
    }

  };
  /////////////////////////////////////////////////////////////////////////////
  /**
   * ColorTemplCalc
   *
   * Calculate a color template
   * Note that the template starts from the upper left corner of the bounding box of the mask
   */
  struct ColorTemplCalc
  {
    vector<ColorTempl> dbctmatch;
    /**
     * void learn_a_template(const Mat &colorOrd, ColorTempl &ct,const Mat &Mask, Point start_pt = Point(0,0), Point stop_pt = Point(0,0))
     *
     * Learn (fill) ColorTempl's hists, weights, template_size according to type.
     * You can use a Mask or an ROI or both, but at least one of Mask or ROI must be set
     * The ColorTempl ct will be cleared and reset according to type
     * skipx and skipy will set the sampling of colors.
     *
     * @param colorOrd  Image from computeColorOrder
     * @param ct         The return template. It will take hist_type from this class's settings
     * @param Mask       This should be the mask you want to collect, but you can use an ROI instead.
     * @param start_pt, stop_pt If these are set, use them, else calculate them from the mask
     */
    void learn_a_template(const Mat &colorOrd, ColorTempl &ct,const Mat &Mask,
                          Point start_pt = Point(0,0), Point stop_pt = Point(0,0))
    {
//      ct.type = hist_type;
      ct.clear(hist_type);
      ct.learn(colorOrd, Mask, start_pt, stop_pt, skipx, skipy);
      //db
//      ofstream os("colortempl.txt");
//      boost::archive::text_oarchive oar(os);
//      oar << ct;
//      int len = (int)dbctmatch.size();
//      if(0 == len)
//        {
//          float scoreit = ct.match_templates(ct);
//          cout << "Match score with itself = " << scoreit << endl;
//        }
//      else
//        {
//          float scoreit = dbctmatch[len-1].match_templates(ct);
//          cout << "match of template[" << len-1 << "] against template[" << len << "] = " << scoreit << endl;
//
//        }
//      dbctmatch.push_back(ct);
    }

    //Virtual functs
    static void declare_params(tendrils& p)
    {
      p.declare<int> (
                      "skipx",
                      "Skip every skipx point in the x direction when collecting a template.",
                      5);
      p.declare<int> (
                      "skipy",
                      "Skip every skipy point in the y direction when collecting a template.t",
                      5);
      p.declare<int> (
                      "hist_type",
                      "Histogram type. 0=>invalid, 1=>4 overlapping regions on a 3x3 grid",
                      1);
    }

    static void declare_io(const tendrils& params, tendrils& inputs,
                           tendrils& outputs)
    {
      inputs.declare<cv::Mat> ("colorord",
                               "A color coded template from the ColorMod module");
      inputs.declare<cv::Mat> ("mask", "A mask of the object, optional");
      inputs.declare<cv::Point> ("start_pt", "Optional upper left corner of an ROI", cv::Point(0,0));
      inputs.declare<cv::Point> ("stop_pt", "Optional lower right corner of an ROI", cv::Point(0,0));
      outputs.declare<ColorTempl> ("colortempl","A color template of type ColorTempl.");
    }

    void configure(const tendrils& params, const tendrils& in, const tendrils& out)
    {
      skipx = params.get<int> ("skipx");
      skipy = params.get<int> ("skipy");
      hist_type = params.get<int> ("hist_type");
    }

    int process(const tendrils& inputs, const tendrils& outputs)
    {
//      std::cout << __PRETTY_FUNCTION__ << std::endl;
      const Mat &colorord = inputs.get<cv::Mat> ("colorord");
      const Mat &mask = inputs.get<cv::Mat> ("mask");
      Point start_pt = inputs.get<cv::Point> ("start_pt");
      Point stop_pt = inputs.get<cv::Point> ("stop_pt");
      ColorTempl &ct = outputs.get<ColorTempl> ("colortempl");
      learn_a_template(colorord, ct, mask, start_pt, stop_pt);
      return 0;
    }
    //settable
    int skipx, skipy; //How many pixels to skip in the X and Y directions when collecting a template
    int hist_type; //For right now: 0=> invalid, 1=> 4 overlapping regions on a 3x3 grid

  };
//  struct ColorTemplCalc
//  {
//    /**
//     * learn_a_template(const Mat &Icolorord, const Mat &Mask, ColorTempl &ct)
//     * @param Icolorord  Image from computeColorOrder
//     * @param Mask       This should be the mask you want ot collect, it should not conflict
//     *                   with prior masking of Icolorord.
//     * @param ct         The return template
//     */
//    void learn_a_template(const Mat &Icolorord, const Mat &Mask, ColorTempl &ct)
//    {
//      ct.clear();
//      if (Icolorord.size() != Mask.size())
//        {
//          throw std::runtime_error(
//                                   "ERROR Icolorord and Mask were not the same size in ColorTemplCalc");
//          return;
//        }
//
//      if ((Icolorord.channels() != Mask.channels()) && (Mask.channels() != 1))
//        {
//          throw std::runtime_error(
//                                   "ERROR: channels missmatch or != 1 in ColorTemplCalc");
//          return;
//        }
//      if ((Icolorord.type() != CV_8UC1) || (Mask.type() != CV_8UC1))
//        {
//          throw std::runtime_error(
//                                   "ERROR: Icolorord and/or Mask is not of type CV_8UC1 in ColorTemplCalc");
//          return;
//        }
//      //Find upperleft corner of mask
//      int cY = -1, cYend, minx = Mask.cols;
//      vector<int> cX;
//      for (int y = 0; y < Mask.rows; y++)
//        {
//          const uchar *m = Mask.ptr<uchar> (y);
//          for (int x = 0; x < Mask.cols; x++, m++)
//            {
//              if (*m)
//                {
//                  if (cY < 0)
//                    cY = y; //Mark the upper left corner, we'll need it for offsets below
//                  cYend = y;
//                  cX.push_back(x);
//                  if (minx > x)
//                    minx = x;
//                  break;
//                }
//            }
//        }
//
//      //Fill the template
//      int Maxx = minx;
//      int Maxy = 0;
//      for (int y = cY; y < cYend; y += skipy)
//        {
//          const uchar *m = Mask.ptr<uchar> (y);
//          const uchar *o = Icolorord.ptr<uchar> (y);
//          m += (int)(cX[y - cY]);
//          o += (int)(cX[y - cY]);
//          for (int x = cX[y - cY], m+=x; x < Icolorord.cols; x += skipx, m += skipx, o
//              += skipx)
//            {
//              if (*m)
//                {
//                  ct.coded_color.push_back(*o);
//                  ct.offsets.push_back(Point(x - minx, y - cY));
//                  if (Maxx < x)
//                    Maxx = x;
//                  if (Maxy < y)
//                    Maxy = y;
//                }
//            }
//        }
//      ct.template_size = Size(Maxx - minx, Maxy - cY); //bounding box of the actual touched points in the mask
//    }
//
//    //Virtual functs
//    static void declare_params(tendrils& p)
//    {
//      p.declare<int> (
//                      "skipx",
//                      "Skip every skipx point in the x direction when collecting a template.",
//                      5);
//      p.declare<int> (
//                      "skipy",
//                      "Skip every skipy point in the y direction when collecting a template.t",
//                      5);
//    }
//
//    static void declare_io(const tendrils& params, tendrils& inputs,
//                           tendrils& outputs)
//    {
//      inputs.declare<cv::Mat> ("image",
//                               "A color coded template from the ColorMod module");
//      inputs.declare<cv::Mat> ("mask", "A mask of the object");
//      outputs.declare<ColorTempl> ("output",
//                                   "A color template of type ColorTempl.");
//    }
//
//    void configure(tendrils& params, tendrils& in, tendrils& out)
//    {
//      skipx = params.get<int> ("skipx");
//      skipy = params.get<int> ("skipy");
//    }
//
//    int process(const tendrils& inputs,const tendrils& outputs)
//    {
//      const Mat &image = inputs.get<cv::Mat> ("image");
//      const Mat &mask = inputs.get<cv::Mat> ("mask");
//      ColorTempl &output = outputs.get<ColorTempl> ("output");
//      learn_a_template(image, mask, output);
//      return 0;
//    }
//    //settable
//    int skipx, skipy; //How many pixels to skip in the X and Y directions when collecting a template
//
//
//  };

  /**
   * colortree
   *
   * Class for growing a ColorTempl matching tree.
   * ToDo: Add serialization
   */
 struct colortree
 {
    vector<ColorTempl>  colortempls;
    vector<colortree>   colortrees;

    float threshold;             //Match threshold this level
    float fract_thresh_incr;     //Fraction threshold should increase over levels
    int level, maxlevel;         //This level, the maximum allowed level (leaf level).
    colortree(float t, float frac, int l, int maxl) { threshold = t; fract_thresh_incr = frac;
              level = l; maxlevel = maxl;}
    colortree(): threshold(0.7), fract_thresh_incr(0.2), level(0), maxlevel(3){}
    /**
     * float match_level(const ColorTempl &ct, int &best_index)
     *
     * Find the score and index of the best match on this level.
     *
     * @param ct  ColorTempl to be matched this level
     * @param best_index returns index of best match; -1 => no match found
     * @return Best matching score at this level. -1 => no match found
     */
    float match_level(const ColorTempl &ct, int &best_index)
    {
      best_index = -1;
      int len = (int)colortempls.size();
      cout << "in ctree.match_level, len = " << len << endl;
      if (0 == len) return -1.0;
      float max_match = -1.0;

      for(int i = 0; i<len; ++i)
        {
          float match = colortempls[i].match_templates(ct); //,1);
          cout << "  in match_level for(" << i << ") loop, match = " << match << " max_match = " << max_match << endl;
          if(match > max_match)
            {
              max_match = match;
              best_index = i;
              cout << " max_match = " << max_match << ", best_index is now " << i << endl;
            }
        }
      return max_match;
    }
    /**
     * float match(const ColorTempl &ct)
     *
     * Return the best match of a ColorTempl with the stored models here. -1 => no match found
     *
     * @param ct ColorTempl to be matched
     * @return best match score [0,1] or -1 => no match found
     */
    float match(const ColorTempl &ct)
    {
      int index = 0;
      float mymatch = match_level(ct,index);
      if(mymatch < 0.0) return mymatch;
      if(level == maxlevel) return mymatch;
      return(colortrees[index].match(ct));
    }
    /**
     * int insert(const ColorTempl &ct, int &level, int &index)
     *
     * insert a color template into the match tree. Sets the level and index of the return,
     * Returns how many colortempls are at that leaf
     *
     * @param ct      ColorTempl to be insterted
     * @param lev   Set this with the final level
     * @param index   Set this with the final index
     * @return  Total number of cts on this leaf
     */
    int insert(const ColorTempl &ct, int &lev, int &index)
    {
      cout << "In insert at level = " << level << ", maxlevel = " << maxlevel << endl;
      if(level == maxlevel)
        {
          cout << "Pushing back at maxlevel. Now: ";
          colortempls.push_back(ct); //The insertion event
          int len = (int)colortempls.size();
          index = len-1;
          lev = level;
          cout << " len = " << len << ", index = " << index << " level = " << level << endl;
          return len;
        }
      int len;
      float score = match_level(ct,index);
      cout << "score result is " << score << " index " << index << endl;
      if(score > threshold) // We found a tree to go down
        {
          cout << "DESCENDING from level " << level << ", index " << index << endl;
          return(colortrees[index].insert(ct,lev,index));
        }
      else //No score good enough
        {
          cout << "Adding another colortree" << endl;
          colortempls.push_back(ct); //New tree stump matcher
          len = (int)colortempls.size(); //New length
          colortree cotree((1.0 - threshold)*fract_thresh_incr + threshold,
                            fract_thresh_incr, level+1, maxlevel);
          colortrees.push_back(cotree);
          return(colortrees[len-1].insert(ct,lev,index));
        }
    }
    /**
     * coutstats()
     *
     * Just print out level, size and threshold. Returns how many colortempls there are
     */
    int coutstats(int detail = 0)
    {
      int len = (int)colortempls.size();
      if(detail)
        cout << "level " << level << " has " << len << " members. Thresh = " << threshold << endl;
      return len;
    }
    /**
     * int report(int detail = 0)
     *
     * Do a breadth first report on what is in this recognition tree
     *
     * @param detail If set, print out level, len, thresholds, else (default) just give the sum of elements
     * @return sum of all the elements
     */
    int report(int detail = 0)
    {
      int sum = 0;
      deque<colortree> Q;
      Q.push_front(*this); //Enque the top of the tree;
      while(!Q.empty())
        {
          colortree &cotree = Q.back(); //work on the last element
          sum += cotree.coutstats(detail);     //print out its stats
          int len = (int)(cotree.colortrees.size()); //Note that there are no trees on the bottom level
          for(int i = 0; i<len; ++i)
            Q.push_front(cotree.colortrees[i]); //Enque all the direct children colortrees
          Q.pop_back(); //Deque this node
        }
      cout << "Total number of elements in tree = " << sum << endl;
      return sum;
    }


    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        ar & threshold;
        ar & fract_thresh_incr;
        ar & level;
        ar & maxlevel;
        ar & colortempls;
        ar & colortrees;
    }



    /**
     * int clear()
     *
     * Clear colortree of all its data. WARNING: This kills the whole tree
     *
     */
    void clear()
    {
      colortempls.clear();
      colortrees.clear();
      threshold = 0.7; fract_thresh_incr = 0.2; level = 0; maxlevel = 3;
    }
    /**
     * void set(float t, float frac, int maxl)
     *
     * Clear out the color tree and set threshold, fract_thresh_incr, maxlevel
     *
     * @param t       threshold
     * @param frac    fract_thresh_incr
     * @param maxl
     */
    void set(float t, float frac, int maxl)
    {
      clear();
      threshold = t;
      fract_thresh_incr = frac;
      level = 0;
      maxlevel = maxl;
    }
 };

 /**
  * TrainColorTempl
  *
  * This class learns color templates into a colortree structure "ctree". It only learns a new
  * template if no existing template matches the new ColorTempl structure better than acceptance_threshold.
  *
  */
 struct TrainColorTempl
 {


   /**
    * int add_a_template(const ColorTempl &ct, colortree *c_tree)
    *
    * Add a template to the colortree structure no other template there
    * matches it above acceptance_threshold.
    *
    * @param ct
    * @param c_tree
    * @return  1=> template was inserted. 0 it was not inserted because existing templates are close
    */
   int add_a_template(const ColorTempl &ct)
   {
     cout << "In TrainColorTempl, add_a_template" << endl;
     int inserted = 0;
     float score = ctree.match(ct);
     if(score < acceptance_threshold)
       {
         int lev, index;
         ctree.insert(ct, lev, index);
         score = 1.0;  //Inserted item now matches itself perfectly
         inserted = 1;
         cout << "Inserted at lev " << lev << ", index " << index << ", with score " << score << endl;
       }
     else
       cout << "Not inserted at score " << score << " >= acceptance_threshold " << acceptance_threshold << endl;
     ctree.report(1);
     return inserted;
   }

   /**
    * int reset()
    *
    * Clears out the existing model (if any) and returns how large that model was that is now clear.
    * Initializes top level number to be 0.
    *
    * @return How large the deleted colortree was.
    */
   int reset()
   {
     int csize = ctree.coutstats();
     ctree.set(threshold, fract_thresh_incr, maxlevel);
     return csize;
   }

   //Virtual functs
   static void declare_params(tendrils& p)
   {
     p.declare<float> (
                     "acceptance_threshold",
                     "Only learn a new template if the best existing template's match is below acceptance_threshold.",
                     0.85);
     p.declare<string> ("filename","Filename to store colortree 'ctree' to.","ctree.txt");
     p.declare<float>("threshold",
                      "Threshold at the root level of the color tree 'ctree'",0.86);
     p.declare<float>("fract_thresh_incr","Fraction of (1-threshold) to increase threshold at each level in the tree",
                      0.25);
     p.declare<int>("maxlevel","Maximum number of levels in the colortree 'ctree'",4);
     p.declare<bool>("trigger_save","Set true to trigger a persist to disk.",false);
   }

   static void declare_io(const tendrils& params, tendrils& inputs,
                          tendrils& outputs)
   {
     //Check: Do the above params have to be declared here?  Or is this just for the data in the graph?
     //inputs and outputs only here. params may have been set by user.
     inputs.declare<ColorTempl> ("colortempl","Input a ColorTempl histogram");
   }

   void trigger_cb(bool /*trigger*/)
   {
     //persist the colortree ctree
     ofstream os(filename.c_str());
     boost::archive::text_oarchive oar(os);
     oar << ctree;
   }

   void configure(const tendrils& params, const tendrils& in, const tendrils& out)
   {
     acceptance_threshold = params.get<float> ("acceptance_threshold");
     if(!ctree.coutstats())
       {
         cout << "In TrainColorTempl configure if !ctree.coutstats()" << endl;
         filename = params.get<string> ("filename");
         threshold = params.get<float> ("threshold");
         fract_thresh_incr = params.get<float> ("fract_thresh_incr");
         maxlevel = params.get<int> ("maxlevel");
         reset(); //clears then sets: ctree.set(threshold, fract_thresh_incr, maxlevel);
       }
     params["trigger_save"]->set_callback<bool>(boost::bind(&TrainColorTempl::trigger_cb,this,_1));
   }

   int process(const tendrils& inputs, const tendrils& outputs)
   {
     const ColorTempl &ct = inputs.get<ColorTempl> ("colortempl");
     add_a_template(ct);
     return 0;
   }
   //settable
   float acceptance_threshold;  //Only learn a new ColorTempl if it has no match better than this
   //member variables
   colortree ctree;             //Color tree matching structure which holds our model for this object
   float threshold;             //Match threshold for colortree at top
   float fract_thresh_incr;     //Fraction match threshold should increase over levels
   int maxlevel;                //The maximum allowed level (leaf level).
   string filename;             //Filename to store ctree to

 };



 struct TestColorTempl
 {
   float test(const ColorTempl &ct, float &score)
   {
     score = ctree.match(ct);
     return score;
   }

   //Virtual functs
   static void declare_params(tendrils& p)
   {
     p.declare<string>("filename","Name of colortree file to be read in.","ctree.txt");
   }

   static void declare_io(const tendrils& params, tendrils& inputs,
                          tendrils& outputs)
   { //Check: Do the above params have to be declared here?  Or is this just for the data in the graph?
     inputs.declare<ColorTempl> ("colortempl","Input a ColorTempl histogram");
     outputs.declare<float> ("score", "The matching score to the input ColorTempls");
   }

   void configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
   {
     filename = params.get<string> ("filename");
     ifstream is(filename.c_str());
     boost::archive::text_iarchive iar(is);
     iar >> ctree;
     int len = ctree.coutstats();
     if(len <= 0)
       throw std::runtime_error("ERROR: ctree in TestColorTempl.configure is of size zero.");
   }

   int process(const tendrils& inputs, const tendrils& outputs)
   {
     const ColorTempl &ct = inputs.get<ColorTempl> ("colortempl");
     test(ct,outputs.get<float> ("score"));
     return 0;
   }
   //settable
   colortree ctree;           //Color tree matching structure which holds our model for this object
   string filename;           //Filename for reading in ctree
   //member variables
 };




  ///////////////////////////////////////////////////////////////////////////////////
  /**
   * Just output debug psuedo colored images give
   */

  struct ColorDebug
  {
    /**
     * Debug function for looking at color coded image
     * @param Iin is single plane image coded by computeColorOrder
     * @param Iout is the color coded debug image
     */
    void idealize_colors(cv::Mat Iin, cv::Mat& Iout)
    {
      //decode binary images here:
      GaussianBlur(Iin, Iin, Size(5, 5), 2, 2);
      if (Iin.size() != Iout.size() || Iout.type() != CV_8UC3) //Make sure Iout is the right size and type
        {
          Iout.create(Iin.size(), CV_8UC3);
        }
      if (Iin.type() != CV_8UC1)
        {
          throw std::runtime_error(
                                   "ERROR: Iin is not of type CV_8UC1 in idealize_colors");
        }
      uchar cratio;
      int foo = 0;
      for (int y = 0; y < Iin.rows; y++)
        {
          uchar *b = Iout.ptr<uchar> (y);
          uchar *g = b + 1;
          uchar *r = b + 2;
          uchar *o = Iin.ptr<uchar> (y);
          for (int x = 0; x < Iin.cols; x++, b += 3, g += 3, r += 3, o++)
            {
              /**
               * 2 bits each: 0=><64, 1=><128, 2=><192 3 >= 192
               * B/G R/B G/R
               */
              *b = 0;
              *r = 0;
              *g = 0;
              if(!((*o)^0xC0)) //Print masked pixels as gray
                {
                  *b = 32;
                  *r = 32;
                  *g = 32;
                  continue;
                }
              //B/G
              cratio = *o & 3;
              if (cratio == 3)
                *b = 255;
              else if (cratio == 2)
                *b = 196;
              else if (cratio == 1)
                *b = 128;
              //R/B
              cratio = *o & 12;
              if (cratio == 12)
                *r = 255;
              else if (cratio == 8)
                *r = 196;
              else if (cratio == 4)
                *r = 128;
              //G/R
              cratio = *o & 48;
              if (cratio == 48)
                *g = 255;
              else if (cratio == 32)
                *g = 196;
              else if (cratio == 16)
                *g = 128;
              //B&W
              cratio = *o & 192;
              if (cratio == 128)
                {
                  if (!(foo % 5))
                    {
                      *g = 255;
                      *r = 255;
                      *b = 255;
                    }
                  foo++;
                }
              else if (cratio == 0)
                {
                  if (!(foo % 5))
                    {
                      *g = 0;
                      *r = 0;
                      *b = 0;
                    }
                  foo++;
                }

            }
        }
    }
    static void declare_io(const tendrils& params, tendrils& in, tendrils& out)
    {
      in.declare<cv::Mat> ("input",
                           "A binarized color image from the color mod module");
      out.declare<cv::Mat> ("output",
                            "A psychodelic looking image for debugging the color idealization");
    }
    int process(const tendrils& in, const tendrils& out)
    {
      idealize_colors(in.get<cv::Mat> ("input"), out.get<cv::Mat> ("output"));
      return 0;
    }
  };




BOOST_PYTHON_MODULE(line_mod)
{
  using namespace line_mod;
  ecto::wrap<ColorMod>("ColorMod");
  ecto::wrap<ColorDebug>("ColorDebug");
  ecto::wrap<ColorTemplCalc>("ColorTemplCalc");
  ecto::wrap<TrainColorTempl>("TrainColorTempl");
  ecto::wrap<TestColorTempl>("TestColorTempl");
}

} //Namespace linemod
} // namespace serialization
} // namespace boost
