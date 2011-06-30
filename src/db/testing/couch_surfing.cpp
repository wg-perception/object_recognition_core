/*
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */
#include <objcog/db/couch.hpp>

#include <boost/serialization/binary_object.hpp>
#include <boost/serialization/split_free.hpp>
#include <boost/format.hpp>
#include <boost/progress.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <iostream>
#include <string>

namespace boost
{
  namespace serialization
  {

    template<class Archive>
    void save(Archive & ar, const cv::Mat & m, const unsigned int version)
    {
      int type = m.type();
      ar & m.rows;
      ar & m.cols;
      ar & type;
      const uchar * data = m.data, *end = m.dataend;
      ar
          & boost::serialization::make_binary_object(const_cast<uchar*>(data),
                                                     size_t(end - data));
    }

    template<class Archive>
    void load(Archive & ar, cv::Mat & m, const unsigned int version)
    {
      int rows, cols, type;
      ar & rows;
      ar & cols;
      ar & type;
      m.create(rows, cols, type);
      uchar * data = m.data, *end = m.dataend;
      ar & boost::serialization::make_binary_object(data, end - data);
    }

  } // namespace serialization
} // namespace boost

BOOST_SERIALIZATION_SPLIT_FREE(cv::Mat)

void test_list_dbs()
{
  std::vector<std::string> dbs;
  couch::Db::all_dbs(std::string(DEFAULT_COUCHDB_URL), dbs);
  std::copy(dbs.begin(), dbs.end(),
            std::ostream_iterator<std::string>(std::cout, " "));
}

struct MyDoc
{
  MyDoc()
  {
  }

  MyDoc(const std::string& name, int number) :
      name(name), favorite_number(number)
  {
  }

  std::string name;
  int favorite_number;

  void operator>>(couch::Document& doc)
  {
    doc.set_value("name", name);
    doc.set_value("favorite_number", favorite_number);
  }
  void operator<<(couch::Document& doc)
  {
    name = doc.get_value<std::string>("name");
    name = doc.get_value<int>("favorite_number");
  }
};

std::ostream& operator<<(std::ostream& out, const MyDoc& value)
{
  return out << "name:" << value.name << " favorite_number:"
      << value.favorite_number;
}
#define STRINGYFY(A) #A

std::string map_fn_01 = STRINGYFY(
    function(doc)
    {
        emit(null,null);
    }
);

void populate()
{
  couch::Db db(std::string(DEFAULT_COUCHDB_URL) + "/mydocs");
  db.create();
  db.print();
  {
    boost::progress_timer timer;
    for (int i = 0; i < 10; i++)
    {
      couch::Document doc(db);
      MyDoc md(boost::str(boost::format("johny%d") % i), i);
      std::cout << md << std::endl;
      md >> doc;
      doc.create();
      doc.print_result(std::cout);
      std::cout << doc.toString() << std::endl;
    }
  }
  db.update_info();
  db.print_info(std::cout);
  std::cout << std::endl;
  //db.delete_();
  couch::View view;
  view.add_map("map",map_fn_01);
  int total_rows,offset;
  std::vector<couch::Document> docs;
  db.run_view(view,-1 /*this will get all docs*/,0,total_rows,offset,docs);
  db.print();
  while(docs.empty() == false)
    {
      docs.back().update();
      std::cout << docs.back().toString() << std::endl;
      docs.pop_back();
    }
}
void run()
{
  couch::Db db(std::string(DEFAULT_COUCHDB_URL) + "/images");
  std::cout << "\n";
  db.create();
  db.print();
  if (!db.create())
  {
    std::cout << "good, already exists" << std::endl;
  }
  else
  {
    throw std::logic_error("should have been a bad create...");
  }

  test_list_dbs();

  couch::Document doc(db);
  doc.set_value("image_type", "rgb");
  doc.set_value("tags", "lena, image");
  doc.create();
  doc.print_result(std::cout);

  doc.set_value("more_tags", "author, b");
  doc.commit();
  doc.print_result(std::cout);
  std::cout << doc.toString() << std::endl;

  cv::Mat lena = cv::imread("lena.bmp");
  doc.attach("lena.bmp", lena);
  doc.print_result(std::cout);
  cv::Mat lena_gotten;
  doc.get_attachment("lena.bmp", lena_gotten);
  cv::imshow("lena_gotten", lena_gotten);
  doc.update();
  std::cout << doc.toString() << std::endl;

  db.update_info();
  std::cout << "doc count : " << db.get_info_item<int>("doc_count")
      << std::endl;
  ;
  db.print_info(std::cout);
  std::cout << std::endl;

  db.delete_();
  db.print();
  cv::waitKey(1000);
}
int main()
{
  try
  {
    populate();
    run();
    //run();
  } catch (std::exception &e)
  {
    std::cout << "caught " << e.what() << std::endl;
    return 1;
  } catch (...)
  {
    std::cout << "unknown exception..." << std::endl;
    return 1;
  }
  return 0;
}
