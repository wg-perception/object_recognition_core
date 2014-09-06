/*
 * twoDToThreeD.cpp
 *
 *  Created on: Jun 16, 2011
 *      Author: vrabaud
 */

#include <ecto/ecto.hpp>

#include <iostream>

#include <object_recognition_core/common/json.hpp>

using ecto::tendrils;

namespace object_recognition_core
{
  namespace io
  {
    /** Ecto implementation of a module that takes object recognition results and writes them to a CSV file
     */
    struct PipelineInfo
    {
      static void
      declare_params(tendrils& p)
      {
        p.declare(&PipelineInfo::parameters_str_, "parameters", "The JSON parameters of the pipeline.");
      }

      static void
      declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
      {
        outputs.declare(&PipelineInfo::parameters_str_, "parameters_str", "The parameters as a JSON string.");
        outputs.declare(&PipelineInfo::parameters_, "parameters", "The parameters as a JSON dict.");
      }

      void
      configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
      {
        *parameters_ = object_recognition_core::to_json(*parameters_str_);
      }

      int
      process(const tendrils& inputs, const tendrils& outputs)
      {
        return ecto::OK;
      }
    private:
      ecto::spore<std::string> parameters_str_;
      ecto::spore<or_json::mValue> parameters_;
    };
  }
}

ECTO_CELL(io, object_recognition_core::io::PipelineInfo, "PipelineInfo",
    "Spits out the parameters given as a JSON sting.")
