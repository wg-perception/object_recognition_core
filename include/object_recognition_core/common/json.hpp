#pragma once
#include <string>
#include <sstream>
#include "json_spirit/json_spirit.h"
namespace object_recognition_core
{
  inline or_json::mValue
  to_json(const std::string& str)
  {
    or_json::mValue value;
    or_json::read(str, value);
    return value;
  }

  inline std::string
  from_json(const or_json::mValue& obj)
  {
    return or_json::write(obj);
  }
}
