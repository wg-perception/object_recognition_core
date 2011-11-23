#pragma once
#include <string>
#include <sstream>
#include "json_spirit/json_spirit.h"
namespace object_recognition
{
  inline or_json::mObject
  to_json(const std::string& str)
  {
    or_json::mValue value;
    or_json::read(str, value);
    return value.get_obj();
  }

  inline std::string
  from_json(const or_json::mObject& obj)
  {
    return or_json::write(obj);
  }
}
