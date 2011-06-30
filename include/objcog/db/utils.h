#pragma once
#include <string>
#include <typeinfo>

namespace objcog
{
  std::string name_of(const std::type_info &ti);
  template<typename T>
  std::string name_of()
  {
    return name_of(typeid(T));
  }
}
