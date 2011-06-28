#include <ecto/ecto.hpp>

void wrap_Camera();
void wrap_PoseRT();


BOOST_PYTHON_MODULE(tod)
{
  wrap_Camera();
  wrap_PoseRT();
}
