get it
^^^^^^
::

  % git clone git://github.com/ethanrublee/sim.git

requires
^^^^^^^^
* boost
* cmake
* vtk 5::
  
  % sudo apt-get install libvtk5-dev

* OpenCV > 2.3 http://opencv.willowgarage.com/wiki/
* *optional* ecto https://github.com/plasmodic/ecto

build it
^^^^^^^^
::

  % cd sim
  % mkdir build
  % cd build
  % cmake ..
  % make

run it
^^^^^^
::

  % cd sim
  % build/plane_render board.png

hack it
^^^^^^^

Main program is plane_render.cpp
