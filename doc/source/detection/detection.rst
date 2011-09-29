Detection
=========

Using the different trained objects, we can now detect them.

Short How-to
------------

Just run the detection.py script in /apps. The ``--help`` option details the different options/arguments of the script. Those have two types for now:
 - which source to read from (kinect, ROS bag ...)
 - which sink to send the data to (terminal, csv file, ...)

Sources
-------

Right now, the pipeline supports the following options:
 - ``--ros_kinect``: it reads the image and depth image from the Kinect when controlled by ROS (TODO link to the model capture)
 -

Sinks
-----

The object_recognition pipeline currently supports the following outputs:
 - terminal TODO
 - CSV file (in the NIST ``challenge in perception 2010``format)
 - ROS topic

Core
----

Once the inputs and outputs of the object_recognition pipeline are chosen, a configuration file must be specified
through the ``-c`` option. It contains a JSON string with the parameters for the following parts of the pipeline:
 - "db": contains two options
  - "type": only "CouchDB" is supported for now
  - "url": e.g. "http://localhost:5984"
 - "object_ids": an array of the unique ids of the objects that can be found
 - and then a JSON dictionary for each pipeline that is used:
  - "tod": contains the following sections:
   - 