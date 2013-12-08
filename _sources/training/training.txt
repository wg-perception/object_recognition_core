:orphan:

.. _training:

Training
########
.. highlight:: ectosh

.. contents::

Once some observations are stored in the database, models can be built from them.

Config File
***********

Training, just like recognition, requires a configuration file but it usually only contains one cell that defines a
pipeline that reads data from the database and computes a model. Then again, nothing prevents you to train several
models at the same time by executing several pipelines.

Use
***

.. toggle_table::
  :arg1: Non-ROS
  :arg2: ROS

.. toggle:: Non-ROS

   Just run the training.py script in ``/apps``. It requires a configuration file through the ``-c`` option. Some of the
   options in there can be overriden through the command line for convenience. Change the following parameters to your needs:
   
      * the ``object_ids`` should be the list of object ids you want to train on. If you want, you can also use object_names,
          that are more human readable. object_ids is of the form ["6b3de86feee4e840280b4caad90003fb"] but there are two special
          options: if it is "all", then all models are recomputed; if it is "missing", only the missing models are computed.


.. toggle:: ROS

   The training script can be run as follow:
   
   .. code-block:: sh
   
      rosrun object_recognition_core training \
      -c `rospack find object_recognition_tod`/conf/training.ork \
      --visualize

  You can choose whatever configuration file; a few are provided in ``object_recognition_server/conf``.


A typical command line session might look like::

   % apps/training -c config_training.txt --commit
   Prepare G2O: processing image 65/65
   Performing full BA:
   iteration= 0     chi2= 168324165740673896546304.000000   time= 39.2803   cumTime= 39.2803        lambda= 154861.907021 edges= 64563     schur= 1
   Persisted

Command Line Interface
**********************
.. program-output:: ../../../apps/training --help
   :in_srcdir:
