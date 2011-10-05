Training
========
.. highlight:: ectosh

.. contents::

Once some observations are stored in the database, models can be built from them.

use
^^^

Just run the training.py script in /apps. It requires a configuration file through the ``-c`` option. Some of the
options in there can be overriden through the command line for convenience.
For now, use the default config_training.txt and change the following parameters to your needs:
  - the db section can point to a different root, another machine for example
  - the object_ids should be the list of object ids you want to train on. If you want, you can also use object_names,
  that are more human readable

A typical command line session might look like::

   % apps/training -c config_training.txt
   Prepare G2O: processing image 65/65
   Performing full BA:
   iteration= 0     chi2= 168324165740673896546304.000000   time= 39.2803   cumTime= 39.2803        lambda= 154861.907021 edges= 64563     schur= 1
   Persisted

command line interface
^^^^^^^^^^^^^^^^^^^^^^
.. program-output:: apps/training --help
   :in_srcdir:

Extras
^^^^^^

To verify that the model is good looking, you can (at least for the TOD models), launch the feature_viewer script in
the app folder.

.. program-output:: apps/feature_viewer --help
   :in_srcdir:
