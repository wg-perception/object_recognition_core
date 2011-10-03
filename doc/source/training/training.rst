Training
========

Once some observations are stored in the database, models can be built from them.

Short How-to
------------

Just run the training.py script in /apps. It requires a configuration file through the ``-c`` option. Some of the
options in there can be overriden by the command line for convenience.
For now, use the default config_training.txt.

Extras
------

To verify that the model is good looking, you can (at least for the TOD models), launch the tod_features.py script in
the test folder.