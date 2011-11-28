Database
========
.. _couchdb: http://couchdb.apache.org

We are using `couchdb`_ as our database.  To set up your local instance, all you
must do is install couchdb, and ensure that the service has started.

.. highlight:: ectosh

The following should download and start the couch on Ubuntu like distros::

  % sudo apt-get install couchdb


To test that it is working properly::
  
  % curl -X GET http://localhost:5984
  {"couchdb":"Welcome","version":"1.0.1"}

The couchdb version should be greater than ``0.10.0``.

Configuration
^^^^^^^^^^^^^
If you would like to expose your couchdb instance to a different port, or bind
to a different interface, besides the loopback, edit ``/etc/couchdb/local.ini``.

For instance, changing the bind_address to ``0.0.0.0`` will allow your couchdb
to be accessed on any interface.

::

  [httpd]
  port = 5984
  bind_address = 0.0.0.0

After editing this file, restart the couchdb service::

  % sudo service couchdb restart

Web UI
^^^^^^

We have a set of webpages that may be pushed to your couchdb instance that help
browse the objects that you train, or models created.

First make sure you have ``couchapp``::
  
  % sudo easy_install couchapp

There is a make target for installing the web ui for your convenience.::

  % make or_web_ui

This will push the app to the location specified in the Cmake cache, by the variable,
``OR_WEB_UI_LOCATION``.  Use ccache or cmake-gui to point it to a different location if you
like.

You can manually push it also, if you need more flexibility, or hate the cmake cache.
cd to the ``object_recognition/web_ui`` directory and run couchapp in a manner similar to the
following.::
  
  % couchapp push . http://localhost:5984/or_web_ui


This will upload the contents of the directory to collection in your couchdb instance,
called ``or_web_ui``.  After this you can browse the web ui using the url
http://localhost:5984/or_web_ui/_design/viewer/index.html

Library
^^^^^^^
Object Recognition tools manipulate the database either using libCURL or python-couchdb.
You may find it helpful to browse the default db HTML interface at http://localhost:5984/_utils

