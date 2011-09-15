Database
========
.. _couchdb: http://couchdb.apache.org

We are using `couchdb`_ as our database.  To set up your local instance, all you
must do is install couchdb, and ensure that the service has started.

.. highlight:: ectosh

The following should download and start the couch on Ubuntu like distros.
::
  
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

Library
^^^^^^^
Object Recognition tools manipulate the database either using libCURL or python-couchdb.
You may find it helpful to browse the default db HTML interface at http://localhost:5984/_utils

