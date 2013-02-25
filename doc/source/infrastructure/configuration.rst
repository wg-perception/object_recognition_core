.. _configuration: 

Configuration File
##################

``ORK`` contains several scripts but two steps (obviously) require heavy configuration: those are
:ref:`Training <training>` and :ref:`Detection <detection>`. Those two steps are very flexible and can use several
inputs/outputs or any pipeline defined on the :ref:`Main Page <index>`.

Those are configured through a config file passed as an argument. For both cases, the file defines one or several
`ecto <http://plasmodic.github.com/ecto>`_ cells that are connected together and executed in a pipeline. Each cell is
defined as follows:

.. code-block:: yaml

   cell_name:
      type: class_of_the_ecto_cell
      module: Python_module_where_the_class_is
      inputs: ['other_cell_name_1', 'other_cell_name_2'] (Optional)
      outputs: ['other_cell_name_3', 'other_cell_name_4'] (Optional)
      parameters: (Optional)
         any_valid_JSON

An example of a pipeline could therefore be:


.. code-block:: yaml

   cell1:
      type: class1
      module: module1
      outputs: [cell2]
      parameters:
         parameter1: value1

   cell2:
      type: class2
      module: module2
      inputs: [cell1] (Optional: actually does not need to be as it's defined for cell1)

The second cell could also have parameters.

Once those relationships are defined, the cells can be properly initialized, linked and executed altogether. That might
seems like sparse information but it really is that simple. The easiest is to look at the different configuration files
for the different pipelines.
