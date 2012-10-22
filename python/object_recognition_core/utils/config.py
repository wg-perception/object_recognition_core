"""
This file implements functions to autogenerate documentation for config files
"""

def config_yaml_for_generator_class(cls):
    """
    Given a pipeline, generate YAML that will be understable in a config file to parameterize it
    """
    return """
           # the type as returned by the type_name function
           type: '%s'
           # the Python module in which that object is
           module: '%s'
           # the parameters that can be used to construct the cells though that object
           """ % (cls.type_name(), cls.__module__)

def config_yaml_for_ecto_cell(cls):
    """
    Given an ecto cell, generate YAML for all the possibles parameters
    """
    return """
           # The type as returned by the type_name function
           type: '%s'
           module: '%s'
           """ % (cls.type_name(), cls.__module__)
