"""
This file implements functions to help documentation:
- autogeneration of config files
"""
import ecto

def config_yaml_for_ecto_cell(cls, header):
    """
    Given an ecto cell, generate YAML for all the possibles parameters

    :param cls: the class of an ecto cell
    :param header: this is just the name of the cell section
    """
    res = '%s:\n' % header
    res += '   type: %s\n' % cls.__name__
    res += '   module: %s\n' % cls.__module__
    # display the parameters
    res += '   parameters:\n'
    p = ecto.Tendrils()
    try:
        cls.declare_params(p)
    except AttributeError:
        p = cls.params
    for tendril_name, tendril in list(p.items()):
        res += '      %s: %s\n' % (tendril_name, tendril.val)

    return res
