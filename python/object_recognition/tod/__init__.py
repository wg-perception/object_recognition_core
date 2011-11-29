import collections
from trainer import *
from detector import *
#from feature_descriptor import *

def merge_dict(a, b):
    """
    Merge two dictionaries recursively (.update would erase co-existing values and not merge them
    """
    c = a.copy()
    for key, val in b.iteritems():
        if key in a:
            if isinstance(val, collections.Mapping) and isinstance(a[key], collections.Mapping):
                c[key] = merge_dict(val, a[key])
            # otherwise, a is preferred as done with the initial copy
        else:
            c[key] = val
    return c
