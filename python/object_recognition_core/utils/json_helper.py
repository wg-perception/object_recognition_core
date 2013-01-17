#!/usr/bin/env python
"""
Module providing simple function to deal with the conversion from file to JSON 
"""

import json

def file_to_json(file_path):
    """
    Given the path of a file containing JSON, load it and make sure there is no unicode
    """
    json_params = json.loads(str(open(file_path).read()))
    json_params = eval(str(json_params).replace("'", '"').replace('u"', '"').replace('{u', '{'))
    return json_params

def obj_to_cpp_json_str(obj):
    """
    Given a dictionary or a list object, convert it to a JSON string for C++ to parse. All unicode
    references are removed
    
    :param obj: a dict or a list
    """
    return json.dumps(obj)
