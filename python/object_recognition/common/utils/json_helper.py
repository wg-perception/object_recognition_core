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

def dict_to_cpp_json_str(dict_obj):
    """
    Given a dictionary object, convert it to a string for C++
    """
    return str(dict_obj).replace("'", '"') #TODO Shouldn't this use json.dumps?

def list_to_cpp_json_str(list_obj):
    """
    Given an iterable, convert it to a string for C++
    """
    return str(json.dumps(list_obj))
