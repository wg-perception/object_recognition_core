#!/usr/bin/env python
"""
Module defining a parser that can read arguments from a file as long as an argument @file_path is given.
It will also ignore every line preceded by //, #
"""
import argparse
import sys as _sys

class ObjectRecognitionParser(argparse.ArgumentParser):
    def __init__(self, *args):
        argparse.ArgumentParser.__init__(self, *args, fromfile_prefix_chars='@')

    # copied and tweaked from http://bugs.python.org/issue10523
    def _read_args_from_files(self, arg_strings):
        filtered_arg_strings = self.remove_launchfile_generated_args(arg_strings)
        # expand arguments referencing files
        new_arg_strings = []
        for arg_string in filtered_arg_strings:

            # for regular arguments, just add them back into the list
            if not arg_string or arg_string[0] not in self.fromfile_prefix_chars:
                new_arg_strings.append(arg_string)

            # replace arguments referencing files with the file content
            else:
                try:
                    args_file = open(arg_string[1:])
                    try:
                        arg_strings = []
                        for line in args_file:
                            if not line:
                                continue
                            if line[0].startswith('//') or line[0].startswith('#'):
                                continue
                            arg_strings.extend(line.strip().split())
                        arg_strings = self._read_args_from_files(arg_strings)
                        
                        new_arg_strings.extend(arg_strings)
                    finally:
                        args_file.close()
                except IOError:
                    err = _sys.exc_info()[1]
                    self.error(str(err))

        # return the modified argument list
        return new_arg_strings

    def remove_launchfile_generated_args(self, arg_strings):
        new_arg_strings = []
        for arg_string in arg_strings:
            if not arg_string.startswith('__name:=') and not arg_string.startswith('__log:='):
                new_arg_strings.append(arg_string)
        return new_arg_strings


    # The following function should be the only one needed but the implementation is now broken in Python
    """
    def convert_arg_line_to_args(self, arg_line):
        if (not arg_line) or arg_line.startswith('//') or arg_line.startswith('#'):
            yield ''
        else:
            for arg in arg_line.split():
                yield arg
    """
