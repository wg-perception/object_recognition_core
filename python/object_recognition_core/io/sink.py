''''
Module defining several outputs for the object recognition pipeline
'''

from object_recognition_core.ecto_cells.io import GuessCsvWriter as GuessCsvWriterCpp

########################################################################################################################

class SinkBase(object):
    """
    This is a base class for a sink: you don't need to have your sink cell inherit from that class but if you do,
    it will make sure that its inputs/outputs fit the ORK standard (which is good if you want to interact with
    the official ORK pipelines).
    You need to call the BlackBox constructor in your __init__ first and then this function. Typically, yout __init__ is

        >>> class Foo(ecto.BlackBox, SinkBase):
        >>>     def __init__(self, *args, **kwargs):
        >>>         ecto.BlackBox.__init__(self, *args, **kwargs)
        >>>         SinkBase.__init__(self)
    """

    def __init__(self):
        """
        This ensures that the given cell exhibits the minimal interface to be
        considered a sink for object recognition
        """
        validate_sink(self)

def validate_sink(cell):
    """
    This ensures that the given cell exhibits the minimal interface to be
    considered a sink for object recognition
    """
    assert(isinstance(cell, SinkBase))
    inputs = dir(cell.inputs)
    # all sources must produce the following
    for x in ['pose_results']:
        if x not in inputs:
            raise NotImplementedError('This cell does not correctly implement the sink interface. Must have an input named %s' % x)
    # type checks
    possible_types_dict = {'pose_results': [
        'std::vector<object_recognition::common::PoseResult, std::allocator<object_recognition::common::PoseResult> >',
        'ecto::tendril::none']}
    for input_name, possible_types in possible_types_dict.items():
        type_name = cell.inputs.at(input_name).type_name
        # test the type: the ecto::tendril::none is here for a passthrough
        if type_name not in type_name:
            raise NotImplementedError('The cell with doc\n%s\n does not correctly implement the sink interface.\n'
                                    'Must have an input named %s, with type one of %s\n'
                                    'This cells input at %s has type %s' % (cell.__doc__, input_name,
                                    ','.join(possible_types.split()), input_name, type_name))
    return cell

########################################################################################################################

class GuessCsvWriter(GuessCsvWriterCpp, SinkBase):

    def __init__(self, *args, **kwargs):
        GuessCsvWriterCpp.__init__(self, *args, **kwargs)
        SinkBase.__init__(self)
