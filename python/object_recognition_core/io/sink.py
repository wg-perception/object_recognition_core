''''
Module defining several outputs for the object recognition pipeline
'''

from object_recognition_core.ecto_cells.io import GuessCsvWriter as GuessCsvWriterCpp

########################################################################################################################

class Sink(object):
    """
    This is a base class for a sink: you don't need to have your sink cell inherit from that class but if you do,
    it will make sure that its inputs/outputs fit the ORK standard (which is good if you want to interact with
    the official ORK pipelines).
    You need to call the BlackBox constructor in your __init__ first and then this function. Typically, yout __init__ is
    class Foo(ecto.BlackBox, Sink):
        def __init__(self, *args, **kwargs):
            ecto.BlackBox.__init__(self, *args, **kwargs)
            Sink.__init__(self)
    """

    def __init__(self):
        """
        This ensures that the given cell exhibits the minimal interface to be
        considered a sink for object recognition
        """
        inputs = dir(self.inputs)
        possible_types_dict = {'pose_results': ['std::vector<object_recognition::common::PoseResult, std::allocator<object_recognition::common::PoseResult> >',
                          'ecto::tendril::none']}
        #all sources must produce the following
        for input_name in possible_types_dict.keys():
            if input_name not in inputs:
                raise NotImplementedError('This cell does not correctly implement the sink interface. '
                                          'Must have an input named %s' % input_name)
        #type checks
        for input_name, possible_types in possible_types_dict.items():
            type_name = self.inputs.at(input_name).type_name
            # test the type: the ecto::tendril::none is here for a passthrough
            if type_name not in type_name:
                raise NotImplementedError('The cell with doc\n%s\n does not correctly implement the sink interface.\n'
                                        'Must have an inputs named %s, with type one of %s\n'
                                        'This cells input at %s has type %s' % (self.__doc__, input_name,
                                        ','.join(possible_types.split()), input_name, type_name))

########################################################################################################################

class GuessCsvWriter(GuessCsvWriterCpp, Sink):

    def __init_(self, *args, **kwargs):
        GuessCsvWriterCpp.__init__(self, *args, **kwargs)
        Sink.__init__(self)
