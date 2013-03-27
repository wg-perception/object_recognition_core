'''
Defines the base class for any training pipeline
'''
import ecto

class TrainerBase(object):
    """
    This is a base class for a training pipeline: you don't need to have your pipeline cell inherit from that class
    but if you do, it will be listed as an official training pipeline
    You need to call the BlackBox constructor in your __init__ first and then this function. Typically, your __init__ is

        >>> class Foo(ecto.BlackBox, TrainerBase):
        >>>     def __init__(self, *args, **kwargs):
        >>>         ecto.BlackBox.__init__(self, *args, **kwargs)
        >>>         TrainerBase.__init__(self)
    """
    pass
