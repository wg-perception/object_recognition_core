''''
Module defining several outputs for the object recognition pipeline
'''

from ecto_object_recognition.io import GuessCsvWriter
from object_recognition.dbtools import db_params_to_db

#SinkTypes is dict of name:name that describes the types of sinks available
#append to the dict sink types.
SinkTypes = type('SourceTypes', (object,),
                   dict(publisher='publisher',
                        csv_writer='csv_writer',
                        )

                   )

def object_to_models_mapping(db, object_ids):
    from object_recognition import models
    #run a view to get all of the meshes for the given objects.
    mapping = {}
    if 'mesh' not in models.Model.by_object_id_and:
        return mapping
    r = models.Model.by_object_id_and['mesh'](db)

    obj = models.Object.all(db)
    for x in r:
        obj_name = "N/A"
        for y in obj:
            if str(x.object_id) == str(y.id):
                obj_name = str(y.object_name)
                break
        mapping[str(x.object_id)] = (str(x.id), obj_name)
    return mapping


class Sink(object):
    '''
    An RGB, Depth Map source.
    '''
    @staticmethod
    def create_sink(sink_type=SinkTypes.publisher, *args, **kwargs):
        from .ros.sink import Publisher
        #extend this dict as necessary
        sinks = {SinkTypes.publisher:Publisher,
                 SinkTypes.csv_writer:GuessCsvWriter
                 }
        if sink_type == SinkTypes.publisher:
            from ecto_object_recognition.object_recognition_db import ObjectDbParameters
            db_params = ObjectDbParameters(kwargs.get('db'))
            mapping = object_to_models_mapping(db_params_to_db(db_params), kwargs.get('object_ids', 'all'))
            return Publisher(mapping=mapping, **kwargs)
        else:
            return sinks[sink_type](*args, **kwargs)

    @staticmethod
    def add_arguments(parser):
        #--ros_kinect is the default.
        sinks = [x for x in dir(SinkTypes) if not x.startswith('__')]
        parser.add_argument('--sink_type', dest='sink_type', choices=(sinks), default=SinkTypes.publisher,
                            help='The source type to use. default(%(default)s)')

    @staticmethod
    def parse_arguments(obj):
        if type(obj).__name__ == 'dict':
            dic = obj
        else:
            dic = obj.__dict__
        if 'sink_type' in dic:
            dic['type'] = dic.pop('sink_type')

        if 'type' in dic:
            sink = Sink.create_sink(sink_type=dic['type'], **dic)
        else:
            raise RuntimeError("Could not create a sink from the given args! %s" % str(dic))
        return _assert_sink_interface(sink)

######################################################################################################
#testing user interface interface
def _assert_sink_interface(cell):
    ''' This ensures that the given cell exhibits the minimal interface to be
    considered a source for object recogntion
    '''
    inputs = dir(cell.inputs)
    #all sources must produce the following
    for x in ('Rs', 'Ts', 'object_ids'):
        if x not in inputs:
            raise NotImplementedError('This cell does not correctly implement the sink interface. Must have an input named %s' % x)
    #type checks
    for x in ('Rs', 'Ts'):
        type_name = cell.inputs.at(x).type_name
        #TODO add more explicit types.
        if type_name not in ['std::vector<cv::Mat>', 'std::vector<cv::Mat, std::allocator<cv::Mat> >']:
            raise NotImplementedError('This cell does not correctly implement the sink interface.\n'
                                      'Must have an output named %s, with type %s\n'
                                      'This cells input at %s has type %s' % (x, 'std::vector<cv::Mat>', x, type_name))
    return cell
###########################################################################################################
