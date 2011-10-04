''''
Module defining several inputs for the object recognition pipeline

All source cells will have the following outputs:
 - K [cv::Mat]
    The camera intrinsics matrix, as a 3x3 double cv::Mat
 - depth [cv::Mat]
    The rescaled depth image.
 - image [cv::Mat]
    A cv::Mat copy.
 - points3d [cv::Mat]
    The 3d points, height by width (or 1 by n_points if mask) with 3 channels (x, y and z)
'''

#SourceTypes is dict of name:name that describes the types of sources available
#append to the dict source types.
SourceTypes = type('SourceTypes', (object,),
                   dict(ros_bag='ros_bag',
                        ros_kinect='ros_kinect',
                        )
                   )

class Source(object):
    '''
    An RGB, Depth Map source.
    '''
    @staticmethod
    def create_source(source_type=SourceTypes.ros_kinect, *args, **kwargs):
        from .ros.source import KinectReader, BagReader
        #extend this dict as necessary
        source = {SourceTypes.ros_bag:BagReader,
                  SourceTypes.ros_kinect:KinectReader,
                  #TODO standalone:StandaloneKinectReader
                  #SourceType : BlackBox
                  }
        return source[source_type](*args, **kwargs)

    @staticmethod
    def add_arguments(parser):
        #--ros_kinect is the default.
        sources = [x for x in dir(SourceTypes) if not x.startswith('__')]
        parser.add_argument('--source_type', dest='source_type', choices=(sources),
                            help='The source type to use. default(%(default)s)')
        parser.add_argument('--ros_bag', dest='ros_bag', type=str,
                            help='The path of a ROS bag to analyze. '
                            'If this is specified it will automatically use'
                            'the source type of, `ros_bag`')

    @staticmethod
    def parse_arguments(obj):
        """
        Function parsing a dictionary or an argument object
        """
        if type(obj).__name__ == 'dict':
            dic = obj
        else:
            dic = obj.__dict__
            dic['bag'] = dic.pop('ros_bag')
            dic['type'] = dic.pop('source_type')
        source = None
        if dic.get('bag', None):
            import os.path as path
            if not path.exists(dic['bag']):
                raise RuntimeError("You must supply a valid path to a bag file: %s does not exist." % dic['bag'])
            source = Source.create_source(source_type=SourceTypes.ros_bag, bag=dic['bag'])
        elif dic.get('type', None):
            source = Source.create_source(source_type=dic['type'])
        else:
            raise RuntimeError("Could not create a source from the given args! %s" % str(dic))
        return _assert_source_interface(source)

######################################################################################################
#testing user interface interface
def _assert_source_interface(cell):
    ''' This ensures that the given cell exhibits the minimal interface to be
    considered a source for object recogntion
    '''
    outputs = dir(cell.outputs)
    #all sources must produce the following
    for x in ('K', 'image', 'depth', 'points3d'):
        if x not in outputs:
            raise NotImplementedError('This cell does not correctly implement the source interface. Must have an output named %s' % x)
    #type checks
    for x in ('K', 'image', 'depth', 'points3d'):
        type_name = cell.outputs.at(x).type_name
        #TODO add more explicit types.
        if type_name != 'cv::Mat':
            raise NotImplementedError('This cell does not correctly implement the source interface.\n'
                                      'Must have an output named %s, with type %s\n'
                                      'This cells output at %s has type %s' % (x, 'cv::Mat', x, type_name))
    return cell
###########################################################################################################
