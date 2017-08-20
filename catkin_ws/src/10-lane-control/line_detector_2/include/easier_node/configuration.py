from collections import namedtuple

import yaml
from yaml.error import YAMLError

from duckietown_utils.exception_utils import raise_wrapped
from duckietown_utils.exceptions import DTConfigException
from duckietown_utils.locate_files_impl import locate_files
from duckietown_utils.path_utils import get_ros_package_path


__all__ = [
    'EasyNodeConfig',
    'load_configuration',
]

EasyNodeConfig = namedtuple('EasyNodeConfig', 'parameters subscriptions contracts publishers')
EasyNodeParameter = namedtuple('EasyNodeParameter', 'name desc type has_default default')
EasyNodeSubscription = namedtuple('EasyNodeSubscription', 'name desc type topic queue_size process')
PROCESS_THREADED = 'threaded'
PROCESS_SYNCHRONOUS = 'synchronous'
PROCESS_VALUES = [PROCESS_THREADED, PROCESS_SYNCHRONOUS]

EasyNodePublisher = namedtuple('EasyNodePublisher', 'name desc type topic queue_size')

# type = int, bool, float, or None (anything)
DEFAULT_NOT_GIVEN = 'default-not-given'

def merge_configuration(c1, c2):
    parameters = {}
    subscriptions = {}
    contracts = {}
    publishers = {}
    for c in [c1, c2]:
        parameters.update(c.parameters)
        subscriptions.update(c.subscriptions)
        contracts.update(c.contracts)
        publishers.update(c.publishers)
    res = EasyNodeConfig(parameters=parameters, 
                         subscriptions=subscriptions, 
                         contracts=contracts,
                         publishers=publishers)
    return res
    
def load_configuration_package_node(package_name, node_type_name):
    path = get_ros_package_path(package_name)
    look_for = '%s.easier_node.yaml' % node_type_name
    found = locate_files(path, look_for)
    if not found:
        msg = 'Could not find EasyNode configuration %r.' % look_for
        raise DTConfigException(msg) # XXX
    
    fn = found[0]
    contents = open(fn).read()
    res = load_configuration(fn, contents)
    return res
        
def load_configuration(realpath, contents):
    # TODO: load "version" string
    try:
        data = yaml.load(contents)
    except YAMLError as e:
        msg = 'Could not read YAML file properly:\n  %s' % realpath
        raise_wrapped(DTConfigException, e, msg, compact=True)
    try:
        parameters = load_configuration_parameters(data['parameters'])
        subscriptions =load_configuration_subscriptions(data['subscriptions'])
        contracts = load_configuration_contracts(data['contracts'])
        publishers = load_configuration_subscriptions(data['publishers'])
        return EasyNodeConfig(parameters=parameters, contracts=contracts, 
                              subscriptions=subscriptions, publishers=publishers)
    except KeyError as e:
        msg = 'Invalid configuration: missing %r in\n %s' % (e, realpath)
        raise  DTConfigException(msg)
    
def load_configuration_parameters(data):
    res = {}
    for k, v in data.items():
        check_good_name(k)
        res[k] = load_configuration_parameter(k, v) 
    return res

def load_configuration_subscriptions(data):
    res = {}
    for k, v in data.items():
        check_good_name(k)
        res[k] = load_configuration_subscription(k, v) 
    return res

def load_configuration_parameter(name, data):
#     verbose:
#         desc: Whether the node is verbose or not.
#         type: bool
#         default: true
#     
    desc = data.pop('desc', None)
    type_ = data.pop('type')
    
    if 'default' in data:
        default = data.pop('default')
        has_default = True
    else:
        default = DEFAULT_NOT_GIVEN
        has_default = False
        
    if data:
        msg = 'Extra keys: %r' % data
        raise DTConfigException(msg)
    
    type2T = {
        'bool': bool,
        'str': str,
        'int': int,
        'float': float,
        'any': None,
    }
    
    if not type_ in type2T:
        raise NotImplementedError(type_)
    T = type2T[type_] 
    
    if has_default and default is not None and T is not None:
        default = T(default) 
    
    return EasyNodeParameter(name=name, desc=desc, type=T, 
                             has_default=has_default, default=default)

def check_good_name(k):
    # TODO
    pass

def message_class_from_string(s):
    from sensor_msgs.msg import CompressedImage, Image  # @UnresolvedImport
    from duckietown_msgs.msg import (AntiInstagramTransform, BoolStamped, Segment, SegmentList, Vector2D)  # @UnresolvedImport

    type2T = {
        'CompressedImage': CompressedImage,
        'BoolStamped': BoolStamped,
        'Image': Image,
        'AntiInstagramTransform': AntiInstagramTransform,
        'Vector2D': Vector2D,
        'SegmentList': SegmentList,
        'Segment': Segment,
    }
    
    if not s in type2T:
        raise NotImplementedError(s)
    
    return type2T[s]
    
def load_configuration_subscription(name, data):
#      image:
#         desc: Image to read
#         topic: ~image
#         type: CompressedImage
#         queue_size: 1
    try:
        desc = data.pop('desc', None)
        topic = data.pop('topic')
        type_ = data.pop('type')
        queue_size = data.pop('queue_size', None)
        process = data.pop('process', PROCESS_SYNCHRONOUS)
        if not process in PROCESS_VALUES:
            msg = 'Invalid value of process %r not in %r.' % (process, PROCESS_VALUES)
            raise DTConfigException(msg)
        
    except KeyError as e:
        msg = 'Could not find field %r.' % e
        raise DTConfigException(msg)
    
    if data:
        msg = 'Extra keys: %r' % data
        raise DTConfigException(msg)
    T = message_class_from_string(type_)  
    
    return EasyNodeSubscription(name=name, desc=desc, topic=topic,
                                type=T, queue_size=queue_size, process=process)


def load_configuration_publisher(name, data):
    try:
        desc = data.pop('desc', None)
        topic = data.pop('topic')
        type_ = data.pop('type')
        queue_size = data.pop('queue_size', None)
    
    except KeyError as e:
        msg = 'Could not find field %r.' % e
        raise DTConfigException(msg)
    
    if data:
        msg = 'Extra keys: %r' % data
        raise DTConfigException(msg)
    
    T = message_class_from_string(type_)  
    
    return EasyNodePublisher(name=name, desc=desc,  topic=topic,
                                type=T, queue_size=queue_size)
    
def load_configuration_contracts(data):
    return {}
    
    
    
    