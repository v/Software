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

EasyNodeConfig = namedtuple('EasyNodeConfig', 'parameters subscriptions contracts')
EasyNodeParameter = namedtuple('EasyNodeParameter', 'name desc type has_default default')
# type = int, bool, float, or None (anything)
DEFAULT_NOT_GIVEN = 'default-not-given'

def merge_configuration(c1, c2):
    parameters = {}
    subscriptions = {}
    contracts = {}
    for c in [c1, c2]:
        parameters.update(c.parameters)
        subscriptions.update(c.subscriptions)
        contracts.update(c.contracts)
    res = EasyNodeConfig(parameters=parameters, 
                         subscriptions=subscriptions, 
                         contracts=contracts)
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
    try:
        data = yaml.load(contents)
    except YAMLError as e:
        msg = 'Could not read YAML file properly:\n  %s' % realpath
        raise_wrapped(DTConfigException, e, msg, compact=True)
    try:
        parameters = load_configuration_parameters(data['parameters'])
        subscriptions =load_configuration_subscriptions(data['subscriptions'])
        contracts = load_configuration_contracts(data['contracts'])
        return EasyNodeConfig(parameters=parameters, contracts=contracts, subscriptions=subscriptions)
    except KeyError as e:
        msg = 'Invalid configuration: missing %r in\n %s' % (e, realpath)
        raise  DTConfigException(msg)
    
def load_configuration_parameters(data):
#     verbose:
#         desc: Whether the node is verbose or not.
#         type: bool
#         default: true
#     
    res = {}
    for k, v in data.items():
        check_good_name(k)
        res[k] = load_configuration_parameter(k, v)
        
    return res


def load_configuration_parameter(name, data):
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
    
    return EasyNodeParameter(name=name, desc=desc, type=T, has_default=has_default, default=default)

def check_good_name(k):
    # TODO
    pass

def load_configuration_subscriptions(data):
    return {}

def load_configuration_contracts(data):
    return {}
    
    
    
    