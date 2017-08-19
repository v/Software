import rospy

from duckietown_utils.exceptions import DTConfigException

from .configuration import load_configuration_package_node, merge_configuration


__all__ = [
    'EasyNode',
]

class EasyNode():
    
    def __init__(self, package_name, node_type_name):
        self.package_name = package_name
        self.node_type_name = node_type_name
        rospy.init_node(node_type_name, anonymous=False) # @UndefinedVariable
    
    def _msg(self, msg):
        return '%s | %s' % (self.node_type_name, msg)
    
    def info(self, msg):
        msg = self._msg(msg) 
        rospy.loginfo(msg)  # @UndefinedVariable
        
    def debug(self, msg):
        msg = self._msg(msg) 
        rospy.logdebug(msg)# @UndefinedVariable
        
    def error(self, msg):
        msg = self._msg(msg) 
        rospy.logerror(msg)# @UndefinedVariable
        
    def on_init(self):
        self.info('on_init (default)')

    def on_parameters_changed(self, first_time, changed):
        self.info('(default) First: %s Parameters changed: %s' % (first_time, changed))

    def on_shutdown(self):
        self.info('on_shutdown (default)')
    
    def _init(self):
        c1 = load_configuration_package_node('line_detector2', 'easier_node')
        c2 = load_configuration_package_node(self.package_name, self.node_type_name)
        self._configuration = merge_configuration(c1, c2)
        self._init_parameters()
        self._init_subscriptions()
        self.info(self._configuration)
        
    def _init_subscriptions(self):
        subscriptions = self._configuration.subscriptions
        class Publishers():
            pass
        self.publishers = Publishers()
        for s in subscriptions.values():
            class Callback():
                def __init__(self, node, s):
                    self.node = node
                    self.s = s
                def __call__(self, data):
                    self.node._sub_callback(self.s, data)
                    
            callback = Callback(self, s)
            self.info('Subscribed to %s')
            S = rospy.Subscriber(s.topic, s.type, callback, queue_size=s.queue_size)  # @UnusedVariable @UndefinedVariable
            setattr(self.publishers, s.name, S)
            
    def _sub_callback(self, subscription, data):
#         self.info('Message %s' % subscription.name)
        callback_name = 'on_received_%s' % subscription.name
        if hasattr(self, callback_name):
            c = getattr(self, callback_name)
            c(data)
        else:
            self.info('No callback %r' % callback_name)
    
    def _init_parameters(self):
        parameters = self._configuration.parameters
        self.info('Loading %d parameters' % len(parameters))
        class Config():
            pass
        self.config = Config()
        values = {}
        for p in parameters.values():
            self.info('Loading parameter %s' % str(p))
            name = '~' + p.name
            if p.has_default:
                val = rospy.get_param(name, p.default)  # @UndefinedVariable
                val = p.type(val)
            else:
                try:
                    val = rospy.get_param(name)  # @UndefinedVariable
                except KeyError:
                    msg = 'Could not load required parameter %r.' % p.name
                    raise DTConfigException(msg)
                
            setattr(self.config, p.name, val)
            self.info('Read %r = %r' % (p.name, val))
            values[p.name] = val
        
        duration = self.config.en_update_params_interval
        duration = rospy.Duration.from_sec(duration)  # @UndefinedVariable
        self.on_parameters_changed(True, values)
        rospy.Timer(duration, self._update_parameters)  # @UndefinedVariable
            
    def _update_parameters(self, _event):
        changed = self._get_changed_parameters()
        if changed:
            for k, v in changed.items():
                setattr(self.config, k, v)
            self.on_parameters_changed(False, changed)
        else:
            pass
            # self.info('No change in parameters.')    
        
    def _get_changed_parameters(self):
        parameters = self._configuration.parameters
        changed = {}
        for p in parameters.values():
            name = '~' + p.name
            if p.has_default:
                val = rospy.get_param(name, p.default)  # @UndefinedVariable
            else:
                val = rospy.get_param(name)  # @UndefinedVariable
            current = getattr(self.config, p.name)
            s1 = current.__repr__()
            s2 = val.__repr__()
            if s1 != s2:
                self.info('change from\n%s\n\nto\n\n%s' % (s1,s2))
                changed[p.name] = current
        return changed
    
    def spin(self):
        rospy.on_shutdown(self.on_shutdown)  # @UndefinedVariable
        self._init()
        self.on_init()
        rospy.spin()  # @UndefinedVariable
        

        