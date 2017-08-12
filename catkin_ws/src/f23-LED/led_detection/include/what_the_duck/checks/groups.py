from duckietown_utils.system_cmd_imp import CmdException, system_cmd_result
from what_the_duck.check import Check, CheckError, CheckFailed
from what_the_duck.resolution import Suggestion


class YouBelongToGroup(Check):
    def __init__(self, group):
        self.group = group
        
    def check(self):
        cmd = 'groups'
        try:
            res = system_cmd_result(None, cmd,
                      display_stdout=False,
                      display_stderr=False,
                      raise_on_error=True,
                      capture_keyboard_interrupt=False,
                      env=None)
        except CmdException as e:
            raise CheckError(str(e))
        
        if not self.group in res.stdout:
            msg = 'Group not found in %r' % self.group
            raise CheckFailed(msg)
        
    def get_suggestion(self):
        import getpass 
        username = getpass.getuser() 
        msg = """
        
You can correct this by adding yourself to the group:

    $ sudo adduser %s %s
""" % (username, self.group)
        return Suggestion(msg)
    
    
class YouAreNotUser(Check):
    def __init__(self, username):
        self.username = username
    def check(self):
        import getpass 
        username = getpass.getuser() 
        if username == self.username:
            msg = 'You are logged in as user %s' % username 
            raise CheckFailed(msg)
        
        
        
        
        
        