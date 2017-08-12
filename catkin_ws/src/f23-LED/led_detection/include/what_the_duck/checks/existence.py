from what_the_duck.check import Check, CheckFailed
import os

class FileOrDirExists(Check):
    
    def __init__(self, filename):
        self.filename = filename
        
    def check(self):
        fn = os.path.expanduser(self.filename)
        if not os.path.exists(fn):
            msg = 'File does not exist: %s' % fn
            raise CheckFailed(msg)