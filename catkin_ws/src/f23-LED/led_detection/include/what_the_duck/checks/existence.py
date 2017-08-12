from what_the_duck.check import Check, CheckFailed
import os
from what_the_duck.checks.fileutils import expand_all

class FileExists(Check):
    
    def __init__(self, filename):
        self.filename = filename
        
    def check(self):
        fn = expand_all(self.filename)
        
        if not os.path.exists(fn):
            msg = 'File does not exist: %s' % fn
            raise CheckFailed(msg)

        if os.path.isdir(fn):
            msg = 'Expect this to be a file, not a directory: %s' % fn
            raise CheckFailed(msg)
        
class DirExists(Check):
    
    def __init__(self, filename):
        self.filename = filename
        
    def check(self):
        fn = expand_all(self.filename)
        
        if not os.path.exists(fn):
            msg = 'Dir does not exist: %s' % fn
            raise CheckFailed(msg)
        
        if not os.path.isdir(fn):
            msg = 'Expect this to be a directory: %s' % fn
            raise CheckFailed(msg)
        