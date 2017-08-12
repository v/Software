import os
import stat

from what_the_duck.check import CheckFailed, CheckError


class CheckPermissions():
    def __init__(self, filename, expected):
        if not isinstance(expected, str) or len(expected) != 4:
            msg = 'Expected "expected" to be a 4-digit string octal ("0700")'
            raise ValueError(msg)
        
        self.filename = filename
        self.expected = expected
        
    def check(self):
        fn = os.path.expanduser(self.filename)
        if not os.path.exists(fn):
            msg = 'Cannot check permissions if file does not exist.'
            raise CheckError(msg)
        
        fstats = os.stat(fn)
        filemode = oct(stat.S_IMODE(fstats.st_mode))
        if len(filemode) > 4:
            filemode = filemode[-4:]
        if filemode != self.expected:
            msg = ('Expected mode %r, obtained %r for %s' % 
                   (self.expected, filemode, fn))
            raise CheckFailed(msg)
            
#         logger.info('%s = %s' % (fn, filemode))
    
#     
#     stat.S_IRWXU
# 
# stat.S_IRUSR
# Owner has read permission.
# 
# stat.S_IWUSR
# Owner has write permission.
# 
# stat.S_IXUSR
# Owner has execute permission.
# 
# stat.S_IRWXG
# Mask for group permissions.
# 
# stat.S_IRGRP
# Group has read permission.
# 
# stat.S_IWGRP
# Group has write permission.
# 
# stat.S_IXGRP
# Group has execute permission.
# 
# stat.S_IRWXO
# Mask for permissions for others (not in group).
# 
# stat.S_IROTH
# Others have read permission.
# 
# stat.S_IWOTH
# Others have write permission.
# 
# stat.S_IXOTH
