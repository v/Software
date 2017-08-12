from .resolution import Suggestion

from .checks import *  # @UnusedWildImport
from .entry import Entry
from .entry import Diagnosis

def get_checks():
    """ Returns a list of Entry """

    entries = [] # 
    def add(only_run_if, desc, check, diagnosis, *resolutions):
        E = Entry(desc=desc, check=check,
                  diagnosis=diagnosis, 
                  resolutions=resolutions, 
                  only_run_if=only_run_if)
        entries.append(E)
        return E
    
    add(None,
        "Camera is detected",
        CommandOutputContains('sudo vcgencmd get_camera', 'detected=1'),
        Diagnosis("The camera is not connected."))
    
    add(None,
        "Scipy is installed",
        CanImportPackages(['scipy', 'scipy.io']),
        Diagnosis("Scipy is not installed correctly."))
    
    add(None,
        "sklearn is installed",
        CanImportPackages(['sklearn']),
        Diagnosis("sklearn is not installed correctly."))
        
    add(None,
        "Date is set correctly.",
        CheckDate(),
        Diagnosis("The date is not set correctly."))

    add(None,
        "Messages are compiled.",
        CheckImportMessages(),
        Diagnosis("The messages are not compiling correctly."))

    add(None,
        "Environment variables are OK",
        CheckEnvironmentVariables(),
        Diagnosis("The messages are not compiling correctly."))

    ssh_is_there = add(None,\
        "~/.ssh/ exists",
        FileOrDirExists('~/.ssh'),
        Diagnosis("SSH config dir does not exist."))

    add(ssh_is_there,
        "~/.ssh/ permissions",
        CheckPermissions('~/.ssh', '0700'),
        Diagnosis("SSH directory has wrong permissions."),
        Suggestion('Run:  chmod 0700 ~/.ssh'))

    add(ssh_is_there,
        "~/.ssh/config exists",
        FileOrDirExists('~/.ssh/config'),
        Diagnosis("SSH config does not exist."))
    
    return entries

