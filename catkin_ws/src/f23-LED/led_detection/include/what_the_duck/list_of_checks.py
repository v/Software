# -*- coding: utf-8 -*-

from .checks import *  # @UnusedWildImport
from .entry import Entry
from .entry import Diagnosis
from what_the_duck.detect_environment import on_duckiebot

def get_checks():
    """ Returns a list of Entry """

    entries = [] # 
    def add(only_run_if, desc, check, diagnosis, *suggestions):
        assert isinstance(check, Check), type(check)
        if not suggestions:
            automated = check.get_suggestion()
            if automated is not None:
                suggestions = [automated]
        E = Entry(desc=desc, check=check,
                  diagnosis=diagnosis, 
                  resolutions=suggestions, 
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
        "Date is set correctly",
        CheckDate(),
        Diagnosis("The date is not set correctly."))

    not_root=add(None,
        "Not running as root",
        YouAreNotUser('root'),
        Diagnosis("You should not run the code as root."))

    not_ubuntu=add(not_root,
        "Not running as ubuntu",
        YouAreNotUser('ubuntu'),
        Diagnosis("You should not run the code as ubuntu."))
   
   
    add(not_ubuntu,
        "Member of group sudo",
        YouBelongToGroup("sudo"),
        Diagnosis("You are not authorized to run sudo."))
    
    add(not_ubuntu,
        "Member of group input",
        YouBelongToGroup("input"),
        Diagnosis("You are not authorized to use the joystick."))
    
    add(not_ubuntu,
        "Member of group i2c",
        YouBelongToGroup("input"),
        Diagnosis("You are not authorized to use the motor shield."))

    ssh_is_there = add(not_ubuntu,\
        "~/.ssh/ exists",
        DirExists('~/.ssh'),
        Diagnosis("SSH config dir does not exist."))

    add(ssh_is_there,
        "~/.ssh/ permissions",
        CheckPermissions('~/.ssh', '0700'),
        Diagnosis("SSH directory has wrong permissions.")
        )

    ssh_config_exists = add(ssh_is_there,
        "~/.ssh/config exists",
        FileExists('~/.ssh/config'),
        Diagnosis("SSH config does not exist."))
    
    identity_file = add(ssh_config_exists,
        "At least one key is configured.",
        FileContains('~/.ssh/config', 'IdentityFile'),
        Diagnosis('You have not enabled any SSH key.'))

    add(ssh_is_there,
        "~/.ssh/authorized_keys exists",
        FileExists('~/.ssh/authorized_keys'),
        Diagnosis("You did not setup the SSH authorized keys."))
 
    gitconfig = add(None,
                    "Git configured",
                    FileExists('~/.gitconfig'),
                    Diagnosis("You did not do the local Git configuration."))
    add(gitconfig, 
        "Git email set",
        FileContains("~/.gitconfig", "email ="),
        Diagnosis("You did not configure your email for Git."))

    add(gitconfig, 
        "Git name set",
        FileContains("~/.gitconfig", "name ="),
        Diagnosis("You did not configure your name for Git."))

    add(gitconfig, 
        "Git polish set",
        FileContains("~/.gitconfig", "[push]"),
        Diagnosis("You did not configure the push policy for Git."))
        
    add(None,
        "Edimax detected",
        CommandOutputContains('iwconfig', 'rtl8822bu'),
        Diagnosis("It seems that the Edimax is not detected."))
    
    add(None,
        'The hostname is configured',
        CheckHostnameConfigured(),
        Diagnosis('You have not completed a proper setup.'))
    
    add(None,
        '/etc/hosts is sane',
        CheckGoodHostsFile(),
        Diagnosis('The contents of /etc/hosts will cause problems later on.'))
    
    add(None,
        'Correct kernel version',
        GoodKernel(),
        Diagnosis('You have been messing with the kernel.'),
        Suggestion('You probably need to start with a pristine SD card.'))
    
    add(None,\
        "~/duckieteam/ exists",
        DirExists('~/duckieteam'),
        Diagnosis("You do not have a duckieteam directory."))


    add(None,
        'Wifi name configured',
        WifiNameConfigured(),
        Diagnosis('You have not completed the Wifi configuration.'))
    
    add(None,
        "Messages are compiled",
        CheckImportMessages(),
        Diagnosis("The messages are not compiling correctly."))

   
    add(None,
        'Shell is bash',
        EnvironmentVariableIsEqualTo('SHELL', '/bin/bash'),
        Diagnosis('You have not set the shell to /bin/bash'),
        Suggestion('You can change the shell using `chsh`.'))
    
    # check if we have internet access, and if so try github
    internet_access = add(identity_file,
                          "Working internet connection",
                          InternetConnected(),
                          Diagnosis('You are not connected to internet.')) 
    
    add(internet_access,
        "Github configured",
        GithubLogin(),
        Diagnosis('You have not successfully setup the Github access.'))

    add(None,
        'Environment variable DUCKIETOWN_ROOT',
        EnvironmentVariableExists('DUCKIETOWN_ROOT'),
        Diagnosis("DUCKIETOWN_ROOT is not set."),
        Suggestion('You have not run the environment script.'))
    
    if not on_duckiebot():
        add(None,
            'Environment variable DUCKIETOWN_DATA',
            EnvironmentVariableExists('DUCKIETOWN_DATA'),
            Diagnosis("DUCKIETOWN_DATA is not set."
"""
The environment variable DUCKIETOWN_DATA must either:
1) be set to "n/a"
2) point to an existing path corresponding to Dropbox/duckietown-data.
   (containing a subdirectory 'logs')
"""                  
                  ))
    
    if on_duckiebot():
        add(None,
            'Environment variable VEHICLE_NAME',
            EnvironmentVariableExists('VEHICLE_NAME'),
            Diagnosis("""
The environment variable VEHICLE_NAME must be the name of your robot
(if you are on the robot)."""),
            Suggestion("""
Add this line to ~/.bashrc: 

    export VEHICLE_NAME= (your vehicle name)
"""))

     
    return entries

