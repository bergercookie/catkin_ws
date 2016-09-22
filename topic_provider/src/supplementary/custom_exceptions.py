# Thu Sep 22 16:42:50 EEST 2016, Nikos Koukis

"""
Custom exceptions to be used in various parts of the current ROS package.

"""

import os

class CustomError(Exception):
    """Base class for custom user exceptions."""
    pass


class FileNotFoundError(CustomError):
    """Exception raised when a file referenced by the command line arguments is
    not found.

    """
    def __init__(self, fname):
        self.fname = fname

    def __str__(self):
        error_msg = "File \"{}\" could not be found. ".format(self.fname)
        return repr(error_msg)

class ExecutableNotFoundError(CustomError):
    """Exception to be called when an executable is not found."""
    def __init__(self, exec_name):
        self.exec_name = exec_name

    def __str__(self):
        error_msg = "Executable \"{}\" can't be found.".format(self.exec_name);
        return error_msg


class MRPTExecutableNotFoundError(ExecutableNotFoundError):
    """Exception to be raised when an MRPT executable is not found (either in
    the bin directory or in the path of the user. """
    def __str__(self):
        error_msg = super(MRPTExecutableNotFoundError, self).__str__()
        sup_msg = "\tCheck the MRPT_DIR shell variable or add {name} to the user's path.".format(name=self.exec_name)
        error_msg = "".join([error_msg, os.linesep, sup_msg])
        return error_msg



