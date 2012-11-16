from iocbuilder import AutoSubstitution, Device
from iocbuilder.modules.asyn import Asyn, AsynPort, _AsynOctetInterface
from iocbuilder.modules.motor import MotorLib
from iocbuilder.arginfo import *

class _zebraTemplate(AutoSubstitution):
    '''Controls the zebra signal converter box'''

    ## Parse this template file for macros
    TemplateFile = 'zebra.template'

class zebra(AsynPort):
    DbdFileList = ["zebraSupport"]
    LibFileList = ["zebra"]
    UniqueName = "PORT"
    Dependencies = (Asyn, MotorLib)
    def __init__(self, serialPort, **args):
        self.__super.__init__(args["PORT"])
        self.PORT = args["PORT"]
        self.serialPort = serialPort
        _zebraTemplate(**args)
    
    def Initialise(self):
        print '#zebraConfig(Port, SerialPort)'
        print 'zebraConfig("%(PORT)s", "%(serialPort)s")' % self.__dict__

    ArgInfo = _zebraTemplate.ArgInfo.filtered(without=["EMPTY"]) + makeArgInfo(__init__,
        serialPort = Ident ("Serial port name", _AsynOctetInterface)) 


