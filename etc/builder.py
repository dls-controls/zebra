from iocbuilder import AutoSubstitution, Device
from iocbuilder.modules.asyn import AsynPort, _AsynOctetInterface
from iocbuilder.arginfo import *

class _zebraTemplate(AutoSubstitution):
    '''Controls the zebra signal converter box'''

    ## Parse this template file for macros
    TemplateFile = 'zebra.template'

class zebra(AsynPort):
    DbdFileList = ["zebraSupport"]
    LibFileList = ["zebra"]
    def __init__(self, name, serialPort, **args):
        self.__super.__init__(name)
        args["PORT"] = name
        self.name = name
        self.serialPort = serialPort
        _zebraTemplate(**args)
    
    def Initialise(self):
        print '#zebraConfig(Port, SerialPort)'
        print 'zebraConfig("%(name)s", "%(serialPort)s")' % self.__dict__

    ArgInfo = _zebraTemplate.ArgInfo.filtered(without=["PORT"]) + makeArgInfo(__init__,
        name       = Simple("Object and asyn port name"),
        serialPort = Ident ("Serial port name", _AsynOctetInterface)) 


