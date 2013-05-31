from iocbuilder import AutoSubstitution, Device
from iocbuilder.modules.asyn import Asyn, AsynPort, _AsynOctetInterface
from iocbuilder.modules.calc import Calc
from iocbuilder.modules.busy import Busy
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
    Dependencies = (Asyn, MotorLib, Calc, Busy)
    def __init__(self, serialPort, **args):
        self.__super.__init__(args["PORT"])
        z = _zebraTemplate(**args)        
        self.PORT = args["PORT"]
        self.serialPort = serialPort
        self.NELM = int(z.args["NELM"])
    
    def Initialise(self):
        print '#zebraConfig(Port, SerialPort, MaxPosCompPoints)'
        print 'zebraConfig("%(PORT)s", "%(serialPort)s", %(NELM)d)' % self.__dict__

    ArgInfo = _zebraTemplate.ArgInfo.filtered(without=["EMPTY"]) + makeArgInfo(__init__,
        serialPort = Ident ("Serial port name", _AsynOctetInterface)) 

class zebraLastDivDiff(AutoSubstitution):
    '''Makes a record pointing to DIV$(DIV) that will display the difference
    between the last two DIV readings in position compare mode'''
    TemplateFile = 'zebraLastDivDiff.template'
