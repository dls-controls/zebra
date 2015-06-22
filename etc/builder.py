from iocbuilder import AutoSubstitution, Device
from iocbuilder.modules.asyn import Asyn, AsynPort, _AsynOctetInterface
from iocbuilder.modules.areaDetector import AreaDetector, _ADBase
from iocbuilder.modules.busy import Busy
from iocbuilder.modules.motor import MotorLib
from iocbuilder.arginfo import *

class _zebraTemplate(AutoSubstitution):
    '''Controls the zebra signal converter box'''

    ## Parse this template file for macros
    TemplateFile = 'zebra.template'

class zebra(_ADBase):
    DbdFileList = ["zebraSupport"]
    LibFileList = ["zebra"]
    _SpecificTemplate = _zebraTemplate
    Dependencies = (Asyn, AreaDetector, MotorLib, Busy)
    def __init__(self, serialPort, MAXBUF=5, MAXMEM=0, **args):
        self.__super.__init__(R=args["Q"] + ":", **args)
        self.serialPort = serialPort
        self.MAXBUF = MAXBUF
        self.MAXMEM = MAXMEM
    
    def Initialise(self):
        print '#zebraConfig(Port, SerialPort, MaxPosCompPoints, MaxBuffers, MaxMemory)'
        print 'zebraConfig("%(PORT)s", "%(serialPort)s", %(NELM)s, %(MAXBUF)d, %(MAXMEM)d)' % self.__dict__

    ArgInfo = _ADBase.ArgInfo.filtered(without = ["R"]) + \
              _zebraTemplate.ArgInfo.filtered(without=["EMPTY"]) + \
              makeArgInfo(__init__,
                          serialPort = Ident ("Serial port name", _AsynOctetInterface),
                          MAXBUF = Simple("Maximum number of buffers (areaDetector)", int),
                          MAXMEM = Simple("Maximum memory (areaDetector)", int))

class zebraLastDivDiff(AutoSubstitution):
    '''Makes a record pointing to DIV$(DIV) that will display the difference
    between the last two DIV readings in position compare mode'''
    TemplateFile = 'zebraLastDivDiff.template'
