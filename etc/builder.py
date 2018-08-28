from iocbuilder import AutoSubstitution, Device
from iocbuilder.modules.asyn import Asyn, AsynPort, _AsynOctetInterface
from iocbuilder.modules.ADCore import ADCore, ADBaseTemplate, makeTemplateInstance
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
    _SpecificTemplate = _zebraTemplate
    Dependencies = (Asyn, ADCore, MotorLib, Busy)
    def __init__(self, PORT, serialPort, NELM=100000, MAXBUF=5, MAXMEM=0, **args):
        self.__super.__init__(PORT)
        self.__dict__.update(locals())
        self.serialPort = serialPort
        self.MAXBUF = MAXBUF
        self.MAXMEM = MAXMEM

        # Make an instance of our template
        makeTemplateInstance(self._SpecificTemplate, locals(), args)
    
    def Initialise(self):
        print '#zebraConfig(Port, SerialPort, MaxPosCompPoints, MaxBuffers, MaxMemory)'
        print 'zebraConfig("%(PORT)s", "%(serialPort)s", %(NELM)s, %(MAXBUF)d, %(MAXMEM)d)' % self.__dict__

    ArgInfo = ADBaseTemplate.ArgInfo.filtered(without = ["R"]) + \
              _zebraTemplate.ArgInfo.filtered(without=["EMPTY"]) + \
              makeArgInfo(__init__,
                          PORT = Simple("asyn Port Name", str),
                          serialPort = Ident ("Serial port name", _AsynOctetInterface),
                          NELM = Simple("Max points for position compare results", int),
                          MAXBUF = Simple("Maximum number of buffers ", int),
                          MAXMEM = Simple("Maximum memory ", int))

class zebraLastDivDiff(AutoSubstitution):
    '''Makes a record pointing to DIV$(DIV) that will display the difference
    between the last two DIV readings in position compare mode'''
    TemplateFile = 'zebraLastDivDiff.template'
