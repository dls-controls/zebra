#!/bin/env dls-python
from pkg_resources import require
require("pyserial")
from serial import Serial
import os, re, time
from ConfigParser import ConfigParser

class zebraRegs:
    bus_lookup = [
    "DISCONNECT",
    "IN1_TTL",
    "IN1_NIM",
    "IN1_LVDS",
    "IN2_TTL",
    "IN2_NIM",
    "IN2_LVDS",
    "IN3_TTL",
    "IN3_OC",
    "IN3_LVDS",
    "IN4_TTL",
    "IN4_CMP",
    "IN4_PECL",
    "IN5_ENCA",
    "IN5_ENCB",
    "IN5_ENCZ",
    "IN5_CONN",
    "IN6_ENCA",
    "IN6_ENCB",
    "IN6_ENCZ",
    "IN6_CONN",
    "IN7_ENCA",
    "IN7_ENCB",
    "IN7_ENCZ",
    "IN7_CONN",
    "IN8_ENCA",
    "IN8_ENCB",
    "IN8_ENCZ",
    "IN8_CONN",
    "PC_ARM",
    "PC_GATE",
    "PC_PULSE",
    "AND1",
    "AND2",
    "AND3",
    "AND4",
    "OR1",
    "OR2",
    "OR3",
    "OR4",
    "GATE1",
    "GATE2",
    "GATE3",
    "GATE4",
    "DIV1_OUTD",
    "DIV2_OUTD",
    "DIV3_OUTD",
    "DIV4_OUTD",
    "DIV1_OUTN",
    "DIV2_OUTN",
    "DIV3_OUTN",
    "DIV4_OUTN",
    "PULSE1",
    "PULSE2",
    "PULSE3",
    "PULSE4",
    "QUAD_OUTA",
    "QUAD_OUTB",
    "CLOCK_1KHZ",
    "CLOCK_1MHZ",
    "SOFT_IN1",
    "SOFT_IN2",
    "SOFT_IN3",
    "SOFT_IN4"]  
    
    def __init__(self):
        reg_lookup_str = open(os.path.realpath(os.path.join(__file__, "..", "zebraRegs.h"))).read()
        self.reg_lookup = {}
        self.name_lookup = {}
        self.reg_type = {}
        for line in re.sub(r"/\*(.|[\r\n])*?\*/", "", reg_lookup_str).splitlines():
            match = re.match(r'\s*\{\s*"([^"]*)"\s*,\s*(\w*),\s*(\w*)\s*\}', line) #,\s*(\w*)\w*}        
            if match:
                name, num, typ = match.groups()
                reg = int(num, 0)
                self.name_lookup[reg] = name
                self.reg_lookup[name] = reg
                self.reg_type[reg] = typ

    def regs(self):
        return self.name_lookup.keys()

    def reg(self, name):
        return self.reg_lookup[name]
                
    def names(self):
        return self.reg_lookup.keys()
    
    def name(self, reg):
        return self.name_lookup[reg]

    def type(self, reg):
        return self.reg_type[reg]

    def bus_index(self, name):
        return self.bus_lookup.index(name)

    def bus_name(self, index):
        return self.bus_lookup[index]

class zebraConnection:
    # The serial port object
    port = None
    
    def __init__(self, portstr):
        self.port = Serial(portstr, 115200, timeout=10)
        self.regs = zebraRegs()
        
    def readReg(self, reg):
        """Takes reg and reads its values from zebra"""
        if reg.upper() in self.regs.names():
            reg = self.regs.reg(reg.upper())
        cmd = "R%02X" % reg
        self.port.write("%s\n" % cmd)
        response = self.port.readline()
        assert response, "Zebra timed out when reading register %02X" % reg
        assert response.startswith(cmd), "Zebra response '%s' should start with '%s'" %(response.rstrip("\n"), cmd)
        out = int(response[len(cmd):], 16)
        if self.regs.type(reg) == "regMux":
            return self.regs.bus_name(out)
        return out
        
    def writeReg(self, reg, value):
        """Takes reg in hex and writes its value to zebra"""
        if reg.upper() in self.regs.names():
            reg = self.regs.reg(reg.upper())        
        if self.regs.type(reg) not in ("regMux", "regRW"):
            return
        if self.regs.type(reg) == "regMux" and type(value) == str:
            value = self.regs.bus_index(value)
        expected = "W%02XOK" % reg
        self.port.write("W%02X%04X\n" % (reg, value))
        response = self.port.readline().rstrip("\n")
        assert response, "Zebra timed out when writing register %02X" % reg
        assert response == expected, "Zebra response '%s' should be '%s'" %(response, expected)

    def systemBus(self):
        return self.regs.bus_lookup

    def getStatus(self, name):
        i = self.regs.bus_index(name)
        stat_reg = "SYS_STAT" + (["1LO", "1HI", "2LO", "2HI"][i / 16])
        stat_off = i % 16
        return self.readReg(stat_reg) >> stat_off & 1          

    def writeCommand(self, cmd):
        self.port.write("%s\n" % cmd)
        expected = "%sOK" % cmd
        response = self.port.readline().rstrip("\n")
        assert response == expected, "Zebra response '%s' should be '%s'" %(response, expected)      
    
class zebraTool:
    def __init__(self, portstr):
        self.zebra = zebraConnection("/dev/ttyS0")
        self.errors = 0
        
    def uploadFile(self, fname):
        parser = ConfigParser()
        parser.read(fname)
        assert parser.has_section("regs"), "Cannot find reg section in %s" % fname
        for reg, value in parser.items("regs"):
            self.zebra.writeReg(reg, int(value))

    def doTest(self, bad="BAD", ok="OK", sleep=0.2):
        version = self.zebra.readReg("SYS_VER")                    
        assert version >= 0x20, "Can only firmware test zebras with 0x20 firmware or higher"

        # first disconnect all the outputs
        for name in self.zebra.regs.names():
            if name.startswith("OUT"):
                self.zebra.writeReg(name, "DISCONNECT")   
        
        # now do the front panel inputs
        for i, signal in enumerate(self.zebra.systemBus()):
            # test inputs against outputs in system bus
            match = re.match(r"IN(\d)", signal)
            if match and int(match.group(1)) < 5:
                # find corresponding output
                if signal == "IN4_CMP":
                    out = "OUT4_NIM"
                else:
                    out = signal.replace("IN", "OUT")
                print "%40s"%("Testing %s and %s..." % (signal, out)),
                # set output to soft input
                self.zebra.writeReg(out, "SOFT_IN1")
                ret = ok
                for expected in (1, 0):
                    # check if we get the input on the system bus                                    
                    # special for PECL
                    if signal == "IN4_PECL":
                        # Latch the PECL inputs
                        self.zebra.writeReg("GATE1_INP1", signal)
                        # Reset on SOFT_IN2
                        self.zebra.writeReg("GATE1_INP2", "SOFT_IN2")
                        # Reset gate                        
                        self.zebra.writeReg("SOFT_IN", 2)                        
                        # Send pulse
                        self.zebra.writeReg("SOFT_IN", expected)
                        # Get pulse
                        if (self.zebra.getStatus("GATE1") != expected):
                            ret = bad                                            
                    else:
                        self.zebra.writeReg("SOFT_IN", expected)                    
                        if (self.zebra.getStatus(signal) != expected):
                            ret = bad
                    # slow it down so we can see it
                    time.sleep(sleep)    
                self.zebra.writeReg(out, "DISCONNECT")
                if ret == "BAD":        
                    self.errors += 1
                print ret

        # now do the encoder inputs
        for i in range(5,9):
            # enable with soft in 2
            self.zebra.writeReg("OUT%d_CONN" % i, "SOFT_IN2")
            for suff in "ABZ":
                signal = "IN%d_ENC%s" % (i, suff)
                out = "OUT%d_ENC%s" % (i, suff)                
                print "%40s"%("Testing %s and %s..." % (signal, out)),
                self.zebra.writeReg(out, "SOFT_IN1")
                ret = ok
                for conn in (1,0):
                    for expected in (0,1):
                        # bit mask, conn is soft2, input is soft1
                        if conn == 0:
                            expected = 1
                        self.zebra.writeReg("SOFT_IN", 2*conn+expected)
                        if (self.zebra.getStatus(signal) != expected):
                            ret = bad  
                    time.sleep(sleep) 
                self.zebra.writeReg(out, "DISCONNECT")         
                if ret == "BAD":        
                    self.errors += 1                             
                print ret
            self.zebra.writeReg("OUT%d_CONN" % i, "DISCONNECT")
            
    def save(self):
        self.zebra.writeCommand("S")

if __name__=="__main__":
    tool = zebraTool("/dev/ttyS0")
    print "Downloading defaults..."
    tool.uploadFile(os.path.realpath(os.path.join(__file__, "..", "..", "..", "etc", "defaults.ini")))
    tool.save()        
    print "Doing hardware test with cables unplugged..."
    tool.doTest(bad="OK", ok="BAD")
    raw_input("Plug in shorting cables and press return when done...")
    tool.doTest()
    print "Test complete: %d Errors" % tool.errors

        
    

