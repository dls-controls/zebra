#!/bin/env dls-python2.6

from pkg_resources import require
require("dls_serial_sim")
from dls_serial_sim import serial_device, CreateSimulation
import time
from random import random

class zebra(serial_device):

    InTerminator = "\n"
    OutTerminator = "\n"    
    ui = None
    diagLevel = 4
    
    def __init__(self):    
        self.i = 0
        self.memory = {}
        for x in range(256):
            self.memory[x] = 0
        self.schedule(self.poll,10)              
    
    def poll(self): 
        self.i+=1
        return "~AA%04X" % self.i

    def reply(self, command):
        command = command.strip("\r")
        if command == "S":
            time.sleep(0.1)
            return "SOK"            
        elif command.startswith("R") and len(command) == 3:
            addr = int(command[1:3], 16)
            if addr in self.memory:
                ret = ""
                #if random() > 0.9:
                #    ret = "~R340000"+self.OutTerminator
                return ret+"R%02X%04X" % (addr, self.memory[addr])
            else:
                return "E1R%02X" % addr
        elif command.startswith("W") and len(command) == 7:
            addr = int(command[1:3], 16)
            value = int(command[3:7], 16)
            time.sleep(0.01)
            if addr in self.memory:
                self.memory[addr] = value
                return "W%02XOK" % addr
            else:
                return "E1W%02X" % addr
        else:
            return "E0"

if __name__=="__main__":
    CreateSimulation(zebra)
    raw_input()
