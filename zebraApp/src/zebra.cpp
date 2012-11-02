#include <stddef.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <epicsTime.h>
#include <epicsThread.h>
#include <epicsMutex.h>
#include <epicsString.h>
#include <epicsStdio.h>
#include <epicsMutex.h>
#include <epicsExport.h>
#include <iocsh.h>
#include "asynOctetSyncIO.h"
#include "asynCommonSyncIO.h"
#include "asynPortDriver.h"
#include "epicsThread.h"
#include "epicsMessageQueue.h"

/* This is the number of messages on our queue */
#define NQUEUE 50

/* The size of our transmit and receive buffers */
#define NBUFF 255

/* The interval in seconds between each poll of zebra */
#define POLL 0.1

/* This is a lookup table of string->register number */
struct reg_lookup {
    const char * str;
    int reg;
    int isMux;
};
static const struct reg_lookup reg_lookup[] = {
    /* AND4 gate */
    { "AND1_INV",        0x00,         0 },
    { "AND2_INV",        0x01,         0 },
    { "AND3_INV",        0x02,         0 },
    { "AND4_INV",        0x03,         0 },
    { "AND1_ENA",        0x04,         0 },
    { "AND2_ENA",        0x05,         0 },
    { "AND3_ENA",        0x06,         0 },
    { "AND4_ENA",        0x07,         0 },
    { "AND1_INP1",       0x08,         1 },
    { "AND1_INP2",       0x09,         1 },
    { "AND1_INP3",       0x0A,         1 },
    { "AND1_INP4",       0x0B,         1 },
    { "AND2_INP1",       0x0C,         1 },
    { "AND2_INP2",       0x0D,         1 },
    { "AND2_INP3",       0x0E,         1 },
    { "AND2_INP4",       0x0F,         1 },
    { "AND3_INP1",       0x10,         1 },
    { "AND3_INP2",       0x11,         1 },
    { "AND3_INP3",       0x12,         1 },
    { "AND3_INP4",       0x13,         1 },
    { "AND4_INP1",       0x14,         1 },
    { "AND4_INP2",       0x15,         1 },
    { "AND4_INP3",       0x16,         1 },
    { "AND4_INP4",       0x17,         1 },
    /* OR4 gate */
    { "OR1_INV",         0x18,         0 },
    { "OR2_INV",         0x19,         0 },
    { "OR3_INV",         0x1A,         0 },
    { "OR4_INV",         0x1B,         0 },
    { "OR1_ENA",         0x1C,         0 },
    { "OR2_ENA",         0x1D,         0 },
    { "OR3_ENA",         0x1E,         0 },
    { "OR4_ENA",         0x1F,         0 },
    { "OR1_INP1",        0x20,         1 },
    { "OR1_INP2",        0x21,         1 },
    { "OR1_INP3",        0x22,         1 },
    { "OR1_INP4",        0x23,         1 },
    { "OR2_INP1",        0x24,         1 },
    { "OR2_INP2",        0x25,         1 },
    { "OR2_INP3",        0x26,         1 },
    { "OR2_INP4",        0x27,         1 },
    { "OR3_INP1",        0x28,         1 },
    { "OR3_INP2",        0x29,         1 },
    { "OR3_INP3",        0x2A,         1 },
    { "OR3_INP4",        0x2B,         1 },
    { "OR4_INP1",        0x2C,         1 },
    { "OR4_INP2",        0x2D,         1 },
    { "OR4_INP3",        0x2E,         1 },
    { "OR4_INP4",        0x2F,         1 },
    /* Gate generator */
    { "GATE1_INP1",      0x30,         1 },
    { "GATE2_INP1",      0x31,         1 },
    { "GATE3_INP1",      0x32,         1 },
    { "GATE4_INP1",      0x33,         1 },
    { "GATE1_INP2",      0x34,         1 },
    { "GATE2_INP2",      0x35,         1 },
    { "GATE3_INP2",      0x36,         1 },
    { "GATE4_INP2",      0x37,         1 },
    /* Pulse divider */
    { "DIV1_DIVLO",      0x38,         0 },
    { "DIV1_DIVHI",      0x39,         0 },
    { "DIV2_DIVLO",      0x3A,         0 },
    { "DIV2_DIVHI",      0x3B,         0 },
    { "DIV3_DIVLO",      0x3C,         0 },
    { "DIV3_DIVHI",      0x3D,         0 },
    { "DIV4_DIVLO",      0x3E,         0 },
    { "DIV4_DIVHI",      0x3F,         0 },
    { "DIV1_INP",        0x40,         1 },
    { "DIV2_INP",        0x41,         1 },
    { "DIV3_INP",        0x42,         1 },
    { "DIV4_INP",        0x43,         1 },
    /* Pulse generator */
    { "PULSE1_DLY",      0x44,         0 },
    { "PULSE2_DLY",      0x45,         0 },
    { "PULSE3_DLY",      0x46,         0 },
    { "PULSE4_DLY",      0x47,         0 },
    { "PULSE1_WID",      0x48,         0 },
    { "PULSE2_WID",      0x49,         0 },
    { "PULSE3_WID",      0x4A,         0 },
    { "PULSE4_WID",      0x4B,         0 },
    { "PULSE1_PRE",      0x4C,         0 },
    { "PULSE2_PRE",      0x4D,         0 },
    { "PULSE3_PRE",      0x4E,         0 },
    { "PULSE4_PRE",      0x4F,         0 },
    { "PULSE1_INP",      0x50,         1 },
    { "PULSE2_INP",      0x51,         1 },
    { "PULSE3_INP",      0x52,         1 },
    { "PULSE4_INP",      0x53,         1 },
    /* Output multiplexer select */
    { "OUT1_TTL",        0x60,         1 },
    { "OUT1_NIM",        0x61,         1 },
    { "OUT1_LVDS",       0x62,         1 },
    { "OUT2_TTL",        0x63,         1 },
    { "OUT2_NIM",        0x64,         1 },
    { "OUT2_LVDS",       0x65,         1 },
    { "OUT3_TTL",        0x66,         1 },
    { "OUT3_OC",         0x67,         1 },
    { "OUT3_LVDS",       0x68,         1 },
    { "OUT4_TTL",        0x69,         1 },
    { "OUT4_NIM",        0x6A,         1 },
    { "OUT4_PECL",       0x6B,         1 },
    { "OUT5_ENCA",       0x6C,         1 },
    { "OUT5_ENCB",       0x6D,         1 },
    { "OUT5_ENCZ",       0x6E,         1 },
    { "OUT6_ENCA",       0x6F,         1 },
    { "OUT6_ENCB",       0x70,         1 },
    { "OUT6_ENCZ",       0x71,         1 },
    { "OUT7_ENCA",       0x72,         1 },
    { "OUT7_ENCB",       0x73,         1 },
    { "OUT7_ENCZ",       0x74,         1 },
    { "OUT8_ENCA",       0x75,         1 },
    { "OUT8_ENCB",       0x76,         1 },
    { "OUT8_ENCZ",       0x77,         1 },
    /* Soft input register */
    { "SYS_RESET",       0x7E,         0 },
    { "SOFT_IN",         0x7F,         0 },
    /* Position compare logic blocks */
#define PC_OFF 0x80
    /* Load position counters */
    { "POS1_SETLO",      0x00 + PC_OFF, 0 },
    { "POS1_SETHI",      0x01 + PC_OFF, 0 },
    { "POS2_SETLO",      0x02 + PC_OFF, 0 },
    { "POS2_SETHI",      0x03 + PC_OFF, 0 },
    { "POS3_SETLO",      0x04 + PC_OFF, 0 },
    { "POS3_SETHI",      0x05 + PC_OFF, 0 },
    { "POS4_SETLO",      0x06 + PC_OFF, 0 },
    { "POS4_SETHI",      0x07 + PC_OFF, 0 },
    /* Select position counter 1,2,3,4,Sum */
    { "PC_ENC",          0x08 + PC_OFF, 0 },
    /* Timestamp clock prescaler */
    { "PC_TSPRE",        0x09 + PC_OFF, 0 },
    /* Arm input Soft,External */
    { "PC_ARM_SEL",      0x0A + PC_OFF, 0 },
    /* Soft arm and disarm commands */
    { "PC_ARM",          0x0B + PC_OFF, 0 },
    { "PC_DISARM",       0x0C + PC_OFF, 0 },
    /* Gate input Position,Time,External */
    { "PC_GATE_SEL",     0x0D + PC_OFF, 0 },
    /* Gate parameters */
    { "PC_GATE_STARTLO", 0x0E + PC_OFF, 0 },
    { "PC_GATE_STARTHI", 0x0F + PC_OFF, 0 },
    { "PC_GATE_WIDLO",   0x10 + PC_OFF, 0 },
    { "PC_GATE_WIDHI",   0x11 + PC_OFF, 0 },
    { "PC_GATE_NGATELO", 0x12 + PC_OFF, 0 },
    { "PC_GATE_NGATEHI", 0x13 + PC_OFF, 0 },
    { "PC_GATE_STEPLO",  0x14 + PC_OFF, 0 },
    { "PC_GATE_STEPHI",  0x15 + PC_OFF, 0 },
    /* Pulse input Position,Time,External */
    { "PC_PULSE_SEL",    0x16 + PC_OFF, 0 },
    /* Pulse parameters */
    { "PC_PULSE_DLYLO",  0x17 + PC_OFF, 0 },
    { "PC_PULSE_DLYHI",  0x18 + PC_OFF, 0 },
    { "PC_PULSE_WIDLO",  0x19 + PC_OFF, 0 },
    { "PC_PULSE_WIDHI",  0x1A + PC_OFF, 0 },
    { "PC_PULSE_STEPLO", 0x1B + PC_OFF, 0 },
    { "PC_PULSE_STEPHI", 0x1C + PC_OFF, 0 },
    /* External inputs for Arm, Gate, Pulse */
    { "PC_ARM_INP",      0x1D + PC_OFF, 1 },
    { "PC_GATE_INP",     0x1E + PC_OFF, 1 },
    { "PC_PULSE_INP",    0x1F + PC_OFF, 1 },
    /* System settings */
#define SYS_OFF 0xF0
    { "SYS_VER",         0x00 + SYS_OFF, 0 },
    /* Status values we should poll */
    { "SYS_STAT1LO",     0x02 + SYS_OFF, 0 },
    { "SYS_STAT1HI",     0x03 + SYS_OFF, 0 },
    { "SYS_STAT2LO",     0x04 + SYS_OFF, 0 },
    { "SYS_STAT2HI",     0x05 + SYS_OFF, 0 },
};
#define NREGS sizeof(reg_lookup) / sizeof(struct reg_lookup)

/* These are the entries on the system bus */
static const char *bus_lookup[] = {
	"CLOCK_1MHZ",
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
    "IN6_ENCA",
    "IN6_ENCB",
    "IN6_ENCZ",
    "IN7_ENCA",
    "IN7_ENCB",
    "IN7_ENCZ",
    "IN8_ENCA",
    "IN8_ENCB",
    "IN8_ENCZ",
    "SOFT_IN1",
    "SOFT_IN2",
    "SOFT_IN3",
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
    "52",
    "53",
    "54",
    "55",
    "PC_ARM",
    "PC_GATE",
    "PC_PULSE",
    "59",
    "60",
    "61",
    "62",
    "63",
};
#define NSYSBUS 64

class zebra : public asynPortDriver
{
public:
    zebra(const char *portName, const char* serialPortName);

    /* These are the methods that we override from asynPortDriver */
    virtual asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);

    /** These should be private, but get called from C, so must be public */
    void pollTask();
    void readTask();

protected:
    /* These are helper methods for the class */
    asynStatus sendReceive(char *txBuffer, int txSize, char** rxBuffer);
    asynStatus setReg(int reg, int value);
    asynStatus getReg(int reg, int *value);
    asynStatus storeFlash();

protected:
    /* Parameter indices */
    #define FIRST_PARAM zebraIsConnected
    int zebraIsConnected;           // int32 read  - is zebra connected?    
    int zebraStore;                 // int32 write - store config to flash
    int zebraSysBus1;               // string read - system bus key first half
    int zebraSysBus2;               // string read - system bus key second half
    int zebraError;                 // int32 read  - error
    #define LAST_PARAM zebraError
    int paramReg[NREGS*2];
    #define NUM_PARAMS (&LAST_PARAM - &FIRST_PARAM + 1) + NREGS*2

private:
    asynUser     *pasynUser;
    asynCommon   *pasynCommon;
    void         *pcommonPvt;
    asynOctet    *pasynOctet;
    void         *octetPvt;
    asynDrvUser  *pasynDrvUser;
    void         *drvUserPvt;
    epicsMessageQueueId msgQId;
};

/* C function to call poll task from epicsThreadCreate */
static void pollTaskC(void *userPvt) {
    zebra *pPvt = (zebra *)userPvt;
    pPvt->pollTask();
}

/* C function to call new message from  task from epicsThreadCreate */
static void readTaskC(void *userPvt) {
    zebra *pPvt = (zebra *)userPvt;
    pPvt->readTask();
}

/* Constructor */
zebra::zebra(const char* portName, const char* serialPortName)
    : asynPortDriver(portName, 1 /*maxAddr*/, NUM_PARAMS,
        asynInt32Mask | asynOctetMask | asynDrvUserMask,
        asynInt32Mask | asynOctetMask,
        ASYN_CANBLOCK, /*ASYN_CANBLOCK=1, ASYN_MULTIDEVICE=0 */
        1, /*autoConnect*/ 0, /*default priority */ 0 /*default stack size*/ )
{
    asynStatus status = asynSuccess;
    asynInterface *pasynInterface;
	char buffer[6400]; /* 100 chars per element on sys bus is overkill... */
	char str[100];

    /* create the parameters */
    createParam("ISCONNECTED", asynParamInt32, &zebraIsConnected);
    setIntegerParam(zebraIsConnected, 0);
    createParam("STORE",       asynParamInt32, &zebraStore);
    createParam("ERROR",       asynParamInt32, &zebraError);
    
    /* create a system bus key */
    createParam("SYS_BUS1",     asynParamOctet, &zebraSysBus1);
    buffer[0] = '\0';
    for (unsigned int i=0; i<NSYSBUS/2; i++) {
    	sprintf(str, "%d: %s\n", i, bus_lookup[i]);
    	strcat(buffer, str);
    }
    setStringParam(zebraSysBus1, buffer);
    createParam("SYS_BUS2",     asynParamOctet, &zebraSysBus2);
    buffer[0] = '\0';
    for (unsigned int i=NSYSBUS/2; i<NSYSBUS; i++) {
    	sprintf(str, "%d: %s\n", i, bus_lookup[i]);
    	strcat(buffer, str);
    }
    setStringParam(zebraSysBus2, buffer);

    /* create parameters for registers */
    for (unsigned int i=0; i<NREGS; i++) {
        createParam(reg_lookup[i].str, asynParamInt32, &paramReg[i]);
    }

    /* create parameters for register string values */
    for (unsigned int i=0; i<NREGS; i++) {
        sprintf(buffer, "%s_STR", reg_lookup[i].str);
        createParam(buffer, asynParamOctet, &paramReg[i+NREGS]);
    }

    /* Create a message queue to hold completed messages */
    this->msgQId = epicsMessageQueueCreate(NQUEUE, sizeof(char*));
    if (!this->msgQId) {
        printf("epicsMessageQueueCreate failure\n");
        return;
    }
    
    /* Connect to the device port */
    /* Copied from asynOctecSyncIO->connect */
	pasynUser = pasynManager->createAsynUser(0,0);
	status = pasynManager->connectDevice(pasynUser, serialPortName, 0);
	if (status != asynSuccess) {
		printf("Connect failed, port=%s, error=%d\n", serialPortName, status);
		return;
	}
	pasynInterface = pasynManager->findInterface(pasynUser, asynCommonType, 1);
	if (!pasynInterface) {
	   epicsSnprintf(pasynUser->errorMessage,pasynUser->errorMessageSize,
		   "%s interface not supported", asynCommonType);
	   return;
	}
	pasynCommon = (asynCommon *)pasynInterface->pinterface;
	pcommonPvt = pasynInterface->drvPvt;
	pasynInterface = pasynManager->findInterface(pasynUser, asynOctetType, 1);
	if (!pasynInterface) {
	   epicsSnprintf(pasynUser->errorMessage,pasynUser->errorMessageSize,
		   "%s interface not supported", asynOctetType);
	   return;
	}
	pasynOctet = (asynOctet *)pasynInterface->pinterface;
	octetPvt = pasynInterface->drvPvt;

	/* Set EOS and flush */
	pasynOctet->flush(octetPvt, pasynUser);
    pasynOctet->setInputEos(octetPvt, pasynUser, "\n", 1);
    pasynOctet->setOutputEos(octetPvt, pasynUser, "\n", 1);

    /* Create the thread that reads from the device  */
    if (epicsThreadCreate("ZebraReadTask",
                          epicsThreadPriorityMedium,
                          epicsThreadGetStackSize(epicsThreadStackMedium),
                          (EPICSTHREADFUNC)readTaskC,
                          this) == NULL) {
        printf("epicsThreadCreate failure for reading task\n");
        return;
    }

    /* Create the thread that polls the device  */
    if (epicsThreadCreate("ZebraPollTask",
                          epicsThreadPriorityMedium,
                          epicsThreadGetStackSize(epicsThreadStackMedium),
                          (EPICSTHREADFUNC)pollTaskC,
                          this) == NULL) {
        printf("epicsThreadCreate failure for polling task\n");
        return;
    }
}

/* This is the function that will be run for the read thread */
void zebra::readTask() {
    asynStatus status = asynSuccess;
    // Create the receive buffer
    asynUser *pasynUserRead = pasynManager->duplicateAsynUser(pasynUser, 0, 0);
    char *rxBuffer;
    size_t nBytesIn;
    int eomReason;

    while (true) {
    	pasynUserRead->timeout = 100000000000000.0;
    	rxBuffer = (char *) malloc(NBUFF);
        status = pasynOctet->read(octetPvt, pasynUserRead, rxBuffer, NBUFF,
            &nBytesIn, &eomReason);
        if (status) {
        	printf("Port not connected\n");
        	free(rxBuffer);
        	epicsThreadSleep(POLL);
        } else if (eomReason & ASYN_EOM_EOS) {
        	if (rxBuffer[0] == '~') {
            	printf("Got an interrupt '%.*s'\n", (int)nBytesIn, rxBuffer);
        		free(rxBuffer);
        	} else {
        		//printf("Got a message '%.*s' %d\n", (int)nBytesIn, rxBuffer, eomReason);
        		rxBuffer[nBytesIn] = '\0';
        		if (epicsMessageQueueTrySend(this->msgQId,
        				&rxBuffer,
        				sizeof(&rxBuffer))) {
        			printf("Message queue full, dropped message\n");
        			free(rxBuffer);
        		}
            }
        } else {
        	printf("Bad message '%.*s'\n", (int)nBytesIn, rxBuffer);
        	free(rxBuffer);
        }
    }
}

/* This is the function that will be run for the poll thread */
void zebra::pollTask() {
    int value;
    unsigned int i, poll=0, sys=0, dosys=1;
    asynStatus status = asynSuccess;
    while (true) {
        // poll registers in turn, system status more often
    	if (dosys) {
    		i = NREGS - 4 + sys++;
	    	if (sys >= 4) sys = 0;
    	} else {
    		i = poll++;
    		if (poll >= NREGS - 4) poll = 0;
    	}
    	dosys = ! dosys;
		this->lock();
		// Get the register value
		status = this->getReg(reg_lookup[i].reg, &value);
		if (status) {
			setIntegerParam(zebraIsConnected, 0);
			printf("pollTask: zebra not connected\n");
		} else {
			setIntegerParam(zebraIsConnected, 1);
			setIntegerParam(this->paramReg[i], value);
			if (reg_lookup[i].isMux && value < NSYSBUS) {
				setStringParam(this->paramReg[i+NREGS],bus_lookup[value]);
			}
		}
		callParamCallbacks();
		this->unlock();
		// Wait for POLL seconds
		epicsThreadSleep(POLL);
    }
}        

/* This function gets the value of a register
   called with the lock taken */
asynStatus zebra::getReg(int i, int *value) {
    asynStatus status = asynSuccess;
    // Create the transmit buffer
    char txBuffer[NBUFF], *rxBuffer;
    int txSize, reg;
    // Send a write
    txSize = sprintf(txBuffer, "R%02X", i);
    status = this->sendReceive(txBuffer, txSize, &rxBuffer);
    // If we got a response
    if(status == asynSuccess) {
        if (sscanf(rxBuffer, "R%02X%04X", &reg, value)) {
        	if (reg == i) {
        		// Good message, everything ok
        		status = asynSuccess;
        	} else {
        		printf("Expected reg %02X got %02X\n", i, reg);
        		status = asynError;
        	}
		} else {
			printf("Unrecognised message '%s'\n", rxBuffer);
			status = asynError;
		}
		free(rxBuffer);
    }
    return status;
}


/* This function sets the value of a register
   called with the lock taken */
asynStatus zebra::setReg(int i, int value) {
    asynStatus status = asynSuccess;
    // Create the transmit buffer
    char txBuffer[NBUFF], *rxBuffer;
    int txSize, reg;
    // Send a write
    txSize = sprintf(txBuffer, "W%02X%04X", i, value & 0xFFFF);
    status = this->sendReceive(txBuffer, txSize, &rxBuffer);
    // If we got a response
    if(status == asynSuccess) {
        if (sscanf(rxBuffer, "W%02XOK", &reg)) {
        	if (reg == i) {
        		// Good message, everything ok
        		status = asynSuccess;
        	} else {
        		printf("Expected reg %02X got %02X\n", i, reg);
        		status = asynError;
        	}
		} else {
			printf("Unrecognised message '%s'\n", rxBuffer);
			status = asynError;
		}
		free(rxBuffer);
    }
    return status;
}

/* This function stores to flash
   called with the lock taken */
asynStatus zebra::storeFlash() {
    asynStatus status = asynSuccess;
    // Create the transmit buffer
    char txBuffer[NBUFF], *rxBuffer;
    int txSize;
    // Send a write
    txSize = sprintf(txBuffer, "S");
    status = this->sendReceive(txBuffer, txSize, &rxBuffer);
    // If we got a response
    if(status == asynSuccess) {
		if (sscanf(rxBuffer, "SOK")) {
			// Good message, everything ok
			status = asynSuccess;
		} else {
			printf("Unrecognised message '%s'\n", rxBuffer);
			status = asynError;
		}
		free(rxBuffer);
    }
    return status;
}

/* Send receive helper function
 * called with lock taken, caller responsible for freeing rxBuffer if return status=asynSuccess
 */
asynStatus zebra::sendReceive(char *txBuffer, int txSize, char** rxBuffer) {
	asynStatus status = asynSuccess;
	size_t nBytesOut;
    pasynUser->timeout = POLL;
    // If there are any responses on the queue they must be junk
    while (epicsMessageQueuePending(this->msgQId)) {
    	epicsMessageQueueReceive(this->msgQId, rxBuffer, sizeof(rxBuffer));
		printf("Junk message in buffer '%s'\n", *rxBuffer);
    	free(*rxBuffer);
    }
    status = pasynOctet->write(octetPvt, pasynUser, txBuffer, txSize, &nBytesOut);
    if(status == asynSuccess) {
        // wait for a response on the message queue
    	if (epicsMessageQueueReceiveWithTimeout(this->msgQId, rxBuffer, sizeof(rxBuffer), POLL) > 0) {
       		status = asynSuccess;
    	} else {
    		printf("Zebra not connected, no response\n");
    		status = asynError;
    	}
    }
    return status;
}

/** Called when asyn clients call pasynInt32->write().
  * This function performs actions for some parameters
  * For all parameters it sets the value in the parameter library and calls any registered callbacks..
  * \param[in] pasynUser pasynUser structure that encodes the reason and address.
  * \param[in] value Value to write. */
asynStatus zebra::writeInt32(asynUser *pasynUser, epicsInt32 value)
{
    /* Base class does most of the work, including writing to the parameter
     * library and doing call backs */
	asynStatus status = asynError;

    /* Any work we need to do */
    int parameter = pasynUser->reason;
    if(parameter >= this->paramReg[0] && parameter < this->paramReg[NREGS-1]) {
    	int i = parameter-this->paramReg[0];
    	int reg = reg_lookup[i].reg;
        printf("Write reg %d\n", reg);
    	setIntegerParam(parameter, value);
    	status = this->setReg(reg, value);
    	if (status == asynSuccess) {
    		status = this->getReg(reg, &value);
    		if (status == asynSuccess) {
				setIntegerParam(parameter, value);
				if (reg_lookup[i].isMux && value < NSYSBUS) {
					printf("Write reg %d isMux\n", reg);
					setStringParam(parameter+NREGS, bus_lookup[value]);
				}
    		}
    	}
    } else if (parameter == zebraStore) {
    	status = this->storeFlash();
    }
    return status;
}

/** Configuration command, called directly or from iocsh */
extern "C" int zebraConfig(const char *portName, const char* serialPortName)
{
    new zebra(portName, serialPortName);
    return(asynSuccess);
}

/** Code for iocsh registration */
static const iocshArg zebraConfigArg0 = {"Port name", iocshArgString};
static const iocshArg zebraConfigArg1 = {"Serial port name", iocshArgString};
static const iocshArg* const zebraConfigArgs[] =  {&zebraConfigArg0, &zebraConfigArg1};
static const iocshFuncDef configzebra = {"zebraConfig", 2, zebraConfigArgs};
static void configzebraCallFunc(const iocshArgBuf *args)
{
    zebraConfig(args[0].sval, args[1].sval);
}

static void zebraRegister(void)
{
    iocshRegister(&configzebra, configzebraCallFunc);
}

extern "C" { epicsExportRegistrar(zebraRegister); }

