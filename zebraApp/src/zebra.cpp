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
#include "ini.h"
#include "zebraRegs.h"

/* This is the number of messages on our queue */
#define NQUEUE 1000

/* The size of our transmit and receive buffers,
 * max filename length and string param buffers */
#define NBUFF 255

/* The timeout waiting for a response from zebra */
#define TIMEOUT 0.5

/* The last FASTREGS should be polled quickly */
#define FASTREGS 6

/* This is the number of waveforms to store */
#define NARRAYS 10

/* This is the number of filtered waveforms to allow */
#define NFILT 4

/* We want to block while waiting on an asyn port forever.
 * Unfortunately putting 0 or a large number causes it to
 * poll and take up lots of CPU. This number seems to work
 * and causes it to block for a reasonably long time (in seconds)
 */
#define LONGWAIT 1000.0

/* The counter in the FPGA is a 32 bit number which increments at
 * 50MHz divided by a prescaler.
 * We set the prescaler to 50 for time units of ms or 50000 for time units
 * of s. This means that each increment of the counter is 0.001 time units.
 * When the counter rolls over we need to add 2^32*0.001 time units, which
 * is this number
 */
#define COUNTERROLLOVER 4294967.296

/* useful defines for converting to and from asyn parameters and zebra regs */
#define NREGS (sizeof(reg_lookup)/sizeof(struct reg))
#define PARAM2REG(/*int*/param) &(reg_lookup[param-this->zebraReg[0]])
#define REG2PARAM(/*reg**/r) (this->zebraReg[0]+(int)(r-reg_lookup))
#define REG2PARAMSTR(/*reg**/r) ((int) (REG2PARAM(r)+NREGS))
#define NSYSBUS (sizeof(bus_lookup)/sizeof(char *))

class zebra : public asynPortDriver
{
public:
    zebra(const char *portName, const char* serialPortName, int maxPts);

    /* These are the methods that we override from asynPortDriver */
    virtual asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);

    /** These should be private, but get called from C, so must be public */
    void pollTask();
    void readTask();
    int configLine(const char* section, const char* name, const char* value);

protected:
    /* These are helper methods for the class */
    asynStatus sendReceive(char *txBuffer, int txSize, char** rxBuffer);
    asynStatus setReg(const reg *r, int value);
    asynStatus getReg(const reg *r, int *value);
    asynStatus storeFlash();
    asynStatus configRead(const char* str);
    asynStatus configWrite(const char* str);
    asynStatus callbackWaveforms();
    asynStatus writeParam(int param, int value);

protected:
    /* Parameter indices */
    #define FIRST_PARAM zebraIsConnected
    int zebraIsConnected;           // int32 read  - is zebra connected?
    int zebraPCTime;                // float64array read - position compare timestamps
    int zebraArrayUpdate;           // int32 write - update arrays
    int zebraArrayAcq;              // int32 read - when data is acquiring
    int zebraNumDown;               // int32 read - number of data points downloaded
    int zebraSysBus1;               // string read - system bus key first half
    int zebraSysBus2;               // string read - system bus key second half
    int zebraStore;                 // int32 write - store config to flash
    int zebraConfigFile;            // charArray write - filename to read/write config to
    int zebraConfigRead;            // int32 write - read config from filename
    int zebraConfigWrite;           // int32 write - write config to filename
    int zebraConfigStatus;          // int32 read - config status message
    #define LAST_PARAM zebraConfigStatus
    int zebraCapArrays[NARRAYS];    // int32array read - position compare capture array
    int zebraCapLast[NARRAYS];      // int32 read - last captured value
    int zebraFiltArrays[NFILT];     // int32array read - position compare sys bus filtered
    int zebraFiltSel[NFILT];        // int32 read/write - which index of system bus to select for zebraFiltArrays
    int zebraFiltSelStr[NFILT];     // string read - the name of the entry in the system bus
    int zebraReg[NREGS*2];          // int32 read/write - all zebra params in reg_lookup
    #define NUM_PARAMS (&LAST_PARAM - &FIRST_PARAM + 1) + NARRAYS*2 + NFILT*3 + NREGS*2

private:
    asynUser     *pasynUser;
    asynCommon   *pasynCommon;
    void         *pcommonPvt;
    asynOctet    *pasynOctet;
    void         *octetPvt;
    asynDrvUser  *pasynDrvUser;
    void         *drvUserPvt;
    epicsMessageQueueId msgQId, intQId;
    int          maxPts, currPt;
    int          *capArrays[NARRAYS];
    int          *filtArrays[NFILT];
    double       *PCTime, tOffset;
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

/* C function to call new message from  task from epicsThreadCreate */
static int configLineC(void* userPvt, const char* section, const char* name, const char* value) {
    zebra *pPvt = (zebra *)userPvt;
    return pPvt->configLine(section, name, value);
}

/* Constructor */
zebra::zebra(const char* portName, const char* serialPortName, int maxPts)
    : asynPortDriver(portName, 1 /*maxAddr*/, NUM_PARAMS,
        asynInt32ArrayMask | asynFloat64ArrayMask | asynInt32Mask | asynFloat64Mask | asynOctetMask | asynDrvUserMask,
        asynInt32ArrayMask | asynFloat64ArrayMask | asynInt32Mask | asynFloat64Mask | asynOctetMask,
        ASYN_CANBLOCK, /*ASYN_CANBLOCK=1, ASYN_MULTIDEVICE=0 */
        1, /*autoConnect*/ 0, /*default priority */ 0 /*default stack size*/ )
{
    asynStatus status = asynSuccess;
    asynInterface *pasynInterface;
    char buffer[6400]; /* 100 chars per element on sys bus is overkill... */
    char str[NBUFF];
    const reg *r;
    
    /* For position compare results */
    this->maxPts = maxPts;
    this->currPt = 0;

    /* Connection status */
    createParam("ISCONNECTED", asynParamInt32, &zebraIsConnected);
    setIntegerParam(zebraIsConnected, 0);

    /* a parameter that controls polling of waveforms */
    createParam("ARRAY_UPDATE", asynParamInt32, &zebraArrayUpdate);

    /* and one that says when we are acquiring arrays */
    createParam("ARRAY_ACQ", asynParamInt32, &zebraArrayAcq);
    setIntegerParam(zebraArrayAcq, 0);

    /* a parameter showing the number of points we have downloaded */
    createParam("PC_NUM_DOWN", asynParamInt32, &zebraNumDown);
    setIntegerParam(zebraNumDown, 0);

    /* create a system bus key */
    createParam("SYS_BUS1", asynParamOctet, &zebraSysBus1);
    buffer[0] = '\0';
    for (unsigned int i=0; i<NSYSBUS/2; i++) {
        epicsSnprintf(str, NBUFF, "%2d: %s\n", i, bus_lookup[i]);
        strcat(buffer, str);
    }
    setStringParam(zebraSysBus1, buffer);
    createParam("SYS_BUS2",     asynParamOctet, &zebraSysBus2);
    buffer[0] = '\0';
    for (unsigned int i=NSYSBUS/2; i<NSYSBUS; i++) {
        epicsSnprintf(str, NBUFF, "%2d: %s\n", i, bus_lookup[i]);
        strcat(buffer, str);
    }
    setStringParam(zebraSysBus2, buffer);

    /* a parameter for a store to flash request */
    createParam("STORE", asynParamInt32, &zebraStore);

    /* parameters for filename reading/writing config */
    createParam("CONFIG_FILE", asynParamOctet, &zebraConfigFile);
    createParam("CONFIG_WRITE", asynParamInt32, &zebraConfigWrite);
    createParam("CONFIG_READ", asynParamInt32, &zebraConfigRead);
    createParam("CONFIG_STATUS", asynParamOctet, &zebraConfigStatus);

    /* position compare time array */
    createParam("PC_TIME", asynParamFloat64Array, &zebraPCTime);
    this->PCTime = (double *)calloc(maxPts, sizeof(double));

    /* create the position compare arrays */
    for (int a=0; a<NARRAYS; a++) {
        epicsSnprintf(str, NBUFF, "PC_CAP%d", a+1);
        createParam(str, asynParamInt32Array, &zebraCapArrays[a]);
        this->capArrays[a] = (int *)calloc(maxPts, sizeof(int));
    }
    
    /* create the last captured interrupt values */
    for (int a=0; a<NARRAYS; a++) {
        epicsSnprintf(str, NBUFF, "PC_CAP%d_LAST", a+1);
        createParam(str, asynParamFloat64, &zebraCapLast[a]);
    }

    /* create filter arrays */
    for (int a=0; a<NFILT; a++) {
        epicsSnprintf(str, NBUFF, "PC_FILT%d", a+1);
        createParam(str, asynParamInt32Array, &zebraFiltArrays[a]);
        this->filtArrays[a] = (int *)calloc(maxPts, sizeof(int));
    }

    /* create values that we can use to filter one element on the system bus with */
    /* NOTE: separate for loop so we get values for params we can do arithmetic with */
    for (int a=0; a<NFILT; a++) {
        epicsSnprintf(str, NBUFF, "PC_FILTSEL%d", a+1);
        createParam(str, asynParamInt32, &zebraFiltSel[a]);
        setIntegerParam(zebraFiltSel[a], 0);
    }

    /* create lookups of string values of these string selects */
    /* NOTE: separate for loop so we get values for params we can do arithmetic with */
    for (int a=0; a<NFILT; a++) {
        epicsSnprintf(str, NBUFF, "PC_FILTSEL%d_STR", a+1);
        createParam(str, asynParamOctet, &zebraFiltSelStr[a]);
        // Check our filter string lookup calcs will work
        assert(zebraFiltSelStr[a] == zebraFiltSel[a] + NFILT);
        setStringParam(zebraFiltSelStr[a], bus_lookup[0]);
    }

    /* create parameters for registers */        
    for (unsigned int i=0; i<NREGS; i++) {
        r = &(reg_lookup[i]);
        createParam(r->str, asynParamInt32, &zebraReg[i]);
        // Check our param -> reg lookup and inverse will work
        assert(r == PARAM2REG(zebraReg[i]));
        assert(REG2PARAM(r) == zebraReg[i]);
    }
    
    /* create parameters for register string values, these are lookups
       of the string values of mux registers from the system bus */
    /* NOTE: separate for loop so we get values for params we can do arithmetic with in REG2PARAMSTR */
    for (unsigned int i=0; i<NREGS; i++) {        
        r = &(reg_lookup[i]);  
        epicsSnprintf(str, NBUFF, "%s_STR", r->str);
        createParam(str, asynParamOctet, &zebraReg[i+NREGS]);
        // Check our reg -> param string lookup will work
        assert(REG2PARAMSTR(r) == zebraReg[i+NREGS]);
    }

    /* Create a message queue to hold completed messages and interrupts */
    this->msgQId = epicsMessageQueueCreate(NQUEUE, sizeof(char*));
    if (!this->msgQId) {
        printf("epicsMessageQueueCreate failure\n");
        return;
    }
    this->intQId = epicsMessageQueueCreate(NQUEUE, sizeof(char*));
    if (!this->intQId) {
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
    char *rxBuffer;
    size_t nBytesIn;
    int eomReason;
    epicsMessageQueueId q;
    asynStatus status = asynSuccess;
    asynUser *pasynUserRead = pasynManager->duplicateAsynUser(pasynUser, 0, 0);

    while (true) {
        pasynUserRead->timeout = LONGWAIT;
        /* Malloc some data to put the reply from zebra. This is freed if there is an
         * error, otherwise it is put on a queue, and the receiving thread should free it
         */
        rxBuffer = (char *) malloc(NBUFF);
        status = pasynOctet->read(octetPvt, pasynUserRead, rxBuffer, NBUFF-1,
            &nBytesIn, &eomReason);
        if (status) {
            printf("Port not connected\n");
            free(rxBuffer);
            epicsThreadSleep(TIMEOUT);
        } else if (eomReason & ASYN_EOM_EOS) {
            // Replace the terminator with a null so we can use it as a string
              rxBuffer[nBytesIn] = '\0';         
              // This is an interrupt, send it to the interrupt queue   
            if (rxBuffer[0] == 'P') {                
                //printf("Got an interrupt '%s' %d\n", rxBuffer, eomReason);
                q = this->intQId;
            // This a zebra response to a command, send it to the message queue
            } else {
                //printf("Got a message '%s' %d\n", rxBuffer, eomReason);
                q = this->msgQId;
            }
            if (epicsMessageQueueTrySend(q,    &rxBuffer, sizeof(&rxBuffer)) != 0) {
                printf("Message queue full, dropped message\n");
                free(rxBuffer);
            }                                        
        } else {
            printf("Bad message '%.*s'\n", (int)nBytesIn, rxBuffer);
            free(rxBuffer);
        }
    }
}

/* This is the function that will be run for the poll thread */
void zebra::pollTask() {
    int value, cap, param, incr;
    unsigned int time;
    const reg *r;
    unsigned int i, poll=0, sys=0, dosys=1;
    char *rxBuffer, *ptr;
    epicsTimeStamp start, end;
    asynStatus status = asynSuccess;
    // Wait until port is up
    epicsThreadSleep(1.0);
    while (true) {
        // poll registers in turn, system status more often
        epicsTimeGetCurrent(&start);
        if (dosys) {
            i = NREGS - FASTREGS + sys++;
            if (sys >= FASTREGS) sys = 0;
        } else {
            i = poll++;
            if (poll >= NREGS - FASTREGS) poll = 0;
        }
        dosys = ! dosys;        
        /* Get the reg pointer */
        r = &(reg_lookup[i]);
        // Get the register value from zebra
        this->lock();
        status = this->getReg(r, &value);
        // set the param value
        if (status) {
            setIntegerParam(zebraIsConnected, 0);
            printf("pollTask: zebra not connected %s\n", r->str);
        } else {
            setIntegerParam(zebraIsConnected, 1);
            // If this is PC_NUM_CAPLO and it has rolled over, then trigger a PC_NUM_CAP_HI update
            if (strcmp(r->str, "PC_NUM_CAPLO")==0) {
                int lastval, hival;
                getIntegerParam(REG2PARAM(r), &lastval);
                if (value < lastval) {
                    //printf("Rollover!\n");
                    findParam("PC_NUM_CAPHI", &param);
                    this->getReg(PARAM2REG(param), &hival);
                    setIntegerParam(param, hival);
                }
            }
            setIntegerParam(REG2PARAM(r), value);
            // If it is a mux, set the string representation from the system bus
            if (r->type == regMux && value >= 0 && (unsigned int) value < NSYSBUS) {
                setStringParam(REG2PARAMSTR(r), bus_lookup[value]);
            }
        }
        // If there are any interrupts, service them
        while (epicsMessageQueuePending(this->intQId) > 0) {
            epicsMessageQueueReceive(this->intQId, &rxBuffer, sizeof(&rxBuffer));
            if (strcmp(rxBuffer, "PR") == 0) {
                // This is zebra telling us to reset our buffers
                this->currPt = 0;
                this->tOffset = 0.0;
                this->callbackWaveforms();       
                setIntegerParam(zebraArrayAcq, 1);                         
                // reset num cap
                findParam("PC_NUM_CAPLO", &param);
                setIntegerParam(param, 0);
                findParam("PC_NUM_CAPHI", &param);
                setIntegerParam(param, 0);
            } else if (strcmp(rxBuffer, "PX") == 0) {
                // This is zebra there is no more data
                this->callbackWaveforms();
                setIntegerParam(zebraArrayAcq, 0);
            } else {
                // This is a data buffer
                ptr = rxBuffer;
                // First get time
                if (sscanf(rxBuffer, "P%08X%n", &time, &incr) == 1) {
                    //printf("Time %X ", time);
                    // put time in time units (s or ms based on TS_PRE)
                    this->PCTime[this->currPt] = time / 1000.0 + this->tOffset;
                    if (this->currPt > 0 && this->PCTime[this->currPt] < this->PCTime[this->currPt-1]) {
                        // we've rolled over the counter, increment the offset
                        this->tOffset += COUNTERROLLOVER;
                        this->PCTime[this->currPt] += COUNTERROLLOVER;
                    }
                    ptr += incr;
                } else {
                    printf("Bad interrupt on time '%s' %d\n", rxBuffer, sscanf(rxBuffer, "P%08X", &time));
                    free(rxBuffer);
                    continue;
                }
                // See which encoders are being captured so we can decode the interrupt
                findParam("PC_BIT_CAP", &param);
                getIntegerParam(param, &cap);
                // Now step through the bytes
                for (int a=0; a<NARRAYS; a++) {
                    if (cap>>a & 1) {
                        if (sscanf(ptr, "%08X%n", &value, &incr) != 1) {
                            printf("Bad interrupt on encoder %d\n", a+1);
                            break;
                        }
                        ptr += incr;
                    } else {
                        value = 0;
                    }
                    // publish value to double param
                    setDoubleParam(zebraCapLast[a], -1);
                    if (a < 4) {
                        // encoders are signed
                        setDoubleParam(zebraCapLast[a], value);
                    } else {
                        // all others are unsigned
                        setDoubleParam(zebraCapLast[a], (unsigned int) value);
                    }
                    // publish value to waveform if we have room
                    // don't worry about signed vs unsigned, the waveform record handles it
                    if (this->currPt < this->maxPts) {
                        this->capArrays[a][this->currPt] = value;
                    }
                    // Note: don't do callParamCallbacks here, or we'll swamp asyn
                 }
                 // sanity check
                 if (ptr[0] != '\0') {
                     printf("Characters remaining in interrupt: %s\n", ptr);
                 }
                 // advance the counter if allowed
                  if (this->currPt < this->maxPts) {
                       this->currPt++;
                  }
            }
            free(rxBuffer);
        }        
        // Update rate of about
        callParamCallbacks();
        this->unlock();
        // We try to do all fastregs at 0.5Hz, so waveform last vals get updated at
        // about 6Hz. This will drop off if we set values
        epicsTimeGetCurrent(&end);
        double timeToSleep = 2.0 / (2 * FASTREGS) - epicsTimeDiffInSeconds(&end, &start);
        if (timeToSleep > 0) {
            epicsThreadSleep(timeToSleep);
        } else {
            //printf("Not enough time to poll properly %f\n", timeToSleep);
        }
    }
}

/* Write the config of a zebra to a file
 * called with the lock taken
 */
asynStatus zebra::configWrite(const char* str) {
    asynStatus status = asynSuccess;
    const reg *r;
    int value;
    FILE *file;
    char buff[NBUFF];
    epicsSnprintf(buff, NBUFF, "Writing '%s'", str);
    setStringParam(zebraConfigStatus, buff);
    callParamCallbacks();
    file = fopen(str,"w");
    if (file == NULL) {
        epicsSnprintf(buff, NBUFF, "Can't open '%s'", str);
        setStringParam(zebraConfigStatus, buff);
        callParamCallbacks();
        return asynError;
    }
    fprintf(file,"; Setup for a zebra box\n");
    fprintf(file,"[regs]\n");
    for (unsigned int i=0; i<NREGS-FASTREGS; i++) {
        r = &(reg_lookup[i]);
        getIntegerParam(REG2PARAM(r), &value);
        fprintf(file,"%s = %d", r->str, value);
        if (r->type == regMux && value >= 0 && (unsigned int) value < NSYSBUS) {
            fprintf(file," ; %s", bus_lookup[value]);
        }
        fprintf(file,"\n");
    }
    fclose(file);
    // Wait so people notice it's doing something!
    epicsThreadSleep(1);
    setStringParam(zebraConfigStatus, "Done");
    callParamCallbacks();
    return status;
}

/* Read the config of a zebra from a zebra
 * called with the lock taken
 */
asynStatus zebra::configRead(const char* str) {
    char buff[NBUFF];
    epicsSnprintf(buff, NBUFF, "Reading '%s'", str);
    setStringParam(zebraConfigStatus, buff);
    callParamCallbacks();
    if (ini_parse(str, configLineC, this) < 0) {
        epicsSnprintf(buff, NBUFF, "Error reading '%s'", str);
        setStringParam(zebraConfigStatus, buff);
        callParamCallbacks();
        return asynError;
    }
    setStringParam(zebraConfigStatus, "Done");
    callParamCallbacks();
    return asynSuccess;
}

int zebra::configLine(const char* section, const char* name, const char* value) {
    char buff[NBUFF];
    if (strcmp(section, "regs") == 0) {
        int param;
        if (findParam(name, &param) == asynSuccess) {
            regType type = (PARAM2REG(param))->type;
            if (type == regMux || type == regRW) {
                this->writeParam(param, atoi(value));
            }
        } else {
            epicsSnprintf(buff, NBUFF, "Can't find param %s", name);
            setStringParam(zebraConfigStatus, buff);
            callParamCallbacks();
        }
        return 1;
    }
    epicsSnprintf(buff, NBUFF, "Can't find section %s", section);
    setStringParam(zebraConfigStatus, buff);
    callParamCallbacks();
    return 0;  /* unknown section, error */
}

/* This function gets the value of a register
   called with the lock taken */
asynStatus zebra::getReg(const reg *r, int *value) {
    asynStatus status = asynSuccess;
    // Create the transmit buffer
    char txBuffer[NBUFF], *rxBuffer;
    int txSize, addr;
    // Send a write
    txSize = epicsSnprintf(txBuffer, NBUFF, "R%02X", r->addr);
    status = this->sendReceive(txBuffer, txSize, &rxBuffer);
    // If we got a response
    if(status == asynSuccess) {
        if (sscanf(rxBuffer, "R%02X%04X", &addr, value)) {
            if (addr == r->addr) {
                // Good message, everything ok
                status = asynSuccess;
            } else {
                printf("Expected addr %02X got %02X\n", r->addr, addr);
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
asynStatus zebra::setReg(const reg *r, int value) {
    asynStatus status = asynSuccess;
    // Create the transmit buffer
    char txBuffer[NBUFF], *rxBuffer;
    int txSize, addr;
    // Send a write
    txSize = epicsSnprintf(txBuffer, NBUFF, "W%02X%04X", r->addr, value & 0xFFFF);
    status = this->sendReceive(txBuffer, txSize, &rxBuffer);
    // If we got a response
    if(status == asynSuccess) {
        if (sscanf(rxBuffer, "W%02XOK", &addr)) {
            if (addr == r->addr) {
                // Good message, everything ok
                status = asynSuccess;
            } else {
                printf("Expected addr %02X got %02X\n", r->addr, addr);
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
    txSize = epicsSnprintf(txBuffer, NBUFF, "S");
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
    pasynUser->timeout = TIMEOUT;
    // If there are any responses on the queue they must be junk
    while (epicsMessageQueuePending(this->msgQId)) {
        epicsMessageQueueReceive(this->msgQId, rxBuffer, sizeof(rxBuffer));
        printf("Junk message in buffer '%s'\n", *rxBuffer);
        free(*rxBuffer);
    }
    status = pasynOctet->write(octetPvt, pasynUser, txBuffer, txSize, &nBytesOut);
    if(status == asynSuccess) {
        // wait for a response on the message queue
        if (epicsMessageQueueReceiveWithTimeout(this->msgQId, rxBuffer, sizeof(rxBuffer), TIMEOUT) > 0) {
               status = asynSuccess;
        } else {
            printf("Zebra not connected, no response to %.*s\n", txSize, txBuffer);
            status = asynError;
        }
    }
    return status;
}

asynStatus zebra::writeParam(int param, int value) {
    asynStatus status = asynError;
    const reg *r = PARAM2REG(param);
    //printf("Write reg %d\n", r->addr);
    // trigger an update every time
    if (r->type == regRO) return status;
    status = this->setReg(r, value);
    printf("Set %s %d\n", r->str, value);
    if (status == asynSuccess && r->type != regCmd) {
        setIntegerParam(param, value-1);
        status = this->getReg(r, &value);
        if (status == asynSuccess) {
            setIntegerParam(param, value);
            if (r->type == regMux && value >= 0 && (unsigned int) value < NSYSBUS) {
                //printf("Write reg %d isMux\n", r->addr);
                setStringParam(REG2PARAMSTR(r), bus_lookup[value]);
            }
        }
        if (strcmp(r->str, "SYS_RESET") == 0) {
            // Reset called, so stop waveform processing
            this->callbackWaveforms();
            setIntegerParam(zebraArrayAcq, 0);
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
    asynStatus status = asynError;

    /* Any work we need to do */
    int param = pasynUser->reason;
    if(param >= this->zebraReg[0] && param < this->zebraReg[NREGS-1]) {
        status = this->writeParam(param, value);
    } else if (param == zebraStore) {
        status = this->storeFlash();
    } else if (param == zebraConfigRead || param == zebraConfigWrite) {
        char fileName[NBUFF];
        getStringParam(zebraConfigFile, NBUFF, fileName);
        if (param == zebraConfigRead) {
            status = this->configRead(fileName);
        } else {
            status = this->configWrite(fileName);
        }
    } else if (param == zebraArrayUpdate) {        
        status = this->callbackWaveforms();
    } else if (param >= zebraFiltSel[0] && param <= zebraFiltSel[NFILT-1]) {
        value = value % NSYSBUS;
        setStringParam(param + NFILT, bus_lookup[value]);
        status = setIntegerParam(param, value);
        setIntegerParam(zebraNumDown, 0);
        this->callbackWaveforms();
    }
    callParamCallbacks();
    return status;
}    
    
/* This function calls back on the time and position waveform values
   called with the lock taken */
asynStatus zebra::callbackWaveforms() {
    int sel, *src, lastUpdatePt;
    getIntegerParam(zebraNumDown, &lastUpdatePt);
    if (lastUpdatePt != this->currPt) {
        // printf("Update %d %d\n", this->lastUpdatePt, this->currPt);  
        // store the last update so we don't get repeated updates
        setIntegerParam(zebraNumDown, this->currPt);

        if (this->currPt < this->maxPts) {
            // horrible hack for edm plotting
            // set the last+1 time point to be the same as the last, this
            // means the last point on the time/pos plot is (last, 0), which
            // gives a straight line back to (0,0) without confusing the user
            this->PCTime[this->currPt] = this->PCTime[this->currPt-1];                
            doCallbacksFloat64Array(this->PCTime, this->currPt+1, zebraPCTime, 0);
        } else {
            doCallbacksFloat64Array(this->PCTime, this->currPt, zebraPCTime, 0);               
        }      

        /* Filter the relevant sys_bus array with filtSel[a] and put the value in filtArray[a] */
        for (int a=0; a<NFILT; a++) {
            getIntegerParam(zebraFiltSel[a], &sel);
            if (sel < 32) {
                src = this->capArrays[4]; // SYS_BUS1
            } else {
                src = this->capArrays[5]; // SYS_BUS2
                sel -= 32;
            }
            for (int i=0; i<this->maxPts; i++) {
                this->filtArrays[a][i] = (src[i] >> sel) & 1;
            }
            doCallbacksInt32Array(this->filtArrays[a], this->currPt, zebraFiltArrays[a], 0);
        }

        // update capture arrays, go backwards so we do PC_ENC1 last
        for (int a=NARRAYS; a>=0; a--) {
            doCallbacksInt32Array(this->capArrays[a], this->currPt, zebraCapArrays[a], 0);
        }
        
        // Note no callParamCallbacks. We will forward link from PC_ENC1 to NumDown
        // so that GDA can monitor NumDown to know when to caget array values 
    }
    return asynSuccess;
}

/** Configuration command, called directly or from iocsh */
extern "C" int zebraConfig(const char *portName, const char* serialPortName, int maxPts)
{
    new zebra(portName, serialPortName, maxPts);
    return(asynSuccess);
}

/** Code for iocsh registration */
static const iocshArg zebraConfigArg0 = {"Port name", iocshArgString};
static const iocshArg zebraConfigArg1 = {"Serial port name", iocshArgString};
static const iocshArg zebraConfigArg2 = {"Max number of points to capture in position compare", iocshArgInt};
static const iocshArg* const zebraConfigArgs[] =  {&zebraConfigArg0, &zebraConfigArg1, &zebraConfigArg2};
static const iocshFuncDef configzebra = {"zebraConfig", 3, zebraConfigArgs};
static void configzebraCallFunc(const iocshArgBuf *args)
{
    zebraConfig(args[0].sval, args[1].sval, args[2].ival);
}

static void zebraRegister(void)
{
    iocshRegister(&configzebra, configzebraCallFunc);
}

extern "C" { epicsExportRegistrar(zebraRegister); }


