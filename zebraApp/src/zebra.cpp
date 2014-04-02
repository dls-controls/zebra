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
#define NQUEUE 10000

/* The size of our transmit and receive buffers,
 * max filename length and string param buffers */
#define NBUFF 255

/* The timeout waiting for a response from zebra */
#define TIMEOUT 1.0

/* The min time between sending 2 read commands without waiting for the reponse */
#define DELAYMULTIREAD 0.01

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
 * We set the prescaler to 5 for time units of ms or 5000 for time units
 * of s. This means that each increment of the counter is 0.0001 time units.
 * When the counter rolls over we need to add 2^32*0.0001 time units, which
 * is this number
 */
#define COUNTERROLLOVER 429496.7296

/* useful defines for converting to and from asyn parameters and zebra regs */
#define NREGS (sizeof(reg_lookup)/sizeof(struct reg))
#define PARAM2REG(/*int*/param) &(reg_lookup[param-this->zebraReg[0]])
#define REG2PARAM(/*reg**/r) (this->zebraReg[0]+(int)(r-reg_lookup))
#define REG2PARAMSTR(/*reg**/r) ((int) (REG2PARAM(r)+NREGS))
#define NSYSBUS (sizeof(bus_lookup)/sizeof(char *))

static const char *driverName = "zebra";

class zebra: public asynPortDriver {
public:
	zebra(const char *portName, const char* serialPortName, int maxPts);

	/* These are the methods that we override from asynPortDriver */
	virtual asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);

	/** These should be private, but get called from C, so must be public */
	void pollTask();
	void readTask();
	void interruptTask();
	int configLine(const char* section, const char* name, const char* value);

protected:
	/* These are helper methods for the class */
	asynStatus send(char *txBuffer, int txSize);
	asynStatus receive(const char* format, int *addr, int *value);
	asynStatus sendSetReg(const reg *r, int value);
	asynStatus receiveSetReg(const reg *r);
	asynStatus setReg(const reg *r, int value);
	asynStatus sendGetReg(const reg *r);
	asynStatus receiveGetReg(const reg *r, int *value);
	asynStatus getReg(const reg *r, int *value);
	asynStatus flashCmd(const char *cmd);
	asynStatus configRead(const char* str);
	asynStatus configWrite(const char* str);
	asynStatus callbackWaveforms();

protected:
	/* Parameter indices */
#define FIRST_PARAM zebraIsConnected
	int zebraIsConnected;        // int32 read  - is zebra connected?
	int zebraArrayUpdate;        // int32 write - update arrays
	int zebraArrayAcq;           // int32 read - when data is acquiring
	int zebraNumDown;            // int32 read - number of data points downloaded
	int zebraSysBus1;            // string read - system bus key first half
	int zebraSysBus2;            // string read - system bus key second half
	int zebraStore;              // int32 write - store config to flash
	int zebraRestore;              // int32 write - restore config from flash	
	int zebraConfigFile;         // charArray write - filename to read/write config to
	int zebraConfigRead;         // int32 write - read config from filename
	int zebraConfigWrite;        // int32 write - write config to filename
	int zebraConfigStatus;       // int32 read - config status message
	int zebraPCTime;             // float64array read - position compare timestamps
#define LAST_PARAM zebraPCTime
	int zebraScale[NARRAYS];     // float64 write - Scale (MRES) of motors
	int zebraOff[NARRAYS];       // float64 write - offset of motors
	int zebraCapArrays[NARRAYS]; // float64array read - position compare capture array
	int zebraCapLast[NARRAYS];   // float64 read - last captured value
	int zebraFiltArrays[NFILT];  // int8array read - position compare sys bus filtered
	int zebraFiltSel[NFILT];     // int32 read/write - which index of system bus to select for zebraFiltArrays
	int zebraFiltSelStr[NFILT];  // string read - the name of the entry in the system bus
	int zebraReg[NREGS * 2];     // int32 read/write - all zebra params in reg_lookup
#define NUM_PARAMS (&LAST_PARAM - &FIRST_PARAM + 1) + NARRAYS*4 + NFILT*3 + NREGS*2

private:
	asynUser *pasynUser;
	asynCommon *pasynCommon;
	void *pcommonPvt;
	asynOctet *pasynOctet;
	void *octetPvt;
	asynDrvUser *pasynDrvUser;
	void *drvUserPvt;
	epicsMessageQueueId msgQId, intQId;
	int maxPts, currPt, configPhase, doneInit;
	char *filtArrays[NFILT];
	double *PCTime, tOffset, *capArrays[NARRAYS];
};

/* C function to call poll task from epicsThreadCreate */
static void pollTaskC(void *userPvt) {
	zebra *pPvt = (zebra *) userPvt;
	pPvt->pollTask();
}

/* C function to call new message from  task from epicsThreadCreate */
static void readTaskC(void *userPvt) {
	zebra *pPvt = (zebra *) userPvt;
	pPvt->readTask();
}

/* C function to call interrupt task from epicsThreadCreate */
static void interruptTaskC(void *userPvt) {
	zebra *pPvt = (zebra *) userPvt;
	pPvt->interruptTask();
}

/* C function to call new message from  task from epicsThreadCreate */
static int configLineC(void* userPvt, const char* section, const char* name,
		const char* value) {
	zebra *pPvt = (zebra *) userPvt;
	return pPvt->configLine(section, name, value);
}

/* Constructor */
zebra::zebra(const char* portName, const char* serialPortName, int maxPts) :
		asynPortDriver(portName, 1 /*maxAddr*/, NUM_PARAMS,
				asynInt8ArrayMask | asynFloat64ArrayMask | asynInt32Mask
						| asynFloat64Mask | asynOctetMask | asynDrvUserMask,
				asynInt8ArrayMask | asynFloat64ArrayMask | asynInt32Mask
						| asynFloat64Mask | asynOctetMask, ASYN_CANBLOCK, /*ASYN_CANBLOCK=1, ASYN_MULTIDEVICE=0 */
				1, /*autoConnect*/0, /*default priority */
				0 /*default stack size*/) {
	const char *functionName = "zebra";
	asynStatus status = asynSuccess;
	asynInterface *pasynInterface;
	char buffer[6400]; /* 100 chars per element on sys bus is overkill... */
	char str[NBUFF];
	const reg *r;

	/* For position compare results */
	this->maxPts = maxPts;
	this->currPt = 0;

	/* So we know when we have a complete set of params that we are allowed to write to file */
	this->doneInit = 0;

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
	for (unsigned int i = 0; i < NSYSBUS / 2; i++) {
		epicsSnprintf(str, NBUFF, "%2d: %s\n", i, bus_lookup[i]);
		strcat(buffer, str);
	}
	setStringParam(zebraSysBus1, buffer);
	createParam("SYS_BUS2", asynParamOctet, &zebraSysBus2);
	buffer[0] = '\0';
	for (unsigned int i = NSYSBUS / 2; i < NSYSBUS; i++) {
		epicsSnprintf(str, NBUFF, "%2d: %s\n", i, bus_lookup[i]);
		strcat(buffer, str);
	}
	setStringParam(zebraSysBus2, buffer);

	/* a parameter for a store to flash request */
	createParam("STORE", asynParamInt32, &zebraStore);
	createParam("RESTORE", asynParamInt32, &zebraRestore);	

	/* parameters for filename reading/writing config */
	createParam("CONFIG_FILE", asynParamOctet, &zebraConfigFile);
	createParam("CONFIG_READ", asynParamInt32, &zebraConfigRead);
	createParam("CONFIG_WRITE", asynParamInt32, &zebraConfigWrite);
	createParam("CONFIG_STATUS", asynParamOctet, &zebraConfigStatus);

	/* position compare time array */
	createParam("PC_TIME", asynParamFloat64Array, &zebraPCTime);
	this->PCTime = (double *) calloc(maxPts, sizeof(double));

	/* position compare array scale (motor resolution) */
	for (int a = 0; a < NARRAYS; a++) {
		epicsSnprintf(str, NBUFF, "M%d_SCALE", a + 1);
		createParam(str, asynParamFloat64, &zebraScale[a]);
		setDoubleParam(zebraScale[a], 1.0);
	}

	/* position compare array offset (motor offset) */
	for (int a = 0; a < NARRAYS; a++) {
		epicsSnprintf(str, NBUFF, "M%d_OFF", a + 1);
		createParam(str, asynParamFloat64, &zebraOff[a]);
		setDoubleParam(zebraOff[a], 0.0);
	}

	/* create the position compare arrays */
	for (int a = 0; a < NARRAYS; a++) {
		epicsSnprintf(str, NBUFF, "PC_CAP%d", a + 1);
		createParam(str, asynParamFloat64Array, &zebraCapArrays[a]);
		this->capArrays[a] = (double *) calloc(maxPts, sizeof(double));
	}

	/* create the last captured interrupt values */
	for (int a = 0; a < NARRAYS; a++) {
		epicsSnprintf(str, NBUFF, "PC_CAP%d_LAST", a + 1);
		createParam(str, asynParamFloat64, &zebraCapLast[a]);
	}

	/* create filter arrays */
	for (int a = 0; a < NFILT; a++) {
		epicsSnprintf(str, NBUFF, "PC_FILT%d", a + 1);
		createParam(str, asynParamInt8Array, &zebraFiltArrays[a]);
		this->filtArrays[a] = (char *) calloc(maxPts, sizeof(char));
	}

	/* create values that we can use to filter one element on the system bus with */
	/* NOTE: separate for loop so we get values for params we can do arithmetic with */
	for (int a = 0; a < NFILT; a++) {
		epicsSnprintf(str, NBUFF, "PC_FILTSEL%d", a + 1);
		createParam(str, asynParamInt32, &zebraFiltSel[a]);
		setIntegerParam(zebraFiltSel[a], 0);
	}

	/* create lookups of string values of these string selects */
	/* NOTE: separate for loop so we get values for params we can do arithmetic with */
	for (int a = 0; a < NFILT; a++) {
		epicsSnprintf(str, NBUFF, "PC_FILTSEL%d_STR", a + 1);
		createParam(str, asynParamOctet, &zebraFiltSelStr[a]);
		// Check our filter string lookup calcs will work
		assert(zebraFiltSelStr[a] == zebraFiltSel[a] + NFILT);
		setStringParam(zebraFiltSelStr[a], bus_lookup[0]);
	}

	/* create parameters for registers */
	for (unsigned int i = 0; i < NREGS; i++) {
		r = &(reg_lookup[i]);
		createParam(r->str, asynParamInt32, &zebraReg[i]);
		// If it is a command then set its value to 0
		if (r->type == regCmd)
			setIntegerParam(zebraReg[i], 0);
		// Check our param -> reg lookup and inverse will work
		assert(r == PARAM2REG(zebraReg[i]));
		assert(REG2PARAM(r) == zebraReg[i]);
	}

	/* create parameters for register string values, these are lookups
	 of the string values of mux registers from the system bus */
	/* NOTE: separate for loop so we get values for params we can do arithmetic with in REG2PARAMSTR */
	for (unsigned int i = 0; i < NREGS; i++) {
		r = &(reg_lookup[i]);
		epicsSnprintf(str, NBUFF, "%s_STR", r->str);
		createParam(str, asynParamOctet, &zebraReg[i + NREGS]);
		// Check our reg -> param string lookup will work
		assert(REG2PARAMSTR(r) == zebraReg[i+NREGS]);
	}

	/* Create a message queue to hold completed messages and interrupts */
	this->msgQId = epicsMessageQueueCreate(NQUEUE, sizeof(char*));
	this->intQId = epicsMessageQueueCreate(NQUEUE, sizeof(char*));
	if (this->msgQId == NULL || this->intQId == NULL) {
		asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
				"%s:%s: epicsMessageQueueCreate failure\n", driverName, functionName);
		return;
	}

	/* Connect to the device port */
	/* Copied from asynOctecSyncIO->connect */
	pasynUser = pasynManager->createAsynUser(0, 0);
	status = pasynManager->connectDevice(pasynUser, serialPortName, 0);
	if (status != asynSuccess) {
		asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
				"%s:%s: Connect failed, port=%s, error=%d\n", driverName, functionName, serialPortName, status);
		return;
	}
	pasynInterface = pasynManager->findInterface(pasynUser, asynCommonType, 1);
	if (!pasynInterface) {
		asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
				"%s:%s: %s interface not supported", driverName, functionName, asynCommonType);
		return;
	}
	pasynCommon = (asynCommon *) pasynInterface->pinterface;
	pcommonPvt = pasynInterface->drvPvt;
	pasynInterface = pasynManager->findInterface(pasynUser, asynOctetType, 1);
	if (!pasynInterface) {
		asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
				"%s:%s: %s interface not supported", driverName, functionName, asynOctetType);
		return;
	}
	pasynOctet = (asynOctet *) pasynInterface->pinterface;
	octetPvt = pasynInterface->drvPvt;

	/* Set EOS and flush */
	pasynOctet->flush(octetPvt, pasynUser);
	pasynOctet->setInputEos(octetPvt, pasynUser, "\n", 1);
	pasynOctet->setOutputEos(octetPvt, pasynUser, "\n", 1);

	/* Create the thread that reads from the device  */
	if (epicsThreadCreate("ZebraReadTask", epicsThreadPriorityMedium,
			epicsThreadGetStackSize(epicsThreadStackMedium),
			(EPICSTHREADFUNC) readTaskC, this) == NULL) {
		asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
				"%s:%s: epicsThreadCreate failure for reading task\n", driverName, functionName);
		return;
	}

	/* Create the thread that polls the device  */
	if (epicsThreadCreate("ZebraPollTask", epicsThreadPriorityMedium,
			epicsThreadGetStackSize(epicsThreadStackMedium),
			(EPICSTHREADFUNC) pollTaskC, this) == NULL) {
		asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
				"%s:%s: epicsThreadCreate failure for polling task\n", driverName, functionName);
		return;
	}

	/* Create the thread that handles interrupts from the device  */
	if (epicsThreadCreate("ZebraInterruptTask", epicsThreadPriorityMedium,
			epicsThreadGetStackSize(epicsThreadStackMedium),
			(EPICSTHREADFUNC) interruptTaskC, this) == NULL) {
		asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
				"%s:%s: epicsThreadCreate failure for interrupt service task\n", driverName, functionName);
		return;
	}
}

/* This is the function that will be run for the read thread */
void zebra::readTask() {
	const char *functionName = "readTask";
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
		status = pasynOctet->read(octetPvt, pasynUserRead, rxBuffer, NBUFF - 1,
				&nBytesIn, &eomReason);
		if (status) {
			//printf("Port not connected\n");
			free(rxBuffer);
			epicsThreadSleep(TIMEOUT);
		} else if (eomReason & ASYN_EOM_EOS) {
			// Replace the terminator with a null so we can use it as a string
			rxBuffer[nBytesIn] = '\0';
			if (rxBuffer[0] == 'P') {
				// This is an interrupt, send it to the interrupt queue
				asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
						"%s:%s: Interrupt: '%s'\n", driverName, functionName, rxBuffer);
				q = this->intQId;
			} else {
				// This a zebra response to a command, send it to the message queue
				asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
						"%s:%s: Message: '%s'\n", driverName, functionName, rxBuffer);
				q = this->msgQId;
			}
			if (epicsMessageQueueTrySend(q, &rxBuffer, sizeof(&rxBuffer))
					!= 0) {
				asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
						"%s:%s: Message queue full, dropped message\n", driverName, functionName);
				free(rxBuffer);
			}
		} else {
			asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
					"%s:%s: Bad message '%.*s'\n", driverName, functionName, (int)nBytesIn, rxBuffer);
			free(rxBuffer);
		}
	}
}

/* This is the function that will be run for the interrupt service thread */
void zebra::interruptTask() {
	const char *functionName = "interruptTask";
	int cap, param, incr;
	unsigned int time, nfound;
	char *rxBuffer, *ptr, escapedbuff[NBUFF];
	epicsTimeStamp start, end;
	while (true) {
		// Get the time we started
		epicsTimeGetCurrent(&start);
		// Lock as we will be updating params
		this->lock();
		// If there are any interrupts, service them
		while (epicsMessageQueuePending(this->intQId) > 0) {
			epicsMessageQueueReceive(this->intQId, &rxBuffer,
					sizeof(&rxBuffer));
			if (strcmp(rxBuffer, "PR") == 0) {
				// This is zebra telling us to reset our buffers
				this->currPt = 0;
				this->tOffset = 0.0;
				// Set it acquiring
                setIntegerParam(zebraArrayAcq, 1);								
				// We need to trigger a waveform update so that PC_NUM_DOWN
				// will unset the busy record set by the arm
				// Setting NumDown to -1 will trigger a waveform update even if
				// the last waveform sent was the same as this one
				setIntegerParam(zebraNumDown, -1);
				this->callbackWaveforms();
				// reset num cap
				findParam("PC_NUM_CAPLO", &param);
				setIntegerParam(param, 0);
				findParam("PC_NUM_CAPHI", &param);
				setIntegerParam(param, 0);
			} else if (strcmp(rxBuffer, "PX") == 0) {
				// This is zebra saying there is no more data
				setIntegerParam(zebraArrayAcq, 0);				
				// Setting NumDown to -1 will trigger a waveform update even if
				// the last waveform sent was the same as this one
				setIntegerParam(zebraNumDown, -1);
				this->callbackWaveforms();
			} else {
				// This is a data buffer
				ptr = rxBuffer;
				// First get time
				nfound = sscanf(rxBuffer, "P%08X%n", &time, &incr);
				if (nfound == 1) {
					// put time in time units (10s, s or ms based on TS_PRE)
					this->PCTime[this->currPt] = time * 0.0001 + this->tOffset;
					if (this->currPt > 0
							&& this->PCTime[this->currPt]
									< this->PCTime[this->currPt - 1]) {
						// we've rolled over the counter, increment the offset
						this->tOffset += COUNTERROLLOVER;
						this->PCTime[this->currPt] += COUNTERROLLOVER;
					}
					ptr += incr;
				} else {
					asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
							"%s:%s: Bad interrupt on time '%s', nfound:%d\n", driverName, functionName, rxBuffer, nfound);
					free(rxBuffer);
					continue;
				}
				// See which encoders are being captured so we can decode the interrupt
				findParam("PC_BIT_CAP", &param);
				getIntegerParam(param, &cap);
				// Now step through the bytes
				for (int a = 0; a < NARRAYS; a++) {
					double scale, off, dvalue = 0;
					int ivalue;
					if (cap >> a & 1) {
						if (sscanf(ptr, "%08X%n", &ivalue, &incr) == 1) {
							getDoubleParam(zebraScale[a], &scale);
							getDoubleParam(zebraOff[a], &off);
							if (a >= 4) {
							    // system bus and dividers are unsigned 32-bit numbers
							    dvalue = ((unsigned int) ivalue) * scale + off;
							} else {
							    // encoders are signed 32-bit numbers
    							dvalue = ivalue * scale + off;
    						}
							ptr += incr;
						} else {
							epicsStrnEscapedFromRaw(escapedbuff, NBUFF, rxBuffer,
									strlen(rxBuffer));
							asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
									"%s:%s: Bad interrupt on encoder %d in '%s'\n", driverName, functionName, a+1, escapedbuff);
							break;
						}
					}
					// publish value to double param
					setDoubleParam(zebraCapLast[a], dvalue-1);
					setDoubleParam(zebraCapLast[a], dvalue);
					// publish value to waveform if we have room
					if (this->currPt < this->maxPts) {
						this->capArrays[a][this->currPt] = dvalue;
					}
					// Note: don't do callParamCallbacks here, or we'll swamp asyn
				}
				// sanity check
				if (ptr[0] != '\0') {
					epicsStrnEscapedFromRaw(escapedbuff, NBUFF, ptr,
							strlen(ptr));
					asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
							"%s:%s: Characters remaining in interrupt: '%s'\n", driverName, functionName, escapedbuff);
				}
				// advance the counter if allowed
				if (this->currPt < this->maxPts) {
					this->currPt++;
				}
			}
			free(rxBuffer);
		}
		// Update any params we have got, this means that the max update rate
		// of the waveform last values is this loop tick (10Hz).
		callParamCallbacks();
		this->unlock();
		// Work out how long to sleep for so each loop iteration takes 0.1s
		epicsTimeGetCurrent(&end);
		double timeToSleep = 0.1 - epicsTimeDiffInSeconds(&end, &start);
		if (timeToSleep > 0) {
			epicsThreadSleep(timeToSleep);
		} else {
			// Got to sleep for a bit in case something else is waiting for the lock
			epicsThreadSleep(0.01);
			//printf("Not enough time to poll properly %f\n", timeToSleep);
		}
	}
}

/* This is the function that will be run for the poll thread */
void zebra::pollTask() {
	const char *functionName = "pollTask";
	int value, caploparam, caphiparam, lastcap, downloading;
	double loopTime;
	const reg *r;
	unsigned int sys, poll = 0, iteration = 0;
	char *rxBuffer, escapedbuff[NBUFF];
	epicsTimeStamp start, end;
	asynStatus status = asynSuccess;
	findParam("PC_NUM_CAPLO", &caploparam);
	findParam("PC_NUM_CAPHI", &caphiparam);
	// Wait 1 second until port is up
	epicsThreadSleep(1.0);
	while (true) {
		// alternate between the next slow reg, and all the fast regs
		epicsTimeGetCurrent(&start);
		this->lock();
		// If there are any responses on the queue they must be junk
		while (epicsMessageQueuePending(this->msgQId)) {
			epicsMessageQueueReceive(this->msgQId, &rxBuffer,
					sizeof(&rxBuffer));
			epicsStrnEscapedFromRaw(escapedbuff, NBUFF, rxBuffer,
					strlen(rxBuffer));
			asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
					"%s:%s: Junk message in buffer '%s'\n", driverName, functionName, escapedbuff);
			status = asynError;
			free(rxBuffer);
		}
		// Work out if we are currently downloading
		getIntegerParam(zebraArrayAcq, &downloading);
		// Check what PC_NUM_CAPLO is now so can check if it rolled over
		getIntegerParam(caploparam, &lastcap);
		if (downloading) {
			// If we are downloading, then must wait for responses as FPGA is heavily loaded
			for (sys = NREGS - FASTREGS; sys < NREGS; sys++) {
				// Send demand to zebra
				this->getReg(&(reg_lookup[sys]), &value);
			}
			// Now wait a second until we do it again
			loopTime = 1.0;
		} else if (iteration == 0) {
			// First send requests for all the system
			for (sys = NREGS - FASTREGS; sys < NREGS; sys++) {
				// Send demand to zebra
				this->sendGetReg(&(reg_lookup[sys]));
				epicsThreadSleep(DELAYMULTIREAD);
			}
			// Now get values back
			for (sys = NREGS - FASTREGS; sys < NREGS; sys++) {
				// wait for a response on the message queue
				this->receiveGetReg(&(reg_lookup[sys]), &value);
			}
			// Now wait 0.25 seconds until we get the regs below
			loopTime = 0.25;
		} else {
			// Get the register value from zebra
			status = this->getReg(&(reg_lookup[poll]), &value);
			// skip one, then keep on skipping until it isn't a command
			for (r = NULL; r == NULL || r->type == regCmd;) {
				// Move to next register
				if (++poll >= NREGS - FASTREGS) {
					poll = 0;
					// Done one complete cycle so write to file allowed.
					this->doneInit = 1;
				}
				r = &(reg_lookup[poll]);
			}
			// Now wait 0.25 seconds until the next poll
			loopTime = 0.25;
		}
		// check what NUM_CAP is now
		getIntegerParam(caploparam, &value);
		// If this is PC_NUM_CAPLO and it has rolled over, then trigger a PC_NUM_CAP_HI update
		if (value < lastcap) {
			//printf("Rollover!\n");
			this->getReg(PARAM2REG(caphiparam), &value);
		}

		// Iteration 0 is all 6 FASTREGS, iterations 1-3 are the next slow polled regs
		iteration++;
		if (iteration > 3) iteration = 0;
		// Update params
		callParamCallbacks();
		this->unlock();
		// We try to run this loop at 4Hz so that system values get done at 1Hz
		// or at 1Hz during download time
		epicsTimeGetCurrent(&end);
		double timeToSleep = loopTime - epicsTimeDiffInSeconds(&end, &start);
		if (timeToSleep > 0) {
			epicsThreadSleep(timeToSleep);
		} else {
			// Got to sleep for a bit in case something else is waiting for the lock
			epicsThreadSleep(0.01);
			//printf("Not enough time to poll properly %f\n", timeToSleep);
		}
	}
}

/* Send helper function
 * called with lock taken
 */
asynStatus zebra::send(char *txBuffer, int txSize) {
	const char *functionName = "send";
	asynStatus status = asynSuccess;
	int connected;
	size_t nBytesOut;
	pasynUser->timeout = TIMEOUT;
	status = pasynOctet->write(octetPvt, pasynUser, txBuffer, txSize,
			&nBytesOut);
	asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
			"%s:%s: Send: '%.*s'\n", driverName, functionName, txSize, txBuffer);
	if (status != asynSuccess) {
		// Can't write, port probably not connected
		getIntegerParam(zebraIsConnected, &connected);
		if (connected) {
			setIntegerParam(zebraIsConnected, 0);
			asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
					"%s:%s: Can't write to zebra: '%.*s'\n", driverName, functionName, txSize, txBuffer);
		}
	}
	return status;
}

/* receive helper function, checks response matches format with optional addr and value args
 * called with lock taken
 */
asynStatus zebra::receive(const char* format, int *addr, int *value) {
	const char *functionName = "receive";
	asynStatus status = asynSuccess;
	char escapedbuff[NBUFF];
	char* rxBuffer;
	int scanned, connected;
	pasynUser->timeout = TIMEOUT;
	// wait for a response on the message queue
	if (epicsMessageQueueReceiveWithTimeout(this->msgQId, &rxBuffer,
			sizeof(&rxBuffer), TIMEOUT) > 0) {
		// scan the return
		if (addr == NULL) {
			scanned = (strcmp(rxBuffer, format) == 0);
		} else if (value == NULL) {
			scanned = sscanf(rxBuffer, format, addr);
		} else {
			scanned = sscanf(rxBuffer, format, addr, value);
		}
		if (!scanned) {
			epicsStrnEscapedFromRaw(escapedbuff, NBUFF, rxBuffer,
					strlen(rxBuffer));
			asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
					"%s:%s: Expected '%s', got '%s'\n", driverName, functionName, format, escapedbuff);
			status = asynError;
		}
		setIntegerParam(zebraIsConnected, 1);
		free(rxBuffer);
	} else {
		getIntegerParam(zebraIsConnected, &connected);
		if (connected) {
			setIntegerParam(zebraIsConnected, 0);
			asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
					"%s:%s: No response from zebra\n", driverName, functionName);
		}
		status = asynTimeout;
	}
	return status;
}

/* This function send an output to Zebra asking for the value of a register
 called with the lock taken */
asynStatus zebra::sendGetReg(const reg *r) {
	//const char *functionName = "sendGetReg";
	char txBuffer[NBUFF];
	int txSize;
	// Create the transmit buffer
	txSize = epicsSnprintf(txBuffer, NBUFF, "R%02X", r->addr);
	// Send a write
	return this->send(txBuffer, txSize);
}

/* This function parses the return from Zebra giving the value of a register
 called with the lock taken */
asynStatus zebra::receiveGetReg(const reg *r, int *value) {
	const char *functionName = "receiveGetReg";
	int addr;
	asynStatus status = asynSuccess;
	// Get the result
	status = this->receive("R%02X%04X", &addr, value);
	// If successful check it matches with what we sent
	if (status == asynSuccess) {
		if (addr == r->addr) {
			// Good message, everything ok
			setIntegerParam(REG2PARAM(r), *value);
			// If it is a mux, set the string representation from the system bus
			if (r->type	== regMux && *value >= 0 && (unsigned int)(*value) < NSYSBUS) {
				setStringParam (REG2PARAMSTR(r), bus_lookup[*value]);
			}
			setIntegerParam(zebraIsConnected, 1);
			status = asynSuccess;
		} else {
			asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
					"%s:%s: Expected addr %02X got %02X\n", driverName, functionName, r->addr, addr);
			status = asynError;
		}
	} else if (status == asynError) {
		asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
				"%s:%s: Mismatched response on addr %02X\n", driverName, functionName, r->addr);
	}
	// Timeout returned without printing error
	return status;
}

/* This function gets the value of a register
 called with the lock taken */
asynStatus zebra::getReg(const reg *r, int *value) {
	//const char *functionName = "sendGetReg";
	asynStatus status = this->sendGetReg(r);
	if (status == asynSuccess) {
		status = this->receiveGetReg(r, value);
	}
	return status;
}

/* This function sets the value of a register
 called with the lock taken */
asynStatus zebra::sendSetReg(const reg *r, int value) {
	//const char *functionName = "sendSetReg";
	char txBuffer[NBUFF];
	int txSize;
	// Not allowed to write a read-only register
	if (r->type == regRO)
		return asynError;
	// Create the transmit buffer
	txSize = epicsSnprintf(txBuffer, NBUFF, "W%02X%04X", r->addr,
			value & 0xFFFF);
	// Send a write
	return this->send(txBuffer, txSize);
}

/* This function sets the value of a register
 called with the lock taken */
asynStatus zebra::receiveSetReg(const reg *r) {
	const char *functionName = "receiveSetReg";
	asynStatus status = asynError;
	int addr;
	// Get the result
	status = this->receive("W%02XOK", &addr, NULL);
	// If successful check it matches with what we sent
	if (status == asynSuccess) {
		if (addr == r->addr) {
			// Good message, everything ok
			setIntegerParam(zebraIsConnected, 1);
			status = asynSuccess;
		} else {
			asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
					"%s:%s: Expected addr %02X got %02X\n", driverName, functionName, r->addr, addr);
			status = asynError;
		}
	} else if (status == asynError) {
		asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
				"%s:%s: Mismatched response on addr %02X\n", driverName, functionName, r->addr);
	}
	// Timeout returned without printing error
	return status;
}

/* This function sets the value of a register
 called with the lock taken */
asynStatus zebra::setReg(const reg *r, int value) {
	const char *functionName = "setReg";
	asynStatus status = this->sendSetReg(r, value);
	if (status == asynSuccess) {
		status = this->receiveSetReg(r);
	}
	if (status) {
		asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
				"%s:%s: Could not set %s to value %d\n", driverName, functionName, r->str, value);
	}
	return status;
}

/* This function stores to flash
 called with the lock taken */
asynStatus zebra::flashCmd(const char * cmd) {
	asynStatus status = asynSuccess;
	// Create the transmit buffer
	char txBuffer[NBUFF];
	int txSize;
	// Send a write
	txSize = epicsSnprintf(txBuffer, NBUFF, cmd);
	status = this->send(txBuffer, txSize);
	// If we could write then get the result
	if (status == asynSuccess) {
	    txSize = epicsSnprintf(txBuffer, NBUFF, "%sOK", cmd);	
		status = this->receive(txBuffer, NULL, NULL);
    }
	return status;
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
	if (this->doneInit != 1) {
		setStringParam(zebraConfigStatus, "Too soon, initial poll not completed, wait a minute");
		callParamCallbacks();
		return asynError;
	}
	epicsSnprintf(buff, NBUFF, "Writing '%s'", str);
	setStringParam(zebraConfigStatus, buff);
	callParamCallbacks();
	file = fopen(str, "w");
	if (file == NULL) {
		epicsSnprintf(buff, NBUFF, "Can't open '%s'", str);
		setStringParam(zebraConfigStatus, buff);
		callParamCallbacks();
		return asynError;
	}
	fprintf(file, "; Setup for a zebra box\n");
	fprintf(file, "[regs]\n");
	for (unsigned int i = 0; i < NREGS - FASTREGS; i++) {
		r = &(reg_lookup[i]);
		getIntegerParam(REG2PARAM(r), &value);
		fprintf(file, "%s = %d", r->str, value);
		if (r->type == regMux && value >= 0 && (unsigned int) value < NSYSBUS) {
			fprintf(file, " ; %s", bus_lookup[value]);
		}
		fprintf(file, "\n");
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
	for (configPhase = 0; configPhase < 4; configPhase++) {
		if (ini_parse(str, configLineC, this) < 0) {
			epicsSnprintf(buff, NBUFF, "Error reading '%s'", str);
			setStringParam(zebraConfigStatus, buff);
			callParamCallbacks();
			return asynError;
		}
	}
	setStringParam(zebraConfigStatus, "Done");
	callParamCallbacks();
	return asynSuccess;
}

int zebra::configLine(const char* section, const char* name,
		const char* value) {
	char buff[NBUFF];
	asynStatus status;
	if (strcmp(section, "regs") == 0) {
		int param, check;
		if (findParam(name, &param) == asynSuccess) {
			regType type = (PARAM2REG(param))->type;
			if (type == regMux || type == regRW) {
				switch (configPhase) {
				case 0:
					status = this->sendSetReg(PARAM2REG(param), atoi(value));
					break;
				case 1:
					status = this->receiveSetReg(PARAM2REG(param));
					break;
				case 2:
					status = this->sendGetReg(PARAM2REG(param));
					epicsThreadSleep(DELAYMULTIREAD);
					break;
				case 3:
					status = this->receiveGetReg(PARAM2REG(param), &check);
					if (status == asynSuccess && atoi(value) != check) status = asynError;
					break;
				default:
					status = asynError;
					break;
				}
				if (status) {
					epicsSnprintf(buff, NBUFF, "Error setting param %s", name);
					setStringParam(zebraConfigStatus, buff);
					callParamCallbacks();
					// Prob comms error, return error
					return 0;
				}
			}
		} else {
			epicsSnprintf(buff, NBUFF, "Can't find param %s", name);
			setStringParam(zebraConfigStatus, buff);
			callParamCallbacks();
			// Not fatal as we might change param names between firmware versions, don't return
		}
		return 1; /* Good */
	}
	epicsSnprintf(buff, NBUFF, "Can't find section %s", section);
	setStringParam(zebraConfigStatus, buff);
	callParamCallbacks();
	return 0; /* unknown section, return error */
}

/** Called when asyn clients call pasynInt32->write().
 * This function performs actions for some parameters
 * For all parameters it sets the value in the parameter library and calls any registered callbacks..
 * \param[in] pasynUser pasynUser structure that encodes the reason and address.
 * \param[in] value Value to write. */
asynStatus zebra::writeInt32(asynUser *pasynUser, epicsInt32 value) {
	const char *functionName = "writeInt32";
	asynStatus status = asynError;
	char buff[NBUFF];

	/* Any work we need to do */
	int param = pasynUser->reason;
	if (param >= this->zebraReg[0] && param < this->zebraReg[NREGS - 1]) {
		const reg *r = PARAM2REG(param);
		status = this->setReg(r, value);
		if (status == asynSuccess && r->type != regCmd) {
		    // do this so we always get an update on the RBV field, essential for clamping 32-bit fields to MRES
    		setIntegerParam(param, -1);
			status = this->getReg(r, &value);
		}
		if (strcmp(r->str, "SYS_RESET") == 0) {
			// Reset called, so stop waveform processing
			setIntegerParam(zebraArrayAcq, 0);						
			// Setting NumDown to -1 will trigger a waveform update even if
			// the last waveform sent was the same as this one
			setIntegerParam(zebraNumDown, -1);
			this->callbackWaveforms();
		}
	} else if (param == zebraStore) {
		status = this->flashCmd("S");
	} else if (param == zebraRestore) {
		status = this->flashCmd("L");		
	} else if (param == zebraConfigRead || param == zebraConfigWrite) {
		char fileName[NBUFF];
		getStringParam(zebraConfigFile, NBUFF, fileName);
		if (param == zebraConfigRead) {
			status = this->configRead(fileName);
		} else {
			status = this->configWrite(fileName);
		}
		if (status) {
			// report error to console too
			getStringParam(zebraConfigStatus, NBUFF, buff);
			asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
					"%s:%s: %s\n", driverName, functionName, buff);
		}
	} else if (param == zebraArrayUpdate) {
		status = this->callbackWaveforms();
	} else if (param >= zebraFiltSel[0] && param <= zebraFiltSel[NFILT-1]) {
		value = value % NSYSBUS;
		setStringParam(param + NFILT, bus_lookup[value]);
		status = setIntegerParam(param, value);
		// Resend all the waveforms as we have changed the filter
		setIntegerParam(zebraNumDown, 0);
		this->callbackWaveforms();
	}
	callParamCallbacks();
	return status;
}

/* This function calls back on the time and position waveform values
 called with the lock taken */
asynStatus zebra::callbackWaveforms() {
	int sel, lastUpdatePt;
	double *src;
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
			this->PCTime[this->currPt] = this->PCTime[this->currPt - 1];
			doCallbacksFloat64Array(this->PCTime, this->currPt + 1, zebraPCTime, 0);
		} else {
			doCallbacksFloat64Array(this->PCTime, this->currPt, zebraPCTime, 0);
		}

		/* Filter the relevant sys_bus array with filtSel[a] and put the value in filtArray[a] */
		for (int a = 0; a < NFILT; a++) {
			getIntegerParam(zebraFiltSel[a], &sel);
			if (sel < 32) {
				src = this->capArrays[4]; // SYS_BUS1
			} else {
				src = this->capArrays[5]; // SYS_BUS2
				sel -= 32;
			}
			for (int i = 0; i < this->maxPts; i++) {
				this->filtArrays[a][i] = (((unsigned int)(src[i]+0.5)) >> sel) & 1;				   
			}
			doCallbacksInt8Array(this->filtArrays[a], this->currPt,
					zebraFiltArrays[a], 0);
		}

		// update capture arrays
		for (int a = 0; a < NARRAYS; a++) {
			doCallbacksFloat64Array(this->capArrays[a], this->currPt,
					zebraCapArrays[a], 0);
		}

		// Note no callParamCallbacks. We will forward link from PC_ENC1 to NumDown
		// so that GDA can monitor NumDown to know when to caget array values
		// This will then FLNK to ARRAY_ACQ so it knows when acquisition is finished
	}
	return asynSuccess;
}

/** Configuration command, called directly or from iocsh */
extern "C" int zebraConfig(const char *portName, const char* serialPortName,
		int maxPts) {
	new zebra(portName, serialPortName, maxPts);
	return (asynSuccess);
}

/** Code for iocsh registration */
static const iocshArg zebraConfigArg0 = { "Port name", iocshArgString };
static const iocshArg zebraConfigArg1 = { "Serial port name", iocshArgString };
static const iocshArg zebraConfigArg2 = {
		"Max number of points to capture in position compare", iocshArgInt };
static const iocshArg* const zebraConfigArgs[] = { &zebraConfigArg0,
		&zebraConfigArg1, &zebraConfigArg2 };
static const iocshFuncDef configzebra = { "zebraConfig", 3, zebraConfigArgs };
static void configzebraCallFunc(const iocshArgBuf *args) {
	zebraConfig(args[0].sval, args[1].sval, args[2].ival);
}

static void zebraRegister(void) {
	iocshRegister(&configzebra, configzebraCallFunc);
}

extern "C" {
epicsExportRegistrar(zebraRegister);
}

