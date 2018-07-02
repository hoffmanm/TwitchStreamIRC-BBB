/*
 * Copyright (c) 2015-2016, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/***** Includes *****/
#include <stdlib.h>
#include <xdc/std.h>
#include <xdc/cfg/global.h>
#include <xdc/runtime/System.h>

#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>

/* Drivers */
#include <ti/drivers/rf/RF.h>
#include <ti/drivers/PIN.h>
#include <driverlib/rf_prop_mailbox.h>
#include <ti/drivers/UART.h>

/* Board Header files */
#include "Board.h"

#include "RFQueue.h"
#include "smartrf_settings/smartrf_settings.h"

#include <stdlib.h>

/* Pin driver handle */
static PIN_Handle ledPinHandle;
static PIN_State ledPinState;

/*
 * Application LED pin configuration table:
 *   - All LEDs board LEDs are off.
 */
PIN_Config pinTable[] =
{
    Board_LED1 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    Board_LED2 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    PIN_TERMINATE
};

enum rf_mode  {
    RX_MODE,
    TX_MODE,
};

volatile enum rf_mode mode = RX_MODE;


/* Packet RX Configuration */
#define DATA_ENTRY_HEADER_SIZE 8  /* Constant header size of a Generic Data Entry */
#define MAX_LENGTH             30 /* Max length byte the radio will accept */
#define NUM_DATA_ENTRIES       2  /* NOTE: Only two data entries supported at the moment */
#define NUM_APPENDED_BYTES     2  /* The Data Entries data field will contain:
                                   * 1 Header byte (RF_cmdPropRx.rxConf.bIncludeHdr = 0x1)
                                   * Max 30 payload bytes
                                   * 1 status byte (RF_cmdPropRx.rxConf.bAppendStatus = 0x1) */


/***** Defines *****/
#define RX_TASK_STACK_SIZE 1024
#define RX_TASK_PRIORITY   1

#define TX_TASK_STACK_SIZE 1024
#define TX_TASK_PRIORITY   3

#define UART_RX_TASK_STACK_SIZE 1024
#define UART_RX_TASK_PRIORITY   2

#define UART_TX_TASK_STACK_SIZE 1024
#define UART_TX_TASK_PRIORITY   4

/* ASCII values of some useful keys */
#define CHAR_LINEFEED                         0x0A
#define CHAR_LINE_END_1                       0x0D   // Enter
#define CHAR_LINE_END_2                       0x03   // Enter on numpad
#define CHAR_SPACE                            0x20
#define CHAR_ZERO                             0x30
#define CHAR_UPPERCASE_START                  0x40

/* TX Configuration */
#define PAYLOAD_LENGTH      30
#define PACKET_INTERVAL     (uint32_t)(4000000*0.5f) /* Set packet interval to 500ms */


/***** Prototypes *****/
static void txTaskFunction(UArg arg0, UArg arg1);
static void rxTaskFunction(UArg arg0, UArg arg1);
static void callback(RF_Handle h, RF_CmdHandle ch, RF_EventMask e);
static void uartRxTaskFunction(UArg arg0, UArg arg1);
static void uartTxTaskFunction(UArg arg0, UArg arg1);
void uart_writePayLoad(uint8_t *packet, uint8_t length);

/***** Variable declarations *****/
static Task_Params rxTaskParams;
Task_Struct rxTask;    /* not static so you can see in ROV */
static uint8_t rxTaskStack[RX_TASK_STACK_SIZE];

static Task_Params txTaskParams;
Task_Struct txTask;    /* not static so you can see in ROV */
static uint8_t txTaskStack[TX_TASK_STACK_SIZE];

static Task_Params uartRxTaskParams;
Task_Struct uartRxTask;    /* not static so you can see in ROV */
static uint8_t uartRxTaskStack[UART_RX_TASK_STACK_SIZE];

static Task_Params uartTxTaskParams;
Task_Struct uartTxTask;    /* not static so you can see in ROV */
static uint8_t uartTxTaskStack[UART_TX_TASK_STACK_SIZE];

Semaphore_Struct semTxStruct;
Semaphore_Handle semTxHandle;

Semaphore_Struct semRxStruct;
Semaphore_Handle semRxHandle;

Semaphore_Struct semUartRxStruct;
Semaphore_Handle semUartRxHandle;

static RF_Object rfObject;
static RF_Handle rfHandle;
static RF_CmdHandle rfRxCmd;
static RF_CmdHandle rfTxCmd;

UART_Handle uart = NULL;
UART_Params uartParams;


/* Buffer which contains all Data Entries for receiving data.
 * Pragmas are needed to make sure this buffer is 4 byte aligned (requirement from the RF Core) */
#if defined(__TI_COMPILER_VERSION__)
    #pragma DATA_ALIGN (rxDataEntryBuffer, 4);
        static uint8_t rxDataEntryBuffer[RF_QUEUE_DATA_ENTRY_BUFFER_SIZE(NUM_DATA_ENTRIES,
                                                                 MAX_LENGTH,
                                                                 NUM_APPENDED_BYTES)];
#elif defined(__IAR_SYSTEMS_ICC__)
    #pragma data_alignment = 4
        static uint8_t rxDataEntryBuffer[RF_QUEUE_DATA_ENTRY_BUFFER_SIZE(NUM_DATA_ENTRIES,
                                                                 MAX_LENGTH,
                                                                 NUM_APPENDED_BYTES)];
#elif defined(__GNUC__)
        static uint8_t rxDataEntryBuffer [RF_QUEUE_DATA_ENTRY_BUFFER_SIZE(NUM_DATA_ENTRIES,
            MAX_LENGTH, NUM_APPENDED_BYTES)] __attribute__ ((aligned (4)));
#else
    #error This compiler is not supported.
#endif

/* Receive dataQueue for RF Core to fill in data */
static dataQueue_t dataQueue;
static rfc_dataEntryGeneral_t* currentDataEntry;
uint8_t packetReady = 0;
static uint8_t packetLength;
static uint8_t* packetDataPointer;
uint32_t time;
static uint8_t txPacket[PAYLOAD_LENGTH];

static PIN_Handle pinHandle;

static uint8_t packet[MAX_LENGTH + NUM_APPENDED_BYTES - 1]; /* The length byte is stored in a separate variable */


/***** Function definitions *****/
void UartRxTask_init(PIN_Handle ledPinHandle) {
    pinHandle = ledPinHandle;

    Task_Params_init(&uartRxTaskParams);
    uartRxTaskParams.stackSize = UART_RX_TASK_STACK_SIZE;
    uartRxTaskParams.priority = UART_RX_TASK_PRIORITY;
    uartRxTaskParams.stack = &uartRxTaskStack;
    uartRxTaskParams.arg0 = (UInt)1000000;

    Task_construct(&uartRxTask, uartRxTaskFunction, &uartRxTaskParams, NULL);
}

void UartTxTask_init(PIN_Handle ledPinHandle) {
    pinHandle = ledPinHandle;

    Task_Params_init(&uartTxTaskParams);
    uartTxTaskParams.stackSize = UART_TX_TASK_STACK_SIZE;
    uartTxTaskParams.priority = UART_TX_TASK_PRIORITY;
    uartTxTaskParams.stack = &uartTxTaskStack;
    uartTxTaskParams.arg0 = (UInt)1000000;

    Task_construct(&uartTxTask, uartTxTaskFunction, &uartTxTaskParams, NULL);
}

void RxTask_init(PIN_Handle ledPinHandle) {
    pinHandle = ledPinHandle;

    Task_Params_init(&rxTaskParams);
    rxTaskParams.stackSize = RX_TASK_STACK_SIZE;
    rxTaskParams.priority = RX_TASK_PRIORITY;
    rxTaskParams.stack = &rxTaskStack;
    rxTaskParams.arg0 = (UInt)1000000;

    Task_construct(&rxTask, rxTaskFunction, &rxTaskParams, NULL);
}

static void rxTaskFunction(UArg arg0, UArg arg1)
{
    RF_Params rfParams;
    RF_Params_init(&rfParams);

    if( RFQueue_defineQueue(&dataQueue,
                            rxDataEntryBuffer,
                            sizeof(rxDataEntryBuffer),
                            NUM_DATA_ENTRIES,
                            MAX_LENGTH + NUM_APPENDED_BYTES))
    {
        /* Failed to allocate space for all data entries */
        while(1);
    }

    /* Modify CMD_PROP_RX command for application needs */
    RF_cmdPropRx.pQueue = &dataQueue;           /* Set the Data Entity queue for received data */
    RF_cmdPropRx.rxConf.bAutoFlushIgnored = 1;  /* Discard ignored packets from Rx queue */
    RF_cmdPropRx.rxConf.bAutoFlushCrcErr = 1;   /* Discard packets with CRC error from Rx queue */
    RF_cmdPropRx.maxPktLen = MAX_LENGTH;        /* Implement packet length filtering to avoid PROP_ERROR_RXBUF */
    RF_cmdPropRx.pktConf.bRepeatOk = 1;
    RF_cmdPropRx.pktConf.bRepeatNok = 1;

    if (!rfHandle) {
        /* Request access to the radio */
        rfHandle = RF_open(&rfObject, &RF_prop, (RF_RadioSetup*)&RF_cmdPropRadioDivSetup, &rfParams);

        /* Set the frequency */
        RF_postCmd(rfHandle, (RF_Op*)&RF_cmdFs, RF_PriorityNormal, NULL, 0);
    }

    while (1) {
    /* Enter RX mode and stay forever in RX */
        rfRxCmd = RF_postCmd(rfHandle, (RF_Op*)&RF_cmdPropRx, RF_PriorityNormal, &callback, IRQ_RX_ENTRY_DONE);

        Semaphore_pend(semRxHandle, BIOS_WAIT_FOREVER);
    }

}

void callback(RF_Handle h, RF_CmdHandle ch, RF_EventMask e)
{
    if (e & RF_EventRxEntryDone)
    {
        /* Toggle pin to indicate RX */
        PIN_setOutputValue(pinHandle, Board_LED2,!PIN_getOutputValue(Board_LED2));

        /* Get current unhandled data entry */
        currentDataEntry = RFQueue_getDataEntry();

        /* Handle the packet data, located at &currentDataEntry->data:
         * - Length is the first byte with the current configuration
         * - Data starts from the second byte */
        packetLength      = *(uint8_t*)(&currentDataEntry->data);
        packetDataPointer = (uint8_t*)(&currentDataEntry->data + 1);

        /* Copy the payload + the status byte to the packet variable */
        memcpy(packet, packetDataPointer, (packetLength + 1));

        packetReady = 1;

        RFQueue_nextEntry();

        Semaphore_post(semUartRxHandle);
    }
}

static void uartRxTaskFunction(UArg arg0, UArg arg1)
{
    if (uart == NULL) {
        /* Create a UART with data processing off. */
        UART_Params_init(&uartParams);
        uartParams.writeDataMode = UART_DATA_BINARY;
        uartParams.readDataMode = UART_DATA_BINARY;
        uartParams.readReturnMode = UART_RETURN_FULL;
        uartParams.readEcho = UART_ECHO_OFF;
        uartParams.baudRate = 115200;
        uart = UART_open(Board_UART0, &uartParams);

        if (uart == NULL) {
            System_abort("Error opening the UART");
        }
    }

    while (1) {
        Semaphore_pend(semUartRxHandle, BIOS_WAIT_FOREVER);
        if (packetReady) {
            uart_writePayLoad(packetDataPointer, packetLength);
            packetReady = 0;
        }
    }
}

#define UART_TX_BUFFER_SIZE 256
char uartTxBuffer[UART_TX_BUFFER_SIZE];

static void uartTxTaskFunction(UArg arg0, UArg arg1)
{
    char input;
    uint8_t charIndex;

    /* Open UART if not already open */
    if (uart == NULL) {
        /* Create a UART with data processing off. */
        UART_Params_init(&uartParams);
        uartParams.writeDataMode = UART_DATA_BINARY;
        uartParams.readDataMode = UART_DATA_BINARY;
        uartParams.readReturnMode = UART_RETURN_FULL;
        uartParams.readEcho = UART_ECHO_OFF;
        uartParams.baudRate = 115200;
        uart = UART_open(Board_UART0, &uartParams);

        if (uart == NULL) {
            System_abort("Error opening the UART");
        }
    }

    while (1) {

        UART_read(uart, &input, 1);

        /* Skip the line feed that appears after an enter as the first character of new line */
        if (input == (char)CHAR_LINEFEED)
            charIndex--;


        if ((input == (char)CHAR_LINE_END_1) | (input == (char)CHAR_LINE_END_2)) // (charIndex < UART_SERIAL_LINE_SIZE))
        {

            /* Cancel Rx */
            RF_cancelCmd(rfHandle, rfRxCmd, 0);
            Semaphore_post(semTxHandle);
            /* reset index to zero to point to begining of the line */
            charIndex = 0;
        }
        else
        {
            /* Store the input character */
            //uartTxBuffer[charIndex++] = input;
            txPacket[charIndex++] = input;
        }

    }
}

/***** Function definitions *****/
void TxTask_init(PIN_Handle inPinHandle)
{
    pinHandle = inPinHandle;

    Task_Params_init(&txTaskParams);
    txTaskParams.stackSize = TX_TASK_STACK_SIZE;
    txTaskParams.priority = TX_TASK_PRIORITY;
    txTaskParams.stack = &txTaskStack;
    txTaskParams.arg0 = (UInt)1000000;

    Task_construct(&txTask, txTaskFunction, &txTaskParams, NULL);
}


static void txTaskFunction(UArg arg0, UArg arg1)
{
    uint32_t time;
    RF_Params rfParams;
    RF_Params_init(&rfParams);

    RF_cmdPropTx.pktLen = PAYLOAD_LENGTH;
    RF_cmdPropTx.pPkt = txPacket;
    RF_cmdPropTx.startTrigger.triggerType = TRIG_ABSTIME;
    RF_cmdPropTx.startTrigger.pastTrig = 1;
    RF_cmdPropTx.startTime = 0;

    while(1)
    {
        Semaphore_pend(semTxHandle, BIOS_WAIT_FOREVER);

        if (!rfHandle) {
            /* Request access to the radio */
            rfHandle = RF_open(&rfObject, &RF_prop, (RF_RadioSetup*)&RF_cmdPropRadioDivSetup, &rfParams);

            /* Set the frequency */
            RF_postCmd(rfHandle, (RF_Op*)&RF_cmdFs, RF_PriorityNormal, NULL, 0);
        }

        /* Get current time */
        time = RF_getCurrentTime();

        /* Set absolute TX time to utilize automatic power management */
        time += PACKET_INTERVAL;
        RF_cmdPropTx.startTime = time;

        /* Send packet */
        //RF_EventMask result = RF_runCmd(rfHandle, (RF_Op*)&RF_cmdPropTx, RF_PriorityNormal, NULL, 0);

        rfTxCmd = RF_postCmd(rfHandle, (RF_Op*)&RF_cmdPropTx, RF_PriorityNormal, NULL, 0);

        RF_EventMask result = RF_pendCmd(rfHandle, rfTxCmd, (RF_EventCmdDone | RF_EventCmdError | RF_EventLastCmdDone |
                RF_EventCmdAborted | RF_EventCmdCancelled | RF_EventCmdStopped));

        if (!(result & RF_EventLastCmdDone))
        {
            /* Error */
            while(1);
        }

        PIN_setOutputValue(pinHandle, Board_LED1,!PIN_getOutputValue(Board_LED1));

        /* clear txPacket buffer */
        memset(txPacket, 0, sizeof(txPacket));

        Semaphore_post(semRxHandle);

    }
}

/*
 *  ======== main ========
 */
int main(void)
{
    Semaphore_Params semParams;

    /* Call board init functions. */
    Board_initGeneral();
    Board_initUART();

    /* Open LED pins */
    ledPinHandle = PIN_open(&ledPinState, pinTable);
    if(!ledPinHandle)
    {
        System_abort("Error initializing board LED pins\n");
    }

    /* Construct a Semaphore object to be used as a resource lock, inital count 0 */
    Semaphore_Params_init(&semParams);
    Semaphore_construct(&semTxStruct, 0, &semParams);
    Semaphore_construct(&semRxStruct, 0, &semParams);
    Semaphore_construct(&semUartRxStruct, 0, &semParams);


    /* Obtain instance handle */
    semTxHandle = Semaphore_handle(&semTxStruct);
    semRxHandle = Semaphore_handle(&semRxStruct);
    semUartRxHandle = Semaphore_handle(&semUartRxStruct);

    /* Initialize task */
    RxTask_init(ledPinHandle);

    /* Initialize task */
    TxTask_init(ledPinHandle);

    UartRxTask_init(ledPinHandle);

    UartTxTask_init(ledPinHandle);

    /* Start BIOS */
    BIOS_start();

    return (0);
}


//*****************************************************************************
//
//! Print payload to UART
//
//*****************************************************************************

void uart_writePayLoad(uint8_t *packet, uint8_t length) {
    char output[2];

    //UART_write(uart, "rx data: ", 9);
    UART_write(uart, packet, length);

    /* Output a carriage return */
    output[0] = CHAR_LINE_END_1;
    UART_write(uart,"\n",1);
    UART_write(uart, output, 1);

}


