#include <stdlib.h>
#include <stdio.h>
#include <xdc/std.h>
#include <xdc/cfg/global.h>
#include <xdc/runtime/System.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/drivers/rf/RF.h>
#include <ti/drivers/PIN.h>
#include <driverlib/rf_prop_mailbox.h>
#include <ti/drivers/UART.h>
#include <ti/drivers/GPIO.h>
#include <ti/drivers/I2C.h>
#include "Board.h"
#include "RFQueue.h"
#include "smartrf_settings/smartrf_settings.h"
#include <stdint.h>
#include <stddef.h>
#include <unistd.h>
#include <string.h>
#include <stdbool.h>

/* Pin driver handle */
static PIN_Handle ledPinHandle;
static PIN_State ledPinState;
// Default Address of CCS811 is 0x5A
#define SLAVE_ADDRESS 0x5A
//#define SLAVE_ADDRESS 0x5B

// Device ID/ADDRESS of CCS811
#define DEVICE_ID_ADDRESS   0x20
#define DEVICE_HW_ID        0x81

// CCS811 MEAS_MODE
// Sets Measurement and Condition
// Page 17 of CCS811 Datasheet
// Will set to 0100_0XXX -> 0x40
// 0001 0xxx
// TESTING 0010_0XXX -> 0x20
// Samples every 250ms, no interrupts
#define MEAS_MODE_REGISTER 0x01 // Address of MEAS
#define MEAS_MODE_SET 0x10  // Settings for MEAS_MODE as described above

 /* Application LED pin configuration table:
 *   - All LEDs board LEDs are off.
 */
PIN_Config pinTable[] =
{
    Board_LED1 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    Board_LED2 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    PIN_TERMINATE
};

enum rf_mode
{
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
#define TX_TASK_STACK_SIZE 1024
#define TX_TASK_PRIORITY   1

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
void uart_writePayLoad(uint8_t *packet, uint8_t length);
void doTransmission(char*);
void outputText(char*); // Simplified Version of UART_write

static Task_Params txTaskParams;
Task_Struct txTask;    /* not static so you can see in ROV */
static uint8_t txTaskStack[TX_TASK_STACK_SIZE];

static RF_Object rfObject;
static RF_Handle rfHandle;
static RF_CmdHandle rfRxCmd;
static RF_CmdHandle rfTxCmd;

UART_Handle uart = NULL;
UART_Params uartParams;

// I2C declarations
uint8_t         txBuffer[10];
uint8_t         rxBuffer[10];
I2C_Handle      i2c;
I2C_Params      i2cParams;
I2C_Transaction i2cTransaction;
uint8_t read8(uint8_t,char*);
char s[20];
uint16_t carbonDioxide; // Store carbonDioxide reading, 2bytes

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

uint8_t packetReady = 0;

uint32_t time;
static uint8_t txPacket[PAYLOAD_LENGTH];

static PIN_Handle pinHandle;

/***** My definitions *****/
#define TASKSTACKSIZE       640
#define Board_I2C_TMP       CC1350_LAUNCHXL_I2C0

// Reverse an array of characters
// Input: An array of characters to reverse
// Output: Null with passed string to reverse
void reverseArray(char s[])
{
    int i, j;
    char c;
    for (i = 0, j = strlen(s)-1; i<j; i++, j--)
    {
        c = s[i];
        s[i] = s[j];
        s[j] = c;
    }
}

// Convert an integer to an array of characters (string)
// Input: integer to convert, array to store characters
// Output: returns char*
void intToString(int n, char s[])
{
    int i, sign;
    if ((sign = n) < 0)  /* record sign */
        n = -n;          /* make n positive */
    i = 0;
    do {       /* generate digits in reverse order */
        s[i++] = n % 10 + '0';   /* get next digit */
    } while ((n /= 10) > 0);     /* delete it */
    if (sign < 0)
        s[i++] = '-';
    s[i] = '\0';
    reverseArray(s);
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
    /* Open UART if not already open */
       if (uart == NULL)
       {
           /* Create a UART with data processing off. */
           UART_Params_init(&uartParams);
           uartParams.writeDataMode = UART_DATA_BINARY;
           uartParams.readDataMode = UART_DATA_BINARY;
           uartParams.readReturnMode = UART_RETURN_FULL;
           uartParams.readEcho = UART_ECHO_OFF;
           uartParams.baudRate = 115200;
           uart = UART_open(Board_UART0, &uartParams);

           if (uart == NULL)
           {
               System_abort("Error opening the UART");
           }
       } // end if (uart == NULL)

    I2C_init();
    outputText("CCS811 Begin\r\n");

    /* Create I2C for usage */
   I2C_Params_init(&i2cParams);
   i2cParams.bitRate = I2C_400kHz;
   i2c = I2C_open(Board_I2C_TMP, &i2cParams);

   if (i2c == NULL)
   {
       outputText("Error Initializing I2C\r\n");
       while (1);
   } // end if (i2c == NULL) begin else
   else
   {
       outputText("I2C Initialized!\r\n");
   } // end else

   uint8_t i = 0; // Used as counter if needed

   //Soft reset of sensor
   txBuffer[0] = 0xFF;
   txBuffer[1] = 0x11;
   txBuffer[2] = 0xE5;
   txBuffer[3] = 0x72;
   txBuffer[4] = 0x8A;
   i2cTransaction.slaveAddress = SLAVE_ADDRESS;
   i2cTransaction.writeBuf = txBuffer;
   i2cTransaction.writeCount = 5;
   i2cTransaction.readBuf = rxBuffer;
   i2cTransaction.readCount = 1;
   if (!I2C_transfer(i2c,&i2cTransaction))
   {
       outputText("SW ERROR");
   } // end if: (!I2C_transfer(i2c,&i2cTransaction))

   //Ensure APP is started
   // Simple Write to Address Starts the App
   txBuffer[0] = 0xF4;
   i2cTransaction.slaveAddress = SLAVE_ADDRESS;
   i2cTransaction.writeBuf = txBuffer;
   i2cTransaction.writeCount = 1;
   i2cTransaction.readBuf = rxBuffer;
   i2cTransaction.readCount = 1;
   if (!I2C_transfer(i2c,&i2cTransaction))
   {
       outputText("APP START ERROR");
   } // end if: (!I2C_transfer(i2c,&i2cTransaction))

   //MEAS_MODE_REGISTER SETTING
   txBuffer[0] = 0x01;
   txBuffer[1] = 0x10;
   i2cTransaction.slaveAddress = SLAVE_ADDRESS;
   i2cTransaction.writeBuf = txBuffer;
   i2cTransaction.writeCount = 2;
   i2cTransaction.readBuf = rxBuffer;
   i2cTransaction.readCount = 1;
   if (!I2C_transfer(i2c,&i2cTransaction))
   {
       outputText("MEAS_MODE ERROR");
   } // end if: (!I2C_transfer(i2c,&i2cTransaction))


  while(1)
  {
      // Check Who Am I Register
      if (DEVICE_HW_ID != read8(DEVICE_ID_ADDRESS,"Error reading device ID")) // Device ID
      {
          outputText("Error reading device ID or Incorrect device ID read\r\n");
          while(1);
      } // end if (DEVICE_ID != read8(0x20,"Error reading device ID"))
//      else // for testing purposes to ensure device is being read
//      {
//          outputText("Found correct device ID\r\n");
//      }

      // READ ALG_RESULT_DATA
      // Read C02 Data
      txBuffer[0] = 0x02;
      i2cTransaction.slaveAddress = SLAVE_ADDRESS;
      i2cTransaction.writeBuf = txBuffer;
      i2cTransaction.writeCount = 1;
      i2cTransaction.readBuf = rxBuffer;
      i2cTransaction.readCount = 2;
      if (!I2C_transfer(i2c,&i2cTransaction))
      {
          outputText("ALG_RESULT_DATA READ ERROR");
      } // end if: (!I2C_transfer(i2c,&i2cTransaction))
      carbonDioxide = (uint16_t)rxBuffer[0] << 8 | ((uint16_t)rxBuffer[1]);
      carbonDioxide = carbonDioxide;
      intToString(carbonDioxide,s);
      s[strlen(s)] = '\n';
      outputText("\r");
      outputText(s);
// FOR TESTING
//      for (i=0;i<20;i++)
//          s[i] = '\0';
      doTransmission(s);
  } // end while(1)
}


//function: int main(void)
int main(void)
{
    /* Call board init functions. */
    Board_initGeneral();
    Board_initUART();
    /* Open LED pins */
    /* Alternating LEDs show connection and packet transmission occurring */
    ledPinHandle = PIN_open(&ledPinState, pinTable);
    if(!ledPinHandle)
    {
        System_abort("Error initializing board LED pins\n");
    }
    /* Initialize task */
    TxTask_init(ledPinHandle);
    /* Start BIOS */
    BIOS_start();
    return (0);
} // end function: int main(void)


//! Print payload to UART
void uart_writePayLoad(uint8_t *packet, uint8_t length)
{
    char output[2];
    //UART_write(uart, "rx data: ", 9);
    UART_write(uart, packet, length);
    /* Output a carriage return */
    output[0] = '\n';
    UART_write(uart, output, 1);
} // end function: void uart_writePayLoad(uint8_t *packet, uint8_t length)

void doTransmission(char* passedValue)
{
    uint32_t time;
    RF_Params rfParams;
    RF_Params_init(&rfParams);

    RF_cmdPropTx.pktLen = PAYLOAD_LENGTH;
    RF_cmdPropTx.pPkt = txPacket;
    RF_cmdPropTx.startTrigger.triggerType = TRIG_ABSTIME;
    RF_cmdPropTx.startTrigger.pastTrig = 1;
    RF_cmdPropTx.startTime = 0;

    char input;
    uint8_t charIndex = 0;

    char *textToSend = passedValue;
    //textToSend[sizeof(textToSend)]= '\n';
    while (1) {
        input = textToSend[charIndex];
        //UART_read(uart, &input, 1);
        if (input == (char)CHAR_LINEFEED) // (charIndex < UART_SERIAL_LINE_SIZE))
        {
            /* Cancel Rx */
            RF_cancelCmd(rfHandle, rfRxCmd, 0);
            /* reset index to zero to point to beginning of the line */
            charIndex = 0;
            break;
        } // end if
        else
        {
            /* Store the input character */
            //uartTxBuffer[charIndex++] = input;
            txPacket[charIndex++] = input;
        } // end else
    } // end while(1)

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
}

// I2C Read Single Byte
// Input[0]: Address to read
// Input[1]: Text to be displayed if cannot be read
// Output: Returns value from register address
uint8_t read8(uint8_t addr,char* textOnError)
{
        txBuffer[0] = addr;
        i2cTransaction.slaveAddress = SLAVE_ADDRESS;
        i2cTransaction.writeBuf = txBuffer;
        i2cTransaction.writeCount = 1;
        i2cTransaction.readBuf = rxBuffer;
        i2cTransaction.readCount = 1;
        if (!I2C_transfer(i2c,&i2cTransaction))
        {
            outputText(textOnError);
        } // end if: (!I2C_transfer(i2c,&i2cTransaction))
        return rxBuffer[0];

}

// Function: void outputText(char*)
// Condensed version of UART_write to output data to terminal
// Primary use: testing
// Input: array of chars / string
// Output: null / output to terminal
void outputText(char* toTerminal)
{
    UART_write(uart,toTerminal,strlen(toTerminal));
} // end function: void outputText(char* toTerminal)
