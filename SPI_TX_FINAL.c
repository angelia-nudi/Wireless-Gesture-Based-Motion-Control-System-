
/*
 *  ======== spiperipheral.c ========
 */
#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

/* POSIX Header files */
#include <pthread.h>
#include <semaphore.h>
#include <unistd.h>
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>

/* Driver Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/SPI.h>
#include <ti/drivers/rf/RF.h>
#include <ti/display/Display.h>

/* Driver configuration */
#include "ti_drivers_config.h"

/* Application Header files */
#include "RFQueue.h"
#include <ti_radio_config.h>

#define THREADSTACKSIZE (1024)

#define SPI_MSG_LENGTH (32)
//#define PERIPHERAL_MSG ("Hello from peripheral, msg#: ")

//#define MAX_LOOP (10)

#ifdef DeviceFamily_CC35XX
    #define CONFIG_GPIO_LED_0 GPIO_INVALID_INDEX
    #define CONFIG_GPIO_LED_1 GPIO_INVALID_INDEX
#endif

static Display_Handle display;

unsigned char peripheralRxBuffer[SPI_MSG_LENGTH];
unsigned char peripheralTxBuffer[SPI_MSG_LENGTH];
typedef struct {
    float roll;
    float pitch;
} GyroData;

static GyroData latestData;
static SemaphoreHandle_t rfSemaphore;
static RF_Object rfObject;

/* Semaphore to block peripheral until transfer is complete */
sem_t peripheralSem;

/* Status indicating whether or not SPI transfer succeeded. */
bool transferStatus;

/*
 *  ======== transferCompleteFxn ========
 *  Callback function for SPI_transfer().
 */
void transferCompleteFxn(SPI_Handle handle, SPI_Transaction *transaction)
{
    if (transaction->status != SPI_TRANSFER_COMPLETED)
    {
        transferStatus = false;
    }
    else
    {
        transferStatus = true;
    }

    sem_post(&peripheralSem);
}

/*
 * ======== peripheralThread ========
 *  Peripheral SPI sends a message to controller while simultaneously receiving a
 *  message from the controller.
 */
void *peripheralThread(void *arg0)
{
    SPI_Handle peripheralSpi;
    SPI_Params spiParams;
    SPI_Transaction transaction;
    uint32_t i;
    bool transferOK;
    int32_t status;

    /*
     * CONFIG_SPI_CONTROLLER_READY & CONFIG_SPI_PERIPHERAL_READY are GPIO pins connected
     * between the controller & peripheral.  These pins are used to synchronize
     * the controller & peripheral applications via a small 'handshake'.  The pins
     * are later used to synchronize transfers & ensure the controller will not
     * start a transfer until the peripheral is ready.  These pins behave
     * differently between spicontroller & spiperipheral examples:
     *
     * spiperipheral example:
     *     * CONFIG_SPI_CONTROLLER_READY is configured as an input pin.  During the
     *       'handshake' this pin is read & a high value will indicate the
     *       controller is ready to run the application.  Afterwards, the pin is
     *       read to determine if the controller has already opened its SPI pins.
     *       The controller will pull this pin low when it has opened its SPI.
     *
     *     * CONFIG_SPI_PERIPHERAL_READY is configured as an output pin.  During the
     *       'handshake' this pin is changed from low to high output.  This
     *       notifies the controller the peripheral is ready to run the application.
     *       Afterwards, the pin is used by the peripheral to notify the controller it
     *       is ready for a transfer.  When ready for a transfer, this pin will
     *       be pulled low.
     *
     * Below we set CONFIG_SPI_CONTROLLER_READY & CONFIG_SPI_PERIPHERAL_READY initial
     * conditions for the 'handshake'.
     */
    GPIO_setConfig(CONFIG_SPI_PERIPHERAL_READY, GPIO_CFG_OUTPUT | GPIO_CFG_OUT_LOW);
    GPIO_setConfig(CONFIG_SPI_CONTROLLER_READY, GPIO_CFG_INPUT);

    /*
     * Handshake - Set CONFIG_SPI_PERIPHERAL_READY high to indicate peripheral is ready
     * to run.  Wait for CONFIG_SPI_CONTROLLER_READY to be high.
     */
    GPIO_write(CONFIG_SPI_PERIPHERAL_READY, 1);
    while (GPIO_read(CONFIG_SPI_CONTROLLER_READY) == 0) {}

    /*
     * Create synchronization semaphore; this semaphore will block the peripheral
     * until a transfer is complete.  The peripheral is configured in callback mode
     * to allow us to configure the SPI transfer & then notify the controller the
     * peripheral is ready.  However, we must still wait for the current transfer
     * to be complete before setting up the next.  Thus, we wait on peripheralSem;
     * once the transfer is complete the callback function will unblock the
     * peripheral.
     */
    status = sem_init(&peripheralSem, 0, 0);
    if (status != 0)
    {
        Display_printf(display, 0, 0, "Error creating peripheralSem\n");

        while (1) {}
    }

    /*
     * Wait until controller SPI is open.  When the controller is configuring SPI pins
     * the clock may toggle from low to high (or high to low depending on
     * polarity).  If using 3-pin SPI & the peripheral has been opened before the
     * controller, clock transitions may cause the peripheral to shift bits out assuming
     * it is an actual transfer.  We can prevent this behavior by opening the
     * controller first & then opening the peripheral. Wait for controller to
     * indicate it's ready by pulling CONFIG_SPI_CONTROLLER_READY low.
     */
    while (GPIO_read(CONFIG_SPI_CONTROLLER_READY)) {}

    /*
     * Open SPI as peripheral in callback mode; callback mode is used to allow us to
     * configure the transfer & then set CONFIG_SPI_PERIPHERAL_READY high.
     */
    SPI_Params_init(&spiParams);
    spiParams.frameFormat         = SPI_POL0_PHA1;
    spiParams.mode                = SPI_PERIPHERAL;
    spiParams.transferCallbackFxn = transferCompleteFxn;
    spiParams.transferMode        = SPI_MODE_CALLBACK;
    spiParams.bitRate             = 1000000;
    peripheralSpi                 = SPI_open(CONFIG_SPI_PERIPHERAL, &spiParams);
    if (peripheralSpi == NULL)
    {
        Display_printf(display, 0, 0, "Error initializing peripheral SPI\n");
        while (1) {}
    }
    else
    {
        Display_printf(display, 0, 0, "Peripheral SPI initialized\n");
    }

    /* Copy message to transmit buffer */
    //strncpy((char *)peripheralTxBuffer, PERIPHERAL_MSG, SPI_MSG_LENGTH);

    //for (i = 0; i < MAX_LOOP; i++)
    while(1)
    {
        /* Initialize peripheral SPI transaction structure */
        //peripheralTxBuffer[sizeof(PERIPHERAL_MSG) - 1] = (i % 10) + '0';
        memset(peripheralTxBuffer, 0, SPI_MSG_LENGTH);
        memset((void *)peripheralRxBuffer, 0, SPI_MSG_LENGTH);
        transaction.count = SPI_MSG_LENGTH;
        transaction.txBuf = (void *)peripheralTxBuffer;
        transaction.rxBuf = (void *)peripheralRxBuffer;

        /* Toggle on user LED, indicating a SPI transfer is in progress */
        GPIO_toggle(CONFIG_GPIO_LED_1);

        /*
         * Setup SPI transfer; CONFIG_SPI_PERIPHERAL_READY will be set low
         * to notify controller the peripheral is ready.
         */
        transferOK = SPI_transfer(peripheralSpi, &transaction);
        if (transferOK)
        {
            GPIO_write(CONFIG_SPI_PERIPHERAL_READY, 0);

            /* Wait until transfer has completed */
            sem_wait(&peripheralSem);

            /*
             * Drive CONFIG_SPI_PERIPHERAL_READY high to indicate peripheral is
             * not ready for another transfer yet.
             */
            GPIO_write(CONFIG_SPI_PERIPHERAL_READY, 1);

            if (transferStatus == false)
            {
                Display_printf(display, 0, 0, "SPI transfer failed!");
            }
            else
            {
                float roll, pitch;
                memcpy(&roll, peripheralRxBuffer, sizeof(float));
                memcpy(&pitch, peripheralRxBuffer + sizeof(float), sizeof(float));
                latestData.roll = roll;
                latestData.pitch = pitch;
                xSemaphoreGive(rfSemaphore);

    //printf("Received SPI Data, Roll: %.2f, Pitch: %.2f\n", roll, pitch);
                usleep(10000);
    //memcpy(peripheralTxBuffer, peripheralRxBuffer, SPI_MSG_LENGTH);

            }
        }
        else
        {
            Display_printf(display, 0, 0, "Unsuccessful peripheral SPI transfer");
        }
    }

    SPI_close(peripheralSpi);

    /* Example complete - set pins to a known state */
    GPIO_setConfig(CONFIG_SPI_CONTROLLER_READY, GPIO_CFG_OUTPUT | GPIO_CFG_OUT_LOW);
    GPIO_write(CONFIG_SPI_PERIPHERAL_READY, 0);

    //Display_printf(display, 0, 0, "\nDone");

    return (NULL);
}

void rfTransmitTask(void *pvParameters) {
    RF_Params rfParams;
    RF_Handle rfHandle;
    RF_CmdHandle rfCmdHandle;
    RF_EventMask terminationReason;
    RF_Op *rfTxCmd;

    // Open RF
    RF_Params_init(&rfParams);
    rfHandle = RF_open(&rfObject, &RF_prop, (RF_RadioSetup*)&RF_cmdPropRadioDivSetup, &rfParams);
    RF_postCmd(rfHandle, (RF_Op*)&RF_cmdFs, RF_PriorityNormal, NULL, 0); // Set frequency

    while (1) {
        // Wait for signal from SPI
        if (xSemaphoreTake(rfSemaphore, portMAX_DELAY) == pdTRUE) {
            uint8_t payload[32] = {0};
            memcpy(payload, &latestData.roll, sizeof(float));
            memcpy(payload + sizeof(float), &latestData.pitch, sizeof(float));

            RF_cmdPropTx.pktLen = 2 * sizeof(float);
            RF_cmdPropTx.pPkt = payload;
            RF_cmdPropTx.startTrigger.triggerType = TRIG_NOW;
            GPIO_toggle(CONFIG_GPIO_LED_0);
            //printf("Received SPI Data within Transmit Task, Roll: %.2f, Pitch: %.2f\n", latestData.roll, latestData.pitch);
            RF_runCmd(rfHandle, (RF_Op*)&RF_cmdPropTx, RF_PriorityNormal, NULL, 0);
        }
    }
}
/*
 *  ======== mainThread ========
 */
void *mainThread(void *arg0)
{
    rfSemaphore = xSemaphoreCreateBinary();
    xTaskCreate(rfTransmitTask, "rfTx", 2048, NULL, 2, NULL);
    pthread_t thread0;
    pthread_attr_t attrs;
    struct sched_param priParam;
    int retc;
    int detachState;

    /* Call driver init functions. */
    Display_init();
    GPIO_init();
    SPI_init();

    /* Configure the LED pins */
    GPIO_setConfig(CONFIG_GPIO_LED_0, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    GPIO_setConfig(CONFIG_GPIO_LED_1, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);

    /* Open the display for output */
    display = Display_open(Display_Type_UART, NULL);
    if (display == NULL)
    {
        /* Failed to open display driver */
        while (1) {}
    }

    /* Turn on user LED */
    GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);

    Display_printf(display, 0, 0, "Starting the SPI peripheral example");
    Display_printf(display,
                   0,
                   0,
                   "This example requires external wires to be "
                   "connected to the header pins. Please see the Readme.html and Board.html for details.\n");

    /* Create application thread */
    pthread_attr_init(&attrs);

    detachState = PTHREAD_CREATE_DETACHED;
    /* Set priority and stack size attributes */
    retc        = pthread_attr_setdetachstate(&attrs, detachState);
    if (retc != 0)
    {
        /* pthread_attr_setdetachstate() failed */
        while (1) {}
    }

    retc |= pthread_attr_setstacksize(&attrs, THREADSTACKSIZE);
    if (retc != 0)
    {
        /* pthread_attr_setstacksize() failed */
        while (1) {}
    }

    /* Create peripheral thread */
    priParam.sched_priority = 1;
    pthread_attr_setschedparam(&attrs, &priParam);

    retc = pthread_create(&thread0, &attrs, peripheralThread, NULL);
    if (retc != 0)
    {
        /* pthread_create() failed */
        while (1) {}
    }

    return (NULL);
}

///*
// *  ======== spiperipheral.c ========
// */
//#include <stddef.h>
//#include <stdbool.h>
//#include <stdint.h>
//#include <string.h>
//
///* POSIX Header files */
//#include <pthread.h>
//#include <semaphore.h>
//#include <unistd.h>
//
///* Driver Header files */
//#include <ti/drivers/GPIO.h>
//#include <ti/drivers/SPI.h>
//#include <ti/display/Display.h>
//
///* Driver configuration */
//#include "ti_drivers_config.h"
//
//#define THREADSTACKSIZE (1024)
//
//#define SPI_MSG_LENGTH (32)
//
//#ifdef DeviceFamily_CC35XX
//    #define CONFIG_GPIO_LED_0 GPIO_INVALID_INDEX
//    #define CONFIG_GPIO_LED_1 GPIO_INVALID_INDEX
//#endif
//
//static Display_Handle display;
//
//unsigned char peripheralRxBuffer[SPI_MSG_LENGTH];
//unsigned char peripheralTxBuffer[SPI_MSG_LENGTH];
//
///* Semaphore to block peripheral until transfer is complete */
//sem_t peripheralSem;
//
///* Status indicating whether or not SPI transfer succeeded. */
//bool transferStatus;
//
///*
// *  ======== transferCompleteFxn ========
// *  Callback function for SPI_transfer().
// */
//void transferCompleteFxn(SPI_Handle handle, SPI_Transaction *transaction)
//{
//    if (transaction->status != SPI_TRANSFER_COMPLETED)
//    {
//        transferStatus = false;
//    }
//    else
//    {
//        transferStatus = true;
//    }
//
//    sem_post(&peripheralSem);
//}
//
///*
// * ======== peripheralThread ========
// *  Peripheral SPI sends a message to controller while simultaneously receiving a
// *  message from the controller.
// */
//void *peripheralThread(void *arg0)
//{
//    SPI_Handle peripheralSpi;
//    SPI_Params spiParams;
//    SPI_Transaction transaction;
//    uint32_t i;
//    bool transferOK;
//    int32_t status;
//
//    GPIO_setConfig(CONFIG_SPI_PERIPHERAL_READY, GPIO_CFG_OUTPUT | GPIO_CFG_OUT_LOW);
//    GPIO_setConfig(CONFIG_SPI_CONTROLLER_READY, GPIO_CFG_INPUT);
//
//    GPIO_write(CONFIG_SPI_PERIPHERAL_READY, 1);
//    while (GPIO_read(CONFIG_SPI_CONTROLLER_READY) == 0) {}
//
//    status = sem_init(&peripheralSem, 0, 0);
//    if (status != 0)
//    {
//        Display_printf(display, 0, 0, "Error creating peripheralSem\n");
//        while (1) {}
//    }
//
//    while (GPIO_read(CONFIG_SPI_CONTROLLER_READY)) {}
//
//    SPI_Params_init(&spiParams);
//    spiParams.frameFormat         = SPI_POL0_PHA1;
//    spiParams.mode                = SPI_PERIPHERAL;
//    spiParams.transferCallbackFxn = transferCompleteFxn;
//    spiParams.transferMode        = SPI_MODE_CALLBACK;
//    spiParams.bitRate             = 1000000;
//    peripheralSpi                 = SPI_open(CONFIG_SPI_PERIPHERAL, &spiParams);
//    if (peripheralSpi == NULL)
//    {
//        Display_printf(display, 0, 0, "Error initializing peripheral SPI\n");
//        while (1) {}
//    }
//    else
//    {
//        Display_printf(display, 0, 0, "Peripheral SPI initialized\n");
//    }
//
//    while (1)
//    {
//        memset(peripheralTxBuffer, 0, SPI_MSG_LENGTH);
//        memset((void *)peripheralRxBuffer, 0, SPI_MSG_LENGTH);
//        transaction.count = SPI_MSG_LENGTH;
//        transaction.txBuf = (void *)peripheralTxBuffer;
//        transaction.rxBuf = (void *)peripheralRxBuffer;
//
//        GPIO_toggle(CONFIG_GPIO_LED_1);
//
//        transferOK = SPI_transfer(peripheralSpi, &transaction);
//        if (transferOK)
//        {
//            GPIO_write(CONFIG_SPI_PERIPHERAL_READY, 0);
//
//            sem_wait(&peripheralSem);
//
//            GPIO_write(CONFIG_SPI_PERIPHERAL_READY, 1);
//
//            if (transferStatus == false)
//            {
//                Display_printf(display, 0, 0, "SPI transfer failed!");
//            }
//            else
//            {
//                float roll, pitch;
//                memcpy(&roll, peripheralRxBuffer, sizeof(float));
//                memcpy(&pitch, peripheralRxBuffer + sizeof(float), sizeof(float));
//
//                printf("Received SPI Data -> Roll: %.2f, Pitch: %.2f\n", roll, pitch);
//            }
//        }
//        else
//        {
//            Display_printf(display, 0, 0, "Unsuccessful peripheral SPI transfer");
//        }
//    }
//
//    SPI_close(peripheralSpi);
//
//    GPIO_setConfig(CONFIG_SPI_CONTROLLER_READY, GPIO_CFG_OUTPUT | GPIO_CFG_OUT_LOW);
//    GPIO_write(CONFIG_SPI_PERIPHERAL_READY, 0);
//
//    Display_printf(display, 0, 0, "\nDone");
//
//    return (NULL);
//}
//
///*
// *  ======== mainThread ========
// */
//void *mainThread(void *arg0)
//{
//    pthread_t thread0;
//    pthread_attr_t attrs;
//    struct sched_param priParam;
//    int retc;
//    int detachState;
//
//    Display_init();
//    GPIO_init();
//    SPI_init();
//
//    GPIO_setConfig(CONFIG_GPIO_LED_0, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
//    GPIO_setConfig(CONFIG_GPIO_LED_1, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
//
//    display = Display_open(Display_Type_UART, NULL);
//    if (display == NULL)
//    {
//        while (1) {}
//    }
//
//    GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);
//
//    Display_printf(display, 0, 0, "Starting the SPI peripheral example");
//    Display_printf(display,
//                   0,
//                   0,
//                   "This example requires external wires to be "
//                   "connected to the header pins. Please see the Readme.html and Board.html for details.\n");
//
//    pthread_attr_init(&attrs);
//
//    detachState = PTHREAD_CREATE_DETACHED;
//    retc        = pthread_attr_setdetachstate(&attrs, detachState);
//    if (retc != 0)
//    {
//        while (1) {}
//    }
//
//    retc |= pthread_attr_setstacksize(&attrs, THREADSTACKSIZE);
//    if (retc != 0)
//    {
//        while (1) {}
//    }
//
//    priParam.sched_priority = 1;
//    pthread_attr_setschedparam(&attrs, &priParam);
//
//    retc = pthread_create(&thread0, &attrs, peripheralThread, NULL);
//    if (retc != 0)
//    {
//        while (1) {}
//    }
//
//    return (NULL);
//}
