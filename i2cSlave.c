#include <stdint.h>
#include <stddef.h>
#include <unistd.h>
#include <stdio.h>
#include <ti/drivers/GPIO.h>
#include <ti/drivers/I2C.h>
#include "ti_drivers_config.h"

#include <ti/devices/cc13x2x7_cc26x2x7/inc/hw_memmap.h>
#include <ti/devices/cc13x2x7_cc26x2x7/inc/hw_i2c.h>
#include <ti/devices/cc13x2x7_cc26x2x7/inc/hw_types.h>
#include <ti/devices/cc13x2x7_cc26x2x7/driverlib/prcm.h>
#include <ti/devices/cc13x2x7_cc26x2x7/inc/hw_wdt.h>
#include <ti/devices/cc13x2x7_cc26x2x7/inc/hw_ioc.h>

#define SLAVE_ADDRESS 0x30

static bool i2c_initialized = false;

I2C_Handle i2cHandle = NULL; // Declare i2cHandle outside

void *mainThread(void *arg0)
{
    if (!i2c_initialized) {
        printf("\n===== CC1312R7 I2C Slave Startup =====\n");

        /* Disable Watchdog Timer */
        HWREG(WDT_BASE + WDT_O_CTL) = 0x0;
        printf("Watchdog Timer Disabled.\n");

        I2C_Handle i2cHandle;
        I2C_Params i2cParams;

        /* Initialize GPIO and I2C */
        printf("Initializing GPIO and I2C...\n");
        GPIO_init();
        I2C_init();
        printf("GPIO and I2C Initialized.\n");

        /* Enable I2C Peripheral Clock */
        printf("Enabling I2C Peripheral Clock...\n");
        PRCMPeripheralRunEnable(PRCM_PERIPH_I2C0);
        PRCMLoadSet();
        while (!PRCMLoadGet());  // Wait for power domain to be active
        printf("I2C Peripheral Clock Enabled.\n");


        uint32_t base = I2C0_BASE;

        /* Step 1: Route I2C_SCL (DIO4) and I2C_SDA (DIO5) to the I2C module */
        HWREG(IOC_BASE + IOC_O_IOCFG4) = (IOC_IOCFG0_PORT_ID_I2C_MSSCL | IOC_IOCFG0_IOMODE_OPENDR);
        HWREG(IOC_BASE + IOC_O_IOCFG5) = (IOC_IOCFG0_PORT_ID_I2C_MSSDA | IOC_IOCFG0_IOMODE_OPENDR);
        printf("SDA and SCL routed to I2C module.\n");

        /* Step 2: Open I2C Peripheral */
        printf("Opening I2C...\n");
        I2C_Params_init(&i2cParams);
        i2cParams.transferMode = I2C_MODE_BLOCKING;
        i2cParams.bitRate = I2C_100kHz;
        i2cHandle = I2C_open(CONFIG_I2C_TMP, &i2cParams);
        if (i2cHandle == NULL) {
            printf("Error: I2C Initialization Failed!\n");
            return NULL;
             }
        printf("I2C opened successfully.\n");

        /* Step 3: Configure Slave Mode */
        printf("Configuring I2C as Slave...\n");

        // Disable both Master and Slave first
        HWREG(base + I2C_O_MCR) = 0x00;


        // Enable slave mode only (disable master mode)
        HWREG(base + I2C_O_MCR) = I2C_MCR_SFE;

        // Verify I2C_MCR is correctly set
        uint32_t mcr_value = HWREG(base + I2C_O_MCR);
        printf("I2C_MCR Set to: 0x%X\n", mcr_value);

        // Configure slave address
        HWREG(base + I2C_O_SOAR) = SLAVE_ADDRESS;

        // Enable slave interrupts
        //HWREG(base + I2C_O_SIMR) = 0x07; // Enable Start, Stop, and Data interrupts

        // Debugging prints

        printf("I2C_MCR = 0x%1X\n", HWREG(base + I2C_O_MCR));
        printf("I2C_SOAR = 0x%1X\n", HWREG(base + I2C_O_SOAR));

        // Finally, enable the Slave mode
        HWREG(base + I2C_O_SCTL) = 0x01;  // Enable Slave Mode
        printf("I2C Slave Mode Enabled.\n");  // Do NOT check I2C_O_SCTL value


        printf("I2C Slave Configured. Waiting for Master Requests...\n");

        while (1) {
        /* Step 4: Read I2C_SSTAT to check if data is being requested */
        uint32_t sstat = HWREG(base + I2C_O_SSTAT);
        printf("I2C Slave Status: SSTAT = 0x%X\n", sstat);

        if (HWREG(base + I2C_O_SSTAT) & I2C_SSTAT_TREQ) {
            printf("TREQ set! Writing dummy data...\n");
            HWREG(base + I2C_O_SDR) = 0x55;  // Dummy data
        }
        }
    }

    return NULL;
}

