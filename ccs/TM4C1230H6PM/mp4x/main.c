
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "inc/hw_nvic.h"
#include "inc/hw_ints.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_can.h"

#include "driverlib/rom.h"
#include "driverlib/ssi.h"
#include "driverlib/can.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "driverlib/systick.h"
#include "driverlib/rom_map.h"

/**
 * main.c
 */
int main(void)
{
    //
    // Set the system clock to run at 50MHz from the PLL
    //
    MAP_SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_XTAL_24MHZ |
                       SYSCTL_OSC_MAIN);

    //
    // Initialize CAN0
    //
    InitCAN0();

    //
    // Transmit message CAN0
    //
    CAN0Transmit();


    //---------------------------------------------------------------//

    while(1)
    {


    }
}
