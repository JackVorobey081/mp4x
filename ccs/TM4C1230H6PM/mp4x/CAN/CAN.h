#ifndef CAN_CAN_H_
#define CAN_CAN_H_

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "inc/hw_nvic.h"
#include "inc/hw_ints.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_can.h"

#include "driverlib/can.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"

void InitCAN0();
//void InitCAN0(enum CAN0RxPin RxPin, enum CAN0TxPin TxPin, uint32_t CANSpeed);
//void SendCAN0Message();
//void InitCAN0MBOX();
void CAN0Transmit();
//void CAN0StatusHandler();


#endif /* SRC_CAN_H_ */
