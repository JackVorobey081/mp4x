#include "CAN.h"

tCANMsgObject g_sCAN0RxMessage;
tCANMsgObject g_sCAN0TxMessage;
#define CAN0RXID 0
#define RXOBJECT 1
#define CAN0TXID 2
#define TXOBJECT 2
uint8_t g_ui8TXMsgData;
uint8_t g_ui8RXMsgData;

void InitCAN0(void) {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    GPIOPinConfigure(GPIO_PE4_CAN0RX);
    GPIOPinConfigure(GPIO_PE5_CAN0TX);
    GPIOPinTypeCAN(GPIO_PORTE_BASE, GPIO_PIN_4 | GPIO_PIN_5);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_CAN0);
    CANInit(CAN0_BASE);
    CANBitRateSet(CAN0_BASE, SysCtlClockGet(), 1000000);
    CANIntEnable(CAN0_BASE, CAN_INT_MASTER | CAN_INT_ERROR | CAN_INT_STATUS);
    IntEnable(INT_CAN0);
    CANEnable(CAN0_BASE);
    g_sCAN0RxMessage.ui32MsgID = CAN0RXID;
    g_sCAN0RxMessage.ui32MsgIDMask = 0;
    g_sCAN0RxMessage.ui32Flags = MSG_OBJ_RX_INT_ENABLE | MSG_OBJ_USE_ID_FILTER;
    g_sCAN0RxMessage.ui32MsgLen = sizeof(g_ui8RXMsgData);
    CANMessageSet(CAN0_BASE, RXOBJECT, &g_sCAN0RxMessage, MSG_OBJ_TYPE_RX);
    g_ui8TXMsgData = 0;
    g_sCAN0TxMessage.ui32MsgID = CAN0TXID;
    g_sCAN0TxMessage.ui32MsgIDMask = 0;
    g_sCAN0TxMessage.ui32Flags = MSG_OBJ_TX_INT_ENABLE;
    g_sCAN0TxMessage.ui32MsgLen = sizeof(g_ui8TXMsgData);
    g_sCAN0TxMessage.pui8MsgData = (uint8_t *)&g_ui8TXMsgData;
}

void CAN0Transmit(void) {
    CANMessageSet(CAN0_BASE, TXOBJECT, &g_sCAN0TxMessage, MSG_OBJ_TYPE_RX);
}
