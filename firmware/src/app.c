/*******************************************************************************
 * Copyright (C) 2019 Microchip Technology Inc. and its subsidiaries.
 *
 * Subject to your compliance with these terms, you may use Microchip software
 * and any derivatives exclusively with Microchip products. It is your
 * responsibility to comply with third party license terms applicable to your
 * use of third party software (including open source software) that may
 * accompany Microchip software.
 *
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
 * EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED
 * WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 * IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
 * INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
 * WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
 * BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE
 * FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN
 * ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
 * THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *******************************************************************************/

/*******************************************************************************
  MPLAB Harmony Application Source File

  Company:
    Microchip Technology Inc.

  File Name:
    app.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It
    implements the logic of the application's state machine and it may call
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include "app.h"
#define SERVER_PORT 80
// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************
#define APP_AT24MAC_DEVICE_MACADDR          (0x5f9A)
#define MAC_ADDR_LENGTH (6)
#define TCP_CLIENT_CONNECTION_TIMEOUT_PERIOD_ms 15000

typedef enum {
    MAC_ADDR_READ_STATE_READ,
    MAC_ADDR_READ_STATE_WAIT,
    MAC_ADDR_READ_STATE_SUCCESS,
    MAC_ADDR_READ_STATE_ERROR,
} AT24_MAC_ADDR_READ_STATE;

char macAddr[6];
char macAddrString[18];
extern TCPIP_NETWORK_CONFIG __attribute__((unused)) TCPIP_HOSTS_CONFIGURATION[];
static void AT24_MacAddr_Read(void);
SYS_MODULE_OBJ TCPIP_STACK_Init();

// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the APP_Initialize function.

    Application strings and buffers are be defined outside this structure.
 */

APP_DATA appData;

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary callback functions.
 */

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************


/* TODO:  Add any necessary local functions.
 */

void AT24_MacAddr_Read_Callback(uintptr_t context) {
    AT24_MAC_ADDR_READ_STATE* transferState = (AT24_MAC_ADDR_READ_STATE*) context;

    if (TWIHS0_ErrorGet() == TWIHS_ERROR_NONE) {
        if (transferState) {
            *transferState = MAC_ADDR_READ_STATE_SUCCESS;
        }
    } else {
        if (transferState) {
            *transferState = MAC_ADDR_READ_STATE_ERROR;
        }
    }
}


static void AT24_MacAddr_Read(void) {
    static AT24_MAC_ADDR_READ_STATE state = MAC_ADDR_READ_STATE_READ;
    switch (state) {
        case MAC_ADDR_READ_STATE_READ:
            /* Register the TWIHS Callback with transfer status as context */
            TWIHS0_CallbackRegister(AT24_MacAddr_Read_Callback, (uintptr_t) & state);
            //Initiate Read AT24 MAC Address
            TWIHS0_Read(APP_AT24MAC_DEVICE_MACADDR, (uint8_t *) (macAddr), MAC_ADDR_LENGTH);
            state = MAC_ADDR_READ_STATE_WAIT;
            break;

        case MAC_ADDR_READ_STATE_WAIT:
            break;

        case MAC_ADDR_READ_STATE_SUCCESS:
            //convert MAC address to string format
            TCPIP_Helper_MACAddressToString((const TCPIP_MAC_ADDR*) macAddr, macAddrString, 18);
            //update host configuration with new MAC address
            (TCPIP_HOSTS_CONFIGURATION[0].macAddr) = (char*) macAddrString;
            SYS_CONSOLE_PRINT("MAC TCPIP_HOSTS_CONFIGURATION[0].macAddr: %s\n\r", TCPIP_HOSTS_CONFIGURATION[0].macAddr);
            appData.state = APP_TCPIP_INIT_TCPIP_STACK;
            break;

        case MAC_ADDR_READ_STATE_ERROR:
            // error; use default MAC address
            appData.state = APP_TCPIP_INIT_TCPIP_STACK;
            break;
    }

}

// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_Initialize ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Initialize(void) {
    /* Place the App state machine in its initial state. */
    appData.state = APP_START_CASE;

    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
}

/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Tasks(void) {
    SYS_STATUS tcpipStat;
    const char *netName, *netBiosName;
    static IPV4_ADDR dwLastIP[2] = {
        {-1},
        {-1}
    };
    IPV4_ADDR ipAddr;
    int i, nNets;
    TCPIP_NET_HANDLE netH;

    SYS_CMD_READY_TO_READ();
    switch (appData.state) {
        
        case APP_START_CASE:
            SYS_CONSOLE_PRINT("\n\r==========================================================\r\n");
            SYS_CONSOLE_PRINT("tcpip_tcp_server_freertos_vm_server %s %s\r\n", __DATE__, __TIME__);
            appData.state = APP_TCPIP_INIT_MAC;
            break;        

        case APP_TCPIP_INIT_MAC:
            // Read MAC address 
            AT24_MacAddr_Read();
            break;
        case APP_TCPIP_INIT_TCPIP_STACK:
            // TCPIP Stack Initialization
            sysObj.tcpip = TCPIP_STACK_Init();
            SYS_ASSERT(sysObj.tcpip != SYS_MODULE_OBJ_INVALID, "TCPIP_STACK_Init Failed");
            appData.state = APP_TCPIP_WAIT_INIT;
            break;
            
        case APP_TCPIP_WAIT_INIT:
            tcpipStat = TCPIP_STACK_Status(sysObj.tcpip);
            if (tcpipStat < 0) { // some error occurred
                SYS_CONSOLE_MESSAGE(" APP: TCP/IP stack initialization failed!\r\n");
                appData.state = APP_TCPIP_ERROR;
            } else if (tcpipStat == SYS_STATUS_READY) {
                // now that the stack is ready we can check the
                // available interfaces
                nNets = TCPIP_STACK_NumberOfNetworksGet();
                for (i = 0; i < nNets; i++) {

                    netH = TCPIP_STACK_IndexToNet(i);
                    netName = TCPIP_STACK_NetNameGet(netH);
                    netBiosName = TCPIP_STACK_NetBIOSName(netH);

#if defined(TCPIP_STACK_USE_NBNS)
                    SYS_CONSOLE_PRINT("    Interface %s on host %s - NBNS enabled\r\n", netName, netBiosName);
#else
                    SYS_CONSOLE_PRINT("    Interface %s on host %s - NBNS disabled\r\n", netName, netBiosName);
#endif  // defined(TCPIP_STACK_USE_NBNS)

                }
                SYS_CONSOLE_PRINT("\r\nWaiting for Client Connection on port: %d\r\n", SERVER_PORT);
                appData.state = APP_TCPIP_WAIT_FOR_IP;

            }
            break;

        case APP_TCPIP_WAIT_FOR_IP:

            // if the IP address of an interface has changed
            // display the new value on the system console
            nNets = TCPIP_STACK_NumberOfNetworksGet();

            for (i = 0; i < nNets; i++) {
                netH = TCPIP_STACK_IndexToNet(i);
                if (!TCPIP_STACK_NetIsReady(netH)) {
                    return; // interface not ready yet!
                }
                ipAddr.Val = TCPIP_STACK_NetAddress(netH);
                if (dwLastIP[i].Val != ipAddr.Val) {
                    dwLastIP[i].Val = ipAddr.Val;

                    SYS_CONSOLE_MESSAGE(TCPIP_STACK_NetNameGet(netH));
                    SYS_CONSOLE_MESSAGE(" IP Address: ");
                    SYS_CONSOLE_PRINT("%d.%d.%d.%d \r\n", ipAddr.v[0], ipAddr.v[1], ipAddr.v[2], ipAddr.v[3]);
                }                
                appData.state = APP_TCPIP_OPENING_SERVER;
            }
            break;
        case APP_TCPIP_OPENING_SERVER:
        {
            //SYS_CONSOLE_PRINT("\r\nWaiting for Client Connection on port: %d\r\n", SERVER_PORT);
            appData.socket = TCPIP_TCP_ServerOpen(IP_ADDRESS_TYPE_IPV4, SERVER_PORT, 0);
            if (appData.socket == INVALID_SOCKET) {
                SYS_CONSOLE_MESSAGE("Couldn't open server socket\r\n");
                break;
            }
            appData.state = APP_TCPIP_WAIT_FOR_CONNECTION;
        }
            break;

        case APP_TCPIP_WAIT_FOR_CONNECTION:
        {
            if (!TCPIP_TCP_IsConnected(appData.socket)) {
                return;
            } else {
                // We got a connection
                appData.state = APP_TCPIP_SERVING_CONNECTION;
                //SYS_CONSOLE_MESSAGE("Received a connection\r\n");
            }
        }
            break;

        case APP_TCPIP_SERVING_CONNECTION:
        {
            if (!TCPIP_TCP_IsConnected(appData.socket)) {
                break;
            }
            uint8_t AppBuffer[128 + 1];

            // Transfer the data out of the TCP RX FIFO and into our local processing buffer.
            uint32_t length = TCPIP_TCP_ArrayGet(appData.socket, AppBuffer, sizeof (AppBuffer));
            AppBuffer[length] = 0;

            // Transfer the data out of our local processing buffer and into the TCP TX FIFO.
            SYS_CONSOLE_PRINT("%s\r\n", AppBuffer);

            /*new*/
            appData.state = APP_TCPIP_CLOSING_CONNECTION;
            // No need to perform any flush.  TCP data in TX FIFO will automatically transmit itself after it accumulates for a while.  If you want to decrease latency (at the expense of wasting network bandwidth on TCP overhead), perform and explicit flush via the TCPFlush() API.

        }
            break;
        case APP_TCPIP_CLOSING_CONNECTION:
        {
            // Close the socket connection.
            TCPIP_TCP_Close(appData.socket);
            //SYS_CONSOLE_MESSAGE("Connection was closed\r\n");
            appData.socket = INVALID_SOCKET;
            appData.state = APP_TCPIP_WAIT_FOR_IP;

        }
            break;
        default:
            break;
    }
}


/*******************************************************************************
 End of File
 */
