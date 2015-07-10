/**************************************************************************************************
Filename:       SampleApp.c
Revised:        $Date: 2009-03-18 15:56:27 -0700 (Wed, 18 Mar 2009) $
Revision:       $Revision: 19453 $

Description:    Sample Application (no Profile).


Copyright 2007 Texas Instruments Incorporated. All rights reserved.

IMPORTANT: Your use of this Software is limited to those specific rights
granted under the terms of a software license agreement between the user
who downloaded the software, his/her employer (which must be your employer)
and Texas Instruments Incorporated (the "License").  You may not use this
Software unless you agree to abide by the terms of the License. The License
limits your use, and you acknowledge, that the Software may not be modified,
copied or distributed unless embedded on a Texas Instruments microcontroller
or used solely and exclusively in conjunction with a Texas Instruments radio
frequency transceiver, which is integrated into your product.  Other than for
the foregoing purpose, you may not use, reproduce, copy, prepare derivative
works of, modify, distribute, perform, display or sell this Software and/or
its documentation for any purpose.

YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
PROVIDED 揂S IS?WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

Should you have any questions regarding your right to use this Software,
contact Texas Instruments Incorporated at www.TI.com.
**************************************************************************************************/

/*********************************************************************
This application isn't intended to do anything useful, it is
intended to be a simple example of an application's structure.

This application sends it's messages either as broadcast or
broadcast filtered group messages.  The other (more normal)
message addressing is unicast.  Most of the other sample
applications are written to support the unicast message model.

Key control:
SW1:  Sends a flash command to all devices in Group 1.
SW2:  Adds/Removes (toggles) this device in and out
of Group 1.  This will enable and disable the
reception of the flash command.
*********************************************************************/

/*********************************************************************
* INCLUDES
*/
#include "OSAL.h"
#include "ZGlobals.h"
#include "AF.h"
#include "aps_groups.h"
#include "ZDApp.h"

#include "SampleApp.h"
#include "SampleAppHw.h"

#include "OnBoard.h"

/* HAL */
#include "hal_lcd.h"
#include "hal_led.h"
#include "hal_key.h"
#include "MT_UART.h"
#include "MT_APP.h"
#include "MT.h"


#include "OSAL_PwrMgr.h"
/*********************************************************************
* MACROS
*/

/*********************************************************************
* CONSTANTS
*/

#if !defined( SAMPLE_APP_PORT )
#define SAMPLE_APP_PORT  0
#endif

#if !defined( SAMPLE_APP_BAUD )
//#define SAMPLE_APP_BAUD  HAL_UART_BR_38400
#define SAMPLE_APP_BAUD  HAL_UART_BR_9600
#endif

// When the Rx buf space is less than this threshold, invoke the Rx callback.
#if !defined( SAMPLE_APP_THRESH )
#define SAMPLE_APP_THRESH  64
#endif

#if !defined( SAMPLE_APP_RX_SZ )
#define SAMPLE_APP_RX_SZ  128
#endif

#if !defined( SAMPLE_APP_TX_SZ )
#define SAMPLE_APP_TX_SZ  128
#endif

// Millisecs of idle time after a byte is received before invoking Rx callback.
#if !defined( SAMPLE_APP_IDLE )
#define SAMPLE_APP_IDLE  6
#endif

// Loopback Rx bytes to Tx for throughput testing.
#if !defined( SAMPLE_APP_LOOPBACK )
#define SAMPLE_APP_LOOPBACK  FALSE
#endif

// This is the max byte count per OTA message.
#if !defined( SAMPLE_APP_TX_MAX )
#define SAMPLE_APP_TX_MAX  80
#endif

#define SAMPLE_APP_RSP_CNT  4


static uint16 Nodeid = 2;
static uint8 packagetype = 4;
/*********************************************************************
* TYPEDEFS
*/

/*********************************************************************
* GLOBAL VARIABLES
*/

// This list should be filled with Application specific Cluster IDs.
const cId_t SampleApp_ClusterList[SAMPLEAPP_MAX_CLUSTERS] =
{
  SAMPLEAPP_PERIODIC_CLUSTERID,
  SAMPLEAPP_FLASH_CLUSTERID
};

const SimpleDescriptionFormat_t SampleApp_SimpleDesc =
{
  SAMPLEAPP_ENDPOINT,              //  int Endpoint;
  SAMPLEAPP_PROFID,                //  uint16 AppProfId[2];
  SAMPLEAPP_DEVICEID,              //  uint16 AppDeviceId[2];
  SAMPLEAPP_DEVICE_VERSION,        //  int   AppDevVer:4;
  SAMPLEAPP_FLAGS,                 //  int   AppFlags:4;
  SAMPLEAPP_MAX_CLUSTERS,          //  uint8  AppNumInClusters;
  (cId_t *)SampleApp_ClusterList,  //  uint8 *pAppInClusterList;
  SAMPLEAPP_MAX_CLUSTERS,          //  uint8  AppNumInClusters;
  (cId_t *)SampleApp_ClusterList   //  uint8 *pAppInClusterList;
};

// This is the Endpoint/Interface description.  It is defined here, but
// filled-in in SampleApp_Init().  Another way to go would be to fill
// in the structure here and make it a "const" (in code space).  The
// way it's defined in this sample app it is define in RAM.
endPointDesc_t SampleApp_epDesc;

/*********************************************************************
* EXTERNAL VARIABLES
*/

/*********************************************************************
* EXTERNAL FUNCTIONS
*/
extern void osal_pwrmgr_device(uint8 pwrmgr_device);
extern uint8 osal_pwrmgr_task_state( uint8 task_id, uint8 state );
/*********************************************************************
* LOCAL VARIABLES
*/
uint8 SampleApp_TaskID;   // Task ID for internal task/event processing
// This variable will be received when
// SampleApp_Init() is called.
devStates_t SampleApp_NwkState;

uint8 SampleApp_TransID;  // This is the unique message ID (counter)

afAddrType_t SampleApp_Periodic_DstAddr;
afAddrType_t SampleApp_Flash_DstAddr;

aps_Group_t SampleApp_Group;

uint8 SampleAppPeriodicCounter = 0;
uint8 SampleAppFlashCounter = 0;


bool ifSendData = false;
//static uint8 SampleApp_TxSeq;
static uint8 SampleApp_TxBuf[SAMPLE_APP_TX_MAX+1];
static uint8 SampleApp_TxLen;

int flag;
uint16 count;
uint8 finaldata[26];
uint8 packagedata[26];
uint8 Let_On_Cmd[6] = {'#',0x00,0xAA,0xAA,'!'};
uint8 Let_Off_Cmd[6] = {'#',0x00,0xAB,0xAB,'!'};

/*********************************************************************
* LOCAL FUNCTIONS
*/
void SampleApp_HandleKeys( uint8 shift, uint8 keys );
void SampleApp_MessageMSGCB( afIncomingMSGPacket_t *pckt );
void SampleApp_SendPeriodicMessage( void );
void SampleApp_SendFlashMessage( uint16 flashTime );
void SampleApp_SerialCMD(mtOSALSerialData_t *cmdMsg);


////////////////////
void Delay_10us(void) ;
void Delay_ms(int time);
void Delay_s(int ms);

static void SampleApp_Send(void);
static void SampleApp_CallBack(uint8 port, uint8 event); 

void datainit(uint8 *array);
void pocessdata(uint8 *array_in,uint8 lenth);
uint8 errorflag(uint8 *shu1);
/*********************************************************************
* NETWORK LAYER CALLBACKS
*/

/*********************************************************************
* PUBLIC FUNCTIONS
*/

/*********************************************************************
* @fn      SampleApp_Init
*
* @brief   Initialization function for the Generic App Task.
*          This is called during initialization and should contain
*          any application specific initialization (ie. hardware
*          initialization/setup, table initialization, power up
*          notificaiton ... ).
*
* @param   task_id - the ID assigned by OSAL.  This ID should be
*                    used to send messages and set timers.
*
* @return  none
*/
void SampleApp_Init( uint8 task_id )
{ 
  halUARTCfg_t uartConfig;
  SampleApp_TaskID = task_id;
  SampleApp_NwkState = DEV_INIT;
  SampleApp_TransID = 0;
  
  //初始化io  
  P0SEL &= 0xBF;
  P0DIR |= 0x40;
  //
  
  //  MT_UartInit();                  //串口初始化
  uartConfig.configured           = TRUE;              // 2x30 don't care - see uart driver.
  uartConfig.baudRate             = SAMPLE_APP_BAUD;
  uartConfig.flowControl          = FALSE;
  uartConfig.flowControlThreshold = SAMPLE_APP_THRESH; // 2x30 don't care - see uart driver.
  uartConfig.rx.maxBufSize        = SAMPLE_APP_RX_SZ;  // 2x30 don't care - see uart driver.
  uartConfig.tx.maxBufSize        = SAMPLE_APP_TX_SZ;  // 2x30 don't care - see uart driver.
  uartConfig.idleTimeout          = SAMPLE_APP_IDLE;   // 2x30 don't care - see uart driver.
  uartConfig.intEnable            = TRUE;              // 2x30 don't care - see uart driver.
  uartConfig.callBackFunc         = SampleApp_CallBack;
  HalUARTOpen (SAMPLE_APP_PORT, &uartConfig);
  
  MT_UartRegisterTaskID(task_id); //注册串口任务
  HalUARTWrite(0,"UartInit OK\n", sizeof("UartInit OK\n"));//提示信息
  
  osal_pwrmgr_task_state(task_id,PWRMGR_CONSERVE);
  // Device hardware initialization can be added here or in main() (Zmain.c).
  // If the hardware is application specific - add it here.
  // If the hardware is other parts of the device add it in main().
  
#if defined ( BUILD_ALL_DEVICES )
  // The "Demo" target is setup to have BUILD_ALL_DEVICES and HOLD_AUTO_START
  // We are looking at a jumper (defined in SampleAppHw.c) to be jumpered
  // together - if they are - we will start up a coordinator. Otherwise,
  // the device will start as a router.
  if ( readCoordinatorJumper() )
    zgDeviceLogicalType = ZG_DEVICETYPE_COORDINATOR;
  else
    zgDeviceLogicalType = ZG_DEVICETYPE_ROUTER;
#endif // BUILD_ALL_DEVICES
  
#if defined ( HOLD_AUTO_START )
  // HOLD_AUTO_START is a compile option that will surpress ZDApp
  //  from starting the device and wait for the application to
  //  start the device.
  ZDOInitDevice(0);
#endif
  
  // Setup for the periodic message's destination address
  // Broadcast to everyone
  SampleApp_Periodic_DstAddr.addrMode = (afAddrMode_t)AddrBroadcast;
  SampleApp_Periodic_DstAddr.endPoint = SAMPLEAPP_ENDPOINT;
  SampleApp_Periodic_DstAddr.addr.shortAddr = 0xFFFF;
  
  // Setup for the flash command's destination address - Group 1
  SampleApp_Flash_DstAddr.addrMode = (afAddrMode_t)afAddrGroup;
  SampleApp_Flash_DstAddr.endPoint = SAMPLEAPP_ENDPOINT;
  SampleApp_Flash_DstAddr.addr.shortAddr = SAMPLEAPP_FLASH_GROUP;
  
  // Fill out the endpoint description.
  SampleApp_epDesc.endPoint = SAMPLEAPP_ENDPOINT;
  SampleApp_epDesc.task_id = &SampleApp_TaskID;
  SampleApp_epDesc.simpleDesc
    = (SimpleDescriptionFormat_t *)&SampleApp_SimpleDesc;
  SampleApp_epDesc.latencyReq = noLatencyReqs;
  
  // Register the endpoint description with the AF
  afRegister( &SampleApp_epDesc );
  
  // Register for all key events - This app will handle all key events
  RegisterForKeys( SampleApp_TaskID );
  
  // By default, all devices start out in Group 1
  SampleApp_Group.ID = 0x0001;
  osal_memcpy( SampleApp_Group.name, "Group 1", 7 );
  aps_AddGroup( SAMPLEAPP_ENDPOINT, &SampleApp_Group );
  
#if defined ( LCD_SUPPORTED )
  HalLcdWriteString( "SampleApp", HAL_LCD_LINE_1 );
#endif
}

/*********************************************************************
* @fn      SampleApp_ProcessEvent
*
* @brief   Generic Application Task event processor.  This function
*          is called to process all events for the task.  Events
*          include timers, messages and any other user defined events.
*
* @param   task_id  - The OSAL assigned task ID.
* @param   events - events to process.  This is a bit map and can
*                   contain more than one event.
*
* @return  none
*/
uint16 SampleApp_ProcessEvent( uint8 task_id, uint16 events )
{
  afIncomingMSGPacket_t *MSGpkt;
  (void)task_id;  // Intentionally unreferenced parameter
  
  if ( events & SYS_EVENT_MSG )
  {
    MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( SampleApp_TaskID );
    while ( MSGpkt )
    {
      switch ( MSGpkt->hdr.event )
      {        
        // Received when a key is pressed
      case KEY_CHANGE:
        SampleApp_HandleKeys( ((keyChange_t *)MSGpkt)->state, ((keyChange_t *)MSGpkt)->keys );
        break;
        
        // Received when a messages is received (OTA) for this endpoint
      case AF_INCOMING_MSG_CMD:
        SampleApp_MessageMSGCB( MSGpkt );
        break;
        
        // Received whenever the device changes state in the network
      case ZDO_STATE_CHANGE:
        SampleApp_NwkState = (devStates_t)(MSGpkt->hdr.status);
        if ( //(SampleApp_NwkState == DEV_ZB_COORD) ||
            (SampleApp_NwkState == DEV_ROUTER)
              || (SampleApp_NwkState == DEV_END_DEVICE) )
        {
          // Start sending the periodic message in a regular interval.
          osal_start_timerEx( SampleApp_TaskID,
                             SAMPLEAPP_SEND_COUNT_EVT,
                             SAMPLEAPP_SEND_PERIODIC_MSG_TIMEOUT );
        }
        else
        {
          // Device is no longer in the network
        }
        break;
        
      default:
        break;
      }
      
      // Release the memory
      osal_msg_deallocate( (uint8 *)MSGpkt );
      
      // Next - if one is available
      MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( SampleApp_TaskID );
    }
    
    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }
  
  if(events & SAMPLEAPP_SEND_COUNT_EVT){
    if(flag < 10){
      flag++;
      osal_start_timerEx( SampleApp_TaskID,
                         SAMPLEAPP_SEND_COUNT_EVT,
                         SAMPLEAPP_SEND_PERIODIC_MSG_TIMEOUT );
    }
    else{
      flag = 0;
      osal_start_timerEx( SampleApp_TaskID,
                         SAMPLEAPP_SEND_PERIODIC_MSG_EVT,
                         SAMPLEAPP_SEND_PERIODIC_MSG_TIMEOUT );
    }
    return (events ^ SAMPLEAPP_SEND_COUNT_EVT);
  }
  
  // Send a message out - This event is generated by a timer
  //  (setup in SampleApp_Init()).
  if ( events & SAMPLEAPP_SEND_PERIODIC_MSG_EVT )
  {
    if(count > 60000){count = 0;}
    count++;
    datainit(packagedata);
    // Send the periodic message
    SampleApp_SendPeriodicMessage();
    
    // Setup to send message again in normal period (+ a little jitter)
    osal_start_timerEx( SampleApp_TaskID, SAMPLEAPP_SEND_COUNT_EVT,
                       (SAMPLEAPP_SEND_PERIODIC_MSG_TIMEOUT + (osal_rand() & 0x00FF)) );
    
    // return unprocessed events
    return (events ^ SAMPLEAPP_SEND_PERIODIC_MSG_EVT);
  }
  
  
  if ( events & SAMPLEAPP_SEND_EVT )
  {
    HalLedSet (HAL_LED_1, HAL_LED_MODE_TOGGLE);   //设置灯状态
    SampleApp_Send();
    return ( events ^ SAMPLEAPP_SEND_EVT );
  }
  
  // Discard unknown events
  return 0;
}

/*********************************************************************
* Event Generation Functions
*/
/*********************************************************************
* @fn      SampleApp_HandleKeys
*
* @brief   Handles all key events for this device.
*
* @param   shift - true if in shift/alt.
* @param   keys - bit field for key events. Valid entries:
*                 HAL_KEY_SW_2
*                 HAL_KEY_SW_1
*
* @return  none
*/
void SampleApp_HandleKeys( uint8 shift, uint8 keys )
{
  (void)shift;  // Intentionally unreferenced parameter
  
  if ( keys & HAL_KEY_SW_1 )
  {
    /* This key sends the Flash Command is sent to Group 1.
    * This device will not receive the Flash Command from this
    * device (even if it belongs to group 1).
    */
    SampleApp_SendFlashMessage( SAMPLEAPP_FLASH_DURATION );
  }
  
  if ( keys & HAL_KEY_SW_2 )
  {
    /* The Flashr Command is sent to Group 1.
    * This key toggles this device in and out of group 1.
    * If this device doesn't belong to group 1, this application
    * will not receive the Flash command sent to group 1.
    */
    aps_Group_t *grp;
    grp = aps_FindGroup( SAMPLEAPP_ENDPOINT, SAMPLEAPP_FLASH_GROUP );
    if ( grp )
    {
      // Remove from the group
      aps_RemoveGroup( SAMPLEAPP_ENDPOINT, SAMPLEAPP_FLASH_GROUP );
    }
    else
    {
      // Add to the flash group
      aps_AddGroup( SAMPLEAPP_ENDPOINT, &SampleApp_Group );
    }
  }
}

/*********************************************************************
* LOCAL FUNCTIONS
*/

/*********************************************************************
* @fn      SampleApp_MessageMSGCB
*
* @brief   Data message processor callback.  This function processes
*          any incoming data - probably from other devices.  So, based
*          on cluster ID, perform the intended action.
*
* @param   none
*
* @return  none
*/
void SampleApp_MessageMSGCB( afIncomingMSGPacket_t *pkt ) //接收数据
{
  uint16 flashTime;
  
  switch ( pkt->clusterId )
  {
  case SAMPLEAPP_PERIODIC_CLUSTERID:
    //HalUARTWrite(0, "Rx:", 3);        //提示信息
    HalUARTWrite(0, pkt->cmd.Data, pkt->cmd.DataLength); //输出接收到的数据
    //HalUARTWrite(0, "\n", 1);         //回车换行
    break;
    
  case SAMPLEAPP_FLASH_CLUSTERID:     //此实验没有使用，到后面实验详解
    flashTime = BUILD_UINT16(pkt->cmd.Data[1], pkt->cmd.Data[2] );
    HalLedBlink( HAL_LED_4, 4, 50, (flashTime / 4) );
    break;
  }
}

/*********************************************************************
* @fn      SampleApp_SendPeriodicMessage
*
* @brief   Send the periodic message.
*
* @param   none
*
* @return  none
*/
void SampleApp_SendPeriodicMessage( void )
{
  osal_pwrmgr_device(PWRMGR_ALWAYS_ON);
  
  Delay_s(1);
  P0_6 = 0;
  Delay_ms(1);
  P0_6 = 1; 
  Delay_ms(1);
  P0_6 = 0;
  Delay_ms(1);  
  Delay_s(6);
  ifSendData = false;  //
  //  发送唤醒指令。。。。
  HalUARTWrite(0, Let_On_Cmd, 5);
  HalUARTPoll();
  //  唤醒LED2
  HalLedSet(HAL_LED_1, HAL_LED_MODE_TOGGLE);  
  //  等待,等待发送数据
  Delay_s(5);
  //  监测是否有UART数据，如果有,发送UART数据；如果没有，发送data中封装的数据 
  //发送心跳
  if ( AF_DataRequest( &SampleApp_Periodic_DstAddr, &SampleApp_epDesc,
                      SAMPLEAPP_PERIODIC_CLUSTERID,
                      26,
                      packagedata,
                      &SampleApp_TransID,
                      AF_DISCV_ROUTE,
                      AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
  {
    HalLedSet(HAL_LED_3, HAL_LED_MODE_TOGGLE);  
  }
  
  //  发送休眠指令。。。。
  HalUARTWrite(0, Let_Off_Cmd, 5);
  //  HalUARTPoll();
  //  关闭LED2
  Delay_s(10);
  osal_pwrmgr_device(PWRMGR_BATTERY);
}

/*********************************************************************
* @fn      SampleApp_SendFlashMessage
*
* @brief   Send the flash message to group 1.
*
* @param   flashTime - in milliseconds
*
* @return  none
*/
void SampleApp_SendFlashMessage( uint16 flashTime )
{
  uint8 buffer[3];
  buffer[0] = (uint8)(SampleAppFlashCounter++);
  buffer[1] = LO_UINT16( flashTime );
  buffer[2] = HI_UINT16( flashTime );
  
  if ( AF_DataRequest( &SampleApp_Flash_DstAddr, &SampleApp_epDesc,
                      SAMPLEAPP_FLASH_CLUSTERID,
                      3,
                      buffer,
                      &SampleApp_TransID,
                      AF_DISCV_ROUTE,
                      AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
  {
  }
  else
  {
    // Error occurred in request to send.
  }
}
/*********************************************************************
*********************************************************************/
/*********************************************************************
* @fn      SampleApp_CallBack
*
* @brief   Send data OTA.
*
* @param   port - UART port.
* @param   event - the UART port event flag.
*
* @return  none
*/
static void SampleApp_CallBack(uint8 port, uint8 event)
{
  (void)port;
  
  if ((event & (HAL_UART_RX_FULL | HAL_UART_RX_ABOUT_FULL | HAL_UART_RX_TIMEOUT)) &&
#if SAMPLE_APP_LOOPBACK
      (SampleApp_TxLen < SAMPLE_APP_TX_MAX))
#else
    !SampleApp_TxLen)
#endif
{
  SampleApp_Send();
}
}

/*********************************************************************
* @fn      SampleApp_Send
*
* @brief   Send data OTA.
*
* @param   none
*
* @return  none
*/
static void SampleApp_Send(void)
{
#if SAMPLE_APP_LOOPBACK
  if (SampleApp_TxLen < SAMPLE_APP_TX_MAX)
  {
    SampleApp_TxLen += HalUARTRead(SAMPLE_APP_PORT, SampleApp_TxBuf+SampleApp_TxLen+1,
                                   SAMPLE_APP_TX_MAX-SampleApp_TxLen);
  }
  
  if (SampleApp_TxLen)
  {
    (void)SampleApp_TxAddr;
    if (HalUARTWrite(SAMPLE_APP_PORT, SampleApp_TxBuf+1, SampleApp_TxLen))
    {
      SampleApp_TxLen = 0;
    }
    else
    {
      osal_set_event(SampleApp_TaskID, SAMPLEAPP_SEND_EVT);
    }
  }
#else
  if (!SampleApp_TxLen && 
      (SampleApp_TxLen = HalUARTRead(SAMPLE_APP_PORT, SampleApp_TxBuf+1, SAMPLE_APP_TX_MAX)))
  {
    // Pre-pend sequence number to the Tx message.
    //SampleApp_TxBuf[0] = ++SampleApp_TxSeq;
    pocessdata(SampleApp_TxBuf,SampleApp_TxLen);
  }
  if (SampleApp_TxLen)
  {
    if ( AF_DataRequest( &SampleApp_Periodic_DstAddr, (endPointDesc_t *)&SampleApp_epDesc,
                        SAMPLEAPP_PERIODIC_CLUSTERID,
                        26, finaldata,/**/
                        &SampleApp_TransID,
                        AF_DISCV_ROUTE,
                        AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
    {//张立超添加 
      HalLedSet (HAL_LED_2, HAL_LED_MODE_TOGGLE);   //设置灯状态
      Delay_ms(2);
      SampleApp_TxLen = 0;
      Delay_ms(2);
      ifSendData = true;
      Delay_ms(2);
      int clear;
      for(clear=0;clear<25;clear++){
        finaldata[clear]=0;
      }
    }
    else
    {
      //        SampleApp_TxBuf[0] = 0;
      osal_set_event(SampleApp_TaskID, SAMPLEAPP_SEND_EVT);
    }
  }
#endif
}


/*********************************************************************
*********************************************************************/
//延时函数
void Delay_10us(void)   //10us延时
{
  MicroWait(10);
}

void Delay_ms(int time)  //ms延时
{
  unsigned char i;
  while(time--)
  {
    for(i = 0; i < 100; i++)
      Delay_10us();
  }
}

void Delay_s(int ms)
{
  while(ms--)
  {
    Delay_ms(1000);
  }
}

void datainit(uint8 *array){
  array[0]=0x7e;    //包头
  array[1]=0x45;    //包头
  array[2]=HI_UINT16(Nodeid);    //nodeid
  array[3]=LO_UINT16(Nodeid);    //nodeid
  array[4]=1;       //控制包
  array[5]=HI_UINT16(count);    //计数
  array[6]=LO_UINT16(count);    //计数
  array[7]=0x00;    //relay1
  array[8]=0x00;    //relay1
  array[9]=0x00;    //relay2
  array[10]=0x00;   //relay2
  array[11]=0x00;   //relay3
  array[12]=0x00;   //relay3
  array[13]=0x00;   //relay4
  array[14]=0x00;   //relay4
  array[15]=0x00;   //relay5
  array[16]=0x00;   //relay5
  array[17]=0x00;   //保留
  array[18]=0x00;   //保留
  array[19]=0x00;   //保留
  array[20]=0x00;   //保留
  array[21]=0x00;   //保留
  array[22]=0x00;   //保留
  array[23]=0x00;   //保留
  array[24]=0x00;   //保留
  array[25]=0x7e;   //包尾 
}

void pocessdata(uint8 *array_in,uint8 lenth){
  int changeflag;
  finaldata[0]=0x7e;
  finaldata[1]=0x45;
  finaldata[2]=HI_UINT16(Nodeid);    //计数
  finaldata[3]=LO_UINT16(Nodeid);
  finaldata[4]=2;//数据包
  finaldata[5]=packagetype;
  if(lenth == 24 && errorflag(array_in)){
    
    for(changeflag=0;changeflag<=4;changeflag++)
    {finaldata[changeflag+6]=array_in[2+changeflag+1];}
    for(changeflag=5;changeflag<=9;changeflag++)
    {finaldata[changeflag+6]=array_in[3+changeflag+1];}
    for(changeflag=10;changeflag<=14;changeflag++)
    {finaldata[changeflag+6]=array_in[4+changeflag+1];}
    for(changeflag=15;changeflag<=16;changeflag++)
    {finaldata[changeflag+6]=array_in[5+changeflag+1];}
    finaldata[23]=0;//传感器保留字节
    finaldata[24]=0;
    finaldata[25]=0x7e;
    //计数
  }
  else{
    for(changeflag=2;changeflag<23;changeflag++)
    {
      finaldata[changeflag]=0x00;
    }
    finaldata[24]=1;
    finaldata[25]=0x7e;
  }
}

uint8 errorflag(uint8 *shu1)
{
  int ii;
  uint8 jiaoyan;
  jiaoyan=(*(shu1+2))^(*(shu1+3));
  for(ii=4;ii<=22 ;ii++)
    jiaoyan=jiaoyan ^ (*(shu1+ii));
  if(jiaoyan == (*(shu1+23)))
    return 1;           //奇偶校验是正确的
  else    return 0;
}
