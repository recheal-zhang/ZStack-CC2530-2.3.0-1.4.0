# ZStack-CC2530-2.3.0-1.4.0
CC2530  Coordinator &amp; EndDevice --ZStack

#**无线传感网CC2530终端、路由以及协调器系统应用**
-------
要做到目视千里，耳听八方是人类长久的梦想，现代卫星技术的出现虽然使人们离这目标又进了一步，但卫星高高在上，洞察全局在行，明察细微就不管用了。这个时候，本文的主角—**无线传感器网络**就排上用场了。将大量的传感器节点遍撒指定区域，数据通过无线电波传回监控中心，监控区域内的所有信息就会尽收观察者的眼中了。

闲话不说，直接进入正题。想让传感数据回来，总得有一套可以“采集传感器数据，打包发送数据给上层”的系统，这里就程序简单说明一下该系统的实现。
-------
主开发程序在/ZStack-CC2530-2.3.0-1.4.0/Projects/zstack/Samples目录下，开发环境是IAR，开发语言C。
该系统具备的功能：
> & 协调器，路由，终端之间能够相互通信
> & 终端，路由可以正常采集传感器的数据并通过路由定时将数据转发给协调器
> & 节点电压信息获取
> & 通过callback实现串口收发数据
> & 休眠调度，节点定时休眠唤醒

程序基本思路如下：
> * 组网：调用协议栈的组网函数、加入网络函数，实现网络的建立与节点的加入
> * 采集：UART采集传感器数据
> * 发送：发送节点调用协议栈的无线数据发送函数，实现无线数据发送
> * 接收：接收节点调用协议栈的无线数据接收函数，实现无线数据接收，UART上传数据
> * 打包：打包处理传感器数据，为服务器控制台解析提供基础

------
##初始化操作
由于节点需要用到UART callback，所以串口需要重新配置，在init()函数中，重新以下部分：
```C
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
  // look out *************************************************************************
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
  
  osal_pwrmgr_task_state(task_id,PWRMGR_CONSERVE);  //休眠功能初始化
  // look out *************************************************************************
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
```
##接收节点调用协议栈的无线数据接收函数
```C
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
    //HalUARTWrite(0, "Rx:", 3);        //prompt message
    HalUARTWrite(0, pkt->cmd.Data, pkt->cmd.DataLength); //send the message to GPRS
    //HalUARTWrite(0, "\n", 1);         
    break;
    
  case SAMPLEAPP_FLASH_CLUSTERID:     
    flashTime = BUILD_UINT16(pkt->cmd.Data[1], pkt->cmd.Data[2] );
    HalLedBlink( HAL_LED_4, 4, 50, (flashTime / 4) );
    break;
  }
}
```
## 终端跟传感器相互通信的时候一定注意延时的重要性！
```C
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
  osal_pwrmgr_device(PWRMGR_ALWAYS_ON);   //唤醒
  
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
  osal_pwrmgr_device(PWRMGR_BATTERY);    //休眠
}
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
    pocessdata(SampleApp_TxBuf, SampleApp_TxLen);
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
      for(clear=0; clear<25; clear++){
        finaldata[clear] = 0;
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
```

## 打包数据校验策略
```C
uint8 errorflag(uint8 *data, int length)
{
  int i;
  uint8 crc;
  jiaoyan=(*(data + 2)) ^ (*(data + 3));
  for(i = 4; i <= 22; i++)
    crc = crc ^ (*(data + i));
  if(crc == (*(data + length)))
    return 1;           //奇偶校验是正确的
  else  
    return 0;
}
```








