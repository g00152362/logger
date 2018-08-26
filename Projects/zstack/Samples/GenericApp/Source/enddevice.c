/******************************************************************************
  Filename:       GenericApp.c
  Revised:        $Date: 2012-03-07 01:04:58 -0800 (Wed, 07 Mar 2012) $
  Revision:       $Revision: 29656 $

  Description:    Generic Application (no Profile).


  Copyright 2004-2012 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License"). You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product. Other than for
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
******************************************************************************/

/*********************************************************************
  This application isn't intended to do anything useful, it is
  intended to be a simple example of an application's structure.

  This application sends "Hello World" to another "Generic"
  application every 5 seconds.  The application will also
  receives "Hello World" packets.

  The "Hello World" messages are sent/received as MSG type message.

  This applications doesn't have a profile, so it handles everything
  directly - itself.

  Key control:
    SW1:
    SW2:  initiates end device binding
    SW3:
    SW4:  initiates a match description request
*********************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include <ioCC2530.h>

#include "OSAL.h"
#include "AF.h"
#include "ZDApp.h"
#include "ZDObject.h"
#include "ZDProfile.h"
#include "OSAL_Nv.h"

#include "GenericApp.h"
#include "DebugTrace.h"

#if !defined( WIN32 )
  #include "OnBoard.h"
#endif

/* HAL */
#include "hal_lcd.h"
#include "hal_led.h"
#include "hal_key.h"
#include "hal_uart.h"
#include "hal_adc.h"

#include "DHT11.h"

/* RTOS */
#if defined( IAR_ARMCM3_LM )
#include "RTOS_App.h"
#endif  

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */
// This list should be filled with Application specific Cluster IDs.
const cId_t GenericApp_ClusterList[GENERICAPP_MAX_CLUSTERS] =
{
  GENERICAPP_HTID,
  GENERICAPP_RSPID

};

const SimpleDescriptionFormat_t GenericApp_SimpleDesc =
{
  GENERICAPP_ENDPOINT,              //  int Endpoint;
  GENERICAPP_PROFID,                //  uint16 AppProfId[2];
  GENERICAPP_DEVICEID,              //  uint16 AppDeviceId[2];
  GENERICAPP_DEVICE_VERSION,        //  int   AppDevVer:4;
  GENERICAPP_FLAGS,                 //  int   AppFlags:4;
  GENERICAPP_MAX_CLUSTERS,          //  byte  AppNumInClusters;
  (cId_t *)GenericApp_ClusterList,  //  byte *pAppInClusterList;
  GENERICAPP_MAX_CLUSTERS,          //  byte  AppNumInClusters;
  (cId_t *)GenericApp_ClusterList   //  byte *pAppInClusterList;
};

// This is the Endpoint/Interface description.  It is defined here, but
// filled-in in GenericApp_Init().  Another way to go would be to fill
// in the structure here and make it a "const" (in code space).  The
// way it's defined in this sample app it is define in RAM.
endPointDesc_t GenericApp_epDesc;

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
byte GenericApp_TaskID;   // Task ID for internal task/event processing
                          // This variable will be received when
                          // GenericApp_Init() is called.
devStates_t GenericApp_NwkState;


byte GenericApp_TransID;  // This is the unique message ID (counter)

afAddrType_t GenericApp_DstAddr;

#define MAX_COLLECT_COUNT  10//10 5分钟发一次 120*GENERICAPP_SEND_MSG_TIMEOUT(0.5MIN)
int collect_count =MAX_COLLECT_COUNT-1; //启动时候先发一次

byte send_timeout_count =0xff;  //一次5秒,ff 为无效值，不检测


byte isr_on =0x0;  

#define PACKET_SEND_TIMEOUT_COUNT 2 //30秒超时检查





/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void GenericApp_ProcessZDOMsgs( zdoIncomingMsg_t *inMsg );
static void GenericApp_HandleKeys( byte shift, byte keys );
static void GenericApp_MessageMSGCB( afIncomingMSGPacket_t *pckt );
static void GenericApp_SendTheMessage( void );
__interrupt void P0_ISR(void) ;

#if defined( IAR_ARMCM3_LM )
static void GenericApp_ProcessRtosMessage( void );
#endif

/****************************************************************************
* 名    称: P0_ISR(void) 中断处理函数 
* 描    述: #pragma vector = 中断向量，紧接着是中断处理程序
****************************************************************************/
#pragma vector = P0INT_VECTOR    
__interrupt void P0_ISR(void) 
{ 
//    DelayMS(10);     //延时去抖
//    LED1 = ~LED1;    //改变LED1状态

//	GenericApp_SendTheMessage();


    P0IFG = 0;       //清中断标志 
    P0IF = 0;        //清中断标志 
	if(isr_on)
	{
		return;
	}




	//启动1s定时器
	osal_start_timerEx( GenericApp_TaskID,
						GENERICAPP_GPIO_MSG_EVT,
						1000 );
	//打开中断控制变量，以免重复进入
	isr_on = 1;
	//启动1min定时器关闭中断，避免重复进入
	osal_start_timerEx( GenericApp_TaskID,
						GENERICAPP_ISR_ON_EVT,
						GENERICAPP_ISR_TIMEOUT );

	
    
} 


/*********************************************************************
 * NETWORK LAYER CALLBACKS
 */

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      GenericApp_Init
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
void GenericApp_Init( uint8 task_id )
{
  GenericApp_TaskID = task_id;
  GenericApp_NwkState = DEV_INIT;
  GenericApp_TransID = 0;

  //add by guojun
  
  APCFG |= 0x40;   //P0.6为ADC 输入，检查电池电压
  P0SEL |= 0x40; 
  P0DIR	&= ~0x40;	



  //init IR interrupt , P0.5

//  P2INP |= 0x20;    //初始化PO为下拉
  P0DIR &= ~0x20;  //P0.5为输入 红外传感

  P0IEN |= 0x20;	// P0.5 设置为中断方式 1：中断使能
  PICTL |= 0x00;	//上升沿触发	
  IEN1 |= 0x20;    //允许P0口中断; 
  P0IFG = 0x00;    //初始化中断标志位
  EA = 1;		   //打开总中断

//led p1.0/1.1 拉高点亮	
  P1DIR |= 0x03; //配置IO口方向

  P1_0 = 1;
  P1_1 = 1;

  
  // 先读一遍，避免取到0值；

  DHT11();


  //end add




  // Device hardware initialization can be added here or in main() (Zmain.c).
  // If the hardware is application specific - add it here.
  // If the hardware is other parts of the device add it in main().

  GenericApp_DstAddr.addrMode = (afAddrMode_t)Addr16Bit;
  GenericApp_DstAddr.endPoint = GENERICAPP_ENDPOINT;
  GenericApp_DstAddr.addr.shortAddr = 0x0000;

  // Fill out the endpoint description.
  GenericApp_epDesc.endPoint = GENERICAPP_ENDPOINT;
  GenericApp_epDesc.task_id = &GenericApp_TaskID;
  GenericApp_epDesc.simpleDesc
            = (SimpleDescriptionFormat_t *)&GenericApp_SimpleDesc;
  GenericApp_epDesc.latencyReq = noLatencyReqs;

  // Register the endpoint description with the AF
  afRegister( &GenericApp_epDesc );

  // Register for all key events - This app will handle all key events
  RegisterForKeys( GenericApp_TaskID );

  // Update the display
#if defined ( LCD_SUPPORTED )
  HalLcdWriteString( "GenericApp", HAL_LCD_LINE_1 );
#endif

  ZDO_RegisterForZDOMsg( GenericApp_TaskID, End_Device_Bind_rsp );
  ZDO_RegisterForZDOMsg( GenericApp_TaskID, Match_Desc_rsp );

#if defined( IAR_ARMCM3_LM )
  // Register this task with RTOS task initiator
  RTOS_RegisterApp( task_id, GENERICAPP_RTOS_MSG_EVT );
#endif

//启动定时器
osal_start_timerEx( GenericApp_TaskID,
					GENERICAPP_SEND_MSG_EVT,
					GENERICAPP_SEND_MSG_TIMEOUT );

}

/*********************************************************************
 * @fn      GenericApp_ProcessEvent
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
uint16 GenericApp_ProcessEvent( uint8 task_id, uint16 events )
{
  afIncomingMSGPacket_t *MSGpkt;
  afDataConfirm_t *afDataConfirm;

  // Data Confirmation message fields
  byte sentEP;
  ZStatus_t sentStatus;
  byte sentTransID;       // This should match the value sent
  (void)task_id;  // Intentionally unreferenced parameter
  if ( events & SYS_EVENT_MSG )
  {
    MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( GenericApp_TaskID );
    while ( MSGpkt )
    {
      switch ( MSGpkt->hdr.event )
      {
        case ZDO_CB_MSG:
          GenericApp_ProcessZDOMsgs( (zdoIncomingMsg_t *)MSGpkt );
          break;

        case KEY_CHANGE:
          GenericApp_HandleKeys( ((keyChange_t *)MSGpkt)->state, ((keyChange_t *)MSGpkt)->keys );
          break;

        case AF_DATA_CONFIRM_CMD:
          // This message is received as a confirmation of a data packet sent.
          // The status is of ZStatus_t type [defined in ZComDef.h]
          // The message fields are defined in AF.h
          afDataConfirm = (afDataConfirm_t *)MSGpkt;
          sentEP = afDataConfirm->endpoint;
          sentStatus = afDataConfirm->hdr.status;
          sentTransID = afDataConfirm->transID;
          (void)sentEP;
          (void)sentTransID;

          // Action taken when confirmation is received.
          if ( sentStatus != ZSuccess )
          {
            // The data wasn't delivered -- Do something
          }
          break;

        case AF_INCOMING_MSG_CMD:
          GenericApp_MessageMSGCB( MSGpkt );
          break;

        case ZDO_STATE_CHANGE:
          GenericApp_NwkState = (devStates_t)(MSGpkt->hdr.status);
          if ( (GenericApp_NwkState == DEV_ZB_COORD)
              || (GenericApp_NwkState == DEV_ROUTER)
              || (GenericApp_NwkState == DEV_END_DEVICE) )
          {
            // Start sending "the" message in a regular interval.
            osal_start_timerEx( GenericApp_TaskID,
                                GENERICAPP_SEND_MSG_EVT,
                                GENERICAPP_SEND_MSG_TIMEOUT );
          }
          break;

        default:
          break;
      }

      // Release the memory
      osal_msg_deallocate( (uint8 *)MSGpkt );

      // Next
      MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( GenericApp_TaskID );
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }

  // Send a message out - This event is generated by a timer
  if ( events & GENERICAPP_SEND_MSG_EVT )
  {
  	if(send_timeout_count!= 0xff) //已经发送
  	{
		if(send_timeout_count >=PACKET_SEND_TIMEOUT_COUNT) //20s 没有响应 认为超时
		{
			// 先用灯表示
			HalLedSet (HAL_LED_1, HAL_LED_MODE_ON);
			HalLedSet (HAL_LED_2, HAL_LED_MODE_ON);
			HalLedSet (HAL_LED_3, HAL_LED_MODE_ON);
            send_timeout_count = 0xff;
			//reboot 
			Delay_ms(5000);
			SystemReset();
		}
		
		send_timeout_count++;
  	}
    // Send "the" message
    GenericApp_SendTheMessage();//周期性调用测温湿度并无线发送给协调器函数

    // Setup to send message again
    osal_start_timerEx( GenericApp_TaskID,
                        GENERICAPP_SEND_MSG_EVT,
                        GENERICAPP_SEND_MSG_TIMEOUT );//这个函数就是启动周期和设置周期长短

    // return unprocessed events
    return (events ^ GENERICAPP_SEND_MSG_EVT);
  }

  // 红外GPIO 中断
  if ( events & GENERICAPP_GPIO_MSG_EVT )
  {
  	//立即发送
 	collect_count = MAX_COLLECT_COUNT;
    GenericApp_SendTheMessage();//周期性调用测温湿度并无线发送给协调器函数
    //osal_stop_timerEx(GenericApp_TaskID, uint16 event_id)
    // return unprocessed events
    return (events ^ GENERICAPP_GPIO_MSG_EVT);
  }
    // 红外GPIO 中断控制定时器
  if ( events & GENERICAPP_ISR_ON_EVT )
  {
  	//立即发送
	isr_on = 0;
    return (events ^ GENERICAPP_ISR_ON_EVT);
  }
  

  

  
#if defined( IAR_ARMCM3_LM )
  // Receive a message from the RTOS queue
  if ( events & GENERICAPP_RTOS_MSG_EVT )
  {
    // Process message from RTOS queue
    GenericApp_ProcessRtosMessage();

    // return unprocessed events
    return (events ^ GENERICAPP_RTOS_MSG_EVT);
  }
#endif

  // Discard unknown events
  return 0;
}

/*********************************************************************
 * Event Generation Functions
 */

/*********************************************************************
 * @fn      GenericApp_ProcessZDOMsgs()
 *
 * @brief   Process response messages
 *
 * @param   none
 *
 * @return  none
 */
static void GenericApp_ProcessZDOMsgs( zdoIncomingMsg_t *inMsg )
{
  switch ( inMsg->clusterID )
  {
    case End_Device_Bind_rsp:
      if ( ZDO_ParseBindRsp( inMsg ) == ZSuccess )
      {
        // Light LED
        HalLedSet( HAL_LED_4, HAL_LED_MODE_ON );
      }
#if defined( BLINK_LEDS )
      else
      {
        // Flash LED to show failure
        HalLedSet ( HAL_LED_4, HAL_LED_MODE_FLASH );
      }
#endif
      break;

    case Match_Desc_rsp:
      {
        ZDO_ActiveEndpointRsp_t *pRsp = ZDO_ParseEPListRsp( inMsg );
        if ( pRsp )
        {
          if ( pRsp->status == ZSuccess && pRsp->cnt )
          {
            GenericApp_DstAddr.addrMode = (afAddrMode_t)Addr16Bit;
            GenericApp_DstAddr.addr.shortAddr = pRsp->nwkAddr;
            // Take the first endpoint, Can be changed to search through endpoints
            GenericApp_DstAddr.endPoint = pRsp->epList[0];

            // Light LED
            HalLedSet( HAL_LED_4, HAL_LED_MODE_ON );
          }
          osal_mem_free( pRsp );
        }
      }
      break;
  }
}

/*********************************************************************
 * @fn      GenericApp_HandleKeys
 *
 * @brief   Handles all key events for this device.
 *
 * @param   shift - true if in shift/alt.
 * @param   keys - bit field for key events. Valid entries:
 *                 HAL_KEY_SW_4
 *                 HAL_KEY_SW_3
 *                 HAL_KEY_SW_2
 *                 HAL_KEY_SW_1
 *
 * @return  none
 */
static void GenericApp_HandleKeys( uint8 shift, uint8 keys )
{
  zAddrType_t dstAddr;

  // Shift is used to make each button/switch dual purpose.
  if ( shift )
  {
    if ( keys & HAL_KEY_SW_1 )
    {
    }
    if ( keys & HAL_KEY_SW_2 )
    {
    }
    if ( keys & HAL_KEY_SW_3 )
    {
    }
    if ( keys & HAL_KEY_SW_4 )
    {
    }
  }
  else
  {
    if ( keys & HAL_KEY_SW_1 )
    {
      // Since SW1 isn't used for anything else in this application...
#if defined( SWITCH1_BIND )
      // we can use SW1 to simulate SW2 for devices that only have one switch,
      keys |= HAL_KEY_SW_2;
#elif defined( SWITCH1_MATCH )
      // or use SW1 to simulate SW4 for devices that only have one switch
      keys |= HAL_KEY_SW_4;
#endif
    }

    if ( keys & HAL_KEY_SW_2 )
    {
      HalLedSet ( HAL_LED_4, HAL_LED_MODE_OFF );

      // Initiate an End Device Bind Request for the mandatory endpoint
      dstAddr.addrMode = Addr16Bit;
      dstAddr.addr.shortAddr = 0x0000; // Coordinator
      ZDP_EndDeviceBindReq( &dstAddr, NLME_GetShortAddr(),
                            GenericApp_epDesc.endPoint,
                            GENERICAPP_PROFID,
                            GENERICAPP_MAX_CLUSTERS, (cId_t *)GenericApp_ClusterList,
                            GENERICAPP_MAX_CLUSTERS, (cId_t *)GenericApp_ClusterList,
                            FALSE );
    }

    if ( keys & HAL_KEY_SW_3 )
    {
    }

    if ( keys & HAL_KEY_SW_4 )
    {
      HalLedSet ( HAL_LED_4, HAL_LED_MODE_OFF );
      // Initiate a Match Description Request (Service Discovery)
      dstAddr.addrMode = AddrBroadcast;
      dstAddr.addr.shortAddr = NWK_BROADCAST_SHORTADDR;
      ZDP_MatchDescReq( &dstAddr, NWK_BROADCAST_SHORTADDR,
                        GENERICAPP_PROFID,
                        GENERICAPP_MAX_CLUSTERS, (cId_t *)GenericApp_ClusterList,
                        GENERICAPP_MAX_CLUSTERS, (cId_t *)GenericApp_ClusterList,
                        FALSE );
    }
  }
}

/*********************************************************************
 * LOCAL FUNCTIONS
 */

/*********************************************************************
 * @fn      GenericApp_MessageMSGCB
 *
 * @brief   Data message processor callback.  This function processes
 *          any incoming data - probably from other devices.  So, based
 *          on cluster ID, perform the intended action.
 *
 * @param   none
 *
 * @return  none
 */
static void GenericApp_MessageMSGCB( afIncomingMSGPacket_t *pkt )
{
int i;
  switch ( pkt->clusterId )
  {
    case GENERICAPP_HTID:
      // "the" message
	  break;

    case GENERICAPP_RSPID:
        //这个地方收到发送的响应
        send_timeout_count =0xff; //表示收到了对端响应
        // test ...  
        for(i=0;i<5;i++)
        {
            HalLedSet (HAL_LED_2, HAL_LED_MODE_ON);
            Delay_ms(100);
            HalLedSet (HAL_LED_2, HAL_LED_MODE_OFF);
        }
      break;
  }
}


// Max ADC input voltage = reference voltage =>
// (VDD/3) max = 1.15 V => max VDD = 3.45 V
// 12 bits resolution means that max ADC value = 0x07FF = 2047 (dec)
// (the ADC value is 2’s complement)
// Battery voltage, VDD = adc value * (3.45 / 2047)
// To avoid using a float, the below function will return the battery voltage * 100
// Battery voltage * 100 = adc value * (3.75 / 2047) * 100
#define XV 0.168539 // (3.45 / 2047) * 100 


/************************************************************
* 名称       get_adc
* 功能       读取ADC通道6电压值
* 入口参数   无
* 出口参数   32 位 电压*100
***************获取ADC通道0电压值************************/
uint16 App_get_adc(void)
{
    uint32 value =0;
    ADCIF = 0;   //清ADC 中断标志
    //采用基准电压avdd5:3.3V，通道6，启动AD转化   ADCCON3 = (0x80 | 0x10 | 0x06);
    ADCCON3 = (HAL_ADC_REF_115V | 0x20 | HAL_ADC_CHN_VDD3); //HAL_ADC_DEC_256
    while ( !ADCIF )
    {
        ;  //等待AD转化结束
    }
    value = ADCH;
    value = value<< 8;
    value |= ADCL;
	value =value >>4;
    value =(uint16)(XV * value) + 10;  //测试下来差0.1V，10作为补偿
    return (uint16)value;
}

  


/*********************************************************************
 * @fn      GenericApp_SendTheMessage
 *
 * @brief   Send "the" message.
 *
 * @param   none
 *
 * @return  none
 */
static void GenericApp_SendTheMessage( void )
{
	char strTemp[40];
	//byte evalue=0;
    byte length=0;
	uint8  *pAddr = 0;
	uint16 adc =0;
	
	if(collect_count <MAX_COLLECT_COUNT)
	{
		 HalLedSet (HAL_LED_1, HAL_LED_MODE_OFF);

		collect_count++;
		return;
	}
	collect_count = 0;	

	
	DHT11();             //获取温湿度
	if((wendu_shi+wendu_ge+shidu_shi+shidu_ge) ==0)
	{
		// 没有读出来，置下次读
		collect_count = MAX_COLLECT_COUNT;
		return;
	}
 

	
	strTemp[0] = 0x01; //SEND CODE
	strTemp[1] = wendu_shi*10+wendu_ge;	//tempture value
	strTemp[2] = shidu_shi*10+shidu_ge;//humdity value
	

  	if(P0_5 == 0)//如果终端的人体红外传感器周围没人活动
	{
		strTemp[3] = 0;
	}
	else
  	{

  		strTemp[3] = 1;
  	}
	length = 4; //TOTAL 4 bytes value


	pAddr= (uint8 *)(PXREG( 0x7800 )+HAL_INFOP_IEEE_OSET);
	osal_memcpy(strTemp+length, pAddr, 6);

	length +=6;

	adc = App_get_adc();
	osal_memcpy(strTemp+length, &adc, 2);
	length +=2;
	

  HalLedSet (HAL_LED_1, HAL_LED_MODE_ON);
 
  if ( AF_DataRequest( &GenericApp_DstAddr, &GenericApp_epDesc,
                       GENERICAPP_HTID,
                       length,
                       (uchar*)strTemp,
                       &GenericApp_TransID,
                       AF_DISCV_ROUTE,
                       AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
  {
  		send_timeout_count = 0; //start timeout count
  }
  else
  {
    // Error occurred in request to send.
  }
  		  HalLedSet (HAL_LED_2, HAL_LED_MODE_OFF);
  send_timeout_count = 0; //start timeout count
  
}

#if defined( IAR_ARMCM3_LM )
/*********************************************************************
 * @fn      GenericApp_ProcessRtosMessage
 *
 * @brief   Receive message from RTOS queue, send response back.
 *
 * @param   none
 *
 * @return  none
 */
static void GenericApp_ProcessRtosMessage( void )
{
  osalQueue_t inMsg;

  if ( osal_queue_receive( OsalQueue, &inMsg, 0 ) == pdPASS )
  {
    uint8 cmndId = inMsg.cmnd;
    uint32 counter = osal_build_uint32( inMsg.cbuf, 4 );

    switch ( cmndId )
    {
      case CMD_INCR:
        counter += 1;  /* Increment the incoming counter */
                       /* Intentionally fall through next case */

      case CMD_ECHO:
      {
        userQueue_t outMsg;

        outMsg.resp = RSP_CODE | cmndId;  /* Response ID */
        osal_buffer_uint32( outMsg.rbuf, counter );    /* Increment counter */
        osal_queue_send( UserQueue1, &outMsg, 0 );  /* Send back to UserTask */
        break;
      }
      
      default:
        break;  /* Ignore unknown command */    
    }
  }
}
#endif

/*********************************************************************
 */
