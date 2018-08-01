
#include "UserTask.h"
#include "AdcDriver.h"
#include "UsartDriver.h"
#include "oled.h"
#include "oled_display.h"
#include "HardwareConfig.h"
#include "ButtonDriver.h"
#include "stm32f1xx_hal.h"

#define HARDWARE_CORE "HV: STM32F103C8T6 MavSwordV1.0\r\n"
#define SOFTWARE_VERSION "SV: RTOS-V1.0\r\n"
#define DESCRIPT_STRING "Desc: Designed by Awesome OY\r\n"
#define BUTTON_TASK_ERROR "ERROR: button task error!"

osThreadId run_led_task_handle;
osThreadId status_led_task_handle;
osThreadId beep_task_handle;
osThreadId feed_dog_task_handle;
osThreadId usart3_rec_task_handle; 
osThreadId usart3_send_task_handle; 

xSemaphoreHandle usart3_send_mutex;  /* 发送数据采用互斥量多任务共享一个发送串行端口 */
xSemaphoreHandle usart1_send_mutex;  /* 发送数据采用互斥量多任务共享一个发送串行端口 */
xQueueHandle button_event_queue;

IWDG_HandleTypeDef hiwdg;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;
ADC_HandleTypeDef hadc1;

static unsigned char status_led_enable = 1;
static unsigned char run_led_enbale = 1;

static SystemFlag system_flag ;
static FailStatus fail_status;

mavlink_system_t gcs_mavlink_system = {0xff,0xbe};

/* 飞行数据缓存 */
DroneMagStatus droneMagStatus[2];
DroneHeatbeat droneHeatbeat;
DroneSysStatus droneSysStatus;
DroneVfr droneVfr;
DroneVibe droneVibe;
DroneGpsRaw droneGPSRawData;
DroneGps2Raw droneGPS2RawData;
DroneAtt droneAtt;
DroneWind droneWind;
DroneIMU droneIMU;
DroneMagCalProgess droneMagCalPro;
mavlink_mag_cal_report_t magCalReport;
mavlink_camera_feedback_t cameraFeedback;
mavlink_rangefinder_t rangefinder;
mavlink_statustext_t statusText;

DroneData droneData;

DroneImuPro droneImuPro;
DroneVibePro droneVibePro;
DroneInfo droneInfo;
DroneCurrent droneCurrent;

void app_run(void)
{
		init_hardware();
		system_init();
	
		button_event_queue = xQueueCreate( 10, sizeof( Button* ));
	
		if(NULL==button_event_queue)
		{
			write(USART1_ID, (char*)BUTTON_TASK_ERROR, sizeof(BUTTON_TASK_ERROR)/sizeof(char));
		}
	
		write(USART1_ID, (char*)HARDWARE_CORE, sizeof(HARDWARE_CORE)/sizeof(char));
		write(USART1_ID, (char*)SOFTWARE_VERSION, sizeof(SOFTWARE_VERSION)/sizeof(char));
		write(USART1_ID, (char*)DESCRIPT_STRING, sizeof(DESCRIPT_STRING)/sizeof(char));
	
		/* 系统LED闪烁任务 */
	  osThreadDef(RunLedThread, run_led_task, osPriorityIdle, 0, 128);
		run_led_task_handle = osThreadCreate(osThread(RunLedThread), NULL);
	
		/* RGB显示任务 */
		osThreadDef(RgbThread, rgb_led_task, osPriorityIdle, 0, 126);
		osThreadCreate(osThread(RgbThread), NULL);
	
		/* 蜂鸣器任务 */
		osThreadDef(BeepThread, beep_task, osPriorityIdle, 0, 128);
		beep_task_handle = osThreadCreate(osThread(BeepThread), NULL);
			
		/* 喂狗任务 */
		osThreadDef(FeedDogThread, feed_dog_task, osPriorityIdle, 0, 128);
		feed_dog_task_handle = osThreadCreate(osThread(FeedDogThread), NULL);	
	
		/* 串口1发送任务 */
		osThreadDef(Usart1SendTask, usart1_send_task, osPriorityNormal, 0, 512);
		osThreadCreate(osThread(Usart1SendTask), NULL);	
		
		/* 串口2接收数据处理任务 */
		osThreadDef(Usart1RecTask, usart1_rec_task, osPriorityNormal, 0, 512);
		osThreadCreate(osThread(Usart1RecTask), NULL);
		
		/* 按键队列处理任务 */
		osThreadDef(ButtonTask, button_event_task, osPriorityRealtime, 0, 128);
		osThreadCreate(osThread(ButtonTask), NULL);	
	
		//osThreadDef(Usart3SendTestTask, usart3_send_test_task, osPriorityIdle, 0, 512);
		//osThreadCreate(osThread(Usart3SendTestTask), NULL);	
		
//		osThreadDef(Usart3RecTask, usart3_rec_task, osPriorityNormal, 0, 512);
//		osThreadCreate(osThread(Usart3RecTask), NULL);	
		
//		osThreadDef(ADC1ProTask, adc1_process_task, osPriorityNormal, 0, 512);
//		osThreadCreate(osThread(ADC1ProTask), NULL);				
		
		osThreadDef(OledTask, update_oled_task, osPriorityIdle, 0, 128);
		osThreadCreate(osThread(OledTask), NULL);	
	
}

/*  同步飞控，10HZ调用频率
***  
*/
void rgb_led_task(void const * argument)
{
	RGB1_Close();
	RGB2_Close();
	
	while(1)
	{
		osDelay(100);   //10HZ 
		rgb_switch();
	}

}

void run_led_task(void const * argument)
{
	reset_run_led();		
	while(1)
	{
		if(1==run_led_enbale)
		{
			set_run_led();
			osDelay(20);
			reset_run_led();
			osDelay(800);
		}
		else
		{
			reset_run_led();
		}
	}
}

void status_led_task(void const * argument)
{
	reset_status_led();
	
	while(1)
	{
		if(1==status_led_enable)
		{
			set_status_led();
			osDelay(5000);
			reset_status_led();
			osDelay(1000);
		}
		else
		{
			reset_status_led();
		}
	}
}

void beep_task(void const * argument)
{
	reset_beep();
	
	while(1)
	{
		osDelay(100);   //10HZ 
		beep_switch();
//		/* auto test code */
//		/* end test code */
//		if(system_flag.test_mask_bit & BEEP_TEST_MASK_BIT_VALID)
//		{
//			set_beep();
//			osDelay(50);
//			reset_beep();
//			osDelay(500);
//			continue;
//		}
//		
//		if(0==system_flag.beep_enable) 
//		{
//			reset_beep();
//			continue;
//		}
//		
//		set_beep();
//		osDelay(1000);
//		reset_beep();
//		osDelay(1000);
	}
}

void feed_dog_task(void const * argument)
{
	while(1)
	{
		HAL_IWDG_Refresh(&hiwdg);
		osDelay(200);  /* 200ms feed the dog, avoid task wait to long time. if no feed dog, the system reset */
	}
}

void usart1_send_task(void const* argument)
{
	uint8_t send_buff[512] = {0};
	unsigned int len = 0;
	LoopQueue* sendQueue = 0;
	unsigned int count = 0;
	unsigned int command_ack_time[5] = {0, 0, 0, 0, 0};
	
	while(1)
	{
		osDelay(20);
		
		/* 发送磁罗盘校准指令 */
		if(COMPASS_CAL_SEND==system_flag.drone_send_compass_cal)
		{
			mavlink_msg_command_long_send(USART1_ID, 0x01, 0x01, MAV_CMD_DO_START_MAG_CAL, 0, system_flag.compass_select, 1, 0, 0, 0, 0, 0);
			system_flag.drone_send_compass_cal = COMPASS_CAL_START;
		}else if(COMPASS_CAL_START==system_flag.drone_send_compass_cal)
		{
			++command_ack_time[0];
			if(command_ack_time[0]*20>=5000) /* 5秒之后没有收到应答信号 */
			{
				system_flag.drone_send_compass_cal = COMPASS_CAL_OLED_PAGE;   /* 没有接受到指令应到消息*/
				command_ack_time[0] = 0;
				system_flag.oled_page = 8;
			}
		}else if(COMPASS_CAL_END==system_flag.drone_send_compass_cal)
		{
			command_ack_time[0] = 0;
		}
		
		++count;
		if(1500==count) /* 每隔30s发送一次请求发送数据控制 */
		{
			//request_send_data(MAVLINK_COMM_1);
			count = 0;
		}
		
		xSemaphoreTake( usart1_send_mutex, portMAX_DELAY );
		
		sendQueue = getUsartSendLoopQueue(USART1_ID); /* get send queue */
		if(sendQueue!=0)
		{		
			len = writeBuffLen(USART1_ID); /* send queue data count */
			if(len>0)
			{
				unsigned int i = 0;
				if(len>512) len=512;
				for( i=0; i<len; ++i)
				{
					send_buff[i] = readCharLoopQueue(sendQueue);
				}
				HAL_UART_Transmit_DMA(&huart1, send_buff, (uint16_t)len); /* DMA send	*/
			}
		}
		
		xSemaphoreGive(usart1_send_mutex);
	}
}

void usart1_rec_task(void const* argument)
{
	unsigned char buff[512] = {0};
	unsigned int len = 0;
	unsigned int i = 0;
	
	while(1)
	{
		osDelay(20);
		
		len = read(USART1_ID, (char*)buff, 512);
		i = 0;
		
		if(len>0)
		{			
			while(len!=0)
			{
				mavlinkV1_parse(USART1_ID, buff[i]);
				++i;
				--len;
			}
		}
	}
}

/* usart3 send task, each 20ms send */
void usart3_send_task(void const* argument)
{
	uint8_t send_buff[512] = {0};
	unsigned int len = 0;
	LoopQueue* sendQueue = 0;
	
	while(1)
	{
		osDelay(20);
		xSemaphoreTake( usart3_send_mutex, portMAX_DELAY );
		
		sendQueue = getUsartSendLoopQueue(USART3_ID); /* get send queue */
		if(sendQueue!=0)
		{		
			len = writeBuffLen(USART3_ID); /* send queue data count */
			if(len>0)
			{
				unsigned int i = 0;
				if(len>512) len=512;
				for( i=0; i<len; ++i)
				{
					send_buff[i] = readCharLoopQueue(sendQueue);
				}
				HAL_UART_Transmit_DMA(&huart3, send_buff, (uint16_t)len); /* DMA send	*/
			}
		}
		
		xSemaphoreGive(usart3_send_mutex);
	}
}

void usart3_rec_task(void const* argument)
{
	unsigned char buff[512] = {0};
	unsigned int len = 0;
	
	while(1)
	{
		osDelay(20);
		
		len = read(USART3_ID, (char*)buff, 512);
		
		if(len>0)
		{			
			xSemaphoreTake( usart3_send_mutex, portMAX_DELAY );
			
			write(USART3_ID, (char*)buff, len);
			
			xSemaphoreGive(usart3_send_mutex);
		}
	}
}

void button_event_task(void const* argument)
{
	init_system_button();
	Button* bt = 0;	
	
	while(1)
	{
		if( pdPASS==xQueueReceive( button_event_queue, &bt, 30 ) )
		{
			ButtonId id = (ButtonId)bt->id;
			
			if(BUTTON_STATUS_DOWN_START==bt->status) /* 识别到第一次按下，有可能是噪声 */
			{
				osDelay(10);    /* 消除抖动 */
				if(GPIO_PIN_RESET==read_button_pin_satus(id)) /* 确定按下 */
				{
					bt->status = BUTTON_STATUS_DOWN_ENTER;
					bt->clicked = 1;
					
					/* 处理按键事件*/
//					system_flag.rgb1_enable = 1;
//					system_flag.rgb2_enable = 0;
					/* end */
					//bt->status = BUTTON_STATUS_NONE;
					//bt->clicked = 0;
				}else
				{
					bt->status = BUTTON_STATUS_NONE;
					bt->down_time = 0;
				}
			}
			else if(BUTTON_STATUS_UP_START==bt->status) /* 按键回弹 */
			{
				osDelay(10);    /* 消除抖动 */
				if(GPIO_PIN_SET==read_button_pin_satus(id)) 
				{
					bt->status = BUTTON_STATUS_UP_ENTER;
					bt->release = 1;
					
					/* 处理按键回弹事件 */
					if(bt->down_time*30<500 && id==1 && system_flag.mavlink_exist && system_flag.drone_complete)
					{
						system_flag.init_loader=0;
						++system_flag.oled_page;
						if(2==system_flag.oled_page || 4==system_flag.oled_page)
						{
							++system_flag.oled_page;
						}
						if(system_flag.oled_page>7)
						{
							system_flag.oled_page = 1;
						}
					}
					
					if(bt->down_time*30<500 && id==0 && (COMPASS_CAL_OLED_PAGE==system_flag.drone_send_compass_cal))
					{
						++system_flag.compass_select;
						if(3==system_flag.compass_select)
						{
							system_flag.compass_select = 0;
						}
					}
					/* end */
					bt->down_time = 0;
					bt->status = BUTTON_STATUS_NONE;
					bt->release = 0;
				}else
				{
					bt->status = BUTTON_STATUS_NONE;
					bt->down_time = 0;
				}
			}
		}
		else
		{
			Button *b = 0;
			unsigned char i = 0;
			for( ; i<BUTTON_MAX_NUM; ++i)
			{
				b = get_button_by_id((ButtonId)i);
				if(b->status==BUTTON_STATUS_DOWN_ENTER)
				{
					++b->down_time;
					if(b->down_time*30>=3000 && 0==i && system_flag.mavlink_exist) /* 按钮一直按下3秒 */
					{
						/* 处理事件 */
						//system_flag.beep_enable = 1;
						if(COMPASS_CAL_END==system_flag.drone_send_compass_cal && b->down_time*30<3500)
						{
							system_flag.drone_send_compass_cal = COMPASS_CAL_OLED_PAGE;
							system_flag.oled_page = 8;
						}else if(COMPASS_CAL_OLED_PAGE==system_flag.drone_send_compass_cal && b->down_time*30>5000)
						{
							system_flag.drone_send_compass_cal = COMPASS_CAL_SEND;
							system_flag.oled_page = 9;
						}
					}else if(b->down_time*50>=10000)
					{
						/* 处理事件 */
						//system_flag.beep_enable = 0;
					}
				}
			}
		}
	}
}

void adc1_process_task(void const* argument)
{
	float temp = 0.0;
	float refvol = 0.0;
	char str[38] = {0};
	
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)get_adc1_dma_buff(), get_adc1_buff_count());
	
	while(1)
	{
		osDelay(100);
		
		temp = get_adc1_temp();
		refvol = get_adc1_refint_vol();
		
		sprintf(str, "%3.2fC\r\n", temp);
				
		xSemaphoreTake( usart3_send_mutex, portMAX_DELAY );		
		//write(USART3_ID, "adc\r\n", 5);
		write(USART3_ID, (char*)str, strlen(str));		
		sprintf(str, "%3.2fV\r\n", refvol);
		write(USART3_ID, (char*)str, strlen(str));
		xSemaphoreGive(usart3_send_mutex);
	}
}

/* init hardware app */
void init_hardware(void)
{
	initUsartBuff(USART1_ID); /* init usart1 send and recive queue */
	usart1_send_mutex = xSemaphoreCreateMutex();
	initUsartIT(&huart1); /* open usart recieve interrupt */
		
}

static void system_init(void)
{
	unsigned short i = 0;
	unsigned char *pc = 0;
	system_flag.armed = 0;
	system_flag.beep_enable = 1;
	system_flag.drone_complete = 0;
	system_flag.init_loader = 0;
	system_flag.mavlink_exist = 0;
	system_flag.mavlink_valid = 0;
	system_flag.oled_enable = 1;
	system_flag.rgb1_enable = 1;
	system_flag.rgb2_enable = 1;
	
	droneData.compass_variance = 0.0;
	droneData.pCameraFeedback = &cameraFeedback;
	droneData.pMagCalReport = &magCalReport;
	droneData.pRangefinder = &rangefinder;
	droneData.pStatusText = &statusText;
	droneData.temperature = 0;
	
	for( i=0,pc=(unsigned char*)&fail_status; i<sizeof(FailStatus); ++i)
	{
		*pc = 0;
		++pc;
	}
	
	init_drone_data();

}

/* 接收中断处理函数中调用，将接收的字节数据保存在接收循环缓冲队列中 */
void usartRecieveITCallBack(unsigned char id, UART_HandleTypeDef *huart)
{
	if(id>=USART_MAX_ID)
	return;
	
	if(USART1_ID==id)
	{
		return;
	}else if(USART2_ID==id)
	{
		return;
	}else if(USART3_ID==id)
	{
		if(huart->Init.Parity == UART_PARITY_NONE)
		{
			insertCharLoopQueue( getUsartRecLoopQueue(id), (unsigned char)huart->Instance->DR & (unsigned char)0x00FF);
		}
		else
		{
			insertCharLoopQueue( getUsartRecLoopQueue(id), (unsigned char)huart->Instance->DR & (unsigned char)0x007F);
		}
	}
	return;
}

/* 打开接收中断，不使用库函数的原因是库函数只支持打开中断指定获取一段数据 */
void initUsartIT(UART_HandleTypeDef *huart)
{
	if(huart==0) return;

	huart->ErrorCode = HAL_UART_ERROR_NONE;
	huart->RxState = HAL_UART_STATE_BUSY_RX;

	/* Process Unlocked */
	__HAL_UNLOCK(huart);

	/* Enable the UART Parity Error Interrupt */
	__HAL_UART_ENABLE_IT(huart, UART_IT_PE);

	/* Enable the UART Error Interrupt: (Frame error, noise error, overrun error) */
	__HAL_UART_ENABLE_IT(huart, UART_IT_ERR);

	/* Enable the UART Data Register not empty Interrupt */
	__HAL_UART_ENABLE_IT(huart, UART_IT_RXNE);
	
	return;
}

/* test code */
void usart3_send_test_task(void const* argument)
{
	char buff[] = "STM32F103VET6 TEST CODE IN FreeRTOS\r\n";
	while(1)
	{
		osDelay(1000);
		xSemaphoreTake( usart3_send_mutex, portMAX_DELAY );
		
		write(USART3_ID, buff, sizeof(buff)/sizeof(char));
		
		xSemaphoreGive(usart3_send_mutex);
	}
}

void update_oled_task(void const* argument)
{	
	unsigned char old_page = 0; 
	unsigned char count = 0;
	char buff[10];
		
	OLED_Init();
	OLED_Clear();
	display_title();
	display_logo();
	OLED_ShowString(0, 6, "link", 8);
	system_flag.init_loader=1;
	system_flag.oled_page = 0;
	
//	system_flag.drone_complete = 1;
//	system_flag.mavlink_exist = 1;
//	
	while(1)
	{
		if(system_flag.pre_fly_count!=system_flag.fly_count)  /* 飞行完成 */
		{
			system_flag.pre_fly_count = system_flag.fly_count;
			system_flag.drone_complete = 1;
			if(!system_flag.armed)
				system_flag.oled_page = 3;
		}
		
		if(system_flag.test_mask_bit & OLED_TEST_MASK_BIT_VALID)
		{
			OLED_Clear();
			OLED_ShowString(0, 2, " MAVSWORD OLED", 16);
			OLED_ShowString(0, 4, " <<< TEST >>>", 16);
			osDelay(1000);
			continue;
		}
		
		if(1==system_flag.init_loader)
		{
			system_flag.oled_page==0;
		}
		
		if(system_flag.init_loader==1 && system_flag.oled_page==0)
		{
			unsigned char *str = ".";
			
			OLED_ShowString(count*8, 7, str, 8);
			osDelay(500);
			++count;
			
			if(count>3)
			{
				count = 0;
				OLED_Clear_Area(8, 7, 40, 7);
				osDelay(500);
				old_page = system_flag.oled_page;
				count = 0;
			}	
		}
		
		osDelay(100);
		
		if(system_flag.init_loader==0)
		{			
			switch(system_flag.oled_page)
			{
				case 1:
				{
					if(old_page!=system_flag.oled_page)
					{
						old_page = system_flag.oled_page;
						OLED_Clear();
						osDelay(10);
						
						/* Mode:None Arm: N */
						display_string_Font8_16(0, 0, "Mode:"); /* 模式显示 x=40-79 */
						//custom_mode_to_string((control_mode_t)droneHeatbeat.custom_mode, buff);
						//display_string_Font8_16(40, 0, (char *)buff);
						display_string_Font8_16(88, 0, "Arm:");    /* 解锁提示x=120-127*/
						
						display_string_Font8_16(0, 2, "Batt:"); /*电压值从x=40开始*/
						display_string_Font8_16(0, 4, "Gps:");  /* GPS固定类型 x=36-71*/ 
						display_string_Font8_16(80, 4, "Num:");		 /* 星数 x=112-127*/
						display_string_Font8_16(0, 6, "EKF:");	 /* EKF提醒 x=32-63 */
						display_north(84, 6);
						OLED_ShowString(68,6,"CW",8);
						OLED_ShowString(104-6*6,7,"",8);
					}
					
					/* 显示有效数据 */
					
					break;
				}
				case 2:
				{
					if(old_page!=system_flag.oled_page)
					{
						old_page = system_flag.oled_page;
						OLED_Clear();
						
						display_string_Font8_16(0, 0, "Compass Cal: "); /*row0*/
						OLED_ShowString(0, 2, "          00% 1", 8);
						OLED_ShowString(0, 4, "          00% 2", 8);
						OLED_ShowString(104, 1, "id", 8);
						OLED_ShowString(0, 5, "offx: ", 8);	
						OLED_ShowString(0, 6, "offy: ", 8);	
						OLED_ShowString(0, 7, "offz: ", 8);	
						display_string_Font8_16(128-4*8, 6, "----");
					}
					break;
				}
				case 3:
				{
					if(old_page!=system_flag.oled_page)
					{
						old_page = system_flag.oled_page;
						OLED_Clear();
						
						display_string_Font8_16(0, 0, "  Fly Complete"); /*row0*/
						display_string_Font8_16(0, 2, "mAH:");
						display_string_Font8_16(0, 4, "Batt:");
						display_string_Font8_16(0, 6, "FlyTime:");
					}
					break;
				}
				
				case 5:
				{
					if(old_page!=system_flag.oled_page)
					{
						old_page = system_flag.oled_page;
						OLED_Clear();
						
						display_string_Font8_16(0, 0, "Vibe  Max  Avr");
						display_string_Font8_16(0, 2, "  X   ");  /* max: 48-71 , min: 88-111*/
						display_string_Font8_16(0, 4, "  Y   ");  
						display_string_Font8_16(0, 6, "  Z   ");
						//display_string_Font8_16(0, 2, "  X   000  000");
						//display_string_Font8_16(0, 4, "  Y   000  000");
						//display_string_Font8_16(0, 6, "  Z   000  000");
					}
										
					break;
				}
				
				case 6:
				{
					if(old_page!=system_flag.oled_page)
					{
						old_page = system_flag.oled_page;
						OLED_Clear();
						
						display_string_Font8_16(0, 0, " ACC  Max  Min");
						display_string_Font8_16(0, 2, "  X   ");
						display_string_Font8_16(0, 4, "  Y   ");
						display_string_Font8_16(0, 6, "  Z   ");
//						display_string_Font8_16(0, 2, "  X   000  000");
//						display_string_Font8_16(0, 4, "  Y   000  000");
//						display_string_Font8_16(0, 6, "  Z   000  000");
					}
										
					break;
				}
				
				case 7:
				{
					if(old_page!=system_flag.oled_page)
					{
						old_page = system_flag.oled_page;
						OLED_Clear();
						
						display_string_Font8_16(0, 0, "       Max  Avr");
						display_string_Font8_16(0, 2, "Speed  "); /* max: 56-79 104-127*/
						display_string_Font8_16(0, 4, "Throt  ");
						display_string_Font8_16(0, 6, "Curr>A ");
						
//						display_string_Font8_16(0, 2, "Speed  000  000");
//						display_string_Font8_16(0, 4, "Throt  000  000");
//						display_string_Font8_16(0, 6, "Curr   000  000");
					}					
					break;
				}
				
								/* 罗盘校准显示界面 */
				case 8:
				{
					if(old_page!=system_flag.oled_page)
					{
						if(COMPASS_CAL_OLED_PAGE==system_flag.drone_send_compass_cal)
						{
							old_page = system_flag.oled_page;
							OLED_Clear();
							display_string_Font8_16(0, 0, " Compass Select");
							display_string_Font8_16(0, 2, "     all");
							OLED_ShowString(0, 3, "  >>", 8);
							display_string_Font8_16(0, 4, "     extern");
							//OLED_ShowString(0, 5, "  >>", 8);
							display_string_Font8_16(0, 6, "     intern");
							//OLED_ShowString(0, 7, "  >>", 8);
						}
					}
					
					break;
				}
				
				case 9:
				{
					if(old_page!=system_flag.oled_page)
					{
						old_page = system_flag.oled_page;
						OLED_Clear();
						display_string_Font8_16(0, 3, " Wait Drone Ack ");							
					}
					
					break;
				}
				
				default: break;
			}
			
			/* 更新页面中的信息 */
			if(system_flag.mavlink_exist)
			{
				switch(old_page)
				{
					case 1:
					{
						if(0!=count%15) break;
						
						OLED_Clear_Area(40, 0, 80, 0); 
						custom_mode_to_string((control_mode_t)droneHeatbeat.custom_mode, buff);
						display_string_Font8_16(40, 0, (char *)buff);

						OLED_Clear_Area(120, 0, 128, 0); 
						if(1==system_flag.armed)
						{
							buff[0] = 'Y';
							buff[1] = '\0';
						}else
						{
							buff[0] = 'N';
							buff[1] = '\0';
						}
						display_string_Font8_16(120, 0, (char *)buff);						
						
						OLED_Clear_Area(40, 2, 128, 2);
						float_to_string((double)droneSysStatus.current_battery/1000.0, (char* )buff, 2, 2);
						display_string_Font8_16(40, 2, (char *)buff);
						display_string_Font8_16(80, 2, "V"); 
						
						OLED_Clear_Area(36, 4, 72, 4);
						gps_type_to_string(GPS_FIX_TYPE_RTK_FIXED, (char*)buff);
						display_string_Font8_16(36, 4, (char *)buff);
						OLED_Clear_Area(112, 4, 128, 4);
						int_to_string(droneGPSRawData.fix_type, (char*)buff, 2);
						display_string_Font8_16(112, 4, (char *)buff);
						
						OLED_Clear_Area(32, 6, 64, 6);
						if(1023==droneData.flags) /* EKF指标报告正常 */
						{
							strcpy(buff, "OK");
						}else
						{
							strcpy(buff, "Fail");
						}
						display_string_Font8_16(32, 6, (char *)buff);
						
						OLED_Clear_Area(68, 7, 98, 7);
						float_to_string((double)droneVfr.heading, (char*)buff, 3, 1);
						OLED_ShowString(68,7,buff,8);
						
						break;
					}
					
					case 2:
					{
						if(!system_flag.compass_select)  /* 两个磁罗盘全部校准 */
						{
							unsigned char i = 0;
							for(i=0; i<2; ++i)
							{
								/* 显示百分比 */
								int_to_string(droneMagStatus[i].completion_pct, (char*)buff, 2);
								if(1==droneMagStatus[i].compass_id)
								{
									OLED_Clear_Area(80, 2, 96, 2);
									OLED_ShowString(80, 2, buff, 8);
									
									if(0==droneMagStatus[i].completion_pct)
									{
										OLED_Clear_Area(0, 2, 80, 2);
									}else
									{
										OLED_set_area(0, 2, (droneMagStatus[i].completion_pct/10)*8, 2);   
									}
									
								}
								else if(2==droneMagStatus[i].compass_id)
								{
									OLED_Clear_Area(80, 4, 96, 4);
									OLED_ShowString(80, 4, buff, 8);
									
									if(0==droneMagStatus[i].completion_pct)
									{
										OLED_Clear_Area(0, 4, 80, 4);
									}else
									{
										OLED_set_area(0, 4, (droneMagStatus[i].completion_pct/10)*8, 4);   
									}
								}
							}
						}else
						{
							if(1==system_flag.compass_select)
							{
								OLED_Clear_Area(80, 2, 96, 2);
								OLED_ShowString(80, 2, buff, 8);
								
								OLED_set_area(0, 2, 10*8, 2);
								
								if(0==droneMagStatus[system_flag.compass_select-1].completion_pct)
								{
									OLED_Clear_Area(0, 2, 80, 2);
								}else
								{
									OLED_set_area(0, 2, (droneMagStatus[system_flag.compass_select-1].completion_pct/10)*8, 4);   
								}
							}else if(2==system_flag.compass_select)
							{
								OLED_Clear_Area(80, 4, 96, 4);
								OLED_ShowString(80, 4, buff, 8);
								
								if(0==droneMagStatus[system_flag.compass_select-1].completion_pct)
								{
									OLED_Clear_Area(0, 4, 80, 4);
								}else
								{
									OLED_set_area(0, 4, (droneMagStatus[system_flag.compass_select-1].completion_pct/10)*8, 4);   
								}
							}

						}
						
						break;
					}
					case 3:
					{
						if(0!=count%15) break;
						
						OLED_Clear_Area(32, 2, 128, 2); 
						int_to_string(28800, (char*)buff, 7);
						display_string_Font8_16(32, 2, (char *)buff);
						display_string_Font8_16(80, 2, "MAH");
						
						OLED_Clear_Area(40, 4, 128, 4);
						float_to_string((double)droneSysStatus.current_battery/1000.0, (char* )buff, 2, 2);
						display_string_Font8_16(40, 4, (char *)buff);
						display_string_Font8_16(80, 4, "V"); 
						
						OLED_Clear_Area(64, 6, 128, 6);
						/* 请加入时间计算转换字符串代码 */
						display_string_Font8_16(64, 6, "00:00:00");
						
						break;
					}
					
					case 5:
					{
						if(system_flag.drone_complete)
						{
							OLED_Clear_Area(48, 2, 128, 2);
							OLED_Clear_Area(48, 4, 128, 4);
							OLED_Clear_Area(48, 6, 128, 6);
							
							vibe_pro(&droneVibePro);
							
							int_to_string(droneVibePro.x_vibe_max, (char* )buff, 3);
							display_string_Font8_16(48, 2, (char *)buff);
							int_to_string(droneVibePro.x_vibe_avr, (char* )buff, 3);
							display_string_Font8_16(88, 2, (char *)buff);
							
							int_to_string(droneVibePro.y_vibe_max, (char* )buff, 3);
							display_string_Font8_16(48, 4, (char *)buff);
							int_to_string(droneVibePro.y_vibe_avr, (char* )buff, 3);
							display_string_Font8_16(88, 4, (char *)buff);
							
							int_to_string(droneVibePro.z_vibe_max, (char* )buff, 3);
							display_string_Font8_16(48, 6, (char *)buff);
							int_to_string(droneVibePro.z_vibe_avr, (char* )buff, 3);
							display_string_Font8_16(88, 6, (char *)buff);
						}
						break;
					}
					
					case 6:
					{
						if(system_flag.drone_complete)
						{
							OLED_Clear_Area(48, 2, 128, 2);
							OLED_Clear_Area(48, 4, 128, 4);
							OLED_Clear_Area(48, 6, 128, 6);
														
							int_to_string(droneImuPro.xacc_max, (char* )buff, 3);
							display_string_Font8_16(48, 2, (char *)buff);
							int_to_string(droneImuPro.xacc_min, (char* )buff, 3);
							display_string_Font8_16(88, 2, (char *)buff);
							
							int_to_string(droneImuPro.yacc_max, (char* )buff, 3);
							display_string_Font8_16(48, 4, (char *)buff);
							int_to_string(droneImuPro.yacc_min, (char* )buff, 3);
							display_string_Font8_16(88, 4, (char *)buff);
							
							int_to_string(droneImuPro.zacc_max, (char* )buff, 3);
							display_string_Font8_16(48, 6, (char *)buff);
							int_to_string(droneImuPro.zacc_min, (char* )buff, 3);
							display_string_Font8_16(88, 6, (char *)buff);		
						}
						break;
					}						
						
					case 7:
					{
						if(system_flag.drone_complete)
						{
							OLED_Clear_Area(56, 2, 128, 2);
							OLED_Clear_Area(56, 4, 128, 4);
							OLED_Clear_Area(56, 6, 128, 6);
							
							info_pro(&droneInfo);
							
							float_to_string(droneInfo.groundspeed_max, (char* )buff, 2, 1);
							display_string_Font8_16(56, 2, (char *)buff);
							float_to_string(droneInfo.groundspeed_avr, (char* )buff, 2, 1);
							display_string_Font8_16(104, 2, (char *)buff);
							
							int_to_string(droneInfo.throttle_max, (char* )buff, 3);
							display_string_Font8_16(56, 4, (char *)buff);
							display_string_Font8_16(72, 4, "%");
							int_to_string(droneInfo.throttle_avr, (char* )buff, 3);
							display_string_Font8_16(104, 4, (char *)buff);
							display_string_Font8_16(120, 4, "%");
							
							float_to_string((droneCurrent.current_max*10.0)/1000, (char* )buff, 2, 2);
							display_string_Font8_16(56, 6, (char *)buff);
							float_to_string((droneCurrent.current_sum*10.0)/1000/droneCurrent.count, (char* )buff, 2, 2);
							display_string_Font8_16(104, 6, (char *)buff);						
						}
						break;
					}
					
					default: break;
				}
			}
			++count;
			/* 公共显示部分 */
			if(system_flag.oled_page<2 && system_flag.oled_page>0 || 9==system_flag.oled_page)
			{
				if(count%10==1 && count<22)
				{
					OLED_ShowString(110+(count/10)*6,7,".",8);
				}else if(count==22)
				{
					OLED_Clear_Area(110, 7, 128, 7);
				}else if(count>25)
				{
					count = 0;
				}
			}
			if(COMPASS_CAL_OLED_PAGE==system_flag.drone_send_compass_cal && 8==system_flag.oled_page)
			{
				OLED_Clear_Area(0, 3, 32, 3);
				OLED_Clear_Area(0, 5, 32, 5);
				OLED_Clear_Area(0, 7, 32, 7);
				if(0==system_flag.compass_select)
				{
					OLED_ShowString(0, 3, "  >>", 8);
				}else if(1==system_flag.compass_select)
				{
					OLED_ShowString(0, 5, "  >>", 8);
				}else if(2==system_flag.compass_select)
				{
					OLED_ShowString(0, 7, "  >>", 8);
				}
			}
		}
		
		
	}
}

/* 调用发送函数 */
void usart_send(unsigned char id, char* pc, unsigned int len)
{
	if(id>=USART_MAX_ID || 0==pc || 0==len)
	return ;
	
	if(USART1_ID==id)
	{
		xSemaphoreTake( usart1_send_mutex, portMAX_DELAY );			
		write(USART1_ID, (char*)pc, len);			
		xSemaphoreGive(usart1_send_mutex);
	}else if(USART2_ID==id)
	{
	}else if(USART3_ID==id)
	{
	}
}

static unsigned char mavlinkV1_parse(uint8_t chan, uint8_t c)
{
	uint8_t parse_result = MAVLINK_FRAMING_INCOMPLETE;
	mavlink_message_t r_message;
	mavlink_status_t r_mavlink_status;
	
	parse_result = mavlink_parse_char(chan, c, &r_message, &r_mavlink_status);
	
	if(MAVLINK_FRAMING_OK==parse_result)
	{
		switch(r_message.msgid)
		{
			/* #0  心跳包 */
			case MAVLINK_MSG_ID_HEARTBEAT:
			{
				droneHeatbeat.custom_mode = mavlink_msg_heartbeat_get_custom_mode(&r_message); 
				droneHeatbeat.base_mode = mavlink_msg_heartbeat_get_base_mode(&r_message); 
				droneHeatbeat.type = mavlink_msg_heartbeat_get_type(&r_message); 
				droneHeatbeat.autopilot = mavlink_msg_heartbeat_get_autopilot(&r_message); /*< Autopilot type / class. defined in MAV_AUTOPILOT ENUM*/
				droneHeatbeat.system_status = mavlink_msg_heartbeat_get_system_status(&r_message); /*< System status flag, see MAV_STATE ENUM*/
				droneHeatbeat.mavlink_version = mavlink_msg_heartbeat_get_mavlink_version(&r_message); /*< MAVLink version, not writable by user, gets added by protocol because of magic data type: uint8_t_mavlink_version*/
				system_flag.mavlink_exist = 1; /* 通道中识别到心跳包，mavlink数据有效 */
				system_flag.init_loader = 0;   /* 跳过启动界面 */
				if((droneHeatbeat.base_mode & MAV_MODE_FLAG_SAFETY_ARMED)&&(system_flag.fly_count==system_flag.pre_fly_count))
				{
					system_flag.armed = 1;
					++system_flag.fly_count;
				}else
				{
					system_flag.armed = 0;
				}
				break;
			}
			
			/* #1  系统标志：板上传感器状态 */
			case MAVLINK_MSG_ID_SYS_STATUS:
			{
				droneSysStatus.onboard_control_sensors_present = mavlink_msg_sys_status_get_onboard_control_sensors_present(&r_message); /*< Bitmask showing which onboard controllers and sensors are present. Value of 0: not present. Value of 1: present. Indices defined by ENUM MAV_SYS_STATUS_SENSOR*/
				droneSysStatus.onboard_control_sensors_enabled = mavlink_msg_sys_status_get_onboard_control_sensors_enabled(&r_message); /*< Bitmask showing which onboard controllers and sensors are enabled:  Value of 0: not enabled. Value of 1: enabled. Indices defined by ENUM MAV_SYS_STATUS_SENSOR*/
				droneSysStatus.onboard_control_sensors_health = mavlink_msg_sys_status_get_onboard_control_sensors_health (&r_message); /*< Bitmask showing which onboard controllers and sensors are operational or have an error:  Value of 0: not enabled. Value of 1: enabled. Indices defined by ENUM MAV_SYS_STATUS_SENSOR*/
				droneSysStatus.load = mavlink_msg_sys_status_get_load(&r_message); /*< Maximum usage in percent of the mainloop time, (0%: 0, 100%: 1000) should be always below 1000*/
				droneSysStatus.voltage_battery = mavlink_msg_sys_status_get_voltage_battery(&r_message); /*< Battery voltage, in millivolts (1 = 1 millivolt)*/
				droneSysStatus.current_battery = mavlink_msg_sys_status_get_current_battery(&r_message); /*< Battery current, in 10*milliamperes (1 = 10 milliampere), -1: autopilot does not measure the current*/
				droneSysStatus.battery_remaining = mavlink_msg_sys_status_get_battery_remaining(&r_message); /*< Remaining battery energy: (0%: 0, 100%: 100), -1: autopilot estimate the remaining battery*/
				
				mav_sys_status_pro(&droneSysStatus);
				if(system_flag.armed)
				{
					droneCurrent.current_sum += droneSysStatus.current_battery;
					if(droneCurrent.current_max<droneSysStatus.current_battery)
						droneCurrent.current_max = droneSysStatus.current_battery;
					++droneCurrent.count;
				}
				
				break;	
			}
			
			/* #2 飞控启动时间 */
			case MAVLINK_MSG_ID_SYSTEM_TIME:
			{
				droneData.drone_time_boot_ms = mavlink_msg_system_time_get_time_boot_ms(&r_message); /*< Timestamp of the component clock since boot time in milliseconds.*/
				break;				
			}
			
			/* #74 飞行器基本信息，高度，速度等 */
			case MAVLINK_MSG_ID_VFR_HUD:
			{
				droneVfr.airspeed = mavlink_msg_vfr_hud_get_airspeed(&r_message); /*< Current airspeed in m/s*/
				droneVfr.groundspeed = mavlink_msg_vfr_hud_get_groundspeed(&r_message); /*< Current ground speed in m/s*/
				droneVfr.alt = mavlink_msg_vfr_hud_get_alt(&r_message); /*< Current altitude (MSL), in meters*/
				droneVfr.climb = mavlink_msg_vfr_hud_get_climb(&r_message); /*< Current climb rate in meters/second*/
				droneVfr.heading = mavlink_msg_vfr_hud_get_heading(&r_message); /*< Current heading in degrees, in compass units (0..360, 0=north)*/
				droneVfr.throttle = mavlink_msg_vfr_hud_get_throttle(&r_message); /*< Current throttle setting in integer percent, 0 to 100*/
				
				info_sum(&droneVfr, &droneInfo);
				break;
			}
			
			/* #241 飞行器震动反馈 */
			case MAVLINK_MSG_ID_VIBRATION:
			{
				if(system_flag.armed)
				{
					droneVibe.vibration_x = mavlink_msg_vibration_get_vibration_x(&r_message); /*< Vibration levels on X-axis*/
					droneVibe.vibration_y = mavlink_msg_vibration_get_vibration_y(&r_message); /*< Vibration levels on Y-axis*/
					droneVibe.vibration_z = mavlink_msg_vibration_get_vibration_z(&r_message); /*< Vibration levels on Z-axis*/
					vibe_sum(&droneVibe, &droneVibePro);
				}
				break;
			}
			
			/* #24 GPS1 原始数据 */
			case MAVLINK_MSG_ID_GPS_RAW_INT:
			{
				droneGPSRawData.lat = mavlink_msg_gps_raw_int_get_lat(&r_message); /*< Latitude (WGS84, EGM96 ellipsoid), in degrees * 1E7*/
				droneGPSRawData.lon = mavlink_msg_gps_raw_int_get_lon(&r_message); /*< Longitude (WGS84, EGM96 ellipsoid), in degrees * 1E7*/
				droneGPSRawData.alt = mavlink_msg_gps_raw_int_get_alt(&r_message); /*< Altitude (AMSL, NOT WGS84), in meters * 1000 (positive for up). Note that virtually all GPS modules provide the AMSL altitude in addition to the WGS84 altitude.*/
				droneGPSRawData.eph = mavlink_msg_gps_raw_int_get_eph(&r_message); /*< GPS HDOP horizontal dilution of position (unitless). If unknown, set to: UINT16_MAX*/
				droneGPSRawData.epv = mavlink_msg_gps_raw_int_get_epv(&r_message); /*< GPS VDOP vertical dilution of position (unitless). If unknown, set to: UINT16_MAX*/
				//uint16_t vel; /*< GPS ground speed (m/s * 100). If unknown, set to: UINT16_MAX*/
				//uint16_t cog; /*< Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX*/
				droneGPSRawData.fix_type = mavlink_msg_gps_raw_int_get_fix_type(&r_message); /*< See the GPS_FIX_TYPE enum.*/
				droneGPSRawData.satellites_visible = mavlink_msg_gps_raw_int_get_satellites_visible(&r_message); /*< Number of satellites visible. If unknown, set to 255*/
				break;
			}
			
			/* #124 GPS2原始数据 */
			case MAVLINK_MSG_ID_GPS2_RAW:
			{
				droneGPS2RawData.lat = mavlink_msg_gps2_raw_get_lat(&r_message); /*< Latitude (WGS84), in degrees * 1E7*/
				droneGPS2RawData.lon = mavlink_msg_gps2_raw_get_lon(&r_message); /*< Longitude (WGS84), in degrees * 1E7*/
				droneGPS2RawData.alt = mavlink_msg_gps2_raw_get_alt(&r_message); /*< Altitude (AMSL, not WGS84), in meters * 1000 (positive for up)*/
				droneGPS2RawData.dgps_age = mavlink_msg_gps2_raw_get_dgps_age(&r_message); /*< Age of DGPS info*/
				droneGPS2RawData.eph = mavlink_msg_gps2_raw_get_eph(&r_message); /*< GPS HDOP horizontal dilution of position in cm (m*100). If unknown, set to: UINT16_MAX*/
				droneGPS2RawData.epv = mavlink_msg_gps2_raw_get_epv(&r_message); /*< GPS VDOP vertical dilution of position in cm (m*100). If unknown, set to: UINT16_MAX*/
				//uint16_t vel; /*< GPS ground speed (m/s * 100). If unknown, set to: UINT16_MAX*/
				//uint16_t cog; /*< Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX*/
				droneGPS2RawData.fix_type =  mavlink_msg_gps2_raw_get_fix_type(&r_message); /*< See the GPS_FIX_TYPE enum.*/
				droneGPS2RawData.satellites_visible =  mavlink_msg_gps2_raw_get_satellites_visible(&r_message); /*< Number of satellites visible. If unknown, set to 255*/
				droneGPS2RawData.dgps_numch = mavlink_msg_gps2_raw_get_dgps_numch(&r_message); /*< Number of DGPS satellites*/
				break;
			}
			
			/* #30 姿态数据 */
			case MAVLINK_MSG_ID_ATTITUDE:
			{
				droneAtt.roll= mavlink_msg_attitude_get_roll(&r_message); /*< Roll angle (rad, -pi..+pi)*/
				droneAtt.pitch = mavlink_msg_attitude_get_pitch(&r_message); /*< Pitch angle (rad, -pi..+pi)*/
				droneAtt.yaw = mavlink_msg_attitude_get_yaw(&r_message); /*< Yaw angle (rad, -pi..+pi)*/
//				float rollspeed; /*< Roll angular speed (rad/s)*/
//				float pitchspeed; /*< Pitch angular speed (rad/s)*/
//				float yawspeed; /*< Yaw angular speed (rad/s)*/
				break;
			}
			
			/* #28 气压和温度 */
			case MAVLINK_MSG_ID_RAW_PRESSURE:
			{
				droneData.temperature = mavlink_msg_raw_pressure_get_temperature(&r_message); /*< Raw Temperature measurement (raw)*/
				break;
			}
			
			/* #168 风速 */
			case MAVLINK_MSG_ID_WIND:
			{
				droneWind.direction = mavlink_msg_wind_get_direction(&r_message); /*< wind direction that wind is coming from (degrees)*/
				droneWind.speed =  mavlink_msg_wind_get_speed(&r_message); /*< wind speed in ground plane (m/s)*/
				//float speed_z; /*< vertical wind speed (m/s)*/
				break;
			}
			
			/* #77 指令应答 */
			case MAVLINK_MSG_ID_COMMAND_ACK:
			{
				uint16_t command = mavlink_msg_command_ack_get_command(&r_message); /*< Command ID, as defined by MAV_CMD enum.*/
				uint8_t result = mavlink_msg_command_ack_get_result(&r_message); /*< See MAV_RESULT enum*/
				break;
			}
			
			/* #193 EKF反馈报告 */
			case MAVLINK_MSG_ID_EKF_STATUS_REPORT:
			{
				droneData.compass_variance = mavlink_msg_ekf_status_report_get_compass_variance(&r_message); /*< Compass variance*/
				droneData.flags = mavlink_msg_ekf_status_report_get_flags(&r_message); /*< Flags*/	
					
				mav_ekf_report_pro(&droneData);
				break;
			}
			
			/* #27 IMU原始数据 */
			case MAVLINK_MSG_ID_RAW_IMU:
			{
				if(system_flag.armed)
				{
					droneIMU.xacc = mavlink_msg_raw_imu_get_xacc(&r_message); /*< X acceleration (raw)*/
					droneIMU.yacc = mavlink_msg_raw_imu_get_yacc(&r_message); /*< Y acceleration (raw)*/
					droneIMU.zacc = mavlink_msg_raw_imu_get_zacc(&r_message); /*< Z acceleration (raw)*/
					acc_sum(&droneIMU, &droneImuPro);
				}
//				int16_t xgyro; /*< Angular speed around X axis (raw)*/
//				int16_t ygyro; /*< Angular speed around Y axis (raw)*/
//				int16_t zgyro; /*< Angular speed around Z axis (raw)*/
//				int16_t xmag; /*< X Magnetic field (raw)*/
//				int16_t ymag; /*< Y Magnetic field (raw)*/
//				int16_t zmag; /*< Z Magnetic field (raw)*/
				break;
			}
			
			/* #180 相机反馈POS */
			case MAVLINK_MSG_ID_CAMERA_FEEDBACK:
			{
				mavlink_msg_camera_feedback_decode(&r_message, droneData.pCameraFeedback);

				break;
			}
			
			/* #191 磁罗盘校准进度 */
			case MAVLINK_MSG_ID_MAG_CAL_PROGRESS:
			{
				unsigned char id = 0;
				droneMagCalPro.compass_id = mavlink_msg_mag_cal_progress_get_compass_id(&r_message);; /*< Compass being calibrated*/
				droneMagCalPro.cal_mask = mavlink_msg_mag_cal_progress_get_cal_mask(&r_message); /*< Bitmask of compasses being calibrated*/
				droneMagCalPro.cal_status = mavlink_msg_mag_cal_progress_get_cal_status(&r_message); /*< Status (see MAG_CAL_STATUS enum)*/
				droneMagCalPro.attempt = mavlink_msg_mag_cal_progress_get_attempt(&r_message); /*< Attempt number 校准次数*/
				droneMagCalPro.completion_pct = mavlink_msg_mag_cal_progress_get_completion_pct(&r_message); /*< Completion percentage 校准进度  百分比*/
				
				/* id=1为外置磁罗盘    id=2为内置磁罗盘 */
				id = droneMagCalPro.compass_id;
				droneMagStatus[id-1].compass_id = id;
				droneMagStatus[id-1].attempt = droneMagCalPro.attempt;
				droneMagStatus[id-1].cal_status = droneMagCalPro.cal_status;
				droneMagStatus[id-1].cal_mask = droneMagCalPro.cal_mask;
				droneMagStatus[id-1].completion_pct = droneMagCalPro.completion_pct;
				
				break;
			}
			
			/* #192 磁罗盘校准反馈报告 */
			case MAVLINK_MSG_ID_MAG_CAL_REPORT:
			{				
				mavlink_msg_mag_cal_report_decode(&r_message, droneData.pMagCalReport);
				if(COMPASS_CAL_PROING==system_flag.drone_send_compass_cal)   /* 磁罗盘校准成功 */
				{
					system_flag.drone_send_compass_cal = COMPASS_CAL_COMPLETE;
				}
				
				break;
			}
			
			
			/* #173 测距模块反馈 */
			case MAVLINK_MSG_ID_RANGEFINDER:
			{
				mavlink_msg_rangefinder_decode(&r_message, droneData.pRangefinder);
				
				break;
			}
			
			/* #253 文本信息  */
			case MAVLINK_MSG_ID_STATUSTEXT:
			{
				mavlink_msg_statustext_decode(&r_message, droneData.pStatusText);
				mav_status_text_pro(droneData.pStatusText);
				break;
			}
			default: break;
		}
		return 1;
	}
	return 0;
}

static void request_send_data(uint8_t chan)
{
	mavlink_request_data_stream_t r_send_data;
	
	r_send_data.req_message_rate = 2;
	r_send_data.req_stream_id = 2;
	r_send_data.start_stop = 1;
	r_send_data.target_component = 0x01;
	r_send_data.target_system = 0x01;
	
	mavlink_msg_request_data_stream_send_struct(chan, &r_send_data);
	mavlink_msg_request_data_stream_send_struct(chan, &r_send_data);
	
	r_send_data.req_stream_id = 6; 
	r_send_data.req_message_rate = 2;	
	mavlink_msg_request_data_stream_send_struct(chan, &r_send_data);
	mavlink_msg_request_data_stream_send_struct(chan, &r_send_data);
	
	r_send_data.req_stream_id = 10; 
	r_send_data.req_message_rate = 4;	
	mavlink_msg_request_data_stream_send_struct(chan, &r_send_data);
	mavlink_msg_request_data_stream_send_struct(chan, &r_send_data);
	
	r_send_data.req_stream_id = 11; 
	r_send_data.req_message_rate = 4;	
	mavlink_msg_request_data_stream_send_struct(chan, &r_send_data);
	mavlink_msg_request_data_stream_send_struct(chan, &r_send_data);
	
	r_send_data.req_stream_id = 12; 
	r_send_data.req_message_rate = 2;	
	mavlink_msg_request_data_stream_send_struct(chan, &r_send_data);
	mavlink_msg_request_data_stream_send_struct(chan, &r_send_data);
	
	r_send_data.req_stream_id = 1; 
	r_send_data.req_message_rate = 2;	
	mavlink_msg_request_data_stream_send_struct(chan, &r_send_data);
	mavlink_msg_request_data_stream_send_struct(chan, &r_send_data);
	
	r_send_data.req_stream_id = 3; 
	r_send_data.req_message_rate = 2;	
	mavlink_msg_request_data_stream_send_struct(chan, &r_send_data);
	mavlink_msg_request_data_stream_send_struct(chan, &r_send_data);
	
}

static void init_drone_data(void)
{
	droneData.compass_variance = 100.0;
	droneData.drone_time_boot_ms = 0;
	droneData.flags = 0;
	droneData.pCameraFeedback = &cameraFeedback;
	droneData.pRangefinder = &rangefinder;
	droneData.pMagCalReport = &magCalReport;
	droneData.pStatusText = &statusText;
	droneData.temperature = 0;
}


static void custom_mode_to_string(control_mode_t mode, char* buff)
{
	switch(mode)
	{
		case STABILIZE:  // manual airframe angle with manual throttle
		{	
			strcpy(buff,"STAB");
			break;
		}
		case ACRO:       // manual body-frame angular rate with manual throttle
		{
			strcpy(buff,"ACRO");
			break;
		}
		case ALT_HOLD:	// manual airframe angle with automatic throttle
		{
			strcpy(buff,"ALTH");
			break;
		}
		case AUTO:			// fully automatic waypoint control using mission commands
		{
			strcpy(buff,"AUTO");
			break;
		}
		case GUIDED:   // fully automatic fly to coordinate or fly at velocity/direction using GCS immediate commands
		{
			strcpy(buff,"GUID");
			break;
		}
		case LOITER:	// automatic horizontal acceleration with automatic throttle
		{
			strcpy(buff,"LOIT");
			break;
		}
		case RTL:
		{
			strcpy(buff,"RTL");
			break;
		}
		case CIRCLE:   // automatic circular flight with automatic throttle
		{
			strcpy(buff,"CIRC");
			break;
		}
		case LAND:		 // automatic landing with horizontal position control
		{
			strcpy(buff,"LAND");
			break;
		}
		case DRIFT:		 // semi-automous position, yaw and throttle control
		{
			strcpy(buff,"DRIFT");
			break;
		}
		case SPORT:		 // manual earth-frame angular rate control with manual throttle
		{
			strcpy(buff,"SPORT");
			break;
		}
		case FLIP:		 // automatically flip the vehicle on the roll axis
		{
			strcpy(buff,"FLIP");
			break;
		}
		case AUTOTUNE: // automatically tune the vehicle's roll and pitch gains
		{
			strcpy(buff,"ATUNE");
			break;
		}
		case POSHOLD:  // automatic position hold with manual override, with automatic throttle
		{
			strcpy(buff,"PHOLD");
			break;
		}
		case BRAKE:		// full-brake using inertial/GPS system, no pilot input
		{
			strcpy(buff,"BRAKE");
			break;
		}
		case THROW:   // throw to launch mode using inertial/GPS system, no pilot input
		{
			strcpy(buff,"THROW");
			break;
		}
		case AVOID_ADSB: // automatic avoidance of obstacles in the macro scale - e.g. full-sized aircraft
		{
			strcpy(buff,"AVOID");
			break;
		}
		case GUIDED_NOGPS:	// guided mode but only accepts attitude and altitude
		{
			strcpy(buff,"GNGPS");
			break;
		}
		case SMART_RTL:			// SMART_RTL returns to home by retracing its steps
		{
			strcpy(buff,"SMRTL");
			break;
		}
		case FLOWHOLD:			// FLOWHOLD holds position with optical flow without rangefinder
		{
			strcpy(buff,"FHOLD");
			break;
		}
		case FOLLOW:
		{
			strcpy(buff,"FOLLW");
			break;
		}
		default: 
		{
			strcpy(buff,"NONE");
			break;
		}
	}
}

static void gps_type_to_string(GPS_FIX_TYPE fix_type, char* buff)
{
	switch(fix_type)
	{
		case GPS_FIX_TYPE_NO_GPS: /* No GPS connected | */
		{
			strcpy(buff,"NONE");
			break;
		}
		case GPS_FIX_TYPE_NO_FIX: /* No position information, GPS is connected | */
		{
			strcpy(buff,"NOFIX");
			break;
		}
		case GPS_FIX_TYPE_2D_FIX:  /* 2D position | */
		{
			strcpy(buff,"2DFIX");
			break;
		}			
		case GPS_FIX_TYPE_3D_FIX: /* 3D position | */
		{
			strcpy(buff,"3DFIX");
			break;
		}
		case GPS_FIX_TYPE_DGPS: /* DGPS/SBAS aided 3D position | */
		{
			strcpy(buff,"DGPS");
			break;
		}
		case GPS_FIX_TYPE_RTK_FLOAT: /* RTK float, 3D position | */
		{
			strcpy(buff,"RTKFT");
			break;
		}
		case GPS_FIX_TYPE_RTK_FIXED: /* RTK Fixed, 3D position | */
		{
			strcpy(buff,"RTKFX");
			break;
		}
		case GPS_FIX_TYPE_STATIC: /* Static fixed, typically used for base stations | */
		{
			strcpy(buff,"STATC");
			break;
		}
		case GPS_FIX_TYPE_PPP: /* PPP, 3D position. | */
		{
			strcpy(buff,"PPP");
			break;
		}
		default:
		{
			strcpy(buff,"UNKNW");
			break;
		}
	}
}

static void vibe_sum(DroneVibe* pvb, DroneVibePro* pvbp)
{
	if(0==pvb || 0==pvbp) return;
	
	pvbp->x_vibe_sum += pvb->vibration_x;
	pvbp->y_vibe_sum += pvb->vibration_y;
	pvbp->z_vibe_sum += pvb->vibration_z;
	if(pvb->vibration_x>pvbp->x_vibe_max) pvbp->x_vibe_max = pvb->vibration_x;
	if(pvb->vibration_y>pvbp->y_vibe_max) pvbp->y_vibe_max = pvb->vibration_y;
	if(pvb->vibration_z>pvbp->z_vibe_max) pvbp->z_vibe_max = pvb->vibration_z;
	++pvbp->vibe_count;
}

static void vibe_pro(DroneVibePro* pvbp)
{
	if(0==pvbp) return;
	
	pvbp->x_vibe_avr = 	(float)(pvbp->x_vibe_sum/pvbp->vibe_count);
	pvbp->y_vibe_avr = 	(float)(pvbp->y_vibe_sum/pvbp->vibe_count);
	pvbp->z_vibe_avr = 	(float)(pvbp->z_vibe_sum/pvbp->vibe_count);
}

static void acc_sum(DroneIMU* pi, DroneImuPro* pip)
{
	if(0==pi || 0==pip) return;
	
//	pip->xacc_sum += pi->xacc;
//	pip->yacc_sum += pi->yacc;
//	pip->zacc_sum += pi->zacc;
	if(pi->xacc > pip->xacc_max) pip->xacc_max = pi->xacc;
	if(pi->yacc > pip->yacc_max) pip->yacc_max = pi->yacc;
	if(pi->zacc > pip->zacc_max) pip->zacc_max = pi->zacc;
	
	if(pi->xacc < pip->xacc_min) pip->xacc_min = pi->xacc;
	if(pi->yacc < pip->yacc_min) pip->yacc_min = pi->yacc;
	if(pi->zacc < pip->zacc_min) pip->zacc_min = pi->zacc;
	++pip->acc_count;
}

static void acc_pro(DroneImuPro* pip)
{
	if(0==pip) return;
//	
//	pip->xacc_avr = (pip->xacc_sum/pip->acc_count);
//	pip->yacc_avr = (pip->yacc_sum/pip->acc_count);
//	pip->zacc_avr = (pip->zacc_sum/pip->acc_count);
}

static void info_pro(DroneInfo* pif)
{
	if(0==pif) return;
	
	pif->groundspeed_avr = pif->groundspeed_sum / pif->count;
	pif->throttle_avr = pif->throttle_sum / pif->count;
}

static void info_sum(DroneVfr* pv, DroneInfo *pif)
{
	if(0==pv || 0==pif) return;
	
	pif->airspeed_sum += pv->airspeed;
	pif->groundspeed_sum += pv->groundspeed;
	pif->throttle_sum += pv->throttle;
	if(pif->airspeed_max < pv->airspeed) pif->airspeed_max = pv->airspeed;
	if(pif->groundspeed_max < pv->groundspeed) pif->groundspeed_max = pv->groundspeed;
	if(pif->throttle_max < pv->throttle) pif->throttle_max = pv->throttle;
	if(pif->alt_max < pv->alt) pif->alt_max = pv->alt;
	++pif->count;
}


/* 每10hz调用一次，灯的指示效果和飞控一致 */
static void rgb_switch(void)
{
	static unsigned char step = 0;
	static unsigned char timeMs = 0;
	
	++step;
	if(10==step)
	{
		step = 0;
	}
	
	/* 初始化,正确获取到mavlink数据说明系统初始化成功 */
	if (system_flag.mavlink_exist) /*AP_Notify::flags.initialising*/
	{
		static unsigned short count = 0;
		++count;
		if(count<100)
		{
			if (step & 1) 
			{
				// odd steps display red light
				RGB1_Red();
				RGB2_Red();
			}else{
				// even display blue light
				RGB1_Blue();
				RGB2_Blue();
			}
			return;
		}else
		{
			count = 100;
		}
			
	}else
	{
		RGB1_Close();
		RGB2_Close();
		return;
	}
	
	// save trim and esc calibration pattern
	if (0) /* AP_Notify::flags.save_trim || AP_Notify::flags.esc_calibration */
	{
		switch(step) 
		{
			case 0:
			case 3:
			case 6:
				// red on
				RGB1_Red();
				RGB2_Red();
				break;

			case 1:
			case 4:
			case 7:
				// blue on
				RGB1_Blue();
				RGB2_Blue();
				break;

			case 2:
			case 5:
			case 8:
				// green on
				RGB1_Green();
				RGB2_Green();
				break;

			case 9:
				// all off
				RGB1_Close();
				RGB2_Close();
				break;
			}
			// exit so no other status modify this pattern
			return;
	}
	
		// radio and battery failsafe patter: flash yellow
    // gps failsafe pattern : flashing yellow and blue
    // ekf_bad pattern : flashing yellow and red
		//这些标志位可以通过 MAVLINK_MSG_ID_SYS_STATUS和 MAVLINK_MSG_ID_EKF_STATUS_REPORT 消息获取
  if (fail_status.failsafe_radio || fail_status.failsafe_battery || fail_status.ekf_bad || fail_status.gps_glitching || fail_status.leak_detected) 
	/*AP_Notify::flags.failsafe_radio || AP_Notify::flags.failsafe_battery || AP_Notify::flags.ekf_bad || AP_Notify::flags.gps_glitching || AP_Notify::flags.leak_detected*/
	{
		switch(step) 
		{
			case 0:
			case 1:
			case 2:
			case 3:
			case 4:
				// yellow on
				RGB1_Yellow();
				RGB2_Yellow();
				break;
			case 5:
			case 6:
			case 7:
			case 8:
			case 9:
//				if (0) { /*AP_Notify::flags.leak_detected --> ardusub 特有 */
//						// purple if leak detected
//						RGB1_Purple();
//						RGB2_Purple();
//				} else 
					if (0) { /* AP_Notify::flags.ekf_bad */
						// red on if ekf bad
						RGB1_Red();
						RGB2_Red();
				} else if (fail_status.gps_glitching) { /* AP_Notify::flags.gps_glitching */
						// blue on gps glitch
						RGB1_Blue();
						RGB2_Blue();
				}else{
						// all off for radio or battery failsafe
						RGB1_Close();
						RGB2_Close();
				}
				break;
      }
      // exit so no other status modify this pattern
      return;
    }
	
	// solid green or blue if armed
	if (system_flag.armed) { /* AP_Notify::flags.armed */
		// solid green if armed with GPS 3d lock
		if (droneGPSRawData.fix_type>=GPS_FIX_TYPE_3D_FIX) { /* AP_Notify::flags.gps_status >= AP_GPS::GPS_OK_FIX_3D */
			RGB1_Green();
			RGB2_Green();
		}else{
			// solid blue if armed with no GPS lock
			RGB1_Blue();
			RGB2_Blue();
		}
		return;
  }else{
		// double flash yellow if failing pre-arm checks
    if (fail_status.pre_arm_check) {  /* !AP_Notify::flags.pre_arm_check, 通过文本获取，解锁失败飞控发送 "PreArm:" */
			switch(step) {
				case 0:
				case 1:
				case 4:
				case 5:
					// yellow on
					RGB1_Yellow();
					RGB2_Yellow();
					break;
				case 2:
				case 3:
				case 6:
				case 7:
				case 8:
				case 9:
					// all off
					RGB1_Close();
					RGB2_Close();
					break;
    }
   }else{
			// fast flashing green if disarmed with GPS 3D lock and DGPS
			// slow flashing green if disarmed with GPS 3d lock (and no DGPS)
			// flashing blue if disarmed with no gps lock or gps pre_arm checks have failed
			char fast_green = droneGPSRawData.fix_type>=GPS_FIX_TYPE_3D_FIX && fail_status.pre_arm_gps_check;//AP_Notify::flags.gps_status >= AP_GPS::GPS_OK_FIX_3D_DGPS && AP_Notify::flags.pre_arm_gps_check;
			switch(step) {
				case 0:
						if (fast_green) {
							RGB1_Green();
							RGB2_Green();
						}
						break;
				case 1:
						if (fast_green) {
							RGB1_Close();
							RGB2_Close();
						}
						break;
				case 2:
						if (fast_green) {
							RGB1_Green();
							RGB2_Green();
						}
						break;
				case 3:
						if (fast_green) {
							RGB1_Close();
							RGB2_Close();
						}
						break;
				case 4:
						RGB1_Close();
						RGB2_Close();
						if (droneGPSRawData.fix_type>=GPS_FIX_TYPE_3D_FIX && fail_status.pre_arm_gps_check) { /* AP_Notify::flags.gps_status >= AP_GPS::GPS_OK_FIX_3D && AP_Notify::flags.pre_arm_gps_check */
								// flashing green if disarmed with GPS 3d lock
								RGB1_Green();
								RGB2_Green();
						}else{
								// flashing blue if disarmed with no gps lock
								RGB1_Blue();
								RGB2_Blue();
						}
						break;
				case 5:
						if (fast_green) {
							RGB1_Close();
							RGB2_Close();
						}
						break;

				case 6:
						if (fast_green) {
							RGB1_Green();
							RGB2_Green();
						}
						break;

				case 7:
						if (fast_green) {
							RGB1_Close();
							RGB2_Close();
						}
						break;
				case 8:
						if (fast_green) {
							RGB1_Green();
							RGB2_Green();
						}
						break;
				case 9:
						// all off
						RGB1_Close();
						RGB2_Close();
						break;
				}
			}
		}
}

/* 蜂鸣器提醒  调用频率10HZ  提示效果与飞控一致 */
static void beep_switch(void)
{
	static unsigned char count = 0;
	static unsigned char bzClass = 0;    /* 0-关闭   1-单响   2-双响   3-Arming提醒   4-EKF_BAD */
	static unsigned char pre_arm = 0;
	static unsigned char pre_ekf_bad = 0;
	
		// check for arming failed event
	if (fail_status.pre_arm_check) {   /* AP_Notify::events.arming_failed */
			// arming failed buzz
		bzClass = 1;
	}
	
	if(0!=bzClass) ++count;
		
	switch (bzClass) {
		case 1:
			// buzz for 10th of a second
			if (1==count) {
					set_beep();
			}else{
					reset_beep();
					bzClass = 0;
					count = 0;
			}
			return;
		case 2:
			// buzz for 10th of a second
			switch (count) {
				case 1:
						set_beep();
						break;
				case 2:
						reset_beep();
						break;
				case 3:
						set_beep();
						break;
				case 4:
				default:
						reset_beep();
						bzClass = 0;
						count = 0;
						break;
			}
			return;
		case 3:
			if(count*100>=3000)
			{
				reset_beep();
				bzClass = 0;
				count = 0;
			}else
			{
				set_beep();
			}
			return;
		case 4:
				// four tones getting shorter)
				switch (count) {
						case 1:
						case 5:
						case 8:
						case 10:
								set_beep();
								break;
						case 4:
						case 7:
						case 9:
								reset_beep();
								break;
						case 11:
								reset_beep();
								bzClass = 0;
								count = 0;
								break;
						default:
								// do nothing
								break;
				}
				return;
		default:
				// do nothing
				break;
		}
		
		// check if armed status has changed
	if (pre_arm!=system_flag.armed) { /* _flags.armed != AP_Notify::flags.armed */
		pre_arm = system_flag.armed;
			if (pre_arm) {  /* _flags.armed */
					// double buzz when armed
					bzClass = 3;
			}else{
					// single buzz when disarmed
					bzClass = 1;
			}
			return;
	}

	// check ekf bad
	if (fail_status.ekf_bad) { /* _flags.ekf_bad != AP_Notify::flags.ekf_bad */
			pre_ekf_bad = fail_status.ekf_bad;//flags.ekf_bad = AP_Notify::flags.ekf_bad;
			if (pre_ekf_bad) {
					// ekf bad warning buzz
					bzClass = 4;
			}
			return;
	}

	// if vehicle lost was enabled, starting beep
	if (fail_status.crash_detect) {   /* AP_Notify::flags.vehicle_lost */
			bzClass = 2;
	}

	// if battery failsafe constantly single buzz
	if (fail_status.failsafe_battery) {  /* AP_Notify::flags.failsafe_battery */
			bzClass = 1;
	}
	
}

static void mav_sys_status_pro(DroneSysStatus *pds)
{
	if(0==pds) return;
	
	if((pds->onboard_control_sensors_present&MAV_SYS_STATUS_SENSOR_RC_RECEIVER)&&!(pds->onboard_control_sensors_health&(~MAV_SYS_STATUS_SENSOR_RC_RECEIVER))) /* 遥控接收机有效，并且状态正常*/
	{
		fail_status.failsafe_radio = 1;
	}else
	{
		fail_status.failsafe_radio = 0;
	}
	
	if((pds->onboard_control_sensors_enabled&MAV_SYS_STATUS_SENSOR_BATTERY)&&!(pds->onboard_control_sensors_health&(~MAV_SYS_STATUS_SENSOR_BATTERY)))   /* 电池检测模块存在并且状态健康 */
	{
		fail_status.failsafe_battery = 1;
	}else
	{
		fail_status.failsafe_battery = 0;
	}	
}

static void mav_ekf_report_pro(DroneData *pdd)
{
	if(0==pdd)	return;
	
	if((pdd->flags&((1<<15))) && system_flag.armed)
	{
		fail_status.gps_glitching = 1;
	}else
	{
		fail_status.gps_glitching = 0;
	}
	
	if(!system_flag.armed)
	{
		if(((pdd->flags&EKF_POS_HORIZ_ABS) || (pdd->flags&EKF_PRED_POS_HORIZ_ABS)) && !fail_status.gps_glitching && pdd->compass_variance<0.8 && droneGPSRawData.eph<120)
		{
			fail_status.pre_arm_gps_check = 0;
		}else fail_status.pre_arm_gps_check = 1;
	}else
	{
		if((pdd->flags&EKF_POS_HORIZ_ABS) && (!(pdd->flags&EKF_CONST_POS_MODE)) && !fail_status.gps_glitching && pdd->compass_variance<0.8  && droneGPSRawData.eph<120)
		{
			fail_status.pre_arm_gps_check = 0;
		}else fail_status.pre_arm_gps_check = 1;
	}
}

static void mav_status_text_pro(mavlink_statustext_t *statusText)
{
	char buff[50];
	unsigned char i = 0;
	if(0==statusText) return;
	
	if(MAV_SEVERITY_CRITICAL==statusText->severity)
	{
		for(i=0; i<8; ++i)
		{
			buff[i] = statusText->text[i];
		}
		buff[i] = '\0';
		if(!strcmp(buff, "PreArm:"))	fail_status.pre_arm_check = 1;
		
		for(i=0; i<12; ++i)
		{
			buff[i] = statusText->text[i];
		}
		buff[i] = '\0';
		if(!strcmp(buff, "EKF variance"))	fail_status.ekf_bad = 1;
  }	

	if(MAV_SEVERITY_EMERGENCY==statusText->severity)
	{
		for(i=0; i<6; ++i)
		{
			buff[i] = statusText->text[i];
		}
		buff[i] = '\0';
		
		if(!strcmp(buff,"Crash:"))
		{
			fail_status.crash_detect = 1;
		}
	}
	
}

static void commadn_ack_pro(uint16_t command, uint8_t result)
{
	switch(command)
	{
		case MAV_CMD_DO_START_MAG_CAL:   /* 发送开始校准磁罗盘指令 */
		{
			if(MAV_RESULT_ACCEPTED==result)
			{
				system_flag.drone_send_compass_cal = COMPASS_CAL_PROING;  /* 进入校准进度条页面 */
				system_flag.oled_page = 2;
			}else
			{
				system_flag.drone_send_compass_cal = COMPASS_CAL_OLED_PAGE;
				system_flag.oled_page = 8;
			}
			break;
		}
		
		default: break;
	}
}

