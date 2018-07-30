
#ifndef USER_TASK_H
#define USER_TASK_H
	#include "HardwareConfig.h"
	#include "stm32f1xx_hal.h"
	#include "cmsis_os.h"
	#include "main.h"
	#include "mavlink.h"
	
	#define RGB_TEST_MASK_BIT_VALID 		(1<<1)
	#define RGB_TEST_MASK_BIT_INVALID		(~(1<<1))
	#define OLED_TEST_MASK_BIT_VALID  	(1<<2)
	#define OLED_TEST_MASK_BIT_INVALID	(~(1<<2))
	#define BEEP_TEST_MASK_BIT_VALID  	(1<<3)
	#define BEEP_TEST_MASK_BIT_INVALID  (~(1<<3))
	
	#define OLED_PAGE1_UPDATE_VALID  		(1<<1)
	#define OLED_PAGE1_UPDATE_INVALID		(~(1<<1))	
	
	typedef struct _drone_current
	{
		uint32_t count;
		int32_t current_sum;
		int16_t current_max;
		int16_t current_avr;
	}DroneCurrent;
	
	typedef struct _drone_info
	{
		double airspeed_sum;
		double groundspeed_sum;
		float airspeed_max;
		float groundspeed_max;
		float groundspeed_avr;
		float alt_max; /*< Current altitude (MSL), in meters*/
		uint32_t count;
		uint32_t throttle_sum; /*< Current throttle setting in integer percent, 0 to 100*/
		uint16_t throttle_max;
		uint16_t throttle_avr;	
	}DroneInfo;
	
	typedef struct _drone_imu_process
	{
//		int64_t xacc_sum;
//		int64_t yacc_sum;
//		int64_t zacc_sum;
		uint32_t acc_count;
		int16_t xacc_max;
		int16_t yacc_max;
		int16_t zacc_max;
		int16_t xacc_min;
		int16_t yacc_min;
		int16_t zacc_min;
//		int16_t xacc_avr;
//		int16_t yacc_avr;
//		int16_t zacc_avr;
	}DroneImuPro;
	
	typedef struct _drone_vibe_process
	{	
		double  x_vibe_sum;
		double  y_vibe_sum;
		double  z_vibe_sum;
		uint32_t  vibe_count;
		float  x_vibe_max;
		float  x_vibe_avr;
		float  y_vibe_max;
		float  y_vibe_avr;
		float  z_vibe_max;
		float  z_vibe_avr;
	}DroneVibePro;
	
	typedef struct _drone_heatbeat
	{
		 uint32_t custom_mode;
		 uint8_t base_mode; 
		 uint8_t type;
		 uint8_t autopilot;
		 uint8_t system_status;
		 uint8_t mavlink_version;
	}DroneHeatbeat;
	
	typedef struct _drone_system_status
	{
		uint32_t onboard_control_sensors_present;
		uint32_t onboard_control_sensors_enabled;
		uint32_t onboard_control_sensors_health;
		uint16_t load;
		uint16_t voltage_battery;
		int16_t current_battery;
		int8_t battery_remaining; 
	}DroneSysStatus;
	
	typedef struct _drone_vfr_hud
	{
		float airspeed;
		float groundspeed;
		float alt;
		float climb;
		int16_t heading;
		uint16_t throttle;
	}DroneVfr;
	
	typedef struct _drone_vibe
	{
		float vibration_x;
		float vibration_y;
		float vibration_z;
	}DroneVibe;
	
	typedef struct _drone_gps_raw
	{
		int32_t lat;
		int32_t lon;
		int32_t alt;
		uint16_t eph;
		uint16_t epv;
		uint8_t fix_type;
		uint8_t satellites_visible;
	}DroneGpsRaw;
	
	typedef struct _drone_gps2_raw
	{
		int32_t lat;
		int32_t lon;
		int32_t alt;
		uint32_t dgps_age;
		uint16_t eph;
		uint16_t epv;
		//uint16_t vel; /*< GPS ground speed (m/s * 100). If unknown, set to: UINT16_MAX*/
		//uint16_t cog; /*< Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX*/
		uint8_t fix_type;
		uint8_t satellites_visible;
		uint8_t dgps_numch;
	}DroneGps2Raw;
	
	typedef struct _drone_att
	{
		float roll;
		float pitch;
		float yaw;
	}DroneAtt;
	
	typedef struct _drone_wind
	{
		float direction;
		float speed;
	}DroneWind;
	
	typedef struct _drone_imu
	{
		int16_t xacc;
		int16_t yacc;
		int16_t zacc;
	}DroneIMU;
	
	typedef struct _drone_mag_cal
	{
		uint8_t compass_id;
		uint8_t cal_mask;
		uint8_t cal_status;
		uint8_t attempt;
		uint8_t completion_pct;
	}DroneMagCalProgess;
	
	typedef struct _drone_data
	{
//		DroneHeatbeat* const pDroneHeatbeat;
//		DroneSysStatus* const pDroneSysStatus;
//		DroneVfr* const pDroneVfr;
//		DroneVibe* const pDroneVibe;
//		DroneGpsRaw* const pDroneGpsRaw;
//		DroneAtt* const pDroneAtt;
//		DroneWind* const pDroneWind;
//		DroneIMU* const pDroneIMU;
//		DroneMagCalProgess* const pDroneMagCalProgess;
		
		mavlink_rangefinder_t* pRangefinder;
		mavlink_mag_cal_report_t* pMagCalReport;
		mavlink_camera_feedback_t* pCameraFeedback;
		mavlink_statustext_t* pStatusText;
		
		uint32_t drone_time_boot_ms;
		float compass_variance;
		uint16_t flags;		
		int16_t temperature;
		
	}DroneData;
	
	typedef enum control_mode_t {
    STABILIZE =     0,  // manual airframe angle with manual throttle
    ACRO =          1,  // manual body-frame angular rate with manual throttle
    ALT_HOLD =      2,  // manual airframe angle with automatic throttle
    AUTO =          3,  // fully automatic waypoint control using mission commands
    GUIDED =        4,  // fully automatic fly to coordinate or fly at velocity/direction using GCS immediate commands
    LOITER =        5,  // automatic horizontal acceleration with automatic throttle
    RTL =           6,  // automatic return to launching point
    CIRCLE =        7,  // automatic circular flight with automatic throttle
    LAND =          9,  // automatic landing with horizontal position control
    DRIFT =        11,  // semi-automous position, yaw and throttle control
    SPORT =        13,  // manual earth-frame angular rate control with manual throttle
    FLIP =         14,  // automatically flip the vehicle on the roll axis
    AUTOTUNE =     15,  // automatically tune the vehicle's roll and pitch gains
    POSHOLD =      16,  // automatic position hold with manual override, with automatic throttle
    BRAKE =        17,  // full-brake using inertial/GPS system, no pilot input
    THROW =        18,  // throw to launch mode using inertial/GPS system, no pilot input
    AVOID_ADSB =   19,  // automatic avoidance of obstacles in the macro scale - e.g. full-sized aircraft
    GUIDED_NOGPS = 20,  // guided mode but only accepts attitude and altitude
    SMART_RTL =    21,  // SMART_RTL returns to home by retracing its steps
    FLOWHOLD  =    22,  // FLOWHOLD holds position with optical flow without rangefinder
    FOLLOW    =    23,  // follow attempts to follow another vehicle or ground station
}control_mode_t;
	
	typedef struct sys_flag
	{
		unsigned int oled_page_bit_mask;
		unsigned char fly_count;       /* 飞行次数 */
		unsigned char pre_fly_count;   /* 上次飞行次数*/
		unsigned char init_loader: 1; /* 启动画面加载进度，如果没有正确连接，则一直处于加载界面 1-loader 0-jump */
		unsigned char armed: 1;          /* 1-armed 0-disarm */
		unsigned char beep_enable:1; 		/* 蜂鸣器使能   0-关闭    1-开启 */
		unsigned char rgb1_enable:1;
		unsigned char rgb2_enable:1;
		unsigned char	oled_enable:1;
		unsigned char mavlink_exist:1;  /* mavlink数据，常亮蓝灯 */
		unsigned char mavlink_valid:1;  /* mavlink中的状态数据有效 */
		unsigned char drone_complete:1; /* 1-飞行完成 */
		unsigned char test_mask_bit;        /*  */
		unsigned char oled_page; 			/* OLED显示页 0-加载页面   1-基本信息页面 */
		
	}SystemFlag;
	
	/* 无人机系统报错标志，1为有效 */
	typedef struct fail_status
	{
		unsigned char failsafe_radio:1;
		unsigned char failsafe_battery:1;
		unsigned char ekf_bad:1;
		unsigned char gps_glitching:1;
		unsigned char leak_detected:1;
		unsigned char pre_arm_check:1;
		unsigned char pre_arm_gps_check:1;
		unsigned char crash_detect:1;
	}FailStatus;
	
	void app_run(void);
	void run_led_task(void const * argument);
	void status_led_task(void const * argument);	
	void rgb_led_task(void const * argument);
	void beep_task(void const * argument);
	void feed_dog_task(void const * argument);
	
	void button_event_task(void const* argument);
	
	void usart1_send_task(void const* argument);
	void usart1_rec_task(void const* argument);
	void usart3_send_task(void const* argument);
	void usart3_rec_task(void const* argument);
	
	void update_oled_task(void const* argument);
	
	void adc1_process_task(void const* argument);
	
	void init_hardware(void);
	void initUsartIT(UART_HandleTypeDef *huart);
	void usartRecieveITCallBack(unsigned char id, UART_HandleTypeDef *huart);
	
	void usart3_send_test_task(void const* argument);
	
	static void system_init(void);
	static unsigned char mavlinkV1_parse(uint8_t chan, uint8_t c);
	static void request_send_data(uint8_t chan);
	static void custom_mode_to_string(enum control_mode_t mode, char* buff);
	static void gps_type_to_string(GPS_FIX_TYPE fix_type, char* buff);
	static void  init_drone_data(void);
	static void vibe_sum(DroneVibe* pvb, DroneVibePro* pvbp);
	static void vibe_pro(DroneVibePro* pvbp);
	static void acc_sum(DroneIMU* pi, DroneImuPro* pip);
	static void acc_pro(DroneImuPro* pip);
	static void info_sum(DroneVfr* pv, DroneInfo *pif);
	static void info_pro(DroneInfo* pif);
	static void rgb_switch(void);
	static void beep_switch(void);
	static void mav_ekf_report_pro(DroneData *pdd);
	static void mav_status_text_pro(mavlink_statustext_t *statusText);
	static void mav_sys_status_pro(DroneSysStatus *pds);
	
	extern void usart_send(unsigned char id, char* pc, unsigned int len);
	extern IWDG_HandleTypeDef hiwdg;
	extern UART_HandleTypeDef huart3;
	extern ADC_HandleTypeDef hadc1;
	extern xQueueHandle button_event_queue;
	extern mavlink_system_t gcs_mavlink_system;

#endif
