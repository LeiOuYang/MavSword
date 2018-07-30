
#ifndef USER_PIXHAWK_TYPE
#define USER_PIXHAWK_TYPE

	typedef  enum  _pixhawk_custom_mode{
		
		STABLILIZE_MODE = 0,
		ACRO_MODE				= 1,
		ALTHOLD_MODE		= 2,
		AUTO_MODE				= 3,
		GUIDED_MODE			= 4,
		LOITER_MODE			= 5,
		RTL_MODE				= 6,
		CIRCLE_MODE     = 7,
		LAND_MODE				= 9,
		DRIFT_MODE			= 11,
		SPORT_MODE 			= 13,
		POSHOLD_MODE		= 16	
	} pixhawk_custom_mode;
	
	typedef  enum  _pixhawk_base_mode{
		
	 CUSTOM_MODE_ENABLED = 1,
   TEST_ENABLED = 2,
   AUTO_ENABLED = 4,
   GUIDED_ENABLED = 8,
   STABILIZE_ENABLED = 16,
   HIL_ENABLED = 32,
   MANUAL_INPUT_ENABLED = 64,
   SAFETY_ARMED = 128,
   ENUM_END = 129,
	} pixhawk_base_mode;
	
#endif
