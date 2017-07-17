
#include "ap_peripheral_handling.h"
#include "ap_state_config.h"
#include "ap_state_handling.h"

//#include "drv_l1_system.h"
#include "driver_l1.h"
#include "drv_l1_cdsp.h"


#define LED_STATUS_FLASH		1
#define LED_STATUS_BLINK		2

#define CRAZY_KEY_TEST			0		// Send key events faster than human finger can do
#define LED_ON					1
#define LED_OFF 				0

static INT8U	led_status; //0: nothing  1: flash	2: blink
static INT8U	led_cnt;

static INT32U	led_mode;
static INT8U	g_led_count;
static INT8U	g_led_r_state; //0 = OFF;	1=ON;	2=Flicker
static INT8U	g_led_g_state;
static INT8U	g_led_flicker_state; //0=同时闪烁	1=交替闪烁
static INT8U	led_red_flag;
static INT8U	led_green_flag;
static INT8U	video_save_led_flicker = 0;

#if TV_DET_ENABLE
INT8U			tv_plug_in_flag;
INT8U			tv_debounce_cnt = 0;

#endif

static INT8U	tv = !TV_DET_ACTIVE;
static INT8U	backlight_tmr = 0;

#if C_SCREEN_SAVER				== CUSTOM_ON
INT8U			auto_off_force_disable = 0;
void ap_peripheral_auto_off_force_disable_set(INT8U);

#endif

static INT8U	led_flash_timerid;
static INT16U	config_cnt;

//----------------------------
typedef struct 
{
INT8U			byRealVal;
INT8U			byCalVal;
} AD_MAP_t;


//----------------------------
extern void avi_adc_gsensor_data_register(void * *msgq_id, INT32U * msg_id);
INT8U			gsensor_data[2][32] =
{
	0
};


static void *	gsensor_msgQId0 = 0;
static INT32U	gsensor_msgId0 = 0;


static INT8U	ad_line_select = 0;

static INT16U	adc_battery_value_new, adc_battery_value_old;
static INT32U	battery_stable_cnt = 0;

#define C_BATTERY_STABLE_THRESHOLD 4  // Defines threshold number that AD value is deemed stable

#if C_BATTERY_DETECT			== CUSTOM_ON
static INT16U	low_voltage_cnt;
static INT32U	battery_value_sum = 0;
static INT8U	bat_ck_cnt = 0;

#endif



#if USE_ADKEY_NO
static INT8U	ad_detect_timerid;
static INT16U	ad_value;
static KEYSTATUS ad_key_map[USE_ADKEY_NO + 1];

//static INT16U ad_key_cnt = 0;
static INT16U	adc_key_release_value_old, adc_key_release_value_new, adc_key_release_value_stable;
static INT32U	key_release_stable_cnt = 0;

#define C_RESISTOR_ACCURACY 	5//josephhsieh@140418 3			// 2% accuracy
#define C_KEY_PRESS_WATERSHED	600//josephhsieh@140418 175
#define C_KEY_STABLE_THRESHOLD	4//josephhsieh@140418 3			// Defines threshold number that AD value of key is deemed stable
#define C_KEY_FAST_JUDGE_THRESHOLD 40			// Defines threshold number that key is should be judge before it is release. 0=Disable
#define C_KEY_RELEASE_STABLE_THRESHOLD 4  // Defines threshold number that AD value is deemed stable

INT16U			adc_key_value;

//static INT8U	ad_value_cnt ;
INT32U			key_pressed_cnt;
INT8U			fast_key_exec_flag;
INT8U			normal_key_exec_flag;
INT8U			long_key_exec_flag;

#endif

static INT32U	key_active_cnt;
static INT8U	lcd_bl_sts;
static INT8U	power_off_timerid;
static INT8U	usbd_detect_io_timerid;
static KEYSTATUS key_map[USE_IOKEY_NO];
static INT8U	key_detect_timerid;
static INT16U	adp_out_cnt;
static INT16U	usbd_cnt;
static INT8U	up_firmware_flag = 0;
static INT8U	flash_flag = 0;

#if USB_PHY_SUSPEND 			== 1
static INT16U	phy_cnt = 0;

#endif

static INT16U	adp_cnt;
INT8U			adp_status;
static INT8U	battery_low_flag = 0;
INT8U			usbd_exit;
INT8U			s_usbd_pin;

//extern INT8U MODE_KEY_flag;
//	prototypes
void ap_peripheral_key_init(void);
void ap_peripheral_rec_key_exe(INT16U * tick_cnt_ptr);
void ap_peripheral_function_key_exe(INT16U * tick_cnt_ptr);
void ap_peripheral_next_key_exe(INT16U * tick_cnt_ptr);
void ap_peripheral_prev_key_exe(INT16U * tick_cnt_ptr);
void ap_peripheral_ok_key_exe(INT16U * tick_cnt_ptr);
void ap_peripheral_sos_key_exe(INT16U * tick_cnt_ptr);
void ap_peripheral_usbd_plug_out_exe(INT16U * tick_cnt_ptr);
void ap_peripheral_pw_key_exe(INT16U * tick_cnt_ptr);
void ap_peripheral_menu_key_exe(INT16U * tick_cnt_ptr);

#if KEY_FUNTION_TYPE			== SAMPLE2
void ap_peripheral_capture_key_exe(INT16U * tick_cnt_ptr);

#endif

void ap_peripheral_null_key_exe(INT16U * tick_cnt_ptr);

#if USE_ADKEY_NO
void ap_peripheral_ad_detect_init(INT8U adc_channel, void(*bat_detect_isr) (INT16U data));
void ap_peripheral_ad_check_isr(INT16U value);

#endif


INT8U ap_video_save_state_get(void)
{
	return video_save_led_flicker;
}


void ap_video_save_state_set(INT8U state)
{
	video_save_led_flicker = state;
}


//==================================================================
//没有使用的IO设置为输入下拉, 并将其接到GND上, 有利于散热
const INT16U	COOLER_PIN_ARRAY[] =
{
	COOLER_PIN_IO_01, 
	COOLER_PIN_IO_02, 
	COOLER_PIN_IO_03, 
	COOLER_PIN_IO_04, 
	COOLER_PIN_IO_05, 
	COOLER_PIN_IO_06, 
	COOLER_PIN_IO_07, 
	COOLER_PIN_IO_08, 
	COOLER_PIN_IO_09, 
	COOLER_PIN_IO_10, 
};


void ap_peripheral_handling_cooler_pin_init(void)
{
	INT8U			i;

	for (i = 0; i < COOLER_PIN_NUM; i++)
	{
		gpio_init_io(COOLER_PIN_ARRAY[i], GPIO_INPUT);
		gpio_set_port_attribute(COOLER_PIN_ARRAY[i], ATTRIBUTE_LOW);
		gpio_write_io(COOLER_PIN_ARRAY[i], DATA_LOW);
	}
}


//==================================================================
void ap_peripheral_init(void)
{
#if TV_DET_ENABLE
	INT32U			i;

#endif

	power_off_timerid	= usbd_detect_io_timerid = led_flash_timerid = 0xFF;
	key_detect_timerid	= 0xFF;

	//LED IO init
	//gpio_init_io(LED, GPIO_OUTPUT);
	//gpio_set_port_attribute(LED, ATTRIBUTE_HIGH);
	//gpio_write_io(LED, DATA_LOW);
	//led_status = 0;
	//led_cnt = 0;
	LED_pin_init();

	gpio_init_io(IR_CTRL, GPIO_OUTPUT);
	gpio_set_port_attribute(IR_CTRL, ATTRIBUTE_HIGH);
	gpio_write_io(IR_CTRL, 0);

	gpio_init_io(AV_IN_DET, GPIO_INPUT);
	gpio_set_port_attribute(AV_IN_DET, ATTRIBUTE_LOW);
	gpio_write_io(AV_IN_DET, !TV_DET_ACTIVE);		//pull high or low

#if TV_DET_ENABLE
	tv_plug_in_flag 	= 0;

	for (i = 0; i < 5; i++)
	{
		if (gpio_read_io(AV_IN_DET) == !TV_DET_ACTIVE)
		{
			break;
		}

		OSTimeDly(1);
	}

	if (i == 5)
	{
		tv					= TV_DET_ACTIVE;
		tv_plug_in_flag 	= 1;
	}

#endif

	gpio_init_io(HDMI_IN_DET, GPIO_INPUT);
	gpio_set_port_attribute(HDMI_IN_DET, ATTRIBUTE_LOW);
	gpio_write_io(HDMI_IN_DET, 0);					//pull low

	gpio_init_io(SPEAKER_EN, GPIO_OUTPUT);
	gpio_set_port_attribute(SPEAKER_EN, ATTRIBUTE_HIGH);

#if TV_DET_ENABLE

	if (tv_plug_in_flag)
	{
		gpio_write_io(SPEAKER_EN, 0);				//mute local speaker
	}
	else 
#endif

	{
		gpio_write_io(SPEAKER_EN, 1);				//enable local speaker
	}

	ap_peripheral_key_init();

#if USE_ADKEY_NO
	ad_detect_timerid	= 0xFF;
	ap_peripheral_ad_detect_init(AD_KEY_DETECT_PIN, ap_peripheral_ad_check_isr);

#else

	adc_init();
#endif

	config_cnt			= 0;

	//MODE_KEY_flag = 2;
}


#ifdef PWM_CTR_LED


void ap_peripheral_PWM_OFF(void)
{
	INT8U			byPole = 0;
	INT16U			wPeriod = 0;
	INT16U			wPreload = 0;
	INT8U			byEnable = 0;

	ext_rtc_pwm0_enable(byPole, wPeriod, wPreload, byEnable);

	//	ext_rtc_pwm1_enable(byPole, wPeriod, wPreload, byEnable); 
	DBG_PRINT("PWM0/1 off!\r\n");

}


void ap_peripheral_PWM_LED_high(void)
{
	INT8U			byPole = 1;
	INT16U			wPeriod = 0x6000;
	INT16U			wPreload = 0x5fff;
	INT8U			byEnable = 0;

	ext_rtc_pwm0_enable(byPole, wPeriod, wPreload, byEnable);

	//	ext_rtc_pwm1_enable(byPole, wPeriod, wPreload, byEnable); 
	DBG_PRINT("PWM0/1 OUT PUT HIGH 750ms, low 68us\r\n");

}


void ap_peripheral_PWM_LED_low(void)
{
	INT8U			byPole = 1;
	INT16U			wPeriod = 0x6000;
	INT16U			wPreload = 0x1;
	INT8U			byEnable = TRUE;

	ext_rtc_pwm0_enable(byPole, wPeriod, wPreload, byEnable);

	//	  ext_rtc_pwm1_enable(byPole, wPeriod, wPreload, byEnable) ; 
	//	  DBG_PRINT("PWM0/1 OUT PUT LOW 750ms, high 68us\r\n"); 
}


#endif

void ap_peripheral_led_set(INT8U type)
{
#ifdef PWM_CTR_LED
	INT8U			byPole;
	INT16U			wPeriod = 0;
	INT16U			wPreload = 0;
	INT8U			byEnable;

	if (type)
	{ //high
		ap_peripheral_PWM_LED_high();
	}
	else 
	{ //low
		ap_peripheral_PWM_LED_low();
	}

#else

	gpio_write_io(LED, type);
	led_status			= 0;
	led_cnt 			= 0;
#endif
}


void ap_peripheral_led_flash_set(void)
{
#ifdef PWM_CTR_LED
	ap_peripheral_PWM_LED_high();
	led_status			= LED_STATUS_FLASH;
	led_cnt 			= 0;

#else

	gpio_write_io(LED, DATA_HIGH);
	led_status			= LED_STATUS_FLASH;
	led_cnt 			= 0;
#endif
}


void ap_peripheral_led_blink_set(void)
{

#ifdef PWM_CTR_LED
	INT8U			byPole = 1;
	INT16U			wPeriod = 0x6000;
	INT16U			wPreload = 0x2fff;
	INT8U			byEnable = TRUE;

	ext_rtc_pwm0_enable(byPole, wPeriod, wPreload, byEnable);

	//	  ext_rtc_pwm1_enable(byPole, wPeriod, wPreload, byEnable) ; 
	DBG_PRINT("PWM0/1 blink on	750ms,380ms \r\n");

#else

	gpio_write_io(LED, DATA_HIGH);
	led_status			= LED_STATUS_BLINK;
	led_cnt 			= 0;
#endif
}


void LED_pin_init(void)
{
	INT32U			type;

	//led init as ouput pull-low
	gpio_init_io(LED1, GPIO_OUTPUT);
	gpio_set_port_attribute(LED1, ATTRIBUTE_HIGH);
	gpio_write_io(LED1, LED1_ACTIVE ^ 1);

	gpio_init_io(LED2, GPIO_OUTPUT);
	gpio_set_port_attribute(LED2, ATTRIBUTE_HIGH);
	gpio_write_io(LED2, LED2_ACTIVE ^ 1);
	led_red_flag		= LED_OFF;
	led_green_flag		= LED_OFF;
	type				= LED_INIT;
	msgQSend(PeripheralTaskQ, MSG_PERIPHERAL_TASK_LED_SET, &type, sizeof(INT32U), MSG_PRI_NORMAL);
	sys_registe_timer_isr(LED_blanking_isr);		//timer base c to start adc convert
}


extern INT8U	card_space_less_flag;
extern volatile INT8U pic_down_flag;


void set_led_mode(LED_MODE_ENUM mode)
{
	INT8U			i;
	static INT8U	prev_mode = 0xaa;

	led_mode			= mode;
	g_led_g_state		= 0;						//3oE??÷oiAAA
	g_led_r_state		= 0;
	g_led_flicker_state = 0;

	//目前做法是容量小于设置的最低容量后，灯不在给出任何响应
	//if(card_space_less_flag)
	//return;
	switch ((INT32U)
	mode)
	{
		case LED_INIT:
			led_red_on();
			led_green_off();
			DBG_PRINT("led_type = LED_INIT\r\n");
			break;

		case LED_UPDATE_PROGRAM:
			led_red_off();
			g_led_g_state = 1;
			DBG_PRINT("led_type = LED_UPDATE_PROGRAM\r\n");
			break;

		case LED_UPDATE_FINISH:
			led_red_off();
			led_green_on();
			DBG_PRINT("led_type = LED_UPDATE_FINISH\r\n");
			break;

		case LED_UPDATE_FAIL:
			sys_release_timer_isr(LED_blanking_isr);

			for (i = 0; i < 2; i++)
			{
				led_all_off();
				OSTimeDly(15);
				led_green_on();
				OSTimeDly(15);
				led_all_off();
			}

			DBG_PRINT("led_type = LED_UPDATE_FAIL\r\n");
			sys_registe_timer_isr(LED_blanking_isr);
			break;

		case LED_USB_CONNECT:
			break;

		case LED_RECORD:
			led_green_off();
			g_led_r_state = 2;
			g_led_flicker_state = 0;
			DBG_PRINT("led_type = LED_RECORD\r\n");
			break;

		case LED_SDC_FULL:
			sys_release_timer_isr(LED_blanking_isr);

			//led_green_off();
			//OSTimeDly(15);
			led_all_off();
			DBG_PRINT("led_type = LED_SDC_FULL\r\n");
			sys_registe_timer_isr(LED_blanking_isr);
		case LED_WAITING_RECORD:
			led_red_off();
			led_green_on();
			DBG_PRINT("led_type = LED_WAITING_RECORD\r\n");
			break;

		case LED_AUDIO_RECORD:
			g_led_r_state = 2;
			g_led_flicker_state = 0;
			led_green_off();
			DBG_PRINT("led_type = LED_AUDIO_RECORD\r\n");
			break;

		case LED_WAITING_AUDIO_RECORD:
			led_red_off();
			led_green_on();
			DBG_PRINT("led_type = LED_WAITING_AUDIO_RECORD\r\n");
			break;

		case LED_CAPTURE:
			led_green_off();
			led_red_on();
			OSTimeDly(25);
			led_red_off();
			DBG_PRINT("led_type = LED_CAPTURE\r\n");
			break;

		case LED_CARD_DETE_SUC:
			if (storage_sd_upgrade_file_flag_get() == 2)
				break;

			sys_release_timer_isr(LED_blanking_isr);

			for (i = 0; i < 3; i++)
			{
				led_all_off();
				led_green_on();
				OSTimeDly(50);
				led_all_off();
				led_red_on();
				OSTimeDly(50);
			}

			sys_registe_timer_isr(LED_blanking_isr);
			DBG_PRINT("led_type = LED_CARD_DETE_SUC\r\n");
			break;

		case LED_CAPTURE_FAIL:
			for (i = 0; i < 2; i++)
			{
				led_all_off();
				OSTimeDly(50);
				led_red_on();
				OSTimeDly(50);
			}

		case LED_WAITING_CAPTURE:
			led_red_off();
			led_green_on();
			DBG_PRINT("led_type = LED_WAITING_CAPTURE\r\n");
			break;

		case LED_MOTION_DETECTION:
			break;

		case LED_NO_SDC:
			if ((prev_mode != mode))
			{
				if ((led_green_flag != LED_ON) && (!card_space_less_flag))
				{
					sys_release_timer_isr(LED_blanking_isr);
					led_all_off();

					//led_green_on();
					//OSTimeDly(10);
					sys_registe_timer_isr(LED_blanking_isr);
				}

				led_all_off();
			}

			DBG_PRINT("led_type = LED_NO_SDC\r\n");
			break;

		case LED_TELL_CARD:
			sys_release_timer_isr(LED_blanking_isr);
			led_red_on();
			OSTimeDly(15);
			led_red_off();
			DBG_PRINT("led_type = LED_TELL_CARD\r\n");
			sys_registe_timer_isr(LED_blanking_isr);
			break;

		case LED_CARD_NO_SPACE:
			//led_all_off();
			if (storage_sd_upgrade_file_flag_get() == 2)
				break;

			g_led_g_state = 3; //3IDo?oE!O?a!MuDAIe!OIFFFFFFG?A?!MoFFFFFGgAI?E!OEAE!M
			g_led_r_state = 3;
			DBG_PRINT("led_type = LED_CARD_NO_SPACE\r\n");
			break;
	}

	prev_mode			= mode;
}


void led_red_on(void)
{
	if (led_red_flag != LED_ON)
	{
		gpio_write_io(LED2, LED2_ACTIVE ^ 0);
		led_red_flag		= LED_ON;
	}
}


void led_green_on(void)
{
	if (led_green_flag != LED_ON)
	{
		gpio_write_io(LED1, LED1_ACTIVE ^ 0);
		led_green_flag		= LED_ON;
	}
}


void led_all_off(void)
{
	if (led_green_flag != LED_OFF)
	{
		gpio_write_io(LED1, LED1_ACTIVE ^ 1);
		led_green_flag		= LED_OFF;
	}

	if (led_red_flag != LED_OFF)
	{
		gpio_write_io(LED2, LED2_ACTIVE ^ 1);
		led_red_flag		= LED_OFF;
	}
}


void led_green_off(void)
{
	if (led_green_flag != LED_OFF)
	{
		gpio_write_io(LED1, LED1_ACTIVE ^ 1);
		led_green_flag		= LED_OFF;
	}
}


void led_red_off(void)
{
	if (led_red_flag != LED_OFF)
	{
		gpio_write_io(LED2, LED2_ACTIVE ^ 1);
		led_red_flag		= LED_OFF;
	}
}


extern INT8U	video_stop_flag;


void LED_blanking_isr(void)
{
	//INT8U type=NULL;
	static INT8U	led_state_flag = 0;
	static INT32U	usb_led_cnt = 0;

	//if(card_space_less_flag)
	//	return;
	if (++g_led_count >= 128)
	{
		g_led_count 		= 0;
	}

	usb_led_cnt++;

	if (video_stop_flag)
		return;

	if (g_led_g_state == 1)
	{
		if (g_led_count % 10 == 0)
		{
			if (up_firmware_flag == 1)
			{
				led_green_off();
				up_firmware_flag	= 0;
			}
			else 
			{
				led_green_on();
				up_firmware_flag	= 1;
			}
		}
	}
	else if (g_led_g_state == 2)
	{
		if (g_led_count / 64 == g_led_flicker_state)
			led_green_on();
		else 
			led_green_off();
	}
	else if (g_led_g_state == 3)
	{

		if (g_led_count % 32 == 0)
		{
			if (flash_flag == 0)
			{
				led_green_on();
				flash_flag			= 1;
			}
			else 
			{
				led_green_off();
				flash_flag			= 0;
			}

		}
	}
	else if (g_led_g_state == 4)
	{ //USB模式

		if (usb_led_cnt >= 256)
		{
			usb_led_cnt 		= 0;
			led_state_flag		^= 1;
		}

		if (led_state_flag)
		{
			led_green_on();
		}
		else 
		{
			led_green_off();
		}
	}

	if (g_led_r_state == 2)
	{
		if (g_led_count < 64)
			led_red_on();
		else 
			led_red_off();
	}
	else if (g_led_r_state == 3)
	{
		if (g_led_count % 32 == 0)
		{
			if (g_led_flicker_state == 1)
			{
				if (flash_flag == 1)
					led_red_off();
				else 
					led_red_on();
			}
			else 
			{
				if (flash_flag == 0)
					led_red_off();
				else 
					led_red_on();
			}
		}
	}
	else if (g_led_r_state == 4)
	{ //视频保存闪灯

		if (g_led_count % 8 == 0)
		{
			if (flash_flag == 0)
			{
				flash_flag			= 1;
				led_red_on();
			}
			else 
			{
				flash_flag			= 0;
				led_red_off();
			}
		}
	}

	//	ap_peripheral_key_judge();
	// msgQSend(PeripheralTaskQ, MSG_PERIPHERAL_TASK_SINGLE_JUGE, &type, sizeof(INT8U), MSG_PRI_NORMAL);
}




#if C_MOTION_DETECTION			== CUSTOM_ON


void ap_peripheral_motion_detect_judge(void)
{
	INT32U			result;

	result				= hwCdsp_MD_get_result();

	//DBG_PRINT("MD_result = 0x%x\r\n",result);
	if (result > 0x40)
	{
		msgQSend(ApQ, MSG_APQ_MOTION_DETECT_ACTIVE, NULL, NULL, MSG_PRI_NORMAL);
	}
}


void ap_peripheral_motion_detect_start(void)
{
	motion_detect_status_set(MOTION_DETECT_STATUS_START);
}


void ap_peripheral_motion_detect_stop(void)
{
	motion_detect_status_set(MOTION_DETECT_STATUS_STOP);
}


#endif

#if USE_ADKEY_NO


void ap_peripheral_ad_detect_init(INT8U adc_channel, void(*ad_detect_isr) (INT16U data))
{
#if C_BATTERY_DETECT				== CUSTOM_ON
	battery_value_sum	= 0;
	bat_ck_cnt			= 0;
#endif

	//	ad_value_cnt = 0;
	adc_init();
	adc_vref_enable_set(TRUE);
	adc_conv_time_sel(1);
	adc_manual_ch_set(adc_channel);
	adc_manual_callback_set(ad_detect_isr);

	if (ad_detect_timerid == 0xFF)
	{
		ad_detect_timerid	= AD_DETECT_TIMER_ID;
		sys_set_timer((void *) msgQSend, (void *) PeripheralTaskQ, MSG_PERIPHERAL_TASK_AD_DETECT_CHECK,
			 ad_detect_timerid, PERI_TIME_INTERVAL_AD_DETECT);
	}
}


void ap_peripheral_ad_check_isr(INT16U value)
{
	ad_value			= value;
}


INT16U adc_key_release_calibration(INT16U value)
{
	return value;
}


void ap_peripheral_clr_screen_saver_timer(void)
{
	key_active_cnt		= 0;
}


#if 0 //(KEY_TYPE == KEY_TYPE1)||(KEY_TYPE==KEY_TYPE2)||(KEY_TYPE==KEY_TYPE3)||(KEY_TYPE==KEY_TYPE4)||(KEY_TYPE==KEY_TYPE5)


#else

/*
	0.41v => 495
	0.39v =>
	0.38v => 460
	0.37v => 
	0.36v => 440
	0.35v =>
	0.34v =>
*/
enum 
{
BATTERY_CNT = 8, 
BATTERY_Lv3 = 495 * BATTERY_CNT, 
BATTERY_Lv2 = 460 * BATTERY_CNT, 
BATTERY_Lv1 = 440 * BATTERY_CNT
};


#if USE_ADKEY_NO				== 6
static INT32U	adc_key_factor_table[USE_ADKEY_NO] =
{ // x1000

	// 6 AD-keys
	//680K, 300K, 150K, 68K, 39K, 22K
	1969, 2933, 4182, 5924, 7104, 8102
};


#else

static INT32U	adc_key_factor_table[USE_ADKEY_NO] =
{ // x1000

	// 1 AD-keys
	//680K
	1969
};


#endif

static INT32U	ad_time_stamp;


INT32U adc_key_judge(INT32U adc_value)
{
	INT32U			candidate_key;
	INT32U			candidate_diff;
	INT32U			i, temp1, temp2, temp3, diff;

	candidate_key		= USE_ADKEY_NO;
	candidate_diff		= 0xFFFFFFFF;
	temp1				= 1000 * adc_value; 		// to avoid "decimal point"

	temp2				= adc_key_release_calibration(adc_key_release_value_stable); // adc_battery_value_stable = stable adc value got when no key press

	// temp2: adc theoretical value 
	for (i = 0; i < USE_ADKEY_NO; i++)
	{
		temp3				= temp2 * adc_key_factor_table[i]; // temp3: the calculated delimiter	

		if (temp1 >= temp3)
		{
			diff				= temp1 - temp3;
		}
		else 
		{
			diff				= temp3 - temp1;
		}

		// DBG_PRINT("adc:[%d], bat:[%d], diff:[%d]\r\n", temp1, temp3, diff);
		if (diff > candidate_diff)
		{
			ASM(NOP);
			ASM(NOP);
			ASM(NOP);
			ASM(NOP);
			ASM(NOP);
			ASM(NOP);

			break;
		}

		candidate_key		= i;
		candidate_diff		= diff;
	}

	if (candidate_key < USE_ADKEY_NO)
	{
		//DBG_PRINT("\r\nKey %d", candidate_key+1);
		//		power_off_time_beep_1 = 0;
		//		power_off_time_beep_2 = 0;
		//		power_off_time_beep_3 = 0;
#if C_SCREEN_SAVER						== CUSTOM_ON
		key_active_cnt		= 0;
		ap_peripheral_lcd_backlight_set(BL_ON);
#endif
	}

	return candidate_key;
}


//#define SA_TIME	50	//seconds, for screen saver time. Temporary use "define" before set in "STATE_SETTING".
void ap_peripheral_ad_key_judge(void)
{
	INT32U			t;
	INT32U			diff;
	INT16U			adc_current_value;
	INT16U			temp_value;

	//DBG_PRINT("ap_peripheral_ad_key_judge()~~~~\r\n");
	//DBG_PRINT(".");

	/*t = OSTimeGet();
	if ((t - ad_time_stamp) < 2) {
		return;
	}
	ad_time_stamp = t;

	ad_line_select++;
	if (ad_line_select & 0x1F) {
		///adc_manual_ch_set(AD_KEY_DETECT_PIN);
	} else {
		adc_manual_ch_set(2);///AD_BAT_DETECT_PIN);
		temp_value = ((R_ADC_MADC_DATA & 0xFFFF) >> 4);
		DBG_PRINT("0x%x \n\r", temp_value);
		ad_line_select = 0;
	}*/
	adc_manual_ch_set(2);							///AD_BAT_DETECT_PIN);

	///temp_value = ((R_ADC_MADC_DATA & 0xFFFF) >> 4);
	///DBG_PRINT("0x%x \n\r", temp_value);
	ad_line_select		= 0;

	//adc_manual_sample_start();
	if (ad_line_select == 1)
	{ //battery detection

		adc_battery_value_old = adc_battery_value_new;
		adc_battery_value_new = ad_value >> 4;

		if (adc_battery_value_new >= adc_battery_value_old)
		{
			diff				= adc_battery_value_new - adc_battery_value_old;
		}
		else 
		{
			diff				= adc_battery_value_old - adc_battery_value_new;
		}

		if (!diff || (100 * diff <= C_RESISTOR_ACCURACY * adc_battery_value_old))
		{
			if (battery_stable_cnt < C_BATTERY_STABLE_THRESHOLD)
				battery_stable_cnt++;
		}
		else 
		{
			battery_stable_cnt	= 1;

#if C_BATTERY_DETECT						== CUSTOM_ON
			bat_ck_cnt			= 0;
			battery_value_sum	= 0;
#endif
		}

		if (battery_stable_cnt >= C_BATTERY_STABLE_THRESHOLD)
		{

			//DBG_PRINT("%d,", adc_battery_value_new);
#if C_BATTERY_DETECT						== CUSTOM_ON
			ap_peripheral_battery_check_calculate();
#endif

		}

		adc_manual_sample_start();
		return;
	}

#if 0
	adc_current_value	= ad_value >> 6;

#else

	// josephhsieh140408
	adc_current_value	= (ad_value >> 4);

	//DBG_PRINT("%d\r\n",adc_current_value);
#endif

	//DBG_PRINT("ad_value = 0x%x \r\n",ad_value);
	//print_string("ad_value = %d \r\n",ad_value);
	if (adc_current_value < C_KEY_PRESS_WATERSHED)
	{ // Key released

		if (adc_key_value)
		{ // adc_key_value: last ADC(KEY) value

			// Key released
			if (!fast_key_exec_flag && !long_key_exec_flag && key_pressed_cnt >= C_KEY_STABLE_THRESHOLD)
			{
				INT32U			pressed_key;

				pressed_key 		= adc_key_judge(adc_key_value);

				if (pressed_key < USE_ADKEY_NO && ad_key_map[pressed_key].key_function)
				{
					ad_key_map[pressed_key].key_function(& (ad_key_map[pressed_key].key_cnt));

					//DBG_PRINT("key_function key%d \r\n",pressed_key);
				}
			}

			adc_key_value		= 0;
			key_pressed_cnt 	= 0;
			fast_key_exec_flag	= 0;
			normal_key_exec_flag = 0;
			long_key_exec_flag	= 0;
		}

		adc_key_release_value_old = adc_key_release_value_new;
		adc_key_release_value_new = adc_current_value;

		if (adc_key_release_value_new >= adc_key_release_value_old)
		{
			diff				= adc_key_release_value_new - adc_key_release_value_old;
		}
		else 
		{
			diff				= adc_key_release_value_old - adc_key_release_value_new;
		}

		if (!diff || (100 * diff <= C_RESISTOR_ACCURACY * adc_key_release_value_old))
		{
			key_release_stable_cnt++;
		}
		else 
		{
			key_release_stable_cnt = 1;
		}

		if (key_release_stable_cnt >= C_KEY_RELEASE_STABLE_THRESHOLD)
		{
			adc_key_release_value_stable = (adc_key_release_value_new + adc_key_release_value_old) >> 1;

			//DBG_PRINT("%d,", adc_key_release_value_stable);
			key_release_stable_cnt = 1;
		}

	}
	else 
	{ // Key pressed

		if (adc_current_value >= adc_key_value)
		{
			diff				= adc_current_value - adc_key_value;
		}
		else 
		{
			diff				= adc_key_value - adc_current_value;
		}

		if (!diff || (100 * diff <= C_RESISTOR_ACCURACY * adc_key_value))
		{
			key_pressed_cnt++;

			// TBD: Handle zoom function
			//if (zoom_key_flag && ad_key_cnt>2 && (adkey_lvl == PREVIOUS_KEY || adkey_lvl == NEXT_KEY)) {
#if C_KEY_FAST_JUDGE_THRESHOLD				!= 0

			if (key_pressed_cnt == C_KEY_FAST_JUDGE_THRESHOLD)
			{ // fast key(long pressed),more than 2 sec
				INT32U			pressed_key;

				pressed_key 		= adc_key_judge(adc_key_value);

				if (pressed_key < USE_ADKEY_NO && ad_key_map[pressed_key].fast_key_fun)
				{
					ad_key_map[pressed_key].fast_key_fun(& (ad_key_map[pressed_key].key_cnt));
					fast_key_exec_flag	= 1;
				}
			}

#endif

			//#if KEY_FUNTION_TYPE == SAMPLE2
			//			DBG_PRINT("key_pressed_cnt = %d  \r\n",key_pressed_cnt);
			if (key_pressed_cnt == 40)
			{ //long key
				INT32U			pressed_key;

				pressed_key 		= adc_key_judge(adc_key_value);

				if (pressed_key < USE_ADKEY_NO && ad_key_map[pressed_key].key_function)
				{
					ad_key_map[pressed_key].key_cnt = key_pressed_cnt;

					//DBG_PRINT("long key cnt = %d	\r\n",ad_key_map[pressed_key].key_cnt);
					ad_key_map[pressed_key].key_function(& (ad_key_map[pressed_key].key_cnt));

					long_key_exec_flag	= 1;
					adc_key_value		= 0;
					key_pressed_cnt 	= 0;
					fast_key_exec_flag	= 0;
					normal_key_exec_flag = 0;
				}
			}

			//#endif
		}
		else 
		{
			// Key released or not stable
			if (adc_key_value && !fast_key_exec_flag && !normal_key_exec_flag && !long_key_exec_flag &&
				 key_pressed_cnt >= C_KEY_STABLE_THRESHOLD)
			{
				INT32U			pressed_key;

				pressed_key 		= adc_key_judge(adc_key_value);

				if (pressed_key < USE_ADKEY_NO && ad_key_map[pressed_key].key_function)
				{
					//ad_key_map[pressed_key].key_function(&(ad_key_map[pressed_key].key_cnt));
					normal_key_exec_flag = 1;

					//DBG_PRINT("key_function 2\r\n");
				}
			}

			if (!fast_key_exec_flag && !normal_key_exec_flag)
			{
				adc_key_value		= adc_current_value;
				key_pressed_cnt 	= 1;

				//fast_key_exec_flag = 0;
			}
		}
	}

	adc_manual_sample_start();
}


#endif // AD-Key

#endif

#if C_BATTERY_DETECT			== CUSTOM_ON

INT32U			previous_direction = 0;
extern void ap_state_handling_led_off(void);
extern INT8U	display_str_battery_low;


#define BATTERY_GAP 			10*BATTERY_CNT


static INT8U ap_peripheral_smith_trigger_battery_level(INT32U direction)
{
	static INT8U	bat_lvl_cal_bak = (INT8U)

	BATTERY_Lv3;
	INT8U			bat_lvl_cal;

	// DBG_PRINT("(%d)\r\n", battery_value_sum);
	if (battery_value_sum >= BATTERY_Lv3)
	{
		bat_lvl_cal 		= 3;
	}
	else if ((battery_value_sum < BATTERY_Lv3) && (battery_value_sum >= BATTERY_Lv2))
	{
		bat_lvl_cal 		= 2;
	}
	else if ((battery_value_sum < BATTERY_Lv2) && (battery_value_sum >= BATTERY_Lv1))
	{
		bat_lvl_cal 		= 1;
	}
	else if (battery_value_sum < BATTERY_Lv1)
	{
		bat_lvl_cal 		= 0;
	}


	if ((direction == 0) && (bat_lvl_cal > bat_lvl_cal_bak))
	{
		if (battery_value_sum >= BATTERY_Lv3 + BATTERY_GAP)
		{
			bat_lvl_cal 		= 3;
		}
		else if ((battery_value_sum < BATTERY_Lv3 + BATTERY_GAP) && (battery_value_sum >= BATTERY_Lv2 + BATTERY_GAP))
		{
			bat_lvl_cal 		= 2;
		}
		else if ((battery_value_sum < BATTERY_Lv2 + BATTERY_GAP) && (battery_value_sum >= BATTERY_Lv1 + BATTERY_GAP))
		{
			bat_lvl_cal 		= 1;
		}
		else if (battery_value_sum < BATTERY_Lv1 + BATTERY_GAP)
		{
			bat_lvl_cal 		= 0;
		}
	}


	if ((direction == 1) && (bat_lvl_cal < bat_lvl_cal_bak))
	{
		if (battery_value_sum >= BATTERY_Lv3 - BATTERY_GAP)
		{
			bat_lvl_cal 		= 3;
		}
		else if ((battery_value_sum < BATTERY_Lv3 - BATTERY_GAP) && (battery_value_sum >= BATTERY_Lv2 - BATTERY_GAP))
		{
			bat_lvl_cal 		= 2;
		}
		else if ((battery_value_sum < BATTERY_Lv2 - BATTERY_GAP) && (battery_value_sum >= BATTERY_Lv1 - BATTERY_GAP))
		{
			bat_lvl_cal 		= 1;
		}
		else if (battery_value_sum < BATTERY_Lv1 - BATTERY_GAP)
		{
			bat_lvl_cal 		= 0;
		}
	}


	bat_lvl_cal_bak 	= bat_lvl_cal;
	return bat_lvl_cal;

}


void ap_peripheral_battery_check_calculate(void)
{
	INT8U			bat_lvl_cal;
	INT32U			direction = 0;

	if (adp_status == 0)
	{
		//unkown state
		return;
	}
	else if (adp_status == 1)
	{
		//adaptor in state
		direction			= 1;					//low voltage to high voltage

		if (previous_direction != direction)
		{
			msgQSend(ApQ, MSG_APQ_BATTERY_CHARGED_SHOW, NULL, NULL, MSG_PRI_NORMAL);
		}

		previous_direction	= direction;
	}
	else 
	{
		//adaptor out state
		direction			= 0;					//high voltage to low voltage

		if (previous_direction != direction)
		{
			msgQSend(ApQ, MSG_APQ_BATTERY_CHARGED_CLEAR, NULL, NULL, MSG_PRI_NORMAL);
		}

		previous_direction	= direction;
	}

	battery_value_sum	+= (ad_value >> 4);

	// DBG_PRINT("%d, ",(ad_value>>4));
	bat_ck_cnt++;

	if (bat_ck_cnt >= BATTERY_CNT)
	{

		bat_lvl_cal 		= ap_peripheral_smith_trigger_battery_level(direction);

		//		DBG_PRINT("%d,", bat_lvl_cal);
		if (!battery_low_flag)
		{
			msgQSend(ApQ, MSG_APQ_BATTERY_LVL_SHOW, &bat_lvl_cal, sizeof(INT8U), MSG_PRI_NORMAL);
		}

		if (bat_lvl_cal == 0 && direction == 0)
		{
			low_voltage_cnt++;

			if (low_voltage_cnt > 5)
			{
				low_voltage_cnt 	= 0;
				ap_state_handling_led_off();

#if C_BATTERY_LOW_POWER_OFF 					== CUSTOM_ON

				if (!battery_low_flag)
				{
					battery_low_flag	= 1;
					{
						INT8U			type;

						msgQSend(StorageServiceQ, MSG_STORAGE_SERVICE_TIMER_STOP, NULL, NULL, MSG_PRI_NORMAL);
						type				= FALSE;
						msgQSend(StorageServiceQ, MSG_STORAGE_SERVICE_FREESIZE_CHECK_SWITCH, &type, sizeof(INT8U),
							 MSG_PRI_NORMAL);
						type				= BETTERY_LOW_STATUS_KEY;
						msgQSend(PeripheralTaskQ, MSG_PERIPHERAL_TASK_KEY_REGISTER, &type, sizeof(INT8U),
							 MSG_PRI_NORMAL);
						msgQSend(ApQ, MSG_APQ_BATTERY_LOW_SHOW, NULL, sizeof(INT8U), MSG_PRI_NORMAL);
					}
				}

#endif

				//OSTimeDly(100);
				//display_str_battery_low = 1;
				//msgQSend(ApQ, MSG_APQ_POWER_KEY_ACTIVE, NULL, NULL, MSG_PRI_NORMAL);
			}
		}
		else 
		{
			if (battery_low_flag)
			{
				INT8U			type;

				battery_low_flag	= 0;
				msgQSend(StorageServiceQ, MSG_STORAGE_SERVICE_TIMER_START, NULL, NULL, MSG_PRI_NORMAL);
				type				= GENERAL_KEY;
				msgQSend(PeripheralTaskQ, MSG_PERIPHERAL_TASK_KEY_REGISTER, &type, sizeof(INT8U), MSG_PRI_NORMAL);
			}

			low_voltage_cnt 	= 0;
		}

		bat_ck_cnt			= 0;
		battery_value_sum	= 0;
	}
}


#endif




#if C_SCREEN_SAVER				== CUSTOM_ON


void ap_peripheral_auto_off_force_disable_set(INT8U auto_off_disable)
{
	auto_off_force_disable = auto_off_disable;
}


void ap_peripheral_lcd_backlight_set(INT8U type)
{
	if (type == BL_ON)
	{
		if (lcd_bl_sts)
		{
			lcd_bl_sts			= 0;

#if _DRV_L1_TFT 							== 1
			tft_backlight_en_set(TRUE);
#endif

			DBG_PRINT("LCD ON\r\n");
		}
	}
	else 
	{
		if (!lcd_bl_sts)
		{
			lcd_bl_sts			= 1;

#if _DRV_L1_TFT 							== 1
			tft_backlight_en_set(FALSE);
#endif

			DBG_PRINT("LCD OFF\r\n");
		}
	}
}


#endif


void ap_peripheral_night_mode_set(INT8U type)
{
	if (type)
	{
		gpio_write_io(IR_CTRL, 1);
	}
	else 
	{
		gpio_write_io(IR_CTRL, 0);
	}
}


static INT8U	RegKey_First_Flag = 0;


void ap_peripheral_key_init(void)
{
	INT32U			i;

	gp_memset((INT8S *) &key_map, NULL, sizeof(KEYSTATUS));
	ap_peripheral_key_register(GENERAL_KEY);

	/*
	if (RegKey_First_Flag == 0)
	{//output 500 ms high level when power on
		RegKey_First_Flag = 1;
		if (key_map[0].key_io)
		{
			//gpio_init_io(key_map[0].key_io, GPIO_INPUT);
			gpio_init_io(key_map[0].key_io, GPIO_OUTPUT);
			gpio_set_port_attribute(key_map[0].key_io, ATTRIBUTE_HIGH);
			//gpio_write_io(key_map[0].key_io, DATA_LOW);
			//OSTimeDly(10);
			gpio_write_io(key_map[0].key_io, DATA_HIGH);
			OSTimeDly(50);
			gpio_write_io(key_map[0].key_io, DATA_LOW);
			OSTimeDly(20);
		}
	}
	*/
	for (i = 0; i < USE_IOKEY_NO; i++)
	{
		if (key_map[i].key_io)
		{
			key_map[i].key_cnt	= 0;
			gpio_init_io(key_map[i].key_io, GPIO_INPUT);
			gpio_set_port_attribute(key_map[i].key_io, ATTRIBUTE_LOW);

			// gpio_write_io(key_map[i].key_io, KEY_ACTIVE^1);
			gpio_write_io(key_map[i].key_io, key_map[i].key_active ^ 1); //Liuxi modified in 2015-01-15
			DBG_PRINT("INIT\r\n");
		}
	}
}


extern void Time_card_storage(TIME_T * intime);


void ap_peripheral_time_set(void)
{
#if 1
	TIME_T			READ_TIME;

	DBG_PRINT("TIME_SET1\r\n");
	ap_state_config_timefile_get(&READ_TIME);
	Time_card_storage(&READ_TIME);

#if USING_EXT_RTC					== CUSTOM_ON
	ap_state_handling_calendar_init();

#else

	ap_state_handing_intime_init(READ_TIME);

#endif

#endif

}


void ap_peripheral_key_register(INT8U type)
{
	INT32U			i;

	if (type == GENERAL_KEY)
	{
		key_map[0].key_io	= VIDEO_KEY;
		key_map[0].key_function = (KEYFUNC)ap_peripheral_pw_key_exe;
		key_map[0].key_active = VIDEO_KEY_ACTIVE;	//1;
		key_map[0].long_key_flag = 0;

		key_map[1].key_io	= CAPTURE_KEY;
		key_map[1].key_function = (KEYFUNC)ap_peripheral_capture_key_exe;
		key_map[1].key_active = CAPTURE_KEY_ACTIVE; //1;
		key_map[1].long_key_flag = 0;

		ad_key_map[0].key_io = FUNCTION_KEY;
		ad_key_map[0].key_function = (KEYFUNC)ap_peripheral_null_key_exe;
	}
	else if (type == USBD_DETECT)
	{
#if USE_IOKEY_NO

		for (i = 0; i < USE_IOKEY_NO; i++)
		{
			if (key_map[i].key_io != VIDEO_KEY)
				key_map[i].key_io = NULL;
		}

#endif

#if USE_ADKEY_NO

		for (i = 0; i < USE_ADKEY_NO; i++)
		{
			ad_key_map[i].key_function = ap_peripheral_null_key_exe;
		}

#endif
	}
	else if (type == DISABLE_KEY)
	{
#if USE_IOKEY_NO

		for (i = 0; i < USE_IOKEY_NO; i++)
		{
			key_map[i].key_io	= NULL;
		}

#endif

#if USE_ADKEY_NO

		for (i = 0; i < USE_ADKEY_NO; i++)
		{
			ad_key_map[i].key_function = ap_peripheral_null_key_exe;
		}

#endif
	}
	else if (type == BETTERY_LOW_STATUS_KEY)
	{
		key_map[0].key_io	= PW_KEY;
		key_map[0].key_function = (KEYFUNC)
		ap_peripheral_pw_key_exe;

#if USE_ADKEY_NO

		for (i = 0; i < USE_ADKEY_NO; i++)
		{
			ad_key_map[i].key_function = ap_peripheral_null_key_exe;
		}

#endif
	}

}


extern INT8U ap_state_config_auto_off_get(void);

INT8U			long_pw_key_pressed = 0;

#if CRAZY_KEY_TEST				== 1
INT8U			crazy_key_enable = 0;
INT32U			crazy_key_cnt = 0;

#endif


void ap_peripheral_key_judge(void)
{
	INT32U			i, key_press = 0;
	INT16U			key_down = 0;

	for (i = 0; i < USE_IOKEY_NO; i++)
	{
		if (key_map[i].key_io)
		{

			if (key_map[i].key_active)
				key_down = gpio_read_io(key_map[i].key_io);
			else 
				key_down = !gpio_read_io(key_map[i].key_io);

			if (key_down)
			{

				if (! (key_map[i].long_key_flag))
				{
					key_map[i].key_cnt	+= 1;

#if SUPPORT_LONGKEY 								== CUSTOM_ON

					if (key_map[i].key_cnt >= Long_Single_width)
					{
						//long_pw_key_pressed = 1;
						key_map[i].long_key_flag = 1;
						key_map[i].key_function(& (key_map[i].key_cnt));
					}

#endif
				}
				else 
				{
					key_map[i].key_cnt	= 0;
				}

				if (key_map[i].key_cnt == 65535)
				{
					key_map[i].key_cnt	= 32;
				}
			}
			else 
			{
				if (key_map[i].long_key_flag)
				{
					key_map[i].long_key_flag = 0;
				}

				// long_pw_key_pressed=0;
				if (key_map[i].key_cnt >= Short_Single_width) //Short_Single_width
				{

					key_map[i].key_function(& (key_map[i].key_cnt));
					key_press			= 1;
				}

				key_map[i].key_cnt	= 0;

			}
		}
	}

}


static int ap_peripheral_power_key_read(int pin)
{
	int 			status;


#if (								KEY_TYPE == KEY_TYPE1)||(KEY_TYPE == KEY_TYPE2)||(KEY_TYPE == KEY_TYPE3)||(KEY_TYPE == KEY_TYPE4)||(KEY_TYPE == KEY_TYPE5)
	status				= gpio_read_io(pin);

#else

	switch (pin)
	{
		case PWR_KEY0:
			status = sys_pwr_key0_read();
			break;

		case PWR_KEY1:
			status = sys_pwr_key1_read();
			break;
	}

#endif

	if (status != 0)
		return 1;

	else 
		return 0;
}


void ap_peripheral_adaptor_out_judge(void)
{
	adp_out_cnt++;

	switch (adp_status)
	{
		case 0: //unkown state
			if (ap_peripheral_power_key_read(ADP_OUT_PIN))
			{
				adp_cnt++;

				if (adp_cnt > 16)
				{
					adp_out_cnt 		= 0;
					adp_cnt 			= 0;
					adp_status			= 1;
					OSQPost(USBTaskQ, (void *) MSG_USBD_INITIAL);

#if C_BATTERY_DETECT								== CUSTOM_ON && USE_ADKEY_NO

					//battery_lvl = 1;
#endif
				}
			}
			else 
			{
				adp_cnt 			= 0;
			}

			if (adp_out_cnt > 24)
			{
				adp_out_cnt 		= 0;
				adp_status			= 3;

#if C_BATTERY_DETECT							== CUSTOM_ON && USE_ADKEY_NO

				//battery_lvl = 2;
				low_voltage_cnt 	= 0;
#endif
			}

			break;

		case 1: //adaptor in state
			if (!ap_peripheral_power_key_read(ADP_OUT_PIN))
			{
				if (adp_out_cnt > 8)
				{
					adp_status			= 2;

#if C_BATTERY_DETECT								== CUSTOM_ON
					low_voltage_cnt 	= 0;
#endif

					// 璝棵辊玂臔秨璶翴獹璉
					if (screen_saver_enable)
					{
						screen_saver_enable = 0;

#if C_SCREEN_SAVER										== CUSTOM_ON
						ap_state_handling_lcd_backlight_switch(1);
#endif
					}
				}
			}
			else 
			{
				adp_out_cnt 		= 0;
			}

			break;

		case 2: //adaptor out state
			if (!ap_peripheral_power_key_read(ADP_OUT_PIN))
			{
				if ((adp_out_cnt > PERI_ADP_OUT_PWR_OFF_TIME))
				{
					//ap_peripheral_pw_key_exe(&adp_out_cnt);
				}

				adp_cnt 			= 0;
			}
			else 
			{
				adp_cnt++;

				if (adp_cnt > 3)
				{
					adp_out_cnt 		= 0;
					adp_status			= 1;
					usbd_exit			= 0;
					OSQPost(USBTaskQ, (void *) MSG_USBD_INITIAL);
				}
			}

			break;

		case 3: //adaptor initial out state
			if (ap_peripheral_power_key_read(ADP_OUT_PIN))
			{
				if (adp_out_cnt > 3)
				{
					adp_out_cnt 		= 0;
					adp_status			= 1;
					OSQPost(USBTaskQ, (void *) MSG_USBD_INITIAL);
				}
			}
			else 
			{
				adp_out_cnt 		= 0;
			}

			break;

		default:
			break;
	}

	///DBG_PRINT("USB_PIN=%d\r\n",s_usbd_pin);
	if (s_usbd_pin == 1)
	{
		usbd_cnt++;

		if (!ap_peripheral_power_key_read(C_USBDEVICE_PIN))
		{
			if (usbd_cnt > 3)
			{
				ap_peripheral_usbd_plug_out_exe(&usbd_cnt);
			}
		}
		else 
		{
			usbd_cnt			= 0;
		}
	}

#if USB_PHY_SUSPEND 				== 1

	if (s_usbd_pin == 0)
	{
		if (ap_peripheral_power_key_read(C_USBDEVICE_PIN))
		{
			if (phy_cnt == PERI_USB_PHY_SUSPEND_TIME)
			{
				// disable USB PHY CLK for saving power
				DBG_PRINT("Turn Off USB PHY clk (TODO)\r\n");
				phy_cnt++;							// ヘ琌 Turn Off 暗Ω
			}
			else if (phy_cnt < PERI_USB_PHY_SUSPEND_TIME)
			{
				phy_cnt++;
			}
		}
		else 
		{
			phy_cnt 			= 0;
		}
	}

#endif

}


void ap_peripheral_function_key_exe(INT16U * tick_cnt_ptr)
{
	msgQSend(ApQ, MSG_APQ_AUDIO_EFFECT_MODE, NULL, NULL, MSG_PRI_NORMAL);

	if (screen_saver_enable)
	{
		screen_saver_enable = 0;
		msgQSend(ApQ, MSG_APQ_KEY_WAKE_UP, NULL, NULL, MSG_PRI_NORMAL);
	}
	else 
	{
		DBG_PRINT("function_Key\r\n");

		if (*tick_cnt_ptr > 24)
		{
		}
		else 
		{
			DBG_PRINT("MODE_ACTIVE\r\n");
			DBG_PRINT("*tick_cnt_ptr=%d\r\n", *tick_cnt_ptr);
			msgQSend(ApQ, MSG_APQ_MODE, NULL, NULL, MSG_PRI_NORMAL);
		}
	}

	*tick_cnt_ptr		= 0;
}


void ap_peripheral_next_key_exe(INT16U * tick_cnt_ptr)
{
	INT8U			data = 0;

	msgQSend(ApQ, MSG_APQ_AUDIO_EFFECT_DOWN, NULL, NULL, MSG_PRI_NORMAL);

	if (screen_saver_enable)
	{
		screen_saver_enable = 0;
		msgQSend(ApQ, MSG_APQ_KEY_WAKE_UP, NULL, NULL, MSG_PRI_NORMAL);
	}
	else 
	{
		if (*tick_cnt_ptr > 24)
		{
			msgQSend(ApQ, MSG_APQ_FORWARD_FAST_PLAY, &data, sizeof(INT8U), MSG_PRI_NORMAL);
		}
		else 
		{
			msgQSend(ApQ, MSG_APQ_NEXT_KEY_ACTIVE, &data, sizeof(INT8U), MSG_PRI_NORMAL);
		}
	}

	*tick_cnt_ptr		= 0;
}


void ap_peripheral_prev_key_exe(INT16U * tick_cnt_ptr)
{
	INT8U			data = 0;

	msgQSend(ApQ, MSG_APQ_AUDIO_EFFECT_UP, NULL, NULL, MSG_PRI_NORMAL);

	if (screen_saver_enable)
	{
		screen_saver_enable = 0;
		msgQSend(ApQ, MSG_APQ_KEY_WAKE_UP, NULL, NULL, MSG_PRI_NORMAL);
	}
	else 
	{
		if (*tick_cnt_ptr > 24)
		{
			msgQSend(ApQ, MSG_APQ_BACKWORD_FAST_PLAY, &data, sizeof(INT8U), MSG_PRI_NORMAL);

		}
		else 
		{
			msgQSend(ApQ, MSG_APQ_PREV_KEY_ACTIVE, &data, sizeof(INT8U), MSG_PRI_NORMAL);
		}
	}

	*tick_cnt_ptr		= 0;
}


void ap_peripheral_ok_key_exe(INT16U * tick_cnt_ptr)
{
	msgQSend(ApQ, MSG_APQ_AUDIO_EFFECT_OK, NULL, NULL, MSG_PRI_NORMAL);

	if (screen_saver_enable)
	{
		screen_saver_enable = 0;
		msgQSend(ApQ, MSG_APQ_KEY_WAKE_UP, NULL, NULL, MSG_PRI_NORMAL);
	}
	else 
	{
		if (*tick_cnt_ptr > 24)
		{
			//for test
			msgQSend(ApQ, MSG_APQ_INIT_THUMBNAIL, NULL, NULL, MSG_PRI_NORMAL);

			//			msgQSend(ApQ, MSG_APQ_FILE_LOCK_DURING_RECORDING, NULL, NULL, MSG_PRI_NORMAL);
		}
		else 
		{
			msgQSend(ApQ, MSG_APQ_FUNCTION_KEY_ACTIVE, NULL, NULL, MSG_PRI_NORMAL);
		}
	}

	*tick_cnt_ptr		= 0;
}


#if KEY_FUNTION_TYPE			== SAMPLE2
extern volatile INT8U pic_down_flag;
extern volatile INT8U video_down_flag;


void ap_peripheral_capture_key_exe(INT16U * tick_cnt_ptr)
{
	INT32U			led_type;

	if (!s_usbd_pin)
	{
		if ((ap_state_handling_storage_id_get() != NO_STORAGE) && (!pic_down_flag) && (!card_space_less_flag))
		{
#if SUPPORT_LONGKEY 						== CUSTOM_ON

			if (*tick_cnt_ptr > 24)
			{

			}
			else 
#endif

			{
				DBG_PRINT("[CAPTUER_ACTIVE...]\r\n");
				msgQSend(ApQ, MSG_APQ_CAPTUER_ACTIVE, NULL, NULL, MSG_PRI_NORMAL);
			}
		}
	}
	else 
	{
		if (*tick_cnt_ptr > 24)
		{
			//OSQPost(USBTaskQ, (void *) MSG_USBD_SWITCH);
		}
		else 
		{
			if (!pic_down_flag)
				OSQPost(USBTaskQ, (void *) MSG_USBD_SWITCH);
		}
	}

	*tick_cnt_ptr		= 0;
}


#endif


void ap_peripheral_sos_key_exe(INT16U * tick_cnt_ptr)
{
#if CRAZY_KEY_TEST					== 1	

	if (!crazy_key_enable)
	{
		crazy_key_enable	= 1;
	}
	else 
	{
		crazy_key_enable	= 0;
	}

	*tick_cnt_ptr		= 0;
	return;

#endif

	if (screen_saver_enable)
	{
		screen_saver_enable = 0;
		msgQSend(ApQ, MSG_APQ_KEY_WAKE_UP, NULL, NULL, MSG_PRI_NORMAL);
	}
	else 
	{
		if (*tick_cnt_ptr > 24)
		{
		}
		else 
		{
			msgQSend(ApQ, MSG_APQ_SOS_KEY_ACTIVE, NULL, NULL, MSG_PRI_NORMAL);
		}
	}

	*tick_cnt_ptr		= 0;
}


void ap_peripheral_usbd_plug_out_exe(INT16U * tick_cnt_ptr)
{
	msgQSend(ApQ, MSG_APQ_DISCONNECT_TO_PC, NULL, NULL, MSG_PRI_NORMAL);
	*tick_cnt_ptr		= 0;
}


extern INT8U ap_video_record_sts_get(void);


void ap_peripheral_pw_key_exe(INT16U * tick_cnt_ptr)
{
	INT32U			led_type;

	if (!s_usbd_pin)
	{
		if (!card_space_less_flag)
		{
			if (ap_state_handling_storage_id_get() != NO_STORAGE)
			{
#if SUPPORT_LONGKEY 							== CUSTOM_ON

				if (*tick_cnt_ptr >= Long_Single_width)
				{
					if (video_stop_flag == 0)
					{
						if (ap_video_record_sts_get() & 0x2)
						{
							//led_type = VIDEO_ENCORD_SAVE_LED;
							//msgQSend(PeripheralTaskQ, MSG_PERIPHERAL_TASK_LED_SET, &led_type, sizeof(INT32U), MSG_PRI_NORMAL);
							ap_video_save_state_set(1);
						}

						DBG_PRINT("[VIDEO_RECORD_...]\r\n");
						msgQSend(ApQ, MSG_APQ_VIDEO_RECORD_ACTIVE, NULL, NULL, MSG_PRI_NORMAL);
					}
				}
				else 
#endif

				{
					if (pic_down_flag == 0)
					{
						if (ap_video_record_sts_get() & 0x2)
						{
							//led_type = VIDEO_ENCORD_SAVE_LED;
							//msgQSend(PeripheralTaskQ, MSG_PERIPHERAL_TASK_LED_SET, &led_type, sizeof(INT32U), MSG_PRI_NORMAL);
							ap_video_save_state_set(0);
						}

						DBG_PRINT("[VIDEO_RECORD_...]\r\n");
						msgQSend(ApQ, MSG_APQ_VIDEO_RECORD_ACTIVE, NULL, NULL, MSG_PRI_NORMAL);

						//DBG_PRINT("[CAPTUER_ACTIVE...]\r\n");
						//msgQSend(ApQ, MSG_APQ_CAPTUER_ACTIVE, NULL, NULL, MSG_PRI_NORMAL);
					}
				}
			}
		}
	}
	else 
	{
		if (*tick_cnt_ptr >= 32)
		{
			//OSQPost(USBTaskQ, (void *) MSG_USBD_SWITCH);
		}
		else 
		{
			OSQPost(USBTaskQ, (void *) MSG_USBD_SWITCH);
		}
	}

	*tick_cnt_ptr		= 0;
}


void ap_peripheral_menu_key_exe(INT16U * tick_cnt_ptr)
{
#if KEY_FUNTION_TYPE				== C6_KEY
	msgQSend(ApQ, MSG_APQ_AUDIO_EFFECT_MENU, NULL, NULL, MSG_PRI_NORMAL);

	if (screen_saver_enable)
	{
		screen_saver_enable = 0;
		msgQSend(ApQ, MSG_APQ_KEY_WAKE_UP, NULL, NULL, MSG_PRI_NORMAL);
	}
	else 
	{
		if (*tick_cnt_ptr > 24)
		{
		}
		else 
		{
			msgQSend(ApQ, MSG_APQ_MENU_KEY_ACTIVE, NULL, NULL, MSG_PRI_NORMAL);
		}
	}

#endif

	* tick_cnt_ptr		= 0;
}


void ap_peripheral_null_key_exe(INT16U * tick_cnt_ptr)
{

}


#if GPDV_BOARD_VERSION			!= GPCV1237A_Aerial_Photo


void ap_TFT_backlight_tmr_check(void)
{
	if (backlight_tmr)
	{
		backlight_tmr--;

		if ((backlight_tmr == 0) && (tv == !TV_DET_ACTIVE))
		{
			//gpio_write_io(TFT_BL, DATA_HIGH);	//turn on LCD backlight
#if _DRV_L1_TFT 							== 1
			tft_backlight_en_set(1);
#endif
		}
	}
}


#endif

//+++ TV_OUT_D1
#if TV_DET_ENABLE


INT8U tv_plug_status_get(void)
{
	return tv_plug_in_flag;
}


#endif

//---
#if GPDV_BOARD_VERSION			!= GPCV1237A_Aerial_Photo


void ap_peripheral_tv_detect(void)
{
#if TV_DET_ENABLE
	INT8U			temp;

	temp				= gpio_read_io(AV_IN_DET);

	if (temp != tv)
	{
		tv_debounce_cnt++;

		if (tv_debounce_cnt > 4)
		{
			tv_debounce_cnt 	= 0;
			tv					= temp;

			if (tv == !TV_DET_ACTIVE)
			{ //display use TFT

				//backlight_tmr = PERI_TIME_BACKLIGHT_DELAY;	//delay some time to enable LCD backlight so that no noise shown on LCD
				gpio_write_io(SPEAKER_EN, DATA_HIGH); //open local speaker

				//+++ TV_OUT_D1
				tv_plug_in_flag 	= 0;
				msgQSend(ApQ, MSG_APQ_TV_PLUG_OUT, NULL, NULL, MSG_PRI_NORMAL);

				//---
			}
			else 
			{ //display use TV
				gpio_write_io(SPEAKER_EN, DATA_LOW); //mute local speaker

				//gpio_write_io(TFT_BL, DATA_LOW);		//turn off LCD backlight
				//+++ TV_OUT_D1
				tv_plug_in_flag 	= 1;
				msgQSend(ApQ, MSG_APQ_TV_PLUG_IN, NULL, NULL, MSG_PRI_NORMAL);

				//---
			}
		}
	}
	else 
	{
		tv_debounce_cnt 	= 0;
	}

#endif
}


void ap_peripheral_gsensor_data_register(void)
{
	avi_adc_gsensor_data_register(&gsensor_msgQId0, (INT32U *) (&gsensor_msgId0));
}


void ap_peripheral_read_gsensor(void)
{
	static INT16U	g_idx = 0;
	INT16U			temp;

	temp				= G_sensor_get_int_active();

	if ((temp != 0xff) && (temp & 0x04)) //active int flag
	{
		G_sensor_clear_int_flag();

		if (ap_state_config_G_sensor_get())
		{
			msgQSend(ApQ, MSG_APQ_SOS_KEY_ACTIVE, NULL, NULL, MSG_PRI_NORMAL);
		}

		DBG_PRINT("gsensor int actived\r\n");
	}

	if (gsensor_msgQId0 != NULL)
	{
		G_Sensor_gps_data_get(gsensor_data[g_idx]);
		OSQPost((OS_EVENT *) gsensor_msgQId0, (void *) (gsensor_msgId0 | g_idx));
		g_idx				^= 0x1;
	}

	//DBG_PRINT("gsensor chipid = 0x%x\r\n", temp);
}


void ap_peripheral_config_store(void)
{
	if (config_cnt++ == PERI_COFING_STORE_INTERVAL)
	{
		config_cnt			= 0;
		msgQSend(ApQ, MSG_APQ_USER_CONFIG_STORE, NULL, NULL, MSG_PRI_NORMAL);
	}
}


void ap_peripheral_hdmi_detect(void)
{
	static BOOLEAN	HDMI_StatusBak = 0;
	static BOOLEAN	HDMI_StateBak = 0;				// HDMI_REMOVE
	static unsigned char HDMI_DetCount = 0;
	BOOLEAN 		cur_status;

	cur_status			= gpio_read_io(HDMI_IN_DET);

	// debounce
	if (HDMI_StatusBak != cur_status)
	{
		HDMI_DetCount		= 0;
	}
	else 
	{
		HDMI_DetCount++;
	}

	if (HDMI_DetCount == 0x10)
	{
		if (cur_status != HDMI_StateBak)
		{
			HDMI_DetCount		= 0;

			if (cur_status) // HDM_IN_DET
			{
				msgQSend(ApQ, MSG_APQ_HDMI_PLUG_IN, NULL, NULL, MSG_PRI_NORMAL);
				gpio_write_io(SPEAKER_EN, DATA_LOW); //mute local speaker
				DBG_PRINT("HDMI Insert\r\n");		// HDMI Insert
			}
			else 
			{
				msgQSend(ApQ, MSG_APQ_HDMI_PLUG_OUT, NULL, NULL, MSG_PRI_NORMAL);
				gpio_write_io(SPEAKER_EN, DATA_HIGH);
				DBG_PRINT("HDMI Remove\r\n");		// HDMI Remove
			}
		}

		HDMI_StateBak		= cur_status;
	}

	HDMI_StatusBak		= cur_status;
}


#endif

#ifdef SDC_DETECT_PIN


void ap_peripheral_SDC_detect_init(void)
{

	gpio_init_io(SDC_DETECT_PIN, GPIO_INPUT);
	gpio_set_port_attribute(SDC_DETECT_PIN, ATTRIBUTE_LOW);
	gpio_write_io(SDC_DETECT_PIN, 1);				//pull high
}


INT32S ap_peripheral_SDC_at_plug_OUT_detect()
{
	INT32S			ret;
	BOOLEAN 		cur_status;

	ap_peripheral_SDC_detect_init();
	cur_status			= gpio_read_io(SDC_DETECT_PIN);
	DBG_PRINT("SDC_DETECT_PIN_=%d\r\n", cur_status);

	if (cur_status)
	{ //plug_out
		ret 				= -1;
	}
	else 
	{ //plug_in	
		ret 				= 0;
	}

	return ret;
}



INT32S ap_peripheral_SDC_at_plug_IN_detect()
{
	INT32S			ret;
	BOOLEAN 		cur_status;

	ap_peripheral_SDC_detect_init();
	cur_status			= gpio_read_io(SDC_DETECT_PIN);
	DBG_PRINT("SDC_DETECT_PIN=%d\r\n", cur_status);

	if (cur_status)
	{ //plug_out
		ret 				= -1;
	}
	else 
	{ //plug_in
		ret 				= 0;
	}

	return ret;
}


#endif

