/******************************************************************************/
/* User Level #define Macros                                                  */
/******************************************************************************/
/* The following definition is based on Rev 2 schematic dated 2/12/2015       */
/* input I/o pins definition ------------------------------------------------ */


#define ADC_REFERENCE 2048L

/* Output I/o pins definition ----------------------------------------------- */

#define pinLowBattLED RA6
#define LedLowBatt LATA6
#define pinRX RC1
#define pinChgLED RC3
#define LedChg LATC3

#define LedOn(x) x=1;                                                           /* Macro to On LED                                              */
#define LedOff(x) x=0;                                                          /* Macro to off LED                                             */
#define LEDToggle(x) x=~x;
#define pinLow(x) x=0;
#define pinHigh(x) x=1;





/* Analog ADC channels -------------------------------------------------------*/
#define chBatteryTop 12
#define chBatteryMid 17
#define chTherm 18
#define chDisCurrent 24
#define chVin 6


#define MOVAVG_FILTER_FACTOR 8 /* Battery voltage reading low pass filter */


/* Watchdog feed values */
#define WATCHDOG_128mSec 0x7
#define WATCHDOG_1Sec 0xA

#define ENABLE_LED_DRIVER() {COG1STRbits.G1STRB=1;COG1STRbits.G1STRA=1;}
#define DISABLE_LED_DRIVER() {COG1STRbits.G1STRB=0;COG1STRbits.G1STRA=0;}

#define ENABLE_DIODE()  {PWM4_LoadDutyValue(511);}
#define DISABLE_DIODE()  {PWM4_LoadDutyValue(0);}

/* Constant for timings */
#define TIMER1_RELOAD_PERIOD 5000                                               /* 5 mSec timer 1 reload period                  */
#define TIMER1_RELOAD_AFTERSLEEP 3                                              /* 3 usec timer 1 reload period - to be used after wake from sleep in discharge mode                  */
/* x is in mSec- gives count timer 1 interrupt  */
#define TimeCount(x) (x/5)


#define POWER_GOOD_FAIL_TIME TimeCount(50)                                      /* Power good will be declared failed if it is fail for more than 10 X 5 mSec */
#define OP_OVER_CURRENT_TIME TimeCount(75)                                      /* 15 X 5 -> 75 mSec before declaring output overcurrent */

#define DISCHG_SLEEP_TIME TimeCount(1000)                                       /*  1000mSec - time to sleep in discharge mode*/
#define LOW_BATT_COUNTER 5                                                      /* battery must have low voltage for 5 consecutive count in order to declare low batt*/
#define OV_VOLTAGE_COUNTER 10                                                    /* Number of samples before declairing over voltage */
#define CHG_STATE_TRAN_COUNT 100                                                /* Wait 500 msec to declare the state transition */
#define WAIT_BEFORE_CHARGING 250                                                /* Wait 250 X 5->  1.25 Sec before stating charging  - 255 will disable the wait */
#define CHARGED_TO_OFF_TIME TimeCount(10000)                                    /* After 10 Sec system will be turned off when battery iss charged  */
#define FAULT_TO_OFF_TIME TimeCount(10000)                                      /* After 10 Sec system will be turned off after any fault */
#define BAL_WITH_CHG_OFF_TIMEOUT TimeCount(60000)                               /* 60 Sec max time the charger will be off for balancing ... if it expires the battrey is declared faulty */




#define HALF_HOUR_TIME_5 1800/5                                                 /* half an hour timer constant */



/*  on time calculation-
 * Count= 1048576/32/(Ton+2)
 * Here 1048576 = 2^20
 *      Ton is in uSec
 * The modified formula Count= 32768000/(Ton+2000)- Where Ton is in nSec
 */

#define MIN_ON_TIME 70                                                         /* 70 nSec is min ON time-- this gives 7.8V at 5V input   */
#define MAX_ON_TIME 3000                                                        /* 2uSec is the max ON time */
#define MIN_ON_TIME_COUNT 0x003DD5
//#define MAX_ON_TIME_COUNT 0x002000
//#define MAX_ON_TIME_COUNT 0x001C71 // for 2.5 uSec
//#define MAX_ON_TIME_COUNT 0x001770 // for 3.46 uSec
//#define MAX_ON_TIME_COUNT 0x001388 // for 4.55 uSec
#define MAX_ON_TIME_COUNT 0xF00 // for 6.1 uSec
#define MIDWAY_ON_TIME_COUNT 0x002F68                                           /* it is count for 700 nsec on time - shoule be changed to identify the CV2 stage */






/* Macros definition */
/* To lock and unlock the PP sel */
#define ppsLock() do{\
        di();\
        PPSLOCK=0x55;\
        PPSLOCK=0xAA;\
        PPSLOCKbits.PPSLOCKED=1;\
        ei();\
        }while(0);

#define ppsUnLock() do{\
        di();\
        PPSLOCK=0x55;\
        PPSLOCK=0xAA;\
        PPSLOCKbits.PPSLOCKED=0;\
        ei();\
        }while(0);

#define LED_PULSING 0x4
#define LED_PORT 0x0

/* Macro to set the charging current */
/* Input is the DAC count - not the current  */
#define setLedCurrent(x) do{DAC1CON1=x;}while(0);
#define getLedCurrentSetting() DAC1CON1

/* Ananlog channel scaling ----------------------------------------------------*/
/* LED voltage reading*/
#define LEDRU 1000  /* 100k*/
#define LEDRL 33    /* 3k3 */

/* input x is in mV */
#define getLedVoltageCount(x) ((x * 1024L * LEDRL/(LEDRL+LEDRU))/ADC_REFERENCE)          /* calculation for LED voltage */

#define targetLedVoltage getLedVoltageCount(52000)                              /* 52V=38V +14V is the max limit of the */
#define ledOverVoltage getLedVoltageCount(58000)                                /* 58V is the over voltage limit */

/* Battery voltage analog channel */
#define BVRU 100                                                                /* 120k is the higher resistance                                    */
#define BVRL 10                                                                /* 100k is the lower resistance                                     */

/* input x is in mV */
#define getBattVoltageCount(x) ((x * 1024L * BVRL/(BVRL+BVRU))/ADC_REFERENCE)* 4L          /* calculation with 2 bit oversampling */

#define BattCalibVoltage getBattVoltageCount(12000)                           /* Top battery calibration voltage */

#define TARGET_CHG_VOLTAGE getBattVoltageCount(14200)                       // 14.2V is typical charging final voltage
#define HighBattVoltage_8V6 getBattVoltageCount(1600)
#define LOW_BATT_THRESHOLD getBattVoltageCount(11200)                           /* 11.2V is the low battery threshold */
#define DEAD_BATT_THRESHOLD getBattVoltageCount(10000)                          /* 10V is the dead battery threshold */
#define LOW_BATT_RESET_VOLTAGE getBattVoltageCount(12400)                       /* 12.4V is the low battery reset threshold */





/* SPV voltage analog channel     */
#define PVRU 1000                                                               /* 100k is the upper resistance */
#define PVRL 33                                                                 /* 3k3 is the lower resistance */
#define PVRLL 10                                                                /* 1k */
/* input x is in mV */
#define getPvVoltageCount(x) ((x * 1024L*PVRL/(PVRL+PVRU))/ADC_REFERENCE)
#define getPvVoltageChgOffCount(x) ((x * 1024L*PVRL/(PVRL+PVRU+PVRLL))/ADC_REFERENCE)

#define dawnOffThreshold getPvVoltageChgOffCount(3000)                          /* 3.0V is the minimum acceptable input-- 4V is necessary to accomodate cable drop in 2.4A charger  */
#define duskOnThreshold getPvVoltageCount(1500)                                 /* 1.5V is the maximum acceptable input */


/* LED current analog  channel scaling */
#define LEDShunt (uint32_t)25                                                   /* input current sensing shunt  25mOhms                             */
#define LedCurrentGain (uint32_t)100                                            /* Gain is 100 - 10( op-amp) X 10( current sense)          */
#define DAC1VoltageRef (uint32_t)2048                                           /* 2.048 is the DAC1 voltage reference                              */


                                                                                /* x is the current in mA- it calculates the count needed to set... */
                                                                                /* ... at DAC1 to get that current*/
#define calcLedCurrCount(x) ((((((uint32_t)(x)*LEDShunt*LedCurrentGain))*(uint32_t)256U)/DAC1VoltageRef)/(uint32_t)1000U)
#define limit256(X) ((X >255) ? 255 : X)
#define getLedCurrCount(x) calcLedCurrCount(x)


#define LedCurrent_10mA   getLedCurrCount(10)
#define LedCurrent_100mA  getLedCurrCount(100)
#define LedCurrent_200mA  getLedCurrCount(200)
#define LedCurrent_350mA  getLedCurrCount(350)
#define LedCurrent_500mA  getLedCurrCount(500)
#define LedCurrent_700mA  getLedCurrCount(700)



/* Input current reading through ADC*/
#define calcLedCurrAdcCount(x) (((uint32_t)x*LEDShunt*LedCurrentGain*(uint32_t)1024U)/(uint32_t)ADC_REFERENCE/(uint32_t)1000U)

#define LedCurrentADC_90mA calcLedCurrAdcCount(90)                             /* 90mA in ADC count */
#define LedCurrentADC_100mA calcLedCurrAdcCount(100)                             /* 100mA in ADC count */
#define LedCurrentADC_750mA calcLedCurrAdcCount(750)                            /* Max LED current for over current */



/* Discharge current- SENSE analog chanel scaling */
#define outputShunt (uint32_t)12                                                /* input current sensing shunt  25mOhms                             */
#define opCurrentGain (uint32_t)10                                              /* with current resistors                                           */
                                                                               
/* x is in mA -- max is 5800mA  */
#define calcChgCurrCount(x) (((uint32_t)x*outputShunt*opCurrentGain*(uint32_t)1024U)/(uint32_t)ADC_REFERENCE/(uint32_t)1000U)

#define diodeOnCurrent calcChgCurrCount(200)                                    /* 40mA min output current required to hold the switcher in ON condition */
#define diodeOffCurrent calcChgCurrCount(100)

/* Tasks within in each state */
typedef enum
{
    entryTask=0,
    bodyTask=1,
            bodyTask_1=2,
    exitTask=3,
}STATE_TASK;

/* Main ststem  states */
typedef enum
{
    sys_BootingUp=0,
    sys_waiting_for_night=1,
    sys_night=2,
    sys_day_night_transition=3,
    sys_day=4,
    sys_sleep=5,
    sys_lowBatt=6,
}SYSTEM_STATES;


/* Led states*/
typedef enum
{
    led_off=0,
    led_prepareToOn=1,
    led_rampingUp=2,
    led_on=3,
    led_fault=4,

}LED_STATES;


/* charging states */
typedef enum
{
    chg_off=0,
    chg_Trickle=1,
    chg_CC=2,
    chg_CV1=3,
    chg_CV2=4,
    chg_Charged=5,
}CHARGING_STATES;

/* discharging battery states */
typedef enum
{
    dis_batt_normal=0,
    dis_batt_lv_flashing,
    dis_batt_lv_cutoff,
    
}DISCHARGING_BATT_STATES;

typedef enum
{
 GSM_OFF=0,
         GSM_PREPARE_TO_ON,
         GSM_WAITING_GSM_READY,
         GSM_TRANSMIT,
         GSM_WAITING_TX_OVER,
         GSM_TURNING_OFF
}GSM_STATES;

/* Balancing task states */
////typedef enum
////{
////    bal_off=0,
////    bal_with_chg_on=1,
////    bal_with_chg_off=2,
////}BALANCING_STATES;

/* charger identification states */
////typedef enum
////{
////    chgID_setup_input=0,
////    chgID_measureV75=1,
////    chgID_setup_1V=2,
////    chgID_measure1V=3,
////}CHARGER_ID_STATES;


/* Faults definitions */
typedef enum
{
   flt_clear=0,
   flt_battery_Under_Voltage=0x1,
   flt_battery_Over_Voltage=0x2,
   flt_discharge_Over_Current=0x4,
   flt_cells_not_charging=0x8,
   flt_smps_pgood_bad=0x10,
   flt_vin_low=0x20,
   flt_vin_high=0x40,
   flt_large_diff=0x80,

}FAULTS;

typedef union Sys_Status{
	struct bits{
	unsigned int iAdcResultReady:1;                                         /* result ready flag to be indicated form interrupt routine */
        unsigned int LocAdcResultReady:1;                                       /* result ready flag to be indicated form local routine */
        unsigned int slowRamp:1;                                                /* To indicate slow ramp of charging current */
        unsigned int lowBattFlag:1;
        unsigned int deadBattFlag:1;
        unsigned int chargerOn:1;
        unsigned int calibrationMode:1;
        unsigned int takeChargingOffset:1;
        unsigned int chgLedON:1;
                                         
	}Flags;
	struct var{
		unsigned char FlagByte;                                         /* reserve  for flags */
		unsigned char FlagBytes1;                                       /* reserve  for flags */
                SYSTEM_STATES SystemState;                                      /* System states */
		CHARGING_STATES ChgState;                                       /* Charging state machines */
		DISCHARGING_BATT_STATES DisBattState;                             /* Discharging battery state machines */
                LED_STATES ledStates;
                unsigned char faults;                                           /* To register faults */
                STATE_TASK sysSytateExe;                                        /* To indicate it is time for entry task execution */
                uint8_t maxLedCurrent;                                     /* System maximum LED current */
                uint8_t presentSetLedCurrent;
                GSM_STATES gsmStates;                                       /* GSM state machine */
                STATE_TASK gsmStatesExe;                                    /* For GSM state machine internal state - enter, body , exit... */


	}Var;
}SYS_STATUS;

typedef union timers_str{
    struct timer_bits{
       uint8_t timerStarted:1;
       uint8_t timerElapsed:1;
    }Flags;
    struct timer_var{
      uint8_t flag;
      uint16_t timer;
    }Var;
}TIMERS;


/******************************************************************************/
/* User Function Prototypes                                                   */
/******************************************************************************/

void InitApp(void);         /* I/O and Peripheral Initialization */
void updateLED(void);       /* Function that updates the LED    */
void  exSystemStates(void);
void readAnalogChannels(void);
void  monitorVinButton(void);
void checkCellBalancing(void);
void checkBattChgLevel(void);
void exLedStates(void);