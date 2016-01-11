/*Auther : Navin Gautam
 *       : 
 * Date  : 9 -July - 2015
 *
 *
 * This file contains the entire application function that are used in the charger
 * 
 *
 *
 */



/******************************************************************************/
/* Files to Include                                                           */
/******************************************************************************/

#if defined(__XC)
    #include <xc.h>         /* XC8 General Include File */
#elif defined(HI_TECH_C)
    #include <htc.h>        /* HiTech General Include File */
#endif

#include <stdint.h>         /* For uint8_t definition */
#include <stdlib.h>
#include <stdbool.h>        /* For true/false definition */

#include "user.h"


#include <xc.h>
#include "mcc.h"

/******************************************************************************/
/* Imported globals                                                           */
/******************************************************************************/

extern SYS_STATUS sysStatus;
extern uint16_t LocBattVoltage,LocThermReading,currLedOffset,LocCurrLed,FinalBatteryVoltage,LocSpv,LocCurChg;
extern uint16_t iBattMidVoltage,iBattVoltage,iThermReading,iCurrLed,iSpv,iCurChg;
extern TIMERS halfHourTimer_5;
//extern int16_t diffVolatge;
extern uint16_t tbattVArr[];

extern short long int ncoCount,intTerm;



extern short long int ncoCount,intTerm;
/******************************************************************************/
/* User Functions                                                             */
/******************************************************************************/
void updateChargingLED();
void updateFaultLED();
void identifyCharger();
void checkCellBalancing();
void  updateDischargingLED();
void assignChgCurrent(uint8_t current);
bool checkTimer(CHARGING_STATES chgState);
void checkBattChgLevel();
/* <Initialize variables in user.h and insert code for user algorithms.> */

/******************************************************************************/
/* Local variables                                                           */
/******************************************************************************/
uint8_t cntCurrLed=0;
uint16_t ledFaultReStartCntr,chargingOffset;

const uint16_t TempReadingTable[14]={914,883,847,806,759,708,653,597,540,484,430,380,333,291};
const uint16_t TempCompValue[15]={
			getBattVoltageCount(135),
			getBattVoltageCount(120),
			getBattVoltageCount(105),
			getBattVoltageCount(90),
			getBattVoltageCount(75),
			getBattVoltageCount(60),
			getBattVoltageCount(45),
			getBattVoltageCount(30),
			getBattVoltageCount(15),
			getBattVoltageCount(0),
			getBattVoltageCount(-15),
			getBattVoltageCount(-30),
			getBattVoltageCount(-45),
			getBattVoltageCount(-60),
			getBattVoltageCount(-75)};


const uint8_t ledItensities[20]={
    100,100, // 1st hour
    100,100, // 2nd Hour
    100,100, // 3rd Hour
    100,100, // 4th Hour
    70,70,    // 5th hour
    70,70,    // 6th hour
    70,70,    // 7th hour
    60,60,    // 8th hour
    60,60,    // 9th hour
    100,100, // 10th hour
};



/* For load compensated battery voltage- used for battery level indication*/
void InitApp(void)
{
                   
////
////    /*
////Configure the I/O pins
//// Setup analog functionality and port direction
////*/
////    	TRISA=0x3E;                                                             /*   0011  RA7-0 (Blue-LED), RA6-0 (Green-LED), RA5-1 (OP_AMP), RA4-1 (OP-AMP)...         */
////                                                                                /*...1010 RA3-1 (CMP), RA2-1 (Comp -in R3), RA1-1 (OPA1OUT), RA0-0 (SHUTDOWN)                         */
////
////	TRISB=0x0F;                                                             /*   0000  RB7-0 (ICSPDAT), RB6-0 (ICSPCLK), RB5-0 (Low Bank Bypass), RB4-0 (Top Bank Bypass)...     */
////                                                                                /*...1111  RB3-1 (CMPin), RB2-1 (OP amp), RB1-1 (OP-Amp), RB0-1 (Batt V)        */
////
////	TRISC=0x7E;                                                             /*   0111 RC7-0 (Switcher Enable), RC6-1 (Therm), RC5-1 (Vmid), RC4-1 (Buton)...        */
////                                                                                /*...1110 RC3-1 (D- Input), RC2-1 (D+ input), RC1-1 (D+ Pull UP), RC0-0 (RED LED)                 */
////
////        TRISD=0x51;                                                             /*   0101 RD7-0 (NC), RD6-1 (Power Good), RD5-0 (+5V EN), RD4-1 (SenseV)...                */
////                                                                                /*...1001 RD3-1 (PWM), RD2-0 (NC), RD1-0 (NC), RD0-1 (D+ pull up)               */
////
////        TRISE=0x0A;                                                             /*   0000 RE7-0 (NA), RE6-0 (NA), RE5-0 (NA), RE4-0 (NA)...                             */
////                                                                                /*...1010 RE3-1 (MCLR), RE2-0 (NC), RE1-1 (Vin), RE0-0 (NC)                    */
////	LATA=0xC0;                                                              /* Make all the output low but LEDs output to make it off */
////	LATB=0x00;                                                              /* Make all the output low */
////	LATC=1;                                                                 /* Make all the output low but LED output to make it off           */
////
////	ANSELA=0x3E;                                                            /*   0011 1010 op-amp,op-amp,CMP,opamp are analog input                                */
////	ANSELB=0x0F;                                                            /*   0000 1111 CMP, op-amp,op-amp, Batt voltage                               */
////        ANSELD=0x11;                                                            /*   0001 0000 Thermister, Vmid in, D- in, D+ in                                 */
////        ANSELE=0x02;                                                            /*   0000 0010 Vin,                                                     */
////        ANSELC=0x64;
////    /* Initialize peripherals                                                   */
////    /* Configure timer 1 to generate interrupt at every 5 mSec                  */
////
////	T1CONbits.TMR1CS=0;                                                     /* Timer1 Clock source Fosc/4 ( for 32MHz FoSC= 8 MHz           */
////	T1CONbits.T1CKPS=3;                                                     /* Prescale 1:8  ( with 32MHz clock, clock for Timer 1 = 1 MHz)  */
////	TMR1H=0xff-(TIMER1_RELOAD_PERIOD & 0xff00)>>8;
////	TMR1L=0xff-TIMER1_RELOAD_PERIOD&0xff;
////	T1CONbits.TMR1ON=1;                                                     /* turn On the timer  */
////	PIE1bits.TMR1IE=1;
////	PIR1bits.TMR1IF=0;
////
////    /* Boost converter setup                                                    */
////    /* Setup MCO as input to CWG
////     * PWM will be setup at 350kHz as fixed off time - this is choosen to suit the inductor on the
////     * circuit which is 4.7uH.
////     */
////////        T2CONbits.T2CKPS=0;                                                     /* Timer 2 Prescalar =1 */
////////	PR2=22;                                                                 /* 8 MHz input to Timer2, with PR2=22 it gives PWM frequency of 347kHz      */
//////////	PWM3DCL=(PWM_MAX_DUTY_CYCLE_VALUE & 0x3)>>6;                                 /* Load PWM duty cycle                                                      */
//////////	PWM3DCH=PWM_MAX_DUTY_CYCLE_VALUE>>2;
////////        /* Start PWM with 0 -*/
////////        PWM3DCL=0;
////////	PWM3DCH=12>>2;
////
////////	PWM3CONbits.PWM3POL =0x0;                                               /* PWM output polarity is active high                                       */
////////	CCPTMRS=0;                                                              /* Select timer 2 as the timer for PWM                                      */
////////        PWM3CONbits.PWM3EN=1;                                                   /* Enable PWM                                               */
////////	T2CONbits.TMR2ON=0;                                                     /* Off timer 2 at start up -- later application will switch on to turn on the boost*/
////
////
////        /* Setup CLC, CMP2, FVR, TIM4 and TIM6 for breathing LED*/
////        CLC1_Initialize();
////        CLC2_Initialize();
////        CLC3_Initialize();
////        TMR4_Initialize();
////        TMR6_Initialize();
////        CMP2_Initialize();
////        FVR_Initialize();
////         /* Setup NCO and COG for PWM   */
////         NCO1_Initialize();
////         COG_Initialize();
////////
////////        ppsUnLock();
////////        RD3PPS=0x09;                                                            /* COG1B is directed to RD3 pin (pin37)     */
////////        ppsLock();
////////        COG1ASD0bits.G1ASE=1;                                                   /* Keep COG disabled - system state will enable it  */
////////
////        /* Configure FVR2 */
////        FVRCON=0x8A;                                                            /* Enable FVR and select 2.048 for DAC buffer*/
////        /* Configure 8 bit DAC */
////        DAC1CON0=0x88;                                                          /* Enabe DAC and select FVR ad vref*/
////        DAC1CON1=LedCurrent_100mA;                                                          /* Select midway--- to start with - must be revised*/
////        /* Configuring the OP-AMP*/
////
////        OPA1CONbits.OPA1UG=0;                                                   /* Select OPA1IN- pin */
////        OPA1CONbits.OPA1PCH=0;                                                  /* Select OPA1IN+ pin */
////        OPA1CONbits.OPA1SP=1;                                                   /* High speed mode  */
////        OPA1CONbits.OPA1EN=1;
////
////        OPA2CONbits.OPA2UG=0;                                                   /* Select OPA2IN- pin */
////        OPA2CONbits.OPA2PCH=2;                                                  /* Select DAC1 output */
////        OPA2CONbits.OPA2SP=1;                                                   /* High speed mode  */
////        OPA2CONbits.OPA2EN=1;
////
////        /* Configure comparator */
////        CM1CON1bits.C1NCH=2;                                                    /* Select C1IN2-*/
////        CM1CON1bits.C1PCH=0;                                                    /* Select C1IN0+-- for Rev 3 PCB  */
////        CM1CON0bits.C1POL=0;                                                    /* Do not Invert the polarity */
////        CM1CON0bits.C1SP=1;                                                     /* High speed*/
////        CM1CON0bits.C1ON=1;
////        ADC_Initialize();
////
////    PIN_MANAGER_Initialize();

    /* Initialize the ADC */
////        /* Initialize system variables */

      OSCILLATOR_Initialize();

    OPA1_Initialize();
    OPA2_Initialize();
    DAC1_Initialize();
    CMP1_Initialize();
    NCO1_Initialize();
    COG_Initialize();
    CLC1_Initialize();
    CLC2_Initialize();
    CLC3_Initialize();

    TMR2_Initialize();
    TMR4_Initialize();
    PWM3_Initialize();
    PWM3_LoadDutyValue(0);  /* Turn off charger */
     PWM4_Initialize();
    FVR_Initialize();


    ADC_Initialize();

    TMR1_Initialize();
    EUSART_Initialize();
    PIN_MANAGER_Initialize();

    /* COG pins port direction is not managed by code generation*/
    TRISC  &= 0x97;  /* makeRC3, RC5 and 6 output*/
    ANSELC &= 0x97;
    TRISA &= ~0x50; // RA4 and RA6- output
    ANSELA &= ~0x50; // RA4- digital

     COG1ASD0bits.G1ASE=1;  
        sysStatus.Var.ChgState=chg_off;
        sysStatus.Var.DisBattState=dis_batt_normal;
        sysStatus.Var.SystemState=sys_BootingUp;
        sysStatus.Var.faults=flt_clear;
        sysStatus.Var.ledStates=led_off;
        sysStatus.Var.FlagByte=0;
        sysStatus.Var.FlagBytes1=0;                                           /* clear all the flags to start with but it can be set as required after..*/
        sysStatus.Var.sysSytateExe=entryTask;
         sysStatus.Var.ledStates=led_off;
        LocBattVoltage=0;
        FinalBatteryVoltage=TARGET_CHG_VOLTAGE;

        tbattVArr[0]=0;
        tbattVArr[1]=0;
        tbattVArr[2]=0;
        tbattVArr[3]=0;

    /* Enable interrupts */
        PEIE=1;                                                                 /* Enable pheripharal interrupt                                     */
	ei();                                                                   /* Enable intrupts                                                  */
        CLRWDT();
////        WDTCONbits.WDTPS=WATCHDOG_128mSec;                                                   /* 128mSec watchdog timer */
////
////        WDTCONbits.SWDTEN=true;                                                 /* enable watchdog */
}

uint24_t avgTemp=0;
uint16_t tempCount=0;

void ExTemperature()
{
    uint8_t i;
	if(sysStatus.Flags.LocAdcResultReady==false)return; // wait for temperature to be ready
	
	avgTemp+=LocThermReading;
	tempCount++;
	if(tempCount<16384)return;
	tempCount=0;
	avgTemp>>=8; // div by 256
	while(avgTemp<TempReadingTable[0] && avgTemp>(TempReadingTable[13]-40))
	{
		for(i=0;i<14;i++)if(avgTemp>TempReadingTable[i])break;
		FinalBatteryVoltage=TARGET_CHG_VOLTAGE+TempCompValue[i];
		avgTemp=0;
		return;

	}// while ends here
	FinalBatteryVoltage=TARGET_CHG_VOLTAGE;
	avgTemp=0;
	return;

}// ExTemperature() ends here



/*
 * Function: updaetLED
 * input:   none
 * Fn:    Updates the red,green and blue LED as required based on the system state
 */
void updateLED()
{
     if(sysStatus.Flags.lowBattFlag==true && sysStatus.Flags.deadBattFlag==false)
     {
        LedOn(pinLowBattLED);
     }
     else
         LedOff(pinLowBattLED);

     if(sysStatus.Flags.chgLedON)
     {
         LedOn(LedChg);
     }
     else
     {
         LedOff(LedChg);
     }


}// UpdateLED() ends


/*
 * Fn Name exGsmStates()
 * Return: None
 * Parameter: None
 * Task: It moves the GSM state machine and does the task as required.
 *       GSM_OFF to GSM_PREPARE must be done by some other process. Once it is in
 *       GSM_OFF, it doe not advance by itself
 */
void exGsmStates()
{
    switch(sysStatus.Var.SystemState)
    {
        case GSM_OFF:
        {
            /* make GSM power OFF*/


            /* make serial peripheral OFF to save power */

            
            break;
        }//case GSM_OFF:
        case GSM_PREPARE_TO_ON:
        {
         /* Turn the power supply ON */

         /* turn the serial peripheral ON*/

        }//case GSM_PREPARE_TO_ON:

    } // switch(sysStatus.Var.SystemState)

/* if charger is on check the battery voltage and operate the PWM as required*/


}// exChargingStates() ends




uint8_t setCur=0,halfHourCounter=0;
uint16_t tmp=0;
void exLedStates()
{
    switch(sysStatus.Var.ledStates)
    {
        case led_off:
        {
           DISABLE_LED_DRIVER();
           cntCurrLed=0;
           currLedOffset=0;
          ncoCount=MAX_ON_TIME_COUNT;
         intTerm=(short long int)MAX_ON_TIME_COUNT*8;
         setLedCurrent(0);  /* This is required to avoid the kick when LED is restarted. */
           break;
        }// case led_off ends
        case led_prepareToOn:
        {
            if(sysStatus.Flags.LocAdcResultReady==false)break;
            currLedOffset+=LocCurrLed;
            cntCurrLed++;
            if(cntCurrLed<32)break;
            currLedOffset>>=7;
            setLedCurrent(2);
            ENABLE_LED_DRIVER();
            tmp=LedCurrent_500mA+currLedOffset;
            if(tmp>255)tmp=255;
            sysStatus.Var.maxLedCurrent=tmp;
            sysStatus.Flags.slowRamp=true;
            sysStatus.Var.presentSetLedCurrent=((uint16_t)sysStatus.Var.maxLedCurrent*ledItensities[halfHourCounter])/100;
          

            setCur=2;
            sysStatus.Var.ledStates=led_rampingUp;
          /* reset Half hour timer counter */
             halfHourTimer_5.Flags.timerElapsed=false;
             halfHourTimer_5.Var.timer=HALF_HOUR_TIME_5;
             halfHourTimer_5.Flags.timerStarted=true;
             halfHourCounter=0;
            break;
        }//case led_prepareToOn ends
        case led_rampingUp:
        {
           if(sysStatus.Flags.LocAdcResultReady==false)break;
           /* ramp up the current if set current is less*/
           if(setCur<(sysStatus.Var.presentSetLedCurrent))
           {
            setCur+=1;
            setLedCurrent(setCur);
           }
           else
           {
              sysStatus.Var.ledStates=led_on;
           }
            break;
        }// case led_rampingUp ends
        case led_on:
        {
       /* check the timer and set the MAX LED current as required*/
           if(halfHourTimer_5.Flags.timerElapsed==true)
           {
             halfHourTimer_5.Flags.timerStarted=false;
             halfHourTimer_5.Flags.timerElapsed=false;
             halfHourTimer_5.Var.timer=HALF_HOUR_TIME_5;
             halfHourTimer_5.Flags.timerStarted=true;
             halfHourCounter++;
             if(halfHourCounter>=20)halfHourCounter--;  /* max table is for 10 Hrs. */
             sysStatus.Var.presentSetLedCurrent=((uint16_t)sysStatus.Var.maxLedCurrent*ledItensities[halfHourCounter])/100;
             setLedCurrent(sysStatus.Var.presentSetLedCurrent);
           }

            
            break;
        }// case led_on ends
        case led_fault:
        {
           DISABLE_LED_DRIVER();
           break;
        }// case led_fault ends

    }// switch sysStatus.Var.ledStates ends


}//void exLedStates() ends


/*
 * Function: exSystemStates
 * input:   none
 * Fn:    walks through the system sates as required, performs task as needed
 */
int8_t DaDuTraCounter=0,dayTraCounter=0,chgCuroffCntr=0;;
void  exSystemStates()
{
    switch(sysStatus.Var.SystemState)
    {
        case sys_BootingUp:
        {

////---- Nothing to do here -- state will be altered by  VinButtonMonitor routine.
            sysStatus.Var.SystemState=sys_night;
            //sysStatus.Var.SystemState=sys_waiting_for_night;
            sysStatus.Var.sysSytateExe=entryTask;

            break;
        }//case sys_BootingUp ends
        case sys_waiting_for_night:
        {
            /* During discharging, turn on the switcher and wait 10 minutes then go to sleep
           
           
             * */
            /* entry task */
            switch(sysStatus.Var.sysSytateExe)
            {
                case entryTask:
                {


                     if(sysStatus.Flags.LocAdcResultReady==false)break;
                     tmp++;
                     if(tmp>50)
                     {
                         //tmp=0;
                         PWM4_LoadDutyValue(0);

                     }
                     if(tmp>100)
                     {
                         tmp=0;
                         PWM4_LoadDutyValue(511);
                     }
                    
                  

                    break;
                }
                case bodyTask:
                {


                    break;
                }
            
            }

            
            break;
        }//case discharging ends
        case sys_night:
        {
           
             switch(sysStatus.Var.sysSytateExe)
            {
                case entryTask:
                {
                    
                    sysStatus.Var.sysSytateExe=bodyTask;
                    /* turn off the charging MOSFET*/

                    /* make sure diode is off too... */
                     DISABLE_DIODE();
                      sysStatus.Flags.chgLedON=false;

                   

                    break;
                }//case entry task ends
                 case bodyTask:
                 {
                 /* check SPV voltage to transit to day mode*/
                     if(LocSpv>dawnOffThreshold)
                     {
                         DaDuTraCounter++;
                         if(DaDuTraCounter>20)
                         {
                             DaDuTraCounter--;
                             /*Turn off LED*/
                             sysStatus.Var.ledStates=led_off;
                             sysStatus.Var.sysSytateExe=entryTask;
                             sysStatus.Var.SystemState=sys_day_night_transition;
                             /* Inform to next stage to take charging current reading for its offet. ... at this stage the charger is still OFF*/
                             sysStatus.Flags.takeChargingOffset=true;
                             chgCuroffCntr=0;
                             break;
                         }
                         
                     }//if(LocSpv>dawnOffThreshold) ends
                     else
                       DaDuTraCounter=0;

                /* check low battery flag and operate LED as required*/
                if(sysStatus.Flags.lowBattFlag==true)
                {
                    if(sysStatus.Var.ledStates!=led_fault)sysStatus.Var.ledStates=led_off;
                }
                else
                {
                    if(sysStatus.Var.ledStates==led_off)sysStatus.Var.ledStates=led_prepareToOn;
                    /* handle if the LED is on fault condition*/
                    if(sysStatus.Var.ledStates==led_fault)
                    {
                        ledFaultReStartCntr++;
                        if(ledFaultReStartCntr>1000)  /* wait for 5 seconds */
                          sysStatus.Var.ledStates=led_off;
                    }
                    else
                        ledFaultReStartCntr=0;
                }
                   
                break;
                }// case bosyTask
              
             }//switch(sysStatus.Var.sysSytateExe) for night mode ends


             break;
        }//case sys_night

       
         case sys_day_night_transition:/* noght mode over but waiting for full sunshine */
                 {
                    switch(sysStatus.Var.sysSytateExe)
                    {
                    case entryTask:
                        {
                            /* Turn off the diode MOFET but turn on the charging */
                         DISABLE_DIODE();
                         /* calculate charging current offset before turning the charger ON*/
                         if(sysStatus.Flags.takeChargingOffset==true)
                         {
                             chgCuroffCntr++;
                             if(chgCuroffCntr<30)break;
                             chgCuroffCntr=0;
                             chargingOffset=LocCurChg;
                             sysStatus.Flags.takeChargingOffset=false;

                         }//if(sysStatus.Flags.takeChargingOffset==true)


                         //PWM3_LoadDutyValue(1023);/* 100% on */
                         sysStatus.Flags.chargerOn=true;
                         /* now transit to body task and wait there */
                         sysStatus.Var.sysSytateExe=bodyTask;

                            break;
                        }//charging entry task ends
                     case   bodyTask:
                     {
                        /* monitor SPV voltage - it might go back to night mode  too*/
                           if(LocSpv<duskOnThreshold)
                         {
                             DaDuTraCounter--;
                             if(DaDuTraCounter<-20)
                             {
                                 DaDuTraCounter++;
                                 /*Turn off LED*/
                                 sysStatus.Var.sysSytateExe=entryTask;
                                 sysStatus.Var.SystemState=sys_night;
                                 sysStatus.Flags.chargerOn=false;
                                 
                                 break;
                             }
                            
                     }//if(LocSpv<duskOnThreshold) ends
                    else
                    DaDuTraCounter=0;

                    /* monitor the charging current- if current is detected then turn on diode and move to day mode*/

                           if(LocCurChg>=(chargingOffset+diodeOnCurrent))
                           {
                              dayTraCounter++;
                              if(dayTraCounter>50)
                              {
                                  dayTraCounter--;
                                  sysStatus.Var.sysSytateExe=entryTask;
                                  sysStatus.Var.SystemState=sys_day;
                                  ENABLE_DIODE();
                                  break;
                              }
                           }
                           else
                               dayTraCounter=0;

                            if(LocCurChg>=(chargingOffset+4))
                            {
                                sysStatus.Flags.chgLedON=true;
                            }
                            else
                                sysStatus.Flags.chgLedON=false;

                         
                         break;
                     }//case   bodyTask:
                    }//switch(sysStatus.Var.sysSytateExe)



                    break;
                    }//case sys_waiting_for_day:
         /* system is in day mode- charging  with diode ON*/
        case sys_day:
        {
                switch(sysStatus.Var.sysSytateExe)
                {
                    case entryTask:
                        {
                            dayTraCounter=0;
                            sysStatus.Var.sysSytateExe=bodyTask;
                             sysStatus.Flags.chgLedON=true;
                            break;
                        }//charging entry task ends
                     case   bodyTask:
                     {

                        if(LocCurChg<=(chargingOffset+diodeOffCurrent))
                        {
                            dayTraCounter--;
                            if(dayTraCounter<-5)
                            {
                                DISABLE_DIODE();
                                sysStatus.Var.sysSytateExe=entryTask;
                                sysStatus.Var.SystemState=sys_day_night_transition;
                                dayTraCounter=0;
                                break;
                            }
                        }//if(LocCurChg<=(chargingOffset+diodeOffCurrent))
                        else
                          dayTraCounter=0;


                         break;
                     }//case   bodyTask:
                }
        }
        case sys_sleep:
        {

            break;
        }//sys_sleep:
        default:
        {

         }// default
    

    }//switch(sysStatus.Var.SystemState)
}

/*
 * Function: checkBattChgLevel
 * input:   None
 * Fn:    checks the battery volatge and sets the discharge LED states as required
 *        wait for about 300mSec before declaring the change in battery status
 *        
 *
 */

int8_t lowBattCounter=0,deadBattCounter=0;
void checkBattChgLevel()
{
    if(sysStatus.Flags.LocAdcResultReady==false)return;
    if(sysStatus.Flags.lowBattFlag==true)
    {
        if(LocBattVoltage>LOW_BATT_RESET_VOLTAGE)
        {
            lowBattCounter--;
            if(lowBattCounter<-20)
            {
                lowBattCounter++;
                sysStatus.Flags.deadBattFlag=false;
                sysStatus.Flags.lowBattFlag=false;
            }
        }

    }
    else  // If not low battery
    {
       if(LocBattVoltage<LOW_BATT_THRESHOLD)
       {
           if(LocBattVoltage<DEAD_BATT_THRESHOLD)
           {
               deadBattCounter++;
           }
           else
           {
              deadBattCounter=0;
           }
           lowBattCounter++;
           if(lowBattCounter>20)
           {
            lowBattCounter--;
            sysStatus.Flags.lowBattFlag=true;
           }
           if(deadBattCounter>20)
           {
                deadBattCounter--;
                sysStatus.Flags.deadBattFlag=true;
                sysStatus.Flags.lowBattFlag=true;
           }
       }
       else// if battery is well above low battery threshold
       {
          deadBattCounter=0;
          lowBattCounter=0;
       }

    }

    

}//checkBattChgLevel() ends







/*
 * Function: monitorVinButton
 * input:   none
 * Fn:    This function monitors the changes in Vin and button state and alters the systemStates and Charging state accordingly
 *
 *
 */
uint8_t buttonPressed=false;
void  monitorSPV()
{


    
}// monitorSPV(); ends








/*
 * Function: readAnalogChannels
 * input:   none
 * Fn:    This function copies analog ch reading from interrupt routine to local variables ans set flags
 *          sets
 */
void readAnalogChannels()
{
    sysStatus.Flags.LocAdcResultReady=false;                            /* Invalidate flags to get new status */
     di();
    if(sysStatus.Flags.iAdcResultReady==true)
    {
       
        sysStatus.Flags.iAdcResultReady=false;
        LocBattVoltage=iBattVoltage;
        LocThermReading=iThermReading;
        LocCurrLed=iCurrLed;
        LocSpv=iSpv;
        LocCurChg=iCurChg;
    
     //   LocCurrInput=iCurrInput;
        sysStatus.Flags.LocAdcResultReady=true;
         
    }
       ei();
////       if(LocSense<outputCurrentOffset)
////       LocSense=0;
////       else
////       LocSense=LocSense-outputCurrentOffset;
     /* find the difference between the top and bottom */
////     diffVolatge=((int16_t)(LocTopBattVoltage)-(int16_t)LocBattMidVoltage);
////     loadCompensatedBattV = LocBattVoltage + ((LocSense<<2)/((opCurrentGain* 4) /7));
 
}

