/******************************************************************************/
/*Files to Include                                                            */
/******************************************************************************/

#if defined(__XC)
    #include <xc.h>         /* XC8 General Include File */
#elif defined(HI_TECH_C)
    #include <htc.h>        /* HiTech General Include File */
#endif

#include <stdint.h>         /* For uint8_t definition */
#include <stdbool.h>        /* For true/false definition */
#include "user.h"          /* User funct/params, such as InitApp */
#include "mcc.h"
/******************************************************************************/
/* Interrupt Routines                                                         */
/******************************************************************************/

/* external globals */
extern uint16_t iBattVoltage,iThermReading,iSpv,iLedV,iCurChg,iCurrLed;
extern SYS_STATUS sysStatus;
extern TIMERS halfHourTimer_5;
extern uint16_t BattVoltageGain,midBattVoltageGain,FinalBatteryVoltage;
extern uint24_t tmpbv;
/* smaller than 350nSec or 400nSec on time can be declaired as charged.
 *
 */

/* Baseline devices don't have interrupts. Note that some PIC16's 
 * are baseline devices.  Unfortunately the baseline detection macro is 
 * _PIC12 */
#ifndef _PIC12



//short long int ncoCount=MIN_ON_TIME_COUNT,intTerm=(short long int)MIN_ON_TIME_COUNT*8;// 24 bit signed int
short long int ncoCount=MAX_ON_TIME_COUNT,intTerm=(short long int)MAX_ON_TIME_COUNT*8;// 24 bit signed int
volatile signed int vErr=0,ki=3,kp=2;
uint16_t sec5Counter=0,tempVal;
uint16_t tbattVArr[MOVAVG_FILTER_FACTOR];
uint8_t samCounter=0,i;
long int propTerm=0,integralTerm=0,PWM=0;

int16_t error;

void interrupt isr(void)
{
   //LED_R_SetLow();
       /* Determine which flag generated the interrupt */
   
    if(TMR1IF)                                                                  /* If  timer 1 interrupt                                */
    {
        TMR1IF=0;                                                               /* clear event flag                                     */
        TMR1H=((65536-TIMER1_RELOAD_PERIOD) & 0xff00)>>8;
        TMR1L=(65536-TIMER1_RELOAD_PERIOD)&0xff;                                /* Interrupt period is 5 mSec                           */


     /* PI loop for voltage control of  battery voltage              */
        iLedV=ADC_GetConversion(VLED_AN19);
        tbattVArr[samCounter]=ADC_GetConversion(BATTV_AN11);
       
        samCounter++;
        if(samCounter>=MOVAVG_FILTER_FACTOR)samCounter=0;
        tempVal=0;
        for(i=0;i<MOVAVG_FILTER_FACTOR;i++)tempVal+=tbattVArr[i];
        iBattVoltage=tempVal>>1;                                                /* Just divide by 2 - we want to get 2 bit oversample to make 12 bit */

        /* If not is calibration mode, use calibration value to calculate back the corrected battery volatge */
        if(sysStatus.Flags.calibrationMode==false)
        {
            tmpbv=iBattVoltage;
            tmpbv<<=11; /* multiply by 2048*/
            iBattVoltage=tmpbv/BattVoltageGain;
        }//if(sysStatus.Flags.calibrationMode==false)
                                           


   
    

     if(sysStatus.Var.ledStates==led_on || sysStatus.Var.ledStates==led_rampingUp)
     {


     vErr=iLedV-targetLedVoltage;



     /* Check if slow ramp is necessary */
     if(sysStatus.Flags.slowRamp==true)
     {
         if(vErr<0)vErr=-(MIN_ON_TIME_COUNT-MAX_ON_TIME_COUNT)/20;                                                     /* This value can be changed to change the ramping speed of the , -1 is the min */
         else
         sysStatus.Flags.slowRamp=false;
     }



         intTerm=(intTerm+vErr*20/ki);
         if(intTerm<(short long int)(MAX_ON_TIME_COUNT)*8)                          /* Max On time reached.....*/
         {
             intTerm=(short long int)(MAX_ON_TIME_COUNT)*8;
         }
             if(intTerm>(short long int)(MIN_ON_TIME_COUNT)*8)                      /* min on time reached... */
         {
             intTerm=(short long int)(MIN_ON_TIME_COUNT)*8;
         }
         ncoCount=(intTerm+vErr/kp)/8;
         if(ncoCount<=MAX_ON_TIME_COUNT)
         {
             ncoCount=MAX_ON_TIME_COUNT;
         }
////         else
////             sysStatus.Flags.lessThanMaxOnTimePWM=true;
         if(ncoCount>MIN_ON_TIME_COUNT)ncoCount=MIN_ON_TIME_COUNT;
         NCO1INCU=(ncoCount>>16)&0xff;
         NCO1INCH=(ncoCount>>8)&0xFF;
         NCO1INCL=(ncoCount)&0xFF;                                                  /* update the NCO to change the on time.. */
       
     }

        if(sysStatus.Flags.chargerOn==true)
        {
            error=(int16_t)(FinalBatteryVoltage-iBattVoltage);
            propTerm=error>>1;     // Divided by 512
            integralTerm+=error<<1;
            //if(integralTerm>8388607)integralTerm=8388607;
            if(integralTerm>130944)integralTerm=130944;
            if(integralTerm<0)integralTerm=0;
            PWM=integralTerm>>7;  //Div by 2048
            PWM+=propTerm;
          // if(PWM<950)SystemStatus.Flags.ChgInCV=1;
           if(PWM<0){PWM=0;}
           if(PWM>1023){PWM=1023;}//SystemStatus.Flags.ChgInCV=0;}
           PWM3DCH = (PWM & 0x03FC)>>2;
           PWM3DCL = (PWM & 0x0003)<<6;

        }//if(sysStatus.Flags.chargerOn==true)
        else
        {
            /*make charging off*/
             PWM3DCH = 0;
             PWM3DCL = 0;
        }





     iThermReading=ADC_GetConversion(TEMP_AN13);
     iSpv=ADC_GetConversion(PV_AN12);
     iCurChg=ADC_GetConversion(ICHG_AN3);
     iCurrLed=ADC_GetConversion(10);

     /* if LED current is more than limit just turn off everything */
     if(iCurrLed>=LedCurrentADC_750mA)
     {
        DISABLE_LED_DRIVER();
        sysStatus.Var.ledStates=led_fault;
        
     }//if(iCurrLed>=LedCurrentADC_750mA)
    // iCurrInput=ADC_GetConversion(10);                                           /* input Current channel */
     sysStatus.Flags.iAdcResultReady=true;

/* Update timers*/
   
     /* heck 5 sec counter */
     sec5Counter++;
  //    if(sec5Counter>=10)/* when it is 5 Sec*/
     if(sec5Counter>=1000)/* when it is 5 Sec*/
     {
         sec5Counter=0;
         /* check timers that are meant for 5 sec count */
        
            if(halfHourTimer_5.Flags.timerStarted==true)
            {
                if(halfHourTimer_5.Var.timer!=0)halfHourTimer_5.Var.timer--;
                else
                halfHourTimer_5.Flags.timerElapsed=true;
            }
       
     }


    }//if(TMR1IF)  ends
    else
    {
        /* Err... Unknown interrupt... Reset... what else.... ? */
        RESET();

    }

 //LED_R_SetHigh();

}
#endif


