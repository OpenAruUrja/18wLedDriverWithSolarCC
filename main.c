/**
  Generated Main Source File

  Company:
    Microchip Technology Inc.

  File Name:
    main.c

  Summary:
    This is the main file generated using MPLAB® Code Configurator

  Description:
    This header file provides implementations for driver APIs for all modules selected in the GUI.
    Generation Information :
        Product Revision  :  MPLAB® Code Configurator - v2.10.3
        Device            :  PIC16F1713
        Driver Version    :  2.00
    The generated drivers are tested against the following:
        Compiler          :  XC8 v1.34
        MPLAB             :  MPLAB X 2.26
*/

/*
Copyright (c) 2013 - 2015 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
*/

#include "mcc_generated_files/mcc.h"
#include "user.h"
#include "flash.h"

void calibrateBattVoltageReading();


uint16_t iBattVoltage,iThermReading,iSpv,iLedV,iCurChg,iCurrLed;
TIMERS LedShutdownTimer,fltLedShutdownTimer,halfHourTimer_5;
volatile uint16_t BattVoltageGain;
uint16_t LocBattVoltage,LocThermReading,currLedOffset,LocCurrLed,FinalBatteryVoltage,LocSpv,LocCurChg;
uint24_t tmpbv;
SYS_STATUS sysStatus;

/* variables to be used in calibration mode */

#define  calib_calculatedGain  LocThermReading
#define  calib_timer LocCurrLed
#define calib_i LocCurrLed
uint8_t calib_checksum;

uint8_t flashBuff[16];
unsigned int fAdd=0xfe0;
// const  __persistent char vCalibGain[16] @ 0xFF0;// ={0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff};
/*
                         Main application
 */
void main(void)
{
    // initialize the device
 
    InitApp();
    __delay_ms(200);



    /* check the calibration condition and perform calibration if required*/
    calibrateBattVoltageReading();

   
    while (1)
    {
        readAnalogChannels();	// Loads the parameters form interrupt to local var ( int will be dissabled)
        exSystemStates();	// Executes the system states and switch betwwen them
        exLedStates();
//        ExChgStates();
//        ExBattStates();  // cleare sthe battey voltage flag
//        ExTemperature(); // Read temperature and adjust the max charging voltage
        updateLED();
        checkBattChgLevel();
	CLRWDT();
    }
}// main() ends


/*
 * Function: calibrateBattVoltageReading
 * input:   none
 * Fn:    Checks the I/O dedicated for calibration mode selection.
 *        If the I/O is high just return back after loading gains from flash-
 *        checksum is verified while reading the gain. Default value of 2048 will be loaded if checksum is found wrong.
 *        Else perform calibration and wait for re-boot
 *        For calibration: Read the top and mid battery voltage analog channels and calculates the gain
 *        Writes the gain to Flash memory with checksum
 *        Gain - will be verified with upper and lower boundaries. If it is beyond the limit, default of 2048 will be loaded.
 *        Turns on the green LED when done
 *
 * Gain Calculation:
 *      Gain =( Reading obtained from ADC ( 12 bit) * 2048)/ expected reading
 *
 * Corrected reading calculation:
 *      Corrected reading(12 bit) = (ADC reading(12 bit) * 2048)/Gain
 *
 * Calibration note:
 *      For simplicity, the calibration assumes the offset is 0. The only gain will be calculated in the region where the voltage is more critical.
 *      The critical voltage is the final set voltage of the battery. Which is 12V .
 */
void calibrateBattVoltageReading()
{
    /* identify if the calibration mode is selected*/
    /* check LB LED pin connected to RX of serial port... (for calibration mode)*/

    for(calib_i=0;calib_i<200;calib_i++)
     {
        if(sysStatus.Flags.iAdcResultReady==false)continue;
        sysStatus.Flags.iAdcResultReady=false;
        /* toggle LB LED pin high */
        pinLow(pinLowBattLED);
        __delay_ms(1);
        /* check the RX pin if it is low  */
        if(pinRX)break;


        pinHigh(pinLowBattLED);
        __delay_ms(1);
        if(!pinRX)break;
         /* check the RX pin if it is high  */
     }//for(calib_i=0;calib_i<200;calib_i++)

    if(calib_i!=200)
    //      if(calib_i==200)
    {
    /* If calibration mode is not selected then the calibration gain- verify with checksum*/
    FlashReadRow(&fAdd, flashBuff); /* Read the row of 16 byte */
    /* verify the checksum */
    calib_checksum=0;
     for(calib_i=0;calib_i<16;calib_i++)
     {
        calib_checksum+=flashBuff[calib_i];

     }
     if(calib_checksum==0)
     {
        BattVoltageGain=(uint16_t)flashBuff[0]|((uint16_t)flashBuff[1]<<8);
     }
     else
     {
        BattVoltageGain=2048; /* load the default if there was error in checksum */
     }
     return;
    }//if(calib_i!=200)
    /* If calibration mode is selected...*/
    /* indicate it is calibration mode*/
    sysStatus.Flags.calibrationMode=true;

    /* Disable watchdog */
     CLRWDT();
     WDTCONbits.SWDTEN=false;
    calib_timer=200;  /* Count for # of ADC sample to wait before calculation */
     while(calib_timer>0)
     {
         if(sysStatus.Flags.iAdcResultReady==true)
         {
            sysStatus.Flags.iAdcResultReady=false;
            calib_timer--;
         }
     }
     /* now read the battery voltages*/
    LocBattVoltage=iBattVoltage;

    tmpbv=(uint24_t)LocBattVoltage;//<11; /* multiply by 2048 */
    tmpbv<<=11;
    LocBattVoltage=BattCalibVoltage;
    calib_calculatedGain=tmpbv/(uint24_t)LocBattVoltage;

    /* make sure the gain are within the limit (+/-7%)*/
    if(calib_calculatedGain< 1904 || calib_calculatedGain > 2091)
    {
        /* Invalid calibration - load the default value- make LED on and wait here */
        calib_calculatedGain=2048;
        LedOn(LedChg);
        while(1);
    }
  

 FlashEraseRow( &fAdd);
 flashBuff[0]=calib_calculatedGain&0xff;
 flashBuff[1]=(calib_calculatedGain>>8)&0xff;
 BattVoltageGain=calib_calculatedGain;
 calib_checksum=0;
 for(calib_i=0;calib_i<15;calib_i++)
 {
    calib_checksum+=flashBuff[calib_i];

 }
  flashBuff[15]=~calib_checksum;  /* Store checksum */
  flashBuff[15]++;

FlashWriteRow(&fAdd, flashBuff);
while(1)
{
    __delay_ms(200);
    LEDToggle(LedChg);

}

}//calibrateBattVoltageReading()  ends



/**
 End of File
*/