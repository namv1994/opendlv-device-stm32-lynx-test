/*
    ChibiOS/RT - Copyright (C) 2006-2013 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/
#include "DiscoveryBoard.h"

/*===========================================================================*/
/* ADC related stuff.                                                        */
/*===========================================================================*/
static void adccb(ADCDriver *adcp, adcsample_t *buffer, size_t n);
/* Total number of channels to be sampled by a single ADC operation.*/
#define ADC_GRP1_NUM_CHANNELS   2
/* Depth of the conversion buffer, channels are sampled four times each.*/
#define ADC_GRP1_BUF_DEPTH      4
/*
 * ADC samples buffer.
 */
static adcsample_t samples[ADC_GRP1_NUM_CHANNELS * ADC_GRP1_BUF_DEPTH];
/*
 * ADC conversion group.
 * Mode:        Linear buffer, 4 samples of 2 channels, SW triggered.
 * Channels:    IN11   (48 cycles sample time)
 *              Sensor (192 cycles sample time)
 */
static const ADCConversionGroup adcgrpcfg = {
  FALSE, //FALSE is linear, TRUE is circular
  ADC_GRP1_NUM_CHANNELS,
  adccb, //callback
  NULL,
  /* HW dependent part.*/
  0, //CR1
  ADC_CR2_SWSTART, //CR2
  ADC_SMPR1_SMP_AN11(ADC_SAMPLE_56) | ADC_SMPR1_SMP_SENSOR(ADC_SAMPLE_56),
  0,
  ADC_SQR1_NUM_CH(ADC_GRP1_NUM_CHANNELS),
  0,
  ADC_SQR3_SQ2_N(ADC_CHANNEL_IN11) | ADC_SQR3_SQ1_N(ADC_CHANNEL_SENSOR)
};

/*
 * ADC end conversion callback.
 * The PWM channels are reprogrammed using the latest ADC samples.
 * The latest samples are transmitted into a single SPI transaction.
 */
void adccb(ADCDriver *adcp, adcsample_t *buffer, size_t n) {

  (void) buffer; (void) n;
  /* Note, only in the ADC_COMPLETE state because the ADC driver fires an
     intermediate callback when the buffer is half full.*/
  if (adcp->state == ADC_COMPLETE) {
    adcsample_t avg_ch1, avg_ch2;
    /* Calculates the average values from the ADC samples.*/
    avg_ch1 = (samples[0] + samples[2] + samples[4] + samples[6]) / 4;
    avg_ch2 = (samples[1] + samples[3] + samples[5] + samples[7]) / 4;
  }
}

/*===========================================================================*/
/* Generic code.                                                             */
/*===========================================================================*/
systime_t timeOut = MS2ST(10); // Receiving and sending timeout is 10ms.
// Custom function: read until reach \n
size_t sdGetLine(SerialDriver *sdp, uint8_t *buf) {
  size_t n;
  uint8_t c;

  n = 0;
  do {
    c = sdGet(sdp);
    *buf++ = c;
    n++;
  } while (c != '\n');
  *buf = 0;
  return n;
}

// Custom function: read until reach \n
size_t sdPutLine(SerialDriver *sdp, uint8_t *buf) {
  size_t n;
  uint8_t c = buf;

  n = 0;
  do {
    sdWrite(sdp, c, 1);
    c++;
    n++;
  } while (c != '\n');
  *buf = 0;
  return n;
}

//Red LED blinker thread, times are in milliseconds.
static WORKING_AREA(usbThreadWA, 128);
static msg_t usbThread(void *arg) {
  (void)arg;
  chRegSetThreadName("usb");
  while (TRUE) {
    systime_t time;
		if(isUSBActive())
    	time=125;
    else
    	time=500;
    palClearPad(GPIOD, GPIOD_LED3);
    chThdSleepMilliseconds(time);
    palSetPad(GPIOD, GPIOD_LED3);
    chThdSleepMilliseconds(time);
  }
}

//Read thread, receive command from docker to turn on/off green led
char bufferRead[64];
bool readOK = false;
static WORKING_AREA(readThreadWA, 128);
static msg_t readThread(void *arg) {
  chRegSetThreadName("read thread");
  while (TRUE) {
  
  	//read data
  	
  	//int bytesRead = sdRead(&SDU1, (uint8_t*)bufferRead, 3);
  	int bytesRead = sdGetLine(&SDU1, (uint8_t*)bufferRead);
  	chThdSleepMilliseconds(1);
  	//set flag for done reading
  	readOK = true;
  	if(bytesRead > 0){
  	//toggle LED4
  		palSetPad(GPIOD, GPIOD_LED4);
  	}
  	else
    	palClearPad(GPIOD, GPIOD_LED4);
  }
}

//WRITE THREAD
bool blinkLedBlue = false;
static WORKING_AREA(writeThreadArea, 128);
static msg_t writeThread(void *arg) {

  (void)arg;
  chRegSetThreadName("write thread");
  while (TRUE) {
  	//CONSTANTLY SEND AIN AND GPIO-0 READINGS TO DOCKER SIDE
  	//------------------------------SYNC THIS CODE WITH MAX--------------------------------------
 /* Currently assumed payload as: "<type: 1 byte>.<pin: 1 byte>.<value: 4bytes><\n>
 - exp: "a.0.1030\n" (raw analog reading from pin 0 (m_analogPinSteerPosition) with value of 1030
 - exp: "d.112.1\n" (digital (GPIO-I) reading from pin 112 (ClampSensor) with value of 1
 - each message has 9 bytes (6 payload + 2 seperator + 1 EOL), and is seperated by a EOL character \n
 */		
  		//char bufferWrite[9] = "";
  		//ANALOG DATA
  		uint32_t raw = (uint32_t)(samples[1] + samples[3] + samples[5] + samples[7]) / 4; //PC1
  		char rawString[4];
  		itoa(raw, rawString, 10);
  		/*
  		strcpy(bufferWrite,'a');
  		strcpy(bufferWrite,'.');
  		strcpy(bufferWrite,'0');
  		strcpy(bufferWrite,'.');
  		strcpy(bufferWrite,rawString);
  		strcpy(bufferWrite,'\n');
  		*/
  		int pin = 1;
  		int value = raw;
  		char bufferWrite[9] = "a.1.1234\n";
  		bufferWrite[4] = rawString[0]; //Very dirty way of writing bytes X)))
  		bufferWrite[5] = rawString[1];
  		bufferWrite[6] = rawString[2];
  		bufferWrite[7] = rawString[3];
  		
			int bytesToWrite = sizeof bufferWrite;
  		int bytesToWriteLeft = bytesToWrite;
  		int bytesWritten = 0;
  		while(bytesToWriteLeft > 0){
  		  blinkLedBlue = true;
  			bytesWritten = sdWriteTimeout(&SDU1, (uint8_t*)bufferWrite, bytesToWrite, timeOut);
  			bytesToWriteLeft -= bytesWritten;
  		}
  		readOK = false;
  		bytesToWrite = 0;
  		blinkLedBlue = false;

  	chThdSleepMilliseconds(1);
  }
}

//WRITE LED BLINK
static WORKING_AREA(writeLedThreadWA, 128);
static msg_t writeLedThread(void *arg) {
  (void)arg;
  chRegSetThreadName("write led blue");
  while (TRUE) {
    systime_t time;
    time = blinkLedBlue == true ? 75 : 500;
    	palClearPad(GPIOD, GPIOD_LED6);
    	chThdSleepMilliseconds(time);
    	palSetPad(GPIOD, GPIOD_LED6);
    	chThdSleepMilliseconds(time);
  }
}

//SAMPLE ADC 
static WORKING_AREA(adcSampleThreadWA, 64);
static msg_t adcSampleThread(void *arg) {
  (void)arg;
  chRegSetThreadName("Sample ADC");
  while (TRUE) {
    //chSysLockFromIsr();
  	adcConvert(&ADCD1, &adcgrpcfg, samples, ADC_GRP1_BUF_DEPTH);
  	//chSysUnlockFromIsr();
  	//adcStopConversion(&ADCD1);
  	//chSysLockFromIsr();

  	//chSysUnlockFromIsr();
  }
}
/*===========================================================================*/
/* Application entry point.                                                             */
/*===========================================================================*/
int main(void) {
  Thread *shelltp = NULL;

  halInit();
  chSysInit();
  // Initialize interface to exchange data.
  initializeUSB(); // USB driver and USB-CDC link.
  /*
   * Initializes the ADC driver 1 and enable the thermal sensor.
   * The pin PC1 on the port GPIOC is programmed as analog input.
   */
  adcStart(&ADCD1, NULL);
  adcSTM32EnableTSVREFE();
  palSetPadMode(GPIOC, 1, PAL_MODE_INPUT_ANALOG); // Positionrack (Nam)
  
  //chSysLockFromIsr();
  	//adcStartConversionI(&ADCD1, &adcgrpcfg, samples, ADC_GRP1_BUF_DEPTH);
  	//chSysUnlockFromIsr();
  

  // Creates the blinker thread. 
  chThdCreateStatic(usbThreadWA, sizeof(usbThreadWA), LOWPRIO, usbThread, NULL);
  // WRITE thread
  chThdCreateStatic(writeThreadArea, sizeof(writeThreadArea), HIGHPRIO, writeThread, NULL);
  chThdCreateStatic(writeLedThreadWA, sizeof(writeLedThreadWA), LOWPRIO, writeLedThread, NULL);
	// READ thread
  chThdCreateStatic(readThreadWA, sizeof(readThreadWA), HIGHPRIO, readThread, NULL);
  // ADC thread
  chThdCreateStatic(adcSampleThreadWA, sizeof(adcSampleThreadWA), LOWPRIO, adcSampleThread, NULL);
  /*
   * Normal main() thread activity, in this demo it does nothing except
   * sleeping in a loop and check the button state.
   */
  while (TRUE) {
    
    //chThdSleepMilliseconds(1000);
    systime_t time;

    if(isUSBActive())
    	time=125;
    else
    	time=500;
    palClearPad(GPIOD, GPIOD_LED3);
    chThdSleepMilliseconds(time);
    palSetPad(GPIOD, GPIOD_LED3);
    chThdSleepMilliseconds(time);
  }
}
