/*
 HABDuino Tracker
 http://www.habduino.org
 (c) Anthony Stirk M0UPU 
 
 March 2015 Version 4.0.0
 
 This is for the Version 4 Habduino Hardware.
 
 Credits :
 
 Interrupt Driven RTTY Code : Evolved from Rob Harrison's RTTY Code.
 Thanks to :  http://www.engblaze.com/microcontroller-tutorial-avr-and-arduino-timer-interrupts/
 http://gammon.com.au/power
 
 Suggestion to lock variables when making the telemetry string & Compare match register calculation from Phil Heron.
 APRS Code mainly by Phil Heron MI0VIM
 
 GPS Code modified from jonsowman and Joey flight computer CUSF
 https://github.com/cuspaceflight/joey-m/tree/master/firmware
 
 Thanks to :
 
 Phil Heron
 James Coxon
 Dave Akerman
 
 The UKHAS Community http://ukhas.org.uk
 
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.
 
 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.
 
 See <http://www.gnu.org/licenses/>.
 
 The Habduino kit is supplied as is with no guarantees of performance or operation. Although the HABDuino 
 is designed for short-term Near Space operation, the conditions may exceed the specification of individual 
 components which may fail to operate correctly because of low temperatures or pressures of increased 
 radiation levels. We have no control over the effectiveness of your payload design especially the 
 temperature and electromagnetic conditions  within, thus we make no warrnaty express or implied as to
 the ability of our products to operate to any degree of effectiveness when used in your application.
 
 If you decide to use this product under a balloon it’s your responsibility to ensure you comply with the 
 local legislation and laws regarding meteorological balloon launching and radio transmission in the air. 
 
 The Radiometrix MTX2 434Mhz is NOT license exempt in the United States of America and does need a radio
 amateur license.  Use of APRS requires a radio amateur license in all countries and a number of countries 
 don’t permit the airborne use of APRS under any circumstances.  
 The Habduino cannot do APRS without an addon Radiometrix HX1 module.
 
 The hardware design & code for Habduino is released under a Creative Commons License 3.0 Attribution-ShareAlike License :
 See : http://creativecommons.org/licenses/by-sa/3.0/
 It is YOUR responsibility to ensure this kit is used safely please review the safety section.
 
 The latest code is available here : https://github.com/HABduino/HABduino
 
 
 */

/* BITS YOU WANT TO AMEND */

#define MTX2_FREQ 434.485 // format 434.XXX  
char callsign[9] = "CHANGEME";  // MAX 9 CHARACTERS!!

/* BELOW HERE YOU PROBABLY DON'T WANT TO BE CHANGING STUFF */

#include <util/crc16.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <SoftwareSerial.h>
#include "ax25modem.h"
static const uint8_t PROGMEM _sine_table[] = {
#include "sine_table.h"
};
#define ASCII 7          // ASCII 7 or 8
#define STOPBITS 2       // Either 1 or 2
#define TXDELAY 0        // Delay between sentence TX's
#define RTTY_BAUD 50     // RTTY Baud rate (Recommended = 50)
#define LED_WARN 9
#define LED_OK 8
#define BATTERY_ADC A0
//#define MTX2_SHIFT 425        
//#define MTX2_OFFSET 0          // 0-100 Slightly adjusts the frequency by increasing the PWM 
#define MTX2_TXD 4
#define MTX2_ENABLE 7


#define HX1_ENABLE 6
#define HX1_TXD 3
#define GPS_ON 2
#define POWERSAVING
#define ONE_WIRE_BUS 5
#define ONE_SECOND F_CPU / 1024 / 16


#define BAUD_RATE      (1200)
#define TABLE_SIZE     (512)
#define PREAMBLE_BYTES (50)
#define REST_BYTES     (5)

#define PLAYBACK_RATE    (F_CPU / 256)
#define SAMPLES_PER_BAUD (PLAYBACK_RATE / BAUD_RATE)
#define PHASE_DELTA_1200 (((TABLE_SIZE * 1200L) << 7) / PLAYBACK_RATE)
#define PHASE_DELTA_2200 (((TABLE_SIZE * 2200L) << 7) / PLAYBACK_RATE)
#define PHASE_DELTA_XOR  (PHASE_DELTA_1200 ^ PHASE_DELTA_2200)

SoftwareSerial MTX2_EN(12, MTX2_ENABLE); // RX, TX
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

volatile int txstatus=1;
volatile int txstringlength=0;
volatile char txc;
volatile int txi;
volatile int txj;
volatile int count=1;
volatile boolean lockvariables = 0;
volatile static uint8_t *_txbuf = 0;
volatile static uint8_t  _txlen = 0;

int errorstatus=0;
/* Error Status Bit Level Field :
 Bit 0 = GPS Error Condition Noted Switch to Max Performance Mode
 Bit 1 = GPS Error Condition Noted Cold Boot GPS
 Bit 2 = DS18B20 temp sensor status 0 = OK 1 = Fault
 Bit 3 = Current Dynamic Model 0 = Flight 1 = Pedestrian
 Bit 4 = PSM Status 0 = PSM On 1 = PSM Off                   
 Bit 5 = Lock 0 = GPS Locked 1= Not Locked
 
 So error 8 means the everything is fine just the GPS is in pedestrian mode. 
 Below 1000 meters the code puts the GPS in the more accurate pedestrian mode. 
 Above 1000 meters it switches to dynamic model 6 i.e flight mode and turns the LED's off for additional power saving. 
 So as an example error code 40 = 101000 means GPS not locked and in pedestrian mode. 
 */

char txstring[100];
uint8_t buf[60]; 
uint8_t lock =0, sats = 0, hour = 0, minute = 0, second = 0;
uint8_t oldhour = 0, oldminute = 0, oldsecond = 0;
int GPSerror = 0,navmode = 0,psm_status = 0,lat_int=0,lon_int=0,tempsensors,temperature1=0,temperature2=0;
int32_t lat = 0, lon = 0, alt = 0, maxalt = 0, lat_dec = 0, lon_dec =0 ,tslf=0;
unsigned long currentMillis;
long previousMillis = 0;
int batteryadc_v,battvaverage=0,aprstxstatus=0; 
int32_t battvsmooth[5] ;
int aprs_tx_status = 0, aprs_attempts = 0;
unsigned long startTime;
char comment[3]={
  ' ', ' ', '\0'};


void setup()  { 
  pinMode(MTX2_TXD, OUTPUT);
  pinMode(LED_WARN, OUTPUT);
  pinMode(HX1_ENABLE, OUTPUT);
  pinMode(LED_OK,OUTPUT);
  pinMode(MTX2_ENABLE, OUTPUT);
  pinMode(GPS_ON, OUTPUT);
  pinMode(BATTERY_ADC, INPUT);
  blinkled(6);
  setMTX2Frequency();
  Serial.begin(9600);
  digitalWrite(MTX2_ENABLE,HIGH);
  blinkled(5);
  digitalWrite(GPS_ON,HIGH);
  blinkled(4);
  resetGPS();
  blinkled(3);

#ifndef APRS
  TCCR2B = TCCR2B & 0b11111000 | 1; // Sets fast PWM on pin 11
#endif

#ifdef APRS
  ax25_init();
#endif

  blinkled(2);
  setupGPS();
  blinkled(1);
  initialise_interrupt();
  sensors.begin();
  tempsensors=sensors.getDeviceCount();
} 

void loop()   {
  oldhour=hour;
  oldminute=minute;
  oldsecond=second;
  gps_check_nav();

  if(lock!=3) // Blink LED to indicate no lock
  {
    errorstatus |=(1 << 5);  // Set bit 5 (Lock 0 = GPS Locked 1= Not Locked)
  }
  else
  {
    errorstatus &= ~(1 << 5); // Unset bit 5 (Lock 0 = GPS Locked 1= Not Locked)
  }
  checkDynamicModel();

#ifdef APRS 
  if(sats>=4){
    if (aprs_tx_status==0)
    {
      startTime=millis();
      aprs_tx_status=1;
    }
    if(millis() - startTime > (APRS_TX_INTERVAL*60000)) {
      aprs_tx_status=0;
      send_APRS();
      aprs_attempts++;
    }
  }  
#endif


#ifdef POWERSAVING
  if((lock==3) && (psm_status==0) && (sats>=5) &&((errorstatus & (1 << 0))==0)&&((errorstatus & (1 << 1))==0)) // Check we aren't in an error condition
  {
    setGPS_PowerSaveMode();
    wait(1000);
    psm_status=1;
    errorstatus &= ~(1 << 4); // Set Bit 4 Indicating PSM is on
  }
#endif
  if(!lockvariables) {

    prepare_data();
    if(alt>maxalt && sats >= 4)
    {
      maxalt=alt;
    }
  }
  if((oldhour==hour&&oldminute==minute&&oldsecond==second)||sats<=4) {
    tslf++;
  }
  else
  {
    tslf=0;
    errorstatus &= ~(1 << 0); // Unset bit 0 (Clear GPS Error Condition Noted Switch to Max Performance Mode)
    errorstatus &= ~(1 << 1); // Unset bit 1 (Clear GPS Error Condition Noted Cold Boot GPS)
  }
  if((tslf>10 && ((errorstatus & (1 << 0))==0)&&((errorstatus & (1 << 1))==0))) {
    setupGPS();
    wait(125);
    setGps_MaxPerformanceMode();
    wait(125);
    errorstatus |=(1 << 0); // Set Bit 1 (GPS Error Condition Noted Switch to Max Performance Mode)
    psm_status=0;
    errorstatus |=(1 << 4); // Set Bit 4 (Indicate PSM is disabled)
  }
  if(tslf>100 && ((errorstatus & (1 << 0))==1)&&((errorstatus & (1 << 1))==0)) {
    errorstatus |=(1 << 0); // Unset Bit 0 we've already tried that didn't work
    errorstatus |=(1 << 1); // Set bit 1 indicating we are cold booting the GPS
    Serial.flush();
    resetGPS();
    wait(125);
    setupGPS();
  }
}

void initialise_interrupt() 
{
  // initialize Timer1
  cli();          // disable global interrupts
  TCCR1A = 0;     // set entire TCCR1A register to 0
  TCCR1B = 0;     // same for TCCR1B
  OCR1A = F_CPU / 1024 / RTTY_BAUD - 1;  // set compare match register to desired timer count:
  TCCR1B |= (1 << WGM12);   // turn on CTC mode:
  // Set CS10 and CS12 bits for:
  TCCR1B |= (1 << CS10);
  TCCR1B |= (1 << CS12);
  // enable timer compare interrupt:
  TIMSK1 |= (1 << OCIE1A);
  sei();          // enable global interrupts
}

ISR(TIMER1_COMPA_vect)
{

  if(alt>1000 && sats >= 4)
  {
    digitalWrite(LED_WARN,LOW);  
    digitalWrite(LED_OK,LOW);  
  }
  else 
  {
    currentMillis = millis();
    if(currentMillis - previousMillis > ONE_SECOND) 
    {
      previousMillis = currentMillis;   
      if(errorstatus!=8)
      {
        digitalWrite(LED_WARN,!digitalRead(LED_WARN));
        digitalWrite(LED_OK,LOW);  
      }
      else 
      {
        digitalWrite(LED_OK, !digitalRead(LED_OK)); 
        digitalWrite(LED_WARN,LOW);    
      }
    }
  }

  switch(txstatus) {
  case 0: // This is the optional delay between transmissions.
    txj++;
    if(txj>(TXDELAY*RTTY_BAUD)) { 
      txj=0;
      txstatus=1;
    }
    break;
  case 1: // Initialise transmission
    if(alt>maxalt && sats >= 4)
    {
      maxalt=alt;
    }
    lockvariables=1;
#ifndef APRS
    if(tempsensors==1)
    {
      snprintf(txstring,100, "$$$$$%s,%i,%02d:%02d:%02d,%s%i.%06ld,%s%i.%06ld,%ld,%d,%i,%i,%02x",callsign,count, hour, minute, second,lat < 0 ? "-" : "",lat_int,lat_dec,lon < 0 ? "-" : "",lon_int,lon_dec, maxalt,sats,temperature1,battvaverage,errorstatus);
    }
    else
    {
      snprintf(txstring,100, "$$$$$%s,%i,%02d:%02d:%02d,%s%i.%06ld,%s%i.%06ld,%ld,%d,%i,%i,%i,%02x",callsign,count, hour, minute, second,lat < 0 ? "-" : "",lat_int,lat_dec,lon < 0 ? "-" : "",lon_int,lon_dec, maxalt,sats,temperature1,temperature2,battvaverage,errorstatus);
    }
#endif
#ifdef APRS
    if(tempsensors==1)
    {
      snprintf(txstring,100, "$$$$$%s,%i,%02d:%02d:%02d,%s%i.%06ld,%s%i.%06ld,%ld,%d,%i,%i,%i,%02x",callsign,count, hour, minute, second,lat < 0 ? "-" : "",lat_int,lat_dec,lon < 0 ? "-" : "",lon_int,lon_dec, maxalt,sats,temperature1,battvaverage,aprs_attempts,errorstatus);
    }
    else
    {
      snprintf(txstring,100, "$$$$$%s,%i,%02d:%02d:%02d,%s%i.%06ld,%s%i.%06ld,%ld,%d,%i,%i,%i,%i,%02x",callsign,count, hour, minute, second,lat < 0 ? "-" : "",lat_int,lat_dec,lon < 0 ? "-" : "",lon_int,lon_dec, maxalt,sats,temperature1,temperature2,battvaverage,aprs_attempts,errorstatus);
    }
#endif
    crccat(txstring);
    maxalt=0;
    lockvariables=0;
    txstringlength=strlen(txstring);
    txstatus=2;
    txj=0;
    break;

  case 2: // Grab a char and lets go transmit it. 
    if ( txj < txstringlength)
    {
      txc = txstring[txj];
      txj++;
      txstatus=3;
      rtty_txbit (0); // Start Bit;
      txi=0;
    }
    else 
    {
      txstatus=0; // Should be finished
      txj=0;
      count++;
    }
    break;
  case 3:
    if(txi<ASCII)
    {
      txi++;
      if (txc & 1) rtty_txbit(1); 
      else rtty_txbit(0);	
      txc = txc >> 1;
      break;
    }
    else 
    {
      rtty_txbit (1); // Stop Bit
      txstatus=4;
      txi=0;
      break;
    } 
  case 4:
    if(STOPBITS==2)
    {
      rtty_txbit (1); // Stop Bit
      txstatus=2;
      break;
    }
    else
    {
      txstatus=2;
      break;
    }

  }
}

void rtty_txbit (int bit)
{
  if (bit)
  {
    digitalWrite(MTX2_TXD,HIGH);
    //#ifdef APRS
//    analogWrite(MTX2_TXD, MTX2_OFFSET+((MTX2_SHIFT*4)/16)); // High
    //#endif
    //#ifndef APRS
    //    analogWrite(MTX2_TXD, MTX2_OFFSET+((MTX2_SHIFT*1.8)/16)); // High
    //#endif

  }
  else
  {
      digitalWrite(MTX2_TXD,LOW);
//    analogWrite(MTX2_TXD, MTX2_OFFSET); // Low
  }
}
void resetGPS() {
  uint8_t set_reset[] = {
    0xB5, 0x62, 0x06, 0x04, 0x04, 0x00, 0xFF, 0x87, 0x00, 0x00, 0x94, 0xF5           };
  sendUBX(set_reset, sizeof(set_reset)/sizeof(uint8_t));
}
void sendUBX(uint8_t *MSG, uint8_t len) {
  Serial.flush();
  Serial.write(0xFF);
  wait(100);
  for(int i=0; i<len; i++) {
    Serial.write(MSG[i]);
  }
}
void setupGPS() {
  // Turning off all GPS NMEA strings apart on the uBlox module
  // Taken from Project Swift (rather than the old way of sending ascii text)
  int gps_set_sucess=0;
  uint8_t setNMEAoff[] = {
    0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, 0x80, 0x25, 0x00, 0x00, 0x07, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0xA0, 0xA9           };
  sendUBX(setNMEAoff, sizeof(setNMEAoff)/sizeof(uint8_t));
  while(!gps_set_sucess)
  {
    sendUBX(setNMEAoff, sizeof(setNMEAoff)/sizeof(uint8_t));
    gps_set_sucess=getUBX_ACK(setNMEAoff);
    if(!gps_set_sucess)
    {
      blinkled(2);
    }
  }
  wait(500);
  setGPS_GNSS();
  wait(500);
  setGPS_DynamicModel6();
  wait(500);
  setGps_MaxPerformanceMode();
  wait(500);
}

void setGPS_DynamicModel6()
{
  int gps_set_sucess=0;
  uint8_t setdm6[] = {
    0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x06,
    0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00,
    0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C,
    0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16, 0xDC           };
  while(!gps_set_sucess)
  {
    sendUBX(setdm6, sizeof(setdm6)/sizeof(uint8_t));
    gps_set_sucess=getUBX_ACK(setdm6);
  }
}
void setGPS_GNSS()
{
  // Sets CFG-GNSS to disable everything other than GPS GNSS
  // solution. Failure to do this means GPS power saving 
  // doesn't work. Not needed for MAX7, needed for MAX8's
  int gps_set_sucess=0;
  uint8_t setgnss[] = {
    0xB5, 0x62, 0x06, 0x3E, 0x2C, 0x00, 0x00, 0x00,
    0x20, 0x05, 0x00, 0x08, 0x10, 0x00, 0x01, 0x00,
    0x01, 0x01, 0x01, 0x01, 0x03, 0x00, 0x00, 0x00,
    0x01, 0x01, 0x03, 0x08, 0x10, 0x00, 0x00, 0x00,
    0x01, 0x01, 0x05, 0x00, 0x03, 0x00, 0x00, 0x00,
    0x01, 0x01, 0x06, 0x08, 0x0E, 0x00, 0x00, 0x00,
    0x01, 0x01, 0xFC, 0x11   };
  while(!gps_set_sucess)
  {
    sendUBX(setgnss, sizeof(setgnss)/sizeof(uint8_t));
    gps_set_sucess=getUBX_ACK(setgnss);
  }
}
void setGPS_DynamicModel3()
{
  int gps_set_sucess=0;
  uint8_t setdm3[] = {
    0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x03,
    0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00,
    0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C,
    0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x13, 0x76           };
  while(!gps_set_sucess)
  {
    sendUBX(setdm3, sizeof(setdm3)/sizeof(uint8_t));
    gps_set_sucess=getUBX_ACK(setdm3);
  }
}
void setGps_MaxPerformanceMode() {
  //Set GPS for Max Performance Mode
  uint8_t setMax[] = { 
    0xB5, 0x62, 0x06, 0x11, 0x02, 0x00, 0x08, 0x00, 0x21, 0x91           }; // Setup for Max Power Mode
  sendUBX(setMax, sizeof(setMax)/sizeof(uint8_t));
}

boolean getUBX_ACK(uint8_t *MSG) {
  uint8_t b;
  uint8_t ackByteID = 0;
  uint8_t ackPacket[10];
  unsigned long startTime = millis();

  // Construct the expected ACK packet    
  ackPacket[0] = 0xB5;	// header
  ackPacket[1] = 0x62;	// header
  ackPacket[2] = 0x05;	// class
  ackPacket[3] = 0x01;	// id
  ackPacket[4] = 0x02;	// length
  ackPacket[5] = 0x00;
  ackPacket[6] = MSG[2];	// ACK class
  ackPacket[7] = MSG[3];	// ACK id
  ackPacket[8] = 0;		// CK_A
  ackPacket[9] = 0;		// CK_B

  // Calculate the checksums
  for (uint8_t ubxi=2; ubxi<8; ubxi++) {
    ackPacket[8] = ackPacket[8] + ackPacket[ubxi];
    ackPacket[9] = ackPacket[9] + ackPacket[8];
  }

  while (1) {

    // Test for success
    if (ackByteID > 9) {
      // All packets in order!
      return true;
    }

    // Timeout if no valid response in 3 seconds
    if (millis() - startTime > 3000) { 
      return false;
    }

    // Make sure data is available to read
    if (Serial.available()) {
      b = Serial.read();

      // Check that bytes arrive in sequence as per expected ACK packet
      if (b == ackPacket[ackByteID]) { 
        ackByteID++;
      } 
      else {
        ackByteID = 0;	// Reset and look again, invalid order
      }

    }
  }
}
uint16_t gps_CRC16_checksum (char *string)
{
  size_t i;
  uint16_t crc;
  uint8_t c;

  crc = 0xFFFF;

  // Calculate checksum ignoring the first two $s
  for (i = 5; i < strlen(string); i++)
  {
    c = string[i];
    crc = _crc_xmodem_update (crc, c);
  }

  return crc;
}

void blinkled(int blinks)
{
  for(int blinkledx = 0; blinkledx <= blinks; blinkledx++) {
    digitalWrite(LED_WARN,HIGH);
    wait(100);
    digitalWrite(LED_WARN,LOW);
    wait(100);
  }    
}    

void wait(unsigned long delaytime) // Arduino Delay doesn't get CPU Speeds below 8Mhz
{
  unsigned long _delaytime=millis();
  while((_delaytime+delaytime)>=millis()){
  }
}
uint8_t gps_check_nav(void)
{
  uint8_t request[8] = {
    0xB5, 0x62, 0x06, 0x24, 0x00, 0x00, 0x2A, 0x84           };
  sendUBX(request, 8);

  // Get the message back from the GPS
  gps_get_data();

  // Verify sync and header bytes
  if( buf[0] != 0xB5 || buf[1] != 0x62 ){
    GPSerror = 41;
  }
  if( buf[2] != 0x06 || buf[3] != 0x24 ){
    GPSerror = 42;
  }
  // Check 40 bytes of message checksum
  if( !_gps_verify_checksum(&buf[2], 40) ) {
    GPSerror = 43;
  }


  // Return the navigation mode and let the caller analyse it
  navmode = buf[8];
}
void gps_get_data()
{
  Serial.flush();
  // Clear buf[i]
  for(int i = 0;i<60;i++) 
  {
    buf[i] = 0; // clearing buffer  
  }  
  int i = 0;
  unsigned long startTime = millis();

  while ((i<60) && ((millis() - startTime) < 1000) ) { 
    if (Serial.available()) {
      buf[i] = Serial.read();
      i++;
    }
  }
}
bool _gps_verify_checksum(uint8_t* data, uint8_t len)
{
  uint8_t a, b;
  gps_ubx_checksum(data, len, &a, &b);
  if( a != *(data + len) || b != *(data + len + 1))
    return false;
  else
    return true;
}
void gps_ubx_checksum(uint8_t* data, uint8_t len, uint8_t* cka,
uint8_t* ckb)
{
  *cka = 0;
  *ckb = 0;
  for( uint8_t i = 0; i < len; i++ )
  {
    *cka += *data;
    *ckb += *cka;
    data++;
  }
}
void checkDynamicModel() {
  if(alt<=1000&&sats>4) {
    if(navmode != 3)
    {
      setGPS_DynamicModel3();
      errorstatus |=(1 << 3);  // Set Bit 3 indicating we are in pedestrian mode    
    }
  }
  else
  {
    if(navmode != 6){
      setGPS_DynamicModel6();
      errorstatus &= ~(1 << 3); // Unset bit 3 indicating we are in flight mode

    }
  }
}
void setGPS_PowerSaveMode() {
  // Power Save Mode 
  uint8_t setPSM[] = { 
    0xB5, 0x62, 0x06, 0x11, 0x02, 0x00, 0x08, 0x01, 0x22, 0x92           }; // Setup for Power Save Mode (Default Cyclic 1s)
  sendUBX(setPSM, sizeof(setPSM)/sizeof(uint8_t));
}
void prepare_data() {
  if(aprstxstatus==0)
  {
    sensors.requestTemperatures();
    temperature1=sensors.getTempCByIndex(0);
    if(tempsensors==2)
    {
      temperature2=sensors.getTempCByIndex(1);
    }
    if(temperature1==-127)
    {
      errorstatus |=(1 << 2); // Set bit 2 indicating the temp sensor is faulty
    }
    else
    {
      errorstatus &= ~(1 << 2); // Unset bit 2 indicating temp sensor is ok.
    }
  } 
  gps_check_lock();
  gps_get_position();
  gps_get_time();
  batteryadc_v=analogRead(BATTERY_ADC)*4.8;
  battvsmooth[4] = battvsmooth[3];
  battvsmooth[3] = battvsmooth[2];
  battvsmooth[2] = battvsmooth[1];
  battvsmooth[1] = battvsmooth[0];
  battvsmooth[0] = batteryadc_v;
  battvaverage = (battvsmooth[0]+battvsmooth[1]+ battvsmooth[2]+battvsmooth[3]+battvsmooth[4])/5;


}
void gps_check_lock()
{
  GPSerror = 0;
  Serial.flush();
  // Construct the request to the GPS
  uint8_t request[8] = {
    0xB5, 0x62, 0x01, 0x06, 0x00, 0x00,
    0x07, 0x16                                                                                                                                                                  };
  sendUBX(request, 8);

  // Get the message back from the GPS
  gps_get_data();
  // Verify the sync and header bits
  if( buf[0] != 0xB5 || buf[1] != 0x62 ) {
    GPSerror = 11;
  }
  if( buf[2] != 0x01 || buf[3] != 0x06 ) {
    GPSerror = 12;
  }

  // Check 60 bytes minus SYNC and CHECKSUM (4 bytes)
  if( !_gps_verify_checksum(&buf[2], 56) ) {
    GPSerror = 13;
  }

  if(GPSerror == 0){
    // Return the value if GPSfixOK is set in 'flags'
    if( buf[17] & 0x01 )
      lock = buf[16];
    else
      lock = 0;

    sats = buf[53];
  }
  else {
    lock = 0;
  }
}
void gps_get_position()
{
  GPSerror = 0;
  Serial.flush();
  // Request a NAV-POSLLH message from the GPS
  uint8_t request[8] = {
    0xB5, 0x62, 0x01, 0x02, 0x00, 0x00, 0x03, 0x0A           };
  sendUBX(request, 8);

  // Get the message back from the GPS
  gps_get_data();

  // Verify the sync and header bits
  if( buf[0] != 0xB5 || buf[1] != 0x62 )
    GPSerror = 21;
  if( buf[2] != 0x01 || buf[3] != 0x02 )
    GPSerror = 22;

  if( !_gps_verify_checksum(&buf[2], 32) ) {
    GPSerror = 23;
  }

  if(GPSerror == 0) {
    if(sats<4)
    {
      lat=0;
      lon=0;
      alt=0;
    }
    else
    {
      lon = (int32_t)buf[10] | (int32_t)buf[11] << 8 | 
        (int32_t)buf[12] << 16 | (int32_t)buf[13] << 24;
      lat = (int32_t)buf[14] | (int32_t)buf[15] << 8 | 
        (int32_t)buf[16] << 16 | (int32_t)buf[17] << 24;
      alt = (int32_t)buf[22] | (int32_t)buf[23] << 8 | 
        (int32_t)buf[24] << 16 | (int32_t)buf[25] << 24;
    }
    // 4 bytes of latitude/longitude (1e-7)
    lon_int=abs(lon/10000000);
    lon_dec=(labs(lon) % 10000000)/10;
    lat_int=abs(lat/10000000);
    lat_dec=(labs(lat) % 10000000)/10;


    // 4 bytes of altitude above MSL (mm)

    alt /= 1000; // Correct to meters
  }

}
void gps_get_time()
{
  GPSerror = 0;
  Serial.flush();
  // Send a NAV-TIMEUTC message to the receiver
  uint8_t request[8] = {
    0xB5, 0x62, 0x01, 0x21, 0x00, 0x00, 0x22, 0x67           };
  sendUBX(request, 8);

  // Get the message back from the GPS
  gps_get_data();

  // Verify the sync and header bits
  if( buf[0] != 0xB5 || buf[1] != 0x62 )
    GPSerror = 31;
  if( buf[2] != 0x01 || buf[3] != 0x21 )
    GPSerror = 32;

  if( !_gps_verify_checksum(&buf[2], 24) ) {
    GPSerror = 33;
  }

  if(GPSerror == 0) {
    if(buf[22] > 23 || buf[23] > 59 || buf[24] > 59)
    {
      GPSerror = 34;
    }
    else {
      hour = buf[22];
      minute = buf[23];
      second = buf[24];
    }
  }
}

void send_APRS() {

  ax25_init();
  tx_aprs();

}

void ax25_init(void)
{
  /* Fast PWM mode, non-inverting output on OC2A */
  TCCR2A = _BV(COM2B1) | _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS20);
  pinMode(HX1_TXD, OUTPUT);
}

void tx_aprs()
{
  aprstxstatus=1;
  PORTD |= _BV(HX1_ENABLE); // Same as digitalWrite(HX1_ENABLE, HIGH); but more efficient
  char slat[5];
  char slng[5];
  char stlm[9];
  static uint16_t seq = 0;
  double aprs_lat, aprs_lon;

  /* Convert the UBLOX-style coordinates to
   	 * the APRS compressed format */
  aprs_lat = 900000000 - lat;
  aprs_lat = aprs_lat / 26 - aprs_lat / 2710 + aprs_lat / 15384615;
  aprs_lon = 900000000 + lon / 2;
  aprs_lon = aprs_lon / 26 - aprs_lon / 2710 + aprs_lon / 15384615;
  int32_t aprs_alt = alt * 32808 / 10000;


  /* Construct the compressed telemetry format */
  ax25_base91enc(stlm + 0, 2, seq);
  if(tempsensors==1)
  {
    ax25_frame(
    APRS_CALLSIGN, APRS_SSID,
    "APRS", 0,
    //0, 0, 0, 0,
    "WIDE1", 1, "WIDE2",1,
    //"WIDE2", 1,
    "!/%s%sO   /A=%06ld|%s|%s/%s,%d,%i,%i'C,http://habduino.org",
    ax25_base91enc(slat, 4, aprs_lat),
    ax25_base91enc(slng, 4, aprs_lon),
    aprs_alt, stlm, comment,APRS_CALLSIGN, count, errorstatus,temperature1
      );
  }
  else
  {
    ax25_frame(
    APRS_CALLSIGN, APRS_SSID,
    "APRS", 0,
    //0, 0, 0, 0,
    "WIDE1", 1, "WIDE2",1,
    //"WIDE2", 1,
    "!/%s%sO   /A=%06ld|%s|%s/%s,%d,%i,%i,%i'C,http://habduino.org",
    ax25_base91enc(slat, 4, aprs_lat),
    ax25_base91enc(slng, 4, aprs_lon),
    aprs_alt, stlm, comment,APRS_CALLSIGN, count, errorstatus,temperature1,temperature2
      );
  }
  seq++;
}

ISR(TIMER2_OVF_vect)
{

  static uint16_t phase  = 0;
  static uint16_t step   = PHASE_DELTA_1200;
  static uint16_t sample = 0;
  static uint8_t rest    = PREAMBLE_BYTES + REST_BYTES;
  static uint8_t byte;
  static uint8_t bit     = 7;
  static int8_t bc       = 0;
  /* Update the PWM output */
  OCR2B = pgm_read_byte(&_sine_table[(phase >> 7) & 0x1FF]);
  phase += step;

  if(++sample < SAMPLES_PER_BAUD) return;
  sample = 0;

  /* Zero-bit insertion */
  if(bc == 5)
  {
    step ^= PHASE_DELTA_XOR;
    bc = 0;
    return;
  }

  /* Load the next byte */
  if(++bit == 8)
  {
    bit = 0;

    if(rest > REST_BYTES || !_txlen)
    {
      if(!--rest)
      {
        /* Disable radio and interrupt */

        PORTD &= ~_BV(HX1_ENABLE); // Turn the HX1 Off
        aprstxstatus=0;
        TIMSK2 &= ~_BV(TOIE2);

        /* Prepare state for next run */
        phase = sample = 0;
        step  = PHASE_DELTA_1200;
        rest  = PREAMBLE_BYTES + REST_BYTES;
        bit   = 7;
        bc    = 0;
        return;
      }

      /* Rest period, transmit ax.25 header */
      byte = 0x7E;
      bc = -1;
    }
    else
    {
      /* Read the next byte from memory */
      byte = *(_txbuf++);
      if(!--_txlen) rest = REST_BYTES + 2;
      if(bc < 0) bc = 0;
    }
  }

  /* Find the next bit */
  if(byte & 1)
  {
    /* 1: Output frequency stays the same */
    if(bc >= 0) bc++;
  }
  else
  {
    /* 0: Toggle the output frequency */
    step ^= PHASE_DELTA_XOR;
    if(bc >= 0) bc = 0;
  }

  byte >>= 1;
}
char *ax25_base91enc(char *s, uint8_t n, uint32_t v)
{
  /* Creates a Base-91 representation of the value in v in the string */
  /* pointed to by s, n-characters long. String length should be n+1. */

  for(s += n, *s = '\0'; n; n--)
  {
    *(--s) = v % 91 + 33;
    v /= 91;
  }

  return(s);
}
void ax25_frame(char *scallsign, char sssid, char *dcallsign, char dssid,
char *path1, char ttl1, char *path2, char ttl2, char *data, ...)
{
  static uint8_t frame[100];
  uint8_t *s;
  uint16_t x;
  va_list va;

  va_start(va, data);

  /* Pause while there is still data transmitting */
  while(_txlen);

  /* Write in the callsigns and paths */
  s = _ax25_callsign(frame, dcallsign, dssid);
  s = _ax25_callsign(s, scallsign, sssid);
  if(path1) s = _ax25_callsign(s, path1, ttl1);
  if(path2) s = _ax25_callsign(s, path2, ttl2);

  /* Mark the end of the callsigns */
  s[-1] |= 1;

  *(s++) = 0x03; /* Control, 0x03 = APRS-UI frame */
  *(s++) = 0xF0; /* Protocol ID: 0xF0 = no layer 3 data */

  vsnprintf((char *) s, 100 - (s - frame) - 2, data, va);
  va_end(va);

  /* Calculate and append the checksum */
  for(x = 0xFFFF, s = frame; *s; s++)
    x = _crc_ccitt_update(x, *s);

  *(s++) = ~(x & 0xFF);
  *(s++) = ~((x >> 8) & 0xFF);

  /* Point the interrupt at the data to be transmit */
  _txbuf = frame;
  _txlen = s - frame;

  /* Enable the timer and key the radio */
  TIMSK2 |= _BV(TOIE2);
  //PORTA |= TXENABLE;
}

static uint8_t *_ax25_callsign(uint8_t *s, char *callsign, char ssid)
{
  char i;
  for(i = 0; i < 6; i++)
  {
    if(*callsign) *(s++) = *(callsign++) << 1;
    else *(s++) = ' ' << 1;
  }
  *(s++) = ('0' + ssid) << 1;
  return(s);
}

void setMTX2Frequency()
{
  float _mtx2comp;
  int _mtx2int;
  long _mtx2fractional;
  char _mtx2command[17];
  MTX2_EN.begin(9600);
  _mtx2comp=(MTX2_FREQ+0.0015)/6.5;
  _mtx2int=_mtx2comp;
  _mtx2fractional=float(((_mtx2comp-_mtx2int)+1)*524288);
  snprintf(_mtx2command,17,"@PRG_%02X%06lX\r",_mtx2int-1, _mtx2fractional);
  delay(100);
  MTX2_EN.print(_mtx2command);
  delay(50);
  MTX2_EN.end();
}  

uint16_t crccat(char *msg)
{
  uint16_t x;


  while(*msg == '$') msg++;


  for(x = 0xFFFF; *msg; msg++)
    x = _crc_xmodem_update(x, *msg);


  snprintf_P(msg, 8, PSTR("*%04X\n"), x);

  return(x);
}

















