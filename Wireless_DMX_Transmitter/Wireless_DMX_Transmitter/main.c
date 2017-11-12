/*******************************************************************************
Firma        : Roche Diagnostics International AG
VET department
Autor        : Ivan Heinzer

Projekt      : Wireless_DMX_Transmitter
Version      : 1.0
Dateiname    : main.c
Erstelldatum : 23/05/2016 9:32:40 AM

********************************************************************************
Chip type         : ATmega32u4
CPU-Clock         : 16.0000 MHz

********************************************************************************
Datum                     Vers.    Kommentar / Aenderungsgrund
23/05/2016 9:32:40 AM     1.0      Creation
23/04/2016 9:32:40 AM     2.0      Send only new
13/05/2016 9:32:40 AM     3.0      Automatic Channel choice

*******************************************************************************/

/*------------------ Definitions for Delay Function --------------------------*/
#define F_CPU 16000000UL
#define USE_RGBLED              //DELET THIS FOR MORE PERFORMANCE

/*------------------ Macros --------------------------------------------------*/
#define SETBIT(Adr,Bit)         (Adr |= (1<<Bit))
#define CLEARBIT(Adr,Bit)       (Adr &= ~(1<<Bit))
#define CHECKBIT(Adr,Bit)       (Adr & (1<<Bit))
#define INVERTBIT(Adr,Bit)      (Adr ^= (1<<Bit))

// Watchdog Software reset
#define soft_reset()        \
do                          \
{                           \
  wdt_enable(WDTO_15MS);  \
  for(;;)                 \
  {                       \
  }                       \
} while(0)

/*------------------------------- Include ------------------------------------*/
#include <avr/io.h>                 // I/O-Definitions
#include <avr/delay.h>              // _delay_ms() and _delay_us()
#include <avr/interrupt.h>          // Interrupt-Definitions

#include "../../Librarys/nrf24.c"   // NRF24 Library
#include "../../Librarys/radioPinFunctions.c" // NRF Pinconfiguration
#include "../../Librarys/DMX512_Lib.c" // DMX Library
#include <avr/wdt.h>                //Watchdog

/*------------------------ const and definitions  ----------------------------*/
#define   RGB_LED   PORTB
#define   DIP_SWITCH PIND
#define   PIN_RED   PORTB0
#define   PIN_GREEN PORTB1

/* CONFIGURE */
#define   CHANNELS_TO_SEND 60
#define   CHANNELS_PER_PACKAGE 30
#define   NUMBER_OF_PACKAGES_TO_SEND (CHANNELS_TO_SEND/CHANNELS_PER_PACKAGE)
#define   PAYLOAD_LENGTH 32
#define   RAWDATASIZE 126

/* MACROS */
#define   DIP_ADDRESS (DIP_SWITCH&0b11110000)>>4

/*------------------------ Global Variables  ---------------------------------*/
uint8_t Package_Data[32];
char data_compare[18];
uint8_t tx_address[5] = {0xE7,0xE7,0xE7,0xE7,0x00};

/*----------------------------Functions--------------------------------------*/
char nRFscanChannels();
int sendNrfPackage(uint8_t *pPackageData);
void setLeds(char Red, char Green);
void wdt_init(void) __attribute__((naked)) __attribute__((section(".init3")));

/*----------------------------- Hauptprogramm --------------------------------*/

int main(void)
{
  // I/O-Konfigurationen
  //RGB LED
  DDRB = 0b00000011;  // two outputs for rg leds
  //DIP-Switch
  DDRD = 0b00000000;
  PORTD= 0b11110000;
  //MAX485
  DDRE = 0b01000000;
  
  // Variables
  int TransmissionFailures = 0;
  int Channel;
  int Package;
  int Checksum = 0;
  char Best_Channel;
  char New_Best_Channel;
  int SendSuccess;
  
  //TODO
  // Get the Adress from DIP's
  //tx_address[4] = DIP_ADDRESS;
  
  // Clear the Channel Array
  for (Channel=0; Channel<513; Channel++)
  {
    DMX_Data[Channel]= 0;
  }
  
  //Startup
  setLeds(1,1); // Show Red/Green
  
  // NRF Init
  nrf24_init();
  nrf24_config(5, PAYLOAD_LENGTH);    // Default Channel
  nrf24_tx_address(tx_address);       // Set the device addresses
  
  Best_Channel = nRFscanChannels();
  nrf24_config(Best_Channel, PAYLOAD_LENGTH);
  
  // DMX Init
  CLEARBIT(PORTE,PORTE6);             // Set to recieve
  init_DMX_RX(MYUBRR);
  sei();                              //Globale Interrupts Enable
  
  while(1)
  {
    //HACK
    // If the Transmitt Adress should be changed, the Device should do a Soft reset
    //if (tx_address[4] != DIP_ADDRESS){
    //soft_reset();
    //}
    
    // every package (contains CHANNELS_PER_PACKAGE channels)
    for (Package = 0; Package < NUMBER_OF_PACKAGES_TO_SEND; Package++)
    {
      
      Package_Data[0] = Package;    // Package Number is first byte
      Checksum = 0;            // Reset data compare
      
      // every channel of the package
      for (Channel = 1; Channel <= CHANNELS_PER_PACKAGE; Channel++)
      {
        /* check, if channel < 512 (protect array DMX Data out of bounds) */
        if (((Package*CHANNELS_PER_PACKAGE)+Channel) < 512)
        {
          // Write dmx channel data to package for transmission
          Package_Data[Channel] = DMX_Data[(Package*CHANNELS_PER_PACKAGE)+Channel-1];  //-1 because DMX_DATA starts at 0
          // calculate checksum
          Checksum += CHECKBIT(Package_Data[Channel],0);  //LSB of every channel => checksum
        }
        else  // if value is out of DMX Data range write zero (prevent wrong dmx data)
        {
          Package_Data[Channel] = 0;
        }
      }
      
      Package_Data[CHANNELS_PER_PACKAGE+1] = Checksum;  // last byte of package is checksum
      
      // Send Package with NRF24
      SendSuccess = sendNrfPackage(Package_Data);
      if(SendSuccess == 1)   //SUCCESS
      {
        TransmissionFailures = 0;
        setLeds(0,1);  //green
      }
      //No Success => try to send next Package and increment Failure Counter
      else if (SendSuccess == 0)
      {
        //setLeds(1,1);   // red and green
        TransmissionFailures++ ;
      }
      // No Success for last 25 tries
      
      if(TransmissionFailures > 100 && TransmissionFailures < 1000)
      {
        setLeds(1,1);
      }
      else if (TransmissionFailures > 1000)
      {
        // No Transmission possible
        setLeds(1,0);  // red
        
        //Searching new channel
        New_Best_Channel = nRFscanChannels();
        //if better channel found => Change channel and reset transmission failure counter
        if(New_Best_Channel != Best_Channel)
        {
          nrf24_config(New_Best_Channel,32);
          TransmissionFailures = 0;
        }
        
      }
      //Send only If Data has been changed. (Receiver will listen to these Packages
      //                                     Package 0 and 8 will be sent every time.)
      //if (data_compare[PackageCounter] != nData_compare  || PackageCounter == 0 || PackageCounter == 8  )
    }
  }
}


void setLeds(char Red, char Green) {
  if (Red) {
    SETBIT(PORTB,PIN_RED);
    } else {
    CLEARBIT(PORTB,PIN_RED);
  }
  if (Green) {
    SETBIT(PORTB,PIN_GREEN);
    } else {
    CLEARBIT(PORTB,PIN_GREEN);
  }
}

int sendNrfPackage(uint8_t *pPackageData)
{
  //int mCounter = 0;
  nrf24_send(pPackageData); //Send Packet
  
  /* Wait for transmission to end */
  
  while(nrf24_isSending()) // Wait for the last transmission to end
  {}
  
  if (nrf24_lastMessageStatus() == NRF24_TRANSMISSON_OK)
  {
    return 1;
  }
  else
  {
    return 0;
  }
}




// scanning all channels in the 2.4GHz band and search cleanest Channel
char nRFscanChannels()
{
  unsigned char numberOfChannels = 20; //there are 126, 0 to 125
  unsigned int i,j;
  uint8_t CD_value;
  char RAWDATA[RAWDATASIZE];
  int CCHANNEL[8+1];
  char BestChannel;
  

  //Start with CE clear
  nrf24_ce_digitalWrite(LOW);
  //Set RX mode
  nrf24_powerUpRx();

  //clear RAWDATA
  for (i = 0; i < RAWDATASIZE; i++){
    RAWDATA[i] = 0;
  }
  
  //clear CCHANNEL
  for (i = 0; i < 9; i++){
    CCHANNEL[i] = 0;
  }

  RAWDATA[0] = (char) numberOfChannels;

  nrf24_ce_digitalWrite(LOW);
  for (j = 0; j < 40; j++) {

    for (i = 0; i < numberOfChannels; i++) {
      // select a new channel
      nrf24_configRegister(RF_CH,i); //will do every second one;
      nrf24_ce_digitalWrite(HIGH);
      //recomended at least 258uS but I found 350uS is need for reliability
      _delay_us(300);

      // received power > -64dBm
      nrf24_readRegister(CD,&CD_value,1);
      if (CD_value)
      {
        RAWDATA[i + 1]++;
      }

      //this seems to be needed after the timeout not before
      nrf24_ce_digitalWrite(LOW);
      _delay_us(4);

    }
  }
  
  // Do average of 10 Channels
  for (j = 1; j < 80; j++)
  {
    i = j / 10;
    CCHANNEL[i] += RAWDATA[j];
  }
  
  //Find best area of 8 areas
  i = 0xFF;
  for (j = 0; j < 8; j++)
  {
    if (CCHANNEL[j] < i){
      i = CCHANNEL[j];
      BestChannel = j * 10;
    }
  }
  
  //Find best channel in the cleanest Area
  i = 0xFF;
  for (j = BestChannel; j < (BestChannel+10); j++)
  {
    if (RAWDATA[j] < i){
      i = RAWDATA[j];
      BestChannel = j;
    }
  }
  
  return BestChannel;
}

// Function Implementation
void wdt_init(void)
{
  MCUSR = 0;
  wdt_disable();

  return;
}