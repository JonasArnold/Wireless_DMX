/*******************************************************************************
Firma        : Roche Diagnostics International AG
VET Department
Autor        : Ivan Heinzer

Projekt      : Wireless_DMX_Receiver
Version      : 1.0
Dateiname    : main.c
Erstelldatum : 23/05/2016 9:32:40 AM

********************************************************************************
Chip type         : ATmega32u4
CPU-Clock         : 16.0000 MHz

********************************************************************************
Datum                     Vers.    Comment / Change due
23/05/2016 9:32:40 AM     1.0      Created
13/09/2016 9:32:40 AM     2.0      Automatic Channel choice

*******************************************************************************/

/*------------------ Definitions for Delay Function --------------------------*/
#define F_CPU 16000000UL

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

/* MACROS */
#define   DIP_ADDRESS (DIP_SWITCH&0b11110000)>>4
/*------------------------ Global Variables  ---------------------------------*/
uint8_t Package_Data[32];
uint8_t rx_address[5] = {0xE7,0xE7,0xE7,0xE7,0x00};

/*----------------------------Functions--------------------------------------*/
void setLeds(char Red, char Green);
char nrf24_search_channel();
int getChecksum(uint8_t * Data);
void wdt_init(void) __attribute__((naked)) __attribute__((section(".init3")));

/*----------------------------- Mainprogram --------------------------------*/

int main(void)
{
  // I/O-Configurations
  //  RGB LED
  DDRB = 0b00000011;  // two outputs for rg leds
  //  DIP-Switch
  DDRD = 0b00000000;
  PORTD= 0b11110000;
  //  MAX485
  DDRE = 0b01000000;
  
  // Variables
  int ReceiveFailures = 0;
  int Channel = 0;
  uint8_t DataReady = 0;
  uint8_t PackageNumber = 0;
  
  //rx_address[4] = DIP_ADDRESS; // Get the Adress from DIP's
  
  // Clear the Channel Array
  for (Channel=0; Channel<513; Channel++)
  {
    DMX_Data[Channel]= 0;
  }
  
  setLeds(1,1);  // red green

  // NRF Init
  nrf24_init();
  nrf24_rx_address(rx_address); // Set the device addresses
  nrf24_config(5,PAYLOAD_LENGTH);  // Default Channel
  //nrf24_search_channel();       // Search Transmit Channel/Frequency
  
  // DMX Init
  SETBIT(PORTE,PORTE6);         // Set SN75176b to transmit
  init_DMX_TX();
  sei();                        // Global Interrupts Enable
  
  while(1)
  {
    // If the Transmitt Adress should be changed, the Device should do a Soft reset
    /*if (rx_address[4] != DIP_ADDRESS){
    soft_reset();
    }
    */
    
    /* Wait for Data from NRF */
    DataReady = nrf24_dataReady();
    if(DataReady == 1)
    {
      setLeds(0,1);          // green
      ReceiveFailures = 0;   // reset failure counter

      // Get the Data from the NRF
      nrf24_getData(Package_Data);
      
      // Check if the calculated Checksum equals the transmitted Checksum
      if (getChecksum(Package_Data) == Package_Data[CHANNELS_PER_PACKAGE+1])
      {
        PackageNumber = Package_Data[0];  // extract the number of the package
        /* Copy the received Data in the DMX Data Array
        (starting at 1 because 0 is the package number) */
        for (Channel = 1; Channel <= CHANNELS_PER_PACKAGE; Channel++)
        {
          // -1 because the DMX Data Array starts at 0
          DMX_Data[(PackageNumber*CHANNELS_PER_PACKAGE) + Channel-1] = Package_Data[Channel];
        }
      }
    }
    /* No Data ready from the NRF */
    else
    {
      DataReady = 0;
      //setLeds(1,1);         // red and green
      ReceiveFailures++ ;   // increase failure counter
    }
    
    /* If there was no Data received for a long time */
    if (ReceiveFailures > 50 && ReceiveFailures < 1000)
    {
      setLeds(1,1);           // red and green
    }
    else if(ReceiveFailures > 1000)
    {
      setLeds(1,0);
      nrf24_search_channel(); // search channels for data (endless until data is found)
    }
    
  }
}

int getChecksum(uint8_t * Data)
{
  int Checksum = 0;
  
  // starting at 1 because first index (0) is the number of the package
  for (int Counter = 1; Counter <= CHANNELS_PER_PACKAGE; Counter++)
  {
    Checksum += CHECKBIT(Package_Data[Counter],0);
  }
  
  return Checksum;
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


// scan for a transmiter device
char nrf24_search_channel()
{
  // Variables
  char WirelessChannel = 0;
  char NrOfTry;

  while (1) //Endless until a channel is found
  {
    // If the Transmitt Adress should be changed, the Device should do a Soft reset
    //if (rx_address[4] != DIP_ADDRESS){
    //  soft_reset();
    //}
    
    // Test only the first 80 Channels
    // Visit: https://en.wikipedia.org/wiki/List_of_WLAN_channels#Interference_concerns
    if (WirelessChannel > 20)
    {
      WirelessChannel = 1;
    }
    else { WirelessChannel++; }
    
    // Set new Channel
    nrf24_powerDown();
    _delay_ms(10);
    nrf24_config(WirelessChannel,PAYLOAD_LENGTH);
    _delay_ms(10);
    nrf24_powerUpRx();
    
    _delay_ms(100);

    for (NrOfTry = 0; NrOfTry < 25; NrOfTry++)    // Check for data 25 times per Channel
    {
      if (nrf24_dataReady())   // check for data
      {
        nrf24_getData(Package_Data);  // get the data
        // Check the Package (first byte must be between 0 and NUMBER_OF_PACKAGES_TO_SEND)
        if ((Package_Data[0] >= 0) && (Package_Data[0] < NUMBER_OF_PACKAGES_TO_SEND))
        {
          return 0;  // New Channel is found and Setup
        }
      }
      _delay_us(1000);   // delay a bit (25*10us = 0,25ms per channel)
    }
  }
}


// Function Implementation
// For more Information please visit: http://www.atmel.com/webdoc/AVRLibcReferenceManual/FAQ_1faq_softreset.html
void wdt_init(void)
{
  MCUSR = 0;
  wdt_disable();

  return;
}

