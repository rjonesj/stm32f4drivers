/*
  Serial Event example

  When new serial data arrives, this sketch adds it to a String.
  When a carriage return is received, the loop prints the string and transmits it over SPI.

  NOTE: The serialEvent() feature is not available on the Leonardo, Micro, or
  other ATmega32U4 based boards.

  created June 16 2020
  by rjonesj
*/
#include <SPI.h>
#define BUFF_LEN    100

bool stringComplete = false; //whether the string is complete
uint8_t t_buffer[BUFF_LEN];  //buffer to hold incoming data
uint32_t cnt = 0;
uint8_t temp;

//Initialize SPI slave.
void SPI_SlaveInit(void) 
{ 
  // Initialize SPI pins.
  pinMode(SCK, INPUT);
  pinMode(MOSI, INPUT);
  pinMode(MISO, OUTPUT);
  pinMode(SS, INPUT);
  
  // Enable SPI as slave.
  SPCR = (1 << SPE);
}

//This function returns SPDR Contents 
uint8_t SPI_SlaveReceive(void)
{
  /* Wait for reception complete */
  while(!(SPSR & (1<<SPIF)));
  /* Return Data Register */
  return SPDR;
}

//sends one byte of data 
void SPI_SlaveTransmit(uint8_t data)
{
  /* Start transmission */
  SPDR = data;
  
  /* Wait for transmission complete */
  while(!(SPSR & (1<<SPIF)));
}

// The setup() function runs after reset.
void setup() 
{
  // Initialize serial for troubleshooting.
  Serial.begin(9600);
 
  // Initialize SPI Slave.
  SPI_SlaveInit();
  
  Serial.println("Slave Initialized");
}

void loop() {
  //1. fist make sure that ss is low . so lets wait until ss is low 
  while(digitalRead(SS) );

  if(stringComplete){
    Serial.println((char*)t_buffer);
    temp = SPI_SlaveReceive(); //dummy read

    //Begin transmission
    SPI_SlaveTransmit(0xf1);
    temp = SPI_SlaveReceive(); //dummy read
    for(uint32_t i = 0 ; i < cnt; i++)
    {
      SPI_SlaveTransmit(t_buffer[i]);    
      temp = SPI_SlaveReceive(); //dummy read
    }
    cnt = 0;
    stringComplete = false;
    Serial.println("sent");
  } else {
    temp = SPI_SlaveReceive();  //dummy read
  }
}

/*
  SerialEvent occurs whenever a new data comes in the hardware serial RX. This
  routine is run between each time loop() runs, so using delay inside loop can
  delay response. Multiple bytes of data may be available.
*/
void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    if(cnt < BUFF_LEN) {
      // add it to the inputString if within BUFF_LEN limit
      t_buffer[cnt++] = inChar;    
    }
    // if the incoming character is a carriage return, set a flag so the main loop can
    // do something about it:
    if (inChar == '\r') {
      if(cnt == BUFF_LEN) {
        t_buffer[BUFF_LEN-1] = '\0';        
      } else {
        t_buffer[cnt++] = '\0';        
      }
      stringComplete = true;
    }  
  }
}
