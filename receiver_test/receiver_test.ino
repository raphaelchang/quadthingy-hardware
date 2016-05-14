/*
 Copyright (C) 2011 James Coliz, Jr. <maniacbug@ymail.com>

 This program is free software; you can redistribute it and/or
 modify it under the terms of the GNU General Public License
 version 2 as published by the Free Software Foundation.
 */

/**
 * Example RF Radio Ping Pair
 *
 * This is an example of how to use the RF24 class.  Write this sketch to two different nodes,
 * connect the role_pin to ground on one.  The ping node sends the current time to the pong node,
 * which responds by sending the value back.  The ping node can then see how long the whole cycle
 * took.
 */

#include <SPI.h>
#include <RF24.h>

# define BAUD 9600
# define CE 9
# define CS 10

RF24 radio(CE, CS);
// Radio pipe addresses for the 2 nodes to communicate.
const uint64_t pipes[2] = { 0xE7E7E7E7E7LL, 0xC1C1C1C1C1LL };
char new_command[10];
int new_index = 0;
unsigned long command = 0;

void setup(void)
{
  Serial.begin(BAUD);

  radio.begin();

  pinMode(14, OUTPUT);
  CORE_PIN13_CONFIG = PORT_PCR_MUX(0);
  CORE_PIN14_CONFIG = PORT_PCR_DSE | PORT_PCR_MUX(2);
  
  radio.openWritingPipe(pipes[1]);
  radio.openReadingPipe(1,pipes[0]);

  radio.enableDynamicPayloads() ;
  radio.setAutoAck( true ) ;
  radio.setCRCLength(RF24_CRC_16);
  radio.setChannel(76);
  //radio.setDataRate(RF24_2MBPS);
  radio.powerUp();
  radio.startListening();

  //
  // Dump the configuration of the rf unit for debugging
  //
  while (!Serial);
  radio.printDetails();
}

void loop(void)
{
  static int e;
  if (e++ % 10000 == 0)
  {
    radio.printDetails();
  }
  if (Serial.available() > 0)
  {
    char b = Serial.read();
    if (b == '\n')
    {
      new_command[new_index] = '\0';
      Serial.printf("%d\n", atoi(new_command));
      new_index = 0;
      command = atoi(new_command);
    }
    new_command[new_index++] = b;
  }
  // First, stop listening so we can talk.
  radio.stopListening();

  Serial.printf("Now sending %lu...",command);
  radio.write( &command, sizeof(unsigned long) );

  // Now, continue listening
  radio.startListening();

  // Wait here until we get a response, or timeout
  unsigned long started_waiting_at = millis();
  bool timeout = false;
  while ( ! radio.available() && ! timeout )
    if (millis() - started_waiting_at > 10 )
      timeout = true;

  // Describe the results
  if ( timeout )
  {
    Serial.printf("Failed, response timed out.\n\r");
//    Serial.printf("Timeout duration: %d\n\r", (1+radio.getMaxTimeout()/1000) ) ;
  }
  else
  {
    // Grab the response, compare, and send to debugging spew
    unsigned long result;
    radio.read( &result, sizeof(unsigned long) );

    // Spew it
    Serial.printf("Got response %lu\n\r",result);
  }

  delay(10);

}
