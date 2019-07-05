#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"
#include "Adafruit_BLEMIDI.h"
#include "Adafruit_MPR121.h"
#include "BluefruitConfig.h"
#include "pitchToNote.h"

//#define DEBUG_MODE 1 //Uncomment this line to operate in debug mode. In debug mode, it will just read the capacitance and analog input values and print them. Comment out this line entirely to operate in normal (non-debug) mode.

#define BLE_FACTORY_RESET_ENABLE       0 //If 1, the BLE device will factory reset and will revert any custom device name you've programmed into it. Avoid this unless something's really gone wrong in the BLE chip.
#define MINIMUM_FIRMWARE_VERSION  "0.7.0"

int xPin = A2;
int yPin = A1;
int zPin = A0;

#define ADC_MAX 1024

#define ACCELEROMETER_MIN 400
#define ACCELEROMETER_MAX 650
#define ACCELEROMETER_RANGE (ACCELEROMETER_MAX - ACCELEROMETER_MIN)

#define PITCH_MAX 16383
int xValRaw, yValRaw, zValRaw;
float xValNorm, yValNorm, zValNorm;

void readRawValues()
{
  xValRaw = analogRead(xPin);
  yValRaw = analogRead(yPin);
  zValRaw = analogRead(zPin);
}

void normalize()
{
  /*
  xValNorm = ((float)xValRaw / ADC_MAX);
  yValNorm = ((float)yValRaw / ADC_MAX);
  zValNorm = ((float)zValRaw / ADC_MAX);
  */
  xValNorm = ((float)(xValRaw-ACCELEROMETER_MIN) / ACCELEROMETER_RANGE);
  if (xValNorm < 0.0) xValNorm = 0.0;
  if (xValNorm > 1.0) xValNorm = 1.0;
  
  yValNorm = ((float)(yValRaw-ACCELEROMETER_MIN) / ACCELEROMETER_RANGE);
  if (yValNorm < 0.0) yValNorm = 0.0;
  if (yValNorm > 1.0) yValNorm = 1.0;
  
  zValNorm = ((float)(zValRaw-ACCELEROMETER_MIN) / ACCELEROMETER_RANGE);
  if (zValNorm < 0.0) zValNorm = 0.0;
  if (zValNorm > 1.0) zValNorm = 1.0;
}

Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);
Adafruit_BLEMIDI midi(ble);
Adafruit_MPR121 cap = Adafruit_MPR121();

int base_notes[4] = {pitchC3, pitchC4, pitchC5, pitchC6};

int current_note = pitchC4;
int new_note = current_note;

int base_note = base_notes[0];
int new_base_note = base_note;

uint16_t current_bend = 0;
uint16_t new_bend = current_bend;

unsigned long curr_time = 0;
unsigned long last_note_on_time = 0;
unsigned long max_note_on_time = 5000;

bool isConnected = false;

// A small helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

void connected(void)
{
  isConnected = true;
  Serial.println(F("BLE connected."));
  delay(1000);
}

void disconnected(void)
{
  Serial.println("BLE disconnected");
  isConnected = false;
}

void BleMidiRX(uint16_t timestamp, uint8_t status, uint8_t byte1, uint8_t byte2)
{
  Serial.print("[MIDI ");
  Serial.print(timestamp);
  Serial.print(" ] ");

  Serial.print(status, HEX); Serial.print(" ");
  Serial.print(byte1 , HEX); Serial.print(" ");
  Serial.print(byte2 , HEX); Serial.print(" ");

  Serial.println();
}

void noteOn(byte channel, byte pitch, byte velocity)
{
  midi.send(0x90 | channel, pitch, velocity);
}

void noteOff(byte channel, byte pitch, byte velocity)
{
  midi.send(0x80 | channel, pitch, velocity);
}

//val should be a 14-bit value representing pitch bend amount. 16383 is max bend up. 0 is max bend down. 8192 is no bend.
void pitchBend(uint16_t val)
{
  byte lsb = val & 0x007F;
  byte msb = (val & 0x3F10) >> 7;
  midi.send(0xE0, lsb, msb);
}

void setup_ble_midi()
{
  Serial.print(F("Initialising the Bluefruit LE module: "));

  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  Serial.println( F("Bluefruit found.") );

  if ( BLE_FACTORY_RESET_ENABLE )
  {
    /* Perform a factory reset to make sure everything is in a known state */
    Serial.println(F("Performing a factory reset: "));
    if ( ! ble.factoryReset() ) {
      error(F("Couldn't factory reset"));
    }
  }

  //ble.sendCommandCheckOK(F("AT+uartflow=off"));
  ble.echo(false);

  Serial.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  ble.info();

  /* Set BLE callbacks */
  ble.setConnectCallback(connected);
  ble.setDisconnectCallback(disconnected);

  // Set MIDI RX callback
  midi.setRxCallback(BleMidiRX);

  Serial.println(F("Enabling MIDI."));
  if ( ! midi.begin(true) )
  {
    error(F("Could not enable MIDI"));
  }

  ble.verbose(false);
  Serial.println(F("Waiting for BLE connection..."));
}

void setup(void)
{
  delay(500);
  Serial.begin(115200);
  
#ifndef DEBUG_MODE //No BLE activity in debug mode. Only sensor readings.  
  setup_ble_midi();
#endif
}

void loop(void)
{  
#ifndef DEBUG_MODE //No BLE activity in debug mode. Only sensor readings.
  // interval for each scanning ~ 500ms (non blocking)
  ble.update(500);

  // bail if not connected
  if (!isConnected)
  {
    //Serial.println(F("Waiting for BLE connection..."));
    delay(1000); //This might not work. It may block future connections. Test disconnecting and reconnecting to see if this needs more work.
  }
#endif

  curr_time = millis();

  readRawValues();
  normalize();
  
#ifdef DEBUG_MODE
  //Serial.print("X: "); Serial.print(xValNorm); Serial.print(" ("); Serial.print(xValRaw); Serial.println(")");
  //Serial.print("Y: "); Serial.print(yValNorm); Serial.print(" ("); Serial.print(yValRaw); Serial.println(")");
  //Serial.print("Z: "); Serial.print(zValNorm); Serial.print(" ("); Serial.print(zValRaw); Serial.println(")\n");
  Serial.print(xValNorm);Serial.print(",");Serial.print(yValNorm);Serial.print(",");Serial.println(zValNorm);
  delay(100);
#else

  new_base_note = base_notes[(int)(4 * zValNorm)];
  if (new_base_note != base_note)
  {
    base_note = new_base_note;
  }

  new_bend = (uint16_t)(PITCH_MAX * yValNorm);
  if (new_bend != current_bend)
  {
    current_bend = new_bend;
    pitchBend(current_bend);
  }

  new_note = (int)(16 * xValNorm) + base_note; //Map the stretch value to a range of 16 notes, beginning at C4
  if (new_note != current_note)
  {
    noteOff(0, current_note, 64);
    current_note = new_note;
    noteOn(0, current_note, 64);
    //delay(50);
  }
  else
  {
    if ((last_note_on_time - curr_time) > max_note_on_time)
    {
      noteOff(0, current_note, 64);
    }
  }
  delay(100);

#endif
}
