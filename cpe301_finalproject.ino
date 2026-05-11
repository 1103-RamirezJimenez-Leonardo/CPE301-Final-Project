//Noah Clark, Luis Angel Sarellano, and Leonardo Ramirez Jimenez

#include <Arduino.h>
#include <Wire.h>
#include <avr/io.h>
#include <avr/interrupt.h>

enum SystemState { OFF_STATE, IDLE_STATE, ACTIVE_STATE, ERROR_STATE };

volatile SystemState currentState = OFF_STATE;
volatile bool onButtonFlag = false;

unsigned long lastDisplayTime = 0;
unsigned long lastSensorReadTime = 0;

const unsigned long DISPLAY_INTERVAL = 2000UL;
const unsigned long SENSOR_INTERVAL = 2000UL;

int temperatureC = 0;
bool tempInitialized = false;
bool dhtGood = false;

int tempWarningThreshold = 30;
const int TEMP_ERROR_OFFSET = 3;
const int TEMP_POT_MIN = 20;
const int TEMP_POT_MAX = 40;

#define BLUE_LED   PA0
#define GREEN_LED  PA1
#define YELLOW_LED PA2
#define RED_LED    PA3
#define FAN_PIN    PA4
#define BUZZER_PIN PA5
#define RESET_BTN  PA6
#define OFF_BTN    PA7

#define LCD_RS PC0
#define LCD_E  PC1
#define LCD_D4 PC2
#define LCD_D5 PC3
#define LCD_D6 PC4
#define LCD_D7 PC5

#define DHT_DDR  DDRH
#define DHT_PORT PORTH
#define DHT_PIN  PINH
#define DHT_BIT  PH4

void waitUs(unsigned long us) {
  unsigned long start = micros();
  while ((micros() - start) < us) {}
}

void waitMs(unsigned long ms) {
  unsigned long start = millis();
  while ((millis() - start) < ms) {}
}

void U0Init() {
  unsigned int ubrr = 103;
  UBRR0H = (unsigned char)(ubrr >> 8);
  UBRR0L = (unsigned char)ubrr;
  UCSR0B = (1 << TXEN0);
  UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

void U0putchar(char c) {
  while (!(UCSR0A & (1 << UDRE0))) {}
  UDR0 = c;
}

void U0puts(const char *str) {
  while (*str) U0putchar(*str++);
}

void U0printInt(int value) {
  char buffer[12];
  itoa(value, buffer, 10);
  U0puts(buffer);
}

void adc_init() {
  ADMUX = (1 << REFS0);
  ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
}

unsigned int adc_read(unsigned char channel) {
  ADMUX = (ADMUX & 0xF0) | (channel & 0x0F);
  ADCSRA |= (1 << ADSC);
  while (ADCSRA & (1 << ADSC)) {}
  return ADC;
}

void lcdPulseEnable() {
  PORTC |= (1 << LCD_E);
  waitUs(1);
  PORTC &= ~(1 << LCD_E);
  waitUs(100);
}

void lcdWriteNibble(unsigned char nibble) {
  PORTC &= ~((1 << LCD_D4) | (1 << LCD_D5) | (1 << LCD_D6) | (1 << LCD_D7));

  if (nibble & 0x01) PORTC |= (1 << LCD_D4);
  if (nibble & 0x02) PORTC |= (1 << LCD_D5);
  if (nibble & 0x04) PORTC |= (1 << LCD_D6);
  if (nibble & 0x08) PORTC |= (1 << LCD_D7);

  lcdPulseEnable();
}

void lcdCommand(unsigned char cmd) {
  PORTC &= ~(1 << LCD_RS);
  lcdWriteNibble(cmd >> 4);
  lcdWriteNibble(cmd & 0x0F);
  waitUs(2000);
}

void lcdData(unsigned char data) {
  PORTC |= (1 << LCD_RS);
  lcdWriteNibble(data >> 4);
  lcdWriteNibble(data & 0x0F);
  waitUs(100);
}

void lcdInit() {
  DDRC |= (1 << LCD_RS) | (1 << LCD_E) | (1 << LCD_D4) |
          (1 << LCD_D5) | (1 << LCD_D6) | (1 << LCD_D7);

  waitMs(50);
  PORTC &= ~(1 << LCD_RS);

  lcdWriteNibble(0x03);
  waitMs(5);
  lcdWriteNibble(0x03);
  waitUs(150);
  lcdWriteNibble(0x03);
  waitUs(150);
  lcdWriteNibble(0x02);

  lcdCommand(0x28);
  lcdCommand(0x0C);
  lcdCommand(0x06);
  lcdCommand(0x01);
  waitMs(2);
}

void lcdClear() {
  lcdCommand(0x01);
  waitMs(2);
}

void lcdSetCursor(unsigned char col, unsigned char row) {
  unsigned char address = row == 0 ? col : 0x40 + col;
  lcdCommand(0x80 | address);
}

void lcdPrint(const char *str) {
  while (*str) lcdData(*str++);
}

void lcdPrintInt(int value) {
  char buffer[8];
  itoa(value, buffer, 10);
  lcdPrint(buffer);
}

byte bcdToDec(byte val) {
  return ((val / 16 * 10) + (val % 16));
}

void getRTC(int &hour, int &minute, int &second) {
  Wire.beginTransmission(0x68);
  Wire.write(0x00);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 3);

  second = bcdToDec(Wire.read() & 0x7F);
  minute = bcdToDec(Wire.read());
  hour = bcdToDec(Wire.read() & 0x3F);
}

void printRTCTimeUART() {
  int h, m, s;
  getRTC(h, m, s);

  if (h < 10) U0putchar('0');
  U0printInt(h);
  U0putchar(':');

  if (m < 10) U0putchar('0');
  U0printInt(m);
  U0putchar(':');

  if (s < 10) U0putchar('0');
  U0printInt(s);
}

int getTempMaxThreshold() {
  return tempWarningThreshold + TEMP_ERROR_OFFSET;
}

void logEvent(const char *eventText) {
  U0puts("[");
  printRTCTimeUART();
  U0puts("] ");
  U0puts(eventText);

  if (tempInitialized) {
    U0puts(" | Temp=");
    U0printInt(temperatureC);
    U0puts("C");
  } else {
    U0puts(" | Temp=Reading");
  }

  U0puts(" | Warning=");
  U0printInt(tempWarningThreshold);
  U0puts("C | Max=");
  U0printInt(getTempMaxThreshold());
  U0puts("C\r\n");
}

bool waitForDHTLevel(bool level, unsigned int timeoutUs) {
  unsigned long start = micros();

  while (((DHT_PIN & (1 << DHT_BIT)) != 0) != level) {
    if ((micros() - start) > timeoutUs) return false;
  }

  return true;
}

bool readDHT11Temperature(int &temp) {
  byte data[5] = {0, 0, 0, 0, 0};

  DHT_DDR |= (1 << DHT_BIT);
  DHT_PORT &= ~(1 << DHT_BIT);
  waitUs(18000);

  DHT_PORT |= (1 << DHT_BIT);
  waitUs(30);

  DHT_DDR &= ~(1 << DHT_BIT);
  DHT_PORT |= (1 << DHT_BIT);

  if (!waitForDHTLevel(false, 100)) return false;
  if (!waitForDHTLevel(true, 100)) return false;
  if (!waitForDHTLevel(false, 100)) return false;

  cli();

  for (int i = 0; i < 40; i++) {
    if (!waitForDHTLevel(true, 100)) {
      sei();
      return false;
    }

    unsigned long startHigh = micros();

    if (!waitForDHTLevel(false, 120)) {
      sei();
      return false;
    }

    unsigned long highTime = micros() - startHigh;
    data[i / 8] <<= 1;

    if (highTime > 40) data[i / 8] |= 1;
  }

  sei();

  byte checksum = data[0] + data[1] + data[2] + data[3];

  if (checksum != data[4]) return false;

  temp = data[2];
  return true;
}

void allOutputsOff() {
  PORTA &= ~((1 << BLUE_LED) | (1 << GREEN_LED) | (1 << YELLOW_LED) |
             (1 << RED_LED) | (1 << FAN_PIN) | (1 << BUZZER_PIN));
}

void outputsOffState() {
  allOutputsOff();
  PORTA |= (1 << BLUE_LED);
}

void outputsIdleState() {
  allOutputsOff();
  PORTA |= (1 << GREEN_LED);
}

void outputsActiveState() {
  allOutputsOff();
  PORTA |= (1 << YELLOW_LED);
  PORTA |= (1 << FAN_PIN);
}

void outputsErrorState() {
  allOutputsOff();
  PORTA |= (1 << RED_LED);

  PORTA |= (1 << BUZZER_PIN);
  waitUs(250);
  PORTA &= ~(1 << BUZZER_PIN);
  waitUs(250);
}

bool resetButtonPressed() {
  return !(PINA & (1 << RESET_BTN));
}

bool offButtonPressed() {
  return !(PINA & (1 << OFF_BTN));
}

void onButtonISR() {
  if (currentState == OFF_STATE) onButtonFlag = true;
}

void readThresholdPot() {
  unsigned int tempPot = adc_read(0);

  tempWarningThreshold = TEMP_POT_MIN +
    ((tempPot * (TEMP_POT_MAX - TEMP_POT_MIN)) / 1023);
}

void readSensorNow() {
  int t;
  dhtGood = readDHT11Temperature(t);

  if (dhtGood) {
    temperatureC = t;
    tempInitialized = true;
  }
}

void readSensorTimed() {
  if (!tempInitialized || millis() - lastSensorReadTime >= SENSOR_INTERVAL) {
    lastSensorReadTime = millis();
    readSensorNow();
  }
}

bool aboveWarningThreshold() {
  return tempInitialized && temperatureC >= tempWarningThreshold;
}

bool belowWarningThreshold() {
  return tempInitialized && temperatureC < tempWarningThreshold;
}

bool aboveMaxThreshold() {
  return tempInitialized && temperatureC >= getTempMaxThreshold();
}

void displayOff() {
  lcdClear();
  lcdSetCursor(0, 0);
  lcdPrint("System Off");
  lcdSetCursor(0, 1);
  lcdPrint("Press ON");
}

void displayIdle() {
  int h, m, s;
  getRTC(h, m, s);

  lcdClear();
  lcdSetCursor(0, 0);

  if (tempInitialized) {
    lcdPrint("T:");
    lcdPrintInt(temperatureC);
    lcdPrint(" L:");
    lcdPrintInt(tempWarningThreshold);
    lcdPrint(" M:");
    lcdPrintInt(getTempMaxThreshold());
  } else {
    lcdPrint("T:Reading...");
  }

  lcdSetCursor(0, 1);
  lcdPrint("IDLE ");

  if (h < 10) lcdPrint("0");
  lcdPrintInt(h);
  lcdPrint(":");

  if (m < 10) lcdPrint("0");
  lcdPrintInt(m);
}

void displayActive() {
  lcdClear();
  lcdSetCursor(0, 0);
  lcdPrint("ACTIVE FAN ON");

  lcdSetCursor(0, 1);

  if (tempInitialized) {
    lcdPrint("T:");
    lcdPrintInt(temperatureC);
    lcdPrint(" L:");
    lcdPrintInt(tempWarningThreshold);
  } else {
    lcdPrint("T:Reading...");
  }
}

void displayError() {
  lcdClear();
  lcdSetCursor(0, 0);
  lcdPrint("ERROR OVERHEAT");

  lcdSetCursor(0, 1);

  if (tempInitialized) {
    lcdPrint("T:");
    lcdPrintInt(temperatureC);
    lcdPrint(" M:");
    lcdPrintInt(getTempMaxThreshold());
  } else {
    lcdPrint("T:Reading...");
  }
}

void enterState(SystemState newState) {
  currentState = newState;
  lastDisplayTime = 0;

  readThresholdPot();

  if (newState == OFF_STATE) {
    outputsOffState();
    displayOff();
  } else if (newState == IDLE_STATE) {
    outputsIdleState();
    displayIdle();
    logEvent("ENTERED IDLE STATE");
  } else if (newState == ACTIVE_STATE) {
    outputsActiveState();
    displayActive();
    logEvent("ENTERED ACTIVE STATE");
  } else if (newState == ERROR_STATE) {
    outputsErrorState();
    displayError();
    logEvent("ENTERED ERROR STATE");
  }
}

void setup() {
  DDRA |= (1 << BLUE_LED) | (1 << GREEN_LED) | (1 << YELLOW_LED) |
          (1 << RED_LED) | (1 << FAN_PIN) | (1 << BUZZER_PIN);

  DDRA &= ~((1 << RESET_BTN) | (1 << OFF_BTN));
  PORTA |= (1 << RESET_BTN) | (1 << OFF_BTN);

  DDRE &= ~(1 << PE4);
  PORTE |= (1 << PE4);

  DHT_DDR &= ~(1 << DHT_BIT);
  DHT_PORT |= (1 << DHT_BIT);

  adc_init();
  U0Init();
  Wire.begin();
  lcdInit();

  attachInterrupt(0, onButtonISR, FALLING);

  U0puts("System booted\r\n");

  waitMs(1200);
  readThresholdPot();
  readSensorNow();

  enterState(OFF_STATE);
}

void loop() {
  if (onButtonFlag) {
    onButtonFlag = false;
    if (currentState == OFF_STATE) enterState(IDLE_STATE);
  }

  switch (currentState) {
    case OFF_STATE:
      outputsOffState();
      break;

    case IDLE_STATE:
      outputsIdleState();
      readThresholdPot();
      readSensorTimed();

      if (offButtonPressed()) {
        enterState(OFF_STATE);
        break;
      }

      if (aboveMaxThreshold()) {
        enterState(ERROR_STATE);
        break;
      }

      if (aboveWarningThreshold()) {
        enterState(ACTIVE_STATE);
        break;
      }

      if (millis() - lastDisplayTime >= DISPLAY_INTERVAL) {
        lastDisplayTime = millis();
        displayIdle();
        logEvent("IDLE DATA LOG");
      }

      break;

    case ACTIVE_STATE:
      outputsActiveState();
      readThresholdPot();
      readSensorTimed();

      if (offButtonPressed()) {
        enterState(OFF_STATE);
        break;
      }

      if (aboveMaxThreshold()) {
        enterState(ERROR_STATE);
        break;
      }

      if (belowWarningThreshold()) {
        enterState(IDLE_STATE);
        break;
      }

      if (millis() - lastDisplayTime >= DISPLAY_INTERVAL) {
        lastDisplayTime = millis();
        displayActive();
        logEvent("ACTIVE DATA LOG");
      }

      break;

    case ERROR_STATE:
      outputsErrorState();
      readThresholdPot();
      readSensorTimed();

      if (resetButtonPressed()) {
        if (aboveWarningThreshold()) enterState(ACTIVE_STATE);
        else enterState(IDLE_STATE);
        break;
      }

      if (millis() - lastDisplayTime >= DISPLAY_INTERVAL) {
        lastDisplayTime = millis();
        displayError();
        logEvent("ERROR STILL ACTIVE");
      }

      break;
  }
}