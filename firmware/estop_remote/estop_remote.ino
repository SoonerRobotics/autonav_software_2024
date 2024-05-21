#include <Adafruit_GFX.h>
#include <Adafruit_SSD1351.h>
#include <SPI.h>
#include <RH_RF95.h>
#include "estop_common.h"

// RFM95 Definitions
#define RFM95_CS   16
#define RFM95_INT  21
#define RFM95_RST  17
#define RF95_FREQ 915.0

// Screen dimensions
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 128 // Change this to 96 for 1.27" OLED.

// SPI
#define SCLK_PIN 14
#define MOSI_PIN 15
#define DC_PIN   12
#define CS_PIN   25
#define RST_PIN  11

// Buttons
#define BTN_MSTART 24
#define BTN_MSTOP 29
#define BTN_ESTOP 28

#define BTN_CENTER 13
#define BTN_UP 10
#define BTN_DOWN 6
#define BTN_LEFT 9
#define BTN_RIGHT 5

// Other
#define BATT_READ 26

// Color definitions
#define	BLACK           0x0000
#define WHITE           0xFFFF

RH_RF95 rf95(RFM95_CS, RFM95_INT);

Adafruit_SSD1351 tft = Adafruit_SSD1351(SCREEN_WIDTH, SCREEN_HEIGHT, &SPI1, CS_PIN, DC_PIN, RST_PIN);

// Parameters
float battery_min_voltage = 3.5f;
float battery_max_voltage = 3.9f;
int screen_refresh_period_ms = 300;
int heartbeat_period_ms = 250;
int handshake_period_ms = 500;
int message_resend_period_ms = 500;
int max_resends = 2;

// Storage variables
unsigned long last_display_update = 0;
unsigned long last_heartbeat = 0;
unsigned long last_handshake_attempt = 0;
unsigned long last_received_message = 0;
unsigned long last_sent_message = 0;
int current_resends = 0;
bool msg_indicator = false;

// int led_status = 0;
float battery_percentage_smoothed = -1.0f;
uint8_t expecting_response_id = MSG_NONE_ID;
bool is_connected = false;

volatile int signal_to_send = NONE_SIGNAL;

RadioPacket outgoing_message;

// Errors
static const int NO_ERROR = 0;
static const int ERROR_WRONG_RESPONSE_ID = 1;
static const int ERROR_WRONG_SIGNAL_RECEIVED = 2;
volatile int error_state = NO_ERROR;

void estopButtonInterrupt()
{
  signal_to_send = ESTOP_SIGNAL;
}

void mobilityStopButtonInterrupt()
{
  if (signal_to_send == NONE_SIGNAL) {
    signal_to_send = MOB_STOP_SIGNAL;
  }
}

void mobilityStartButtonInterrupt()
{
  if (signal_to_send == NONE_SIGNAL) {
    signal_to_send = MOB_START_SIGNAL;
  }
}


void setup()
{
  // delay(2000);
  Serial.begin(115200);

  // LED
  // pinMode(LED_PIN, OUTPUT);
  // digitalWrite(LED_PIN, led_status);

  // Setup SPI
  SPI1.setTX(MOSI_PIN);
  SPI1.setRX(8);
  SPI1.setSCK(SCLK_PIN);
  SPI1.begin();

  // display init to blank screen
  tft.begin(32000000);

  tft.setTextSize(2);
  tft.setTextColor(WHITE, BLACK);
  
  delay(400);
  tft.fillScreen(BLACK);

  // reset rfm95
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  // init rfm95
  int result = rf95.init();
  int result2 = rf95.setFrequency(RF95_FREQ);
  rf95.setTxPower(23, false);

  Serial.println(result);
  Serial.println(result2);

  // button interrupt setup
  pinMode(BTN_ESTOP, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BTN_ESTOP), estopButtonInterrupt, FALLING);

  pinMode(BTN_MSTOP, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BTN_MSTOP), mobilityStopButtonInterrupt, FALLING);

  pinMode(BTN_MSTART, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BTN_MSTART), mobilityStartButtonInterrupt, FALLING);

  strncpy(outgoing_message.password, GLOBAL_PASSWORD, sizeof(GLOBAL_PASSWORD));

  pinMode(BATT_READ, INPUT);
}

void updateBatteryDisplay() {
  // Battery status
  float battery_volage = analogRead(BATT_READ);
  battery_volage *= 2;    // we divided by 2, so multiply back
  battery_volage *= 3.3;  // Multiply by 3.3V, our reference voltage
  battery_volage /= 1024; // convert to voltage

  float estimated_battery_percentage = (battery_volage - battery_min_voltage) / (battery_max_voltage - battery_min_voltage) * 100.0f; // lerp between bat min and bat max

  if (estimated_battery_percentage > 100) {
    estimated_battery_percentage = 100;
  }

  if (estimated_battery_percentage < 0) {
    estimated_battery_percentage = 0;
  }

  // Smooth out battery percentage by averaging it with the last value
  if (battery_percentage_smoothed < 0) {
    battery_percentage_smoothed = estimated_battery_percentage;
  } else {
    battery_percentage_smoothed = 0.7 * battery_percentage_smoothed + 0.3 * estimated_battery_percentage;
  }

  tft.setCursor(80, 110);

  // Print leading 0
  if (battery_percentage_smoothed < 10) {
    tft.printf("LOW");
    return;
  }

  // lcd.print(battery_percentage_smoothed, 0);
  // lcd.print("%");
  tft.printf("%3.0f%%", battery_percentage_smoothed);
}

void updateDisplay() {

  if (error_state != NO_ERROR) {
    tft.setCursor(0, 0);
    tft.print("ERROR!");
    tft.setCursor(0, 20);
    tft.print("Code: ");
    tft.print((float)error_state, 0);

    updateBatteryDisplay();
    return;
  }

  if (is_connected && signal_to_send != NONE_SIGNAL) {
    tft.setCursor(0, 0);
    tft.print("Connected");
    tft.setCursor(0, 110);
    if (msg_indicator) {
      tft.write(0xDB);
    } else {
      tft.write(0xDE);
    }
    tft.printf(" %02d", rf95.lastSNR());
    tft.setCursor(0, 20);
    tft.print("Sending: ");
    tft.print((float)signal_to_send, 0);

    updateBatteryDisplay();
    return;
  }

  // Connection status
  if (is_connected) {
    tft.setCursor(0, 0);
    tft.print("Connected");
    tft.setCursor(0, 110);
    if (msg_indicator) {
      tft.write(0xDB);
    } else {
      tft.write(0xDE);
    }
    tft.printf(" %02d", rf95.lastSNR());
    if (current_resends > 0) {
        tft.setCursor(0, 20);
        tft.printf("Resends: %01d", current_resends);
    } else {
        tft.setCursor(0, 20);
        tft.print("          ");
    }
  } else {
    tft.setCursor(0, 0);
    tft.print("Waiting.");
    int cur_secs = millis() / 1000;
    switch (cur_secs % 3) {
      case 0: tft.print("  "); break;
      case 1: tft.print(". "); break;
      case 2: tft.print(".."); break;
    }

    tft.setCursor(0, 20);
    tft.print("          ");
  }

  updateBatteryDisplay();
}

void reachedError(int error_type) {
    error_state = error_type;
    signal_to_send = ESTOP_SIGNAL;

    outgoing_message.id = MSG_SIGNAL_ID;
    outgoing_message.signal = signal_to_send;

    rf95.send((uint8_t*)&outgoing_message, sizeof(outgoing_message));
    rf95.waitPacketSent();}

void loop()
{

  if (rf95.available())
  {
    // Blink LED on receive
    // led_status = !led_status;
    // digitalWrite(LED_PIN, led_status);

    uint8_t buf[sizeof(RadioPacket)];
    uint8_t len = sizeof(buf);

    if (rf95.recv(buf, &len))
    {
      Serial.println("Received!");
      // Convert packet to character array to determine message contents
      RadioPacket incoming_message = *(RadioPacket*)buf;

      // Check password
      if (strcmp(GLOBAL_PASSWORD, incoming_message.password) == 0) {

        msg_indicator = !msg_indicator;

        last_received_message = millis();

        // Clear expecting response
        if (incoming_message.id == expecting_response_id) {
            expecting_response_id = MSG_NONE_ID;
            current_resends = 0;
        } else {
            // Error! Relay responded with wrong response!
            // EStop due to unknown state!
            reachedError(ERROR_WRONG_RESPONSE_ID);
        }

        // Handshake reply
        if (!is_connected && incoming_message.id == MSG_HANDSHAKE_REPLY_ID) {
            is_connected = true;
        }

        // Signal reply
        if (incoming_message.id == MSG_SIGNAL_REPLY_ID) {
            if (incoming_message.signal == signal_to_send) {
                signal_to_send = NONE_SIGNAL;
            } else {
                // Error! Relay responded with wrong signal!
                // EStop due to unknown state!
                reachedError(ERROR_WRONG_SIGNAL_RECEIVED);
            }
        }
      }
    }
  }

  // Update display regularly, except when we are expecting a response
  if ((expecting_response_id == MSG_NONE_ID || expecting_response_id == MSG_HANDSHAKE_REPLY_ID) && (millis() - last_display_update) > screen_refresh_period_ms) {
    last_display_update = millis();
    updateDisplay();
  }

//   // Mark us as disconnected if we haven't received an ACK in a while
//   if (is_connected && (millis() - last_received_message) > connection_timeout_ms) {
//     is_connected = false;
//   }

  // Resend messages
  if (is_connected && expecting_response_id != MSG_NONE_ID && (millis() - last_sent_message) > message_resend_period_ms) {
    // Disconnect after certain number of tries
    if (current_resends >= max_resends) {
        is_connected = false;
        expecting_response_id = MSG_NONE_ID;
        signal_to_send = NONE_SIGNAL;
    }

    current_resends += 1;

    // Update the display to reflect resend
    updateDisplay();

    rf95.send((uint8_t*)&outgoing_message, sizeof(outgoing_message));
    rf95.waitPacketSent();

    last_sent_message = millis();
  }

  // Send a signal if we have one waiting
  if (is_connected && expecting_response_id == MSG_NONE_ID && signal_to_send != NONE_SIGNAL) {
    outgoing_message.id = MSG_SIGNAL_ID;
    outgoing_message.signal = signal_to_send;

    rf95.send((uint8_t*)&outgoing_message, sizeof(outgoing_message));
    rf95.waitPacketSent();

    last_sent_message = millis();
    expecting_response_id = MSG_SIGNAL_REPLY_ID;
  }

  // Send a hearbeat periodically if we haven't received an ACK in a while
  if (is_connected && expecting_response_id == MSG_NONE_ID && (millis() - last_heartbeat) > heartbeat_period_ms) {
    last_heartbeat = millis();
    outgoing_message.id = MSG_INIT_HEARTBEAT_ID;

    rf95.send((uint8_t*)&outgoing_message, sizeof(outgoing_message));
    rf95.waitPacketSent();

    last_sent_message = millis();
    expecting_response_id = MSG_HEARTBEAT_REPLY_ID;
  }

  // Send a hearbeat periodically if we are not connected
  if (!is_connected && (millis() - last_handshake_attempt) > handshake_period_ms) {
    last_handshake_attempt = millis();
    outgoing_message.id = MSG_INIT_HANDSHAKE_ID;

    rf95.send((uint8_t*)&outgoing_message, sizeof(outgoing_message));
    rf95.waitPacketSent();

    last_sent_message = millis();
    expecting_response_id = MSG_HANDSHAKE_REPLY_ID;
  }
}