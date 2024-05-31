#include <SPI.h>
#include <ACAN2515.h>
#include <Adafruit_NeoPixel.h>
#include "ArduinoJson.h"
#include <CONBus.h>
#include <CANBusDriver.h>

static const int BLINK_PERIOD_MS = 500;
static const int DEFAULT_BRIGHTNESS = 50;

static const int CLED_COUNT = 24;
static const int CLED_PIN = 20;
static const int ESTOP_PIN = 6;
static const int W_PIN = 19;

static const byte MCP2515_SCK = 2;
static const byte MCP2515_MOSI = 3;
static const byte MCP2515_MISO = 0;

static const byte MCP2515_CS  = 1 ; // CS input of MCP2515 (adapt to your design) 
static const byte MCP2515_INT =  4 ; // INT output of MCP2515 (adapt to your design)

Adafruit_NeoPixel strip(CLED_COUNT, CLED_PIN, NEO_GRB);

// Start as mobility start and not autonomous
bool is_estopped = false;
bool is_mobility_stopped = false;
bool is_autonomous = false;
bool is_eco = false;
int current_brightness = DEFAULT_BRIGHTNESS;

// Json packet
StaticJsonDocument<256> json_in;

// Default to fading purple as a "connecting" state
int color_mode = 0;
int current_color = strip.Color(0,255,255);

// Setup CONBus variables
CONBus::CONBus conbus;
CONBus::CANBusDriver conbus_can(conbus, 13); // device id 13

int current_blink_period = BLINK_PERIOD_MS; // how fast the light would blink per

typedef struct SafetyLightsMessage { 
    uint8_t autonomous: 1;
    uint8_t eco: 1;
    uint8_t mode: 6;
    uint8_t brightness;

    uint8_t red;
    uint8_t green;
    uint8_t blue;
    uint8_t blink_period;
} SafetyLightsMessage;

SafetyLightsMessage last_CAN_message;

ACAN2515 can (MCP2515_CS, SPI, MCP2515_INT) ;

static const uint32_t QUARTZ_FREQUENCY = 8UL * 1000UL * 1000UL ; // 8 MHz

void modeSelector(){ // RGB mode selector
  switch(color_mode){
    case 0: //Solid
      colorSolid();
      break;
    case 1: //Flash
      colorFlash();
      break;
    case 2: //Fade
      colorFade();
      break;
    // case 3: //party mode?
    //   partyTime();
    //   break;
    default:
      colorSolid();
      break;
   }
}

CANMessage frame;

void onCanReceive() {
  can.isr();
  can.receive(frame);

  conbus_can.readCanMessage(frame.id, frame.data);
}

void setup () {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(W_PIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  Serial.begin();

  SPI.setSCK(MCP2515_SCK);
  SPI.setTX(MCP2515_MOSI);
  SPI.setRX(MCP2515_MISO);
  SPI.setCS(MCP2515_CS);
  SPI.begin();

  strip.begin();
  strip.show();
  strip.setBrightness(current_brightness);

  analogWriteFreq(100);  

  Serial.println ("Configure ACAN2515") ;
  ACAN2515Settings settings (QUARTZ_FREQUENCY, 100UL * 1000UL) ; // CAN bit rate 100 kb/s
  settings.mRequestedMode = ACAN2515Settings::NormalMode ; // Select Normal mode
  const uint16_t errorCode = can.begin (settings, [] { can.isr () ; }) ;

  if (errorCode == 0) {
    Serial.println("L");
  }
  else{
    Serial.print ("Configuration error 0x") ;
    Serial.println (errorCode, HEX) ;
  }

  // Create CONBus registers
  // The addresses can be anything from 0 to 255
  // Both the blink period and the color of the leds
  conbus.addRegister(7, &current_blink_period);
  conbus.addRegister(99, &current_color);

}

void loop() {
  // Serial.println("test");
  // frame.id = 0x13;
  // frame.len = 6;
  // can.tryToSend (frame) ;

  is_estopped = !digitalRead(ESTOP_PIN);

  if(is_estopped == false) {
    digitalWrite(LED_BUILTIN, HIGH);
  }  
  else {
    digitalWrite(LED_BUILTIN, LOW);
  }

  if (Serial.available()) {
    // Read the JSON document from the "link" serial port
    DeserializationError err = deserializeJson(json_in, Serial);

    if (err == DeserializationError::Ok) 
    {
      int json_red = json_in["red"].as<int>();
      int json_green = json_in["green"].as<int>();
      int json_blue = json_in["blue"].as<int>();

      is_autonomous = json_in["autonomous"].as<bool>();
      is_eco = json_in["eco"].as<bool>();
      color_mode = json_in["mode"].as<int>();
      current_brightness = json_in["brightness"].as<int>();
      current_blink_period = json_in["blink_period"].as<int>();
      current_color = strip.Color(json_green, json_red, json_blue);
    } 
    else 
    {
      // Flush all bytes in the "link" serial port buffer
      while (Serial.available() > 0)
        Serial.read();
    }
  }

  if (can.available()) {
    can.receive(frame);
    Serial.print("Received CAN ");
    Serial.println(frame.id);

    switch (frame.id) {
      case 1: // Mobility stop
        is_mobility_stopped = true;
        break;
      case 9: // Mobility start
        is_mobility_stopped = false;
        break;
      case 13: // Safety Lights
        last_CAN_message = *(SafetyLightsMessage*)frame.data;
        is_autonomous = last_CAN_message.autonomous;
        is_eco = last_CAN_message.eco;
        color_mode = last_CAN_message.mode;
        current_brightness = last_CAN_message.brightness;
        current_blink_period = 10*last_CAN_message.blink_period;
        current_color = strip.Color(last_CAN_message.green, last_CAN_message.red, last_CAN_message.blue);
        break;
    }
  }
  whiteFlash();
  modeSelector();
  // delay(1000); // Just so we aren't looping too fast
}

void whiteFlash() {
  if (!is_autonomous || is_estopped || is_mobility_stopped) {

    if (!is_eco) {
      digitalWrite(W_PIN, LOW);
    } else {
      analogWrite(W_PIN, 128);
    }

    return;
  }
  if (!is_eco) {
    digitalWrite(W_PIN, (millis() / current_blink_period) % 2);
  } else {
    analogWrite(W_PIN,(255-((millis() / current_blink_period) % 2) * 127));
  }
}

void colorSolid() { // LED strip solid effect
  strip.setBrightness(current_brightness);
  strip.fill(current_color);
  strip.show();
}

void colorFlash(){ // LED strip flashing effect
  strip.setBrightness(current_brightness);
  if ((millis() / current_blink_period) % 2 == 0){
    strip.fill(current_color);
    strip.show();
  }
  else{
    colorClear();
  }
}

void colorFade(){ // LED strip fading effect
  strip.setBrightness(abs(sin(millis() / (float)current_blink_period)) * current_brightness);
  strip.fill(current_color);
  strip.show();
}

void colorClear(){ // Clear LED strip
  strip.fill(0);
  strip.show();
}

// void partyTime() // I love parties
// {
//   strip.setBrightness(current_brightness);
//   if((millis() / current_blink_period / 2) % 2 == 0) {
//     for(int i=0;i<CLED_COUNT;i++){
//       uint32_t color = strip.Color(random(256), random(256), random(256));
//       strip.setPixelColor(i, color);
//     }
//   }
//   strip.show();
// }



