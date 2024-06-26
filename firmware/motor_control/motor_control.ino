#include <Servo.h>
#include "time.h"
#include <ACAN2515.h>
#include "differential_drive.h"
#include "motor_with_encoder.h"
#include "common.h"
#include "CONBus.h" 
#include "CANBusDriver.h"

//LED
static int LED1 = 22;
static int LED2 = 21;

//ESTOP
static int ESTOP = 27;

//PWM
static int PWM0 = 6; //right motor controller
static int PWM1 = 7; //left motor controller

//ENCODER
static int ENC0A = 9; //right encoder
static int ENC0B = 8; //right encoder
static int ENC1A = 11; //left encoder
static int ENC1B = 10; //left encoder

//EEPROM
static int EEPROMSDA = 0;
static int EEPROMSCL = 1;
static int EEPROMWP = 2;

//SPI
static int MCP2515_MISO = 16;
static int MCP2515_CS = 17;
static int MCP2515_SCK = 18;
static int MCP2515_MOSI = 19;
static int MCP2515_INT = 20;

//Quartz oscillator - 8MHz
static uint32_t QUARTZ_FREQUENCY = 8UL * 1000UL * 1000UL;

ACAN2515 can(MCP2515_CS, SPI, MCP2515_INT);

TickTwo motor_update_timer(setMotorUpdateFlag, 25);

CANMessage frame;
CANMessage outFrame;

// Setup CONBus variables
CONBus::CONBus conbus;
CONBus::CANBusDriver conbus_can(conbus, 0x10); // device id 0x10 (16)

robotStatus_t roboStatus;
distance motorDistances;
MotorCommand motorCommand;

// motor leftMotor(PWM1, true);
// motor rightMotor(PWM0, false);

MotorWithEncoder leftMotor(PWM1, ENC1A, ENC1B, true);
MotorWithEncoder rightMotor(PWM0, ENC0A, ENC0B, false);
DifferentialDrive drivetrain(leftMotor, rightMotor, 0.025);

void configureCan();
void printCanMsg();
void updateMsgData();
void updateLeft();
void updateRight();
void sendCanOdomMsgOut();
void resetDelta();

int motor_updates_in_deltaodom = 0;
uint32_t motor_updates_between_deltaodom = 3;
bool canBlinky = false;

bool useObstacleAvoidance = false;
uint32_t collisonBoxDist = 20;
bool isDetectingObstacle = false;

bool sendStatistics = true;

float delta_x = 0;
float delta_y = 0;
float delta_theta = 0;

float desired_forward_velocity;
float desired_angular_velocity;

void setup() {
  Serial.begin(9600);
  //while(!Serial){
    //delay(50);
  //}
  delay(50);
  Serial.println("Serial Connected");

  drivetrain.setup();

  pinMode(ENC0A, INPUT);
  pinMode(ENC0B, INPUT);
  pinMode(ENC1A, INPUT);
  pinMode(ENC1B, INPUT);

  attachInterrupt(ENC1A, updateLeft, CHANGE);
  attachInterrupt(ENC0A, updateRight, CHANGE);

  motor_update_timer.start();

  //setup1
  delay(50);
  configureCan();

  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  digitalWrite(LED1, HIGH);
  digitalWrite(LED2, HIGH);
  delay(1000);
  digitalWrite(LED1, LOW);
  digitalWrite(LED2, LOW);
  pinMode(ESTOP, INPUT);

  conbus.addReadOnlyRegister(0x00, drivetrain.getUpdatePeriod());
  conbus.addRegister(0x01, drivetrain.getPulsesPerRadian());
  conbus.addRegister(0x02, drivetrain.getWheelRadius());
  conbus.addRegister(0x03, drivetrain.getWheelbaseLength());
  conbus.addRegister(0x04, drivetrain.getSlewRateLimit());
  conbus.addRegister(0x05, drivetrain.getLeftEncoderFactor());
  conbus.addRegister(0x06, drivetrain.getRightEncoderFactor());

  conbus.addRegister(0x10, drivetrain.getVelocitykP());
  conbus.addRegister(0x11, drivetrain.getVelocitykI());
  conbus.addRegister(0x12, drivetrain.getVelocitykD());
  conbus.addRegister(0x13, drivetrain.getVelocitykF());

  conbus.addRegister(0x20, drivetrain.getAngularkP());
  conbus.addRegister(0x21, drivetrain.getAngularkI());
  conbus.addRegister(0x22, drivetrain.getAngularkD());
  conbus.addRegister(0x23, drivetrain.getAngularkF());

  conbus.addRegister(0x30, &useObstacleAvoidance);
  conbus.addRegister(0x31, &collisonBoxDist);

  conbus.addRegister(0x40, &sendStatistics);

  conbus.addRegister(0x50, &motor_updates_between_deltaodom);
}

void loop() {

  updateTimers();
  if (MOTOR_UPDATE_FLAG) {

    drivetrain.updateState(delta_x, delta_y, delta_theta);
    motor_updates_in_deltaodom++;

    MOTOR_UPDATE_FLAG = false;
  }

  //loop1
  if (motor_updates_in_deltaodom >= motor_updates_between_deltaodom) {
    motor_updates_in_deltaodom = 0;
    sendCanOdomMsgOut();
    resetDelta();
  }

  if (conbus_can.isReplyReady()) {
    conbus_can.peekReply(outFrame.id, outFrame.len, outFrame.data);

    bool success = can.tryToSend(outFrame);

    if (success) {
      conbus_can.popReply();
    }
  }

}

void configureCan() {
  SPI.setSCK(MCP2515_SCK);
  SPI.setTX(MCP2515_MOSI);
  SPI.setRX(MCP2515_MISO);
  SPI.setCS(MCP2515_CS);
  SPI.begin();

  Serial.println("Configure ACAN2515");
  ACAN2515Settings settings(QUARTZ_FREQUENCY, 100UL * 1000UL);  // CAN bit rate 100 kb/s
  settings.mRequestedMode = ACAN2515Settings::NormalMode ; // Select Normal mode
  const uint16_t errorCode = can.begin(settings, onCanRecieve );
  if (errorCode == 0) {
    Serial.println("CAN Configured");
  }
  else{
    Serial.print("Error: ");
    Serial.println(errorCode);
  }
}
void onCanRecieve() {
  can.isr();

  if (!can.available()) {
    return;
  }

  can.receive(frame);  

  conbus_can.readCanMessage(frame.id, frame.data);

  printCanMsg(frame);

  switch (frame.id) {  
    case 10:
      motorCommand = *(MotorCommand*)(frame.data);

      desired_forward_velocity = (float)motorCommand.setpoint_forward_velocity / SPEED_SCALE_FACTOR;
      desired_angular_velocity = (float)motorCommand.setpoint_angular_velocity / SPEED_SCALE_FACTOR;


      if (useObstacleAvoidance && isDetectingObstacle && desired_forward_velocity > 0) {
        desired_forward_velocity = 0;
      }

      drivetrain.setOutput(desired_forward_velocity, desired_angular_velocity);
      break;
    case 20:
      isDetectingObstacle = (frame.data[1] < collisonBoxDist || frame.data[2] < collisonBoxDist || frame.data[3] < collisonBoxDist);

      if (useObstacleAvoidance && isDetectingObstacle && desired_forward_velocity > 0) {
        desired_forward_velocity = 0;
      }

      drivetrain.setOutput(desired_forward_velocity, desired_angular_velocity);
      break;
  }
  
}

PIDSetpoints pid_setpoints;
PIDControl pid_controls;

void sendCanOdomMsgOut(){
  outFrame.id = ODOM_OUT_ID;
  outFrame.len = ODOM_OUT_LEN;

  motorDistances.xn = delta_x * ODOM_SCALE_FACTOR;
  motorDistances.yn = delta_y * ODOM_SCALE_FACTOR;
  motorDistances.on = delta_theta * ODOM_SCALE_FACTOR;

  memcpy(outFrame.data, &motorDistances, ODOM_OUT_LEN );
  const bool ok = can.tryToSend (outFrame) ;

  if (sendStatistics) {
    drivetrain.getSetpoints(pid_setpoints);
    outFrame.id = 50;
    outFrame.len = 8;
    memcpy(outFrame.data, &pid_setpoints, 8);
    can.tryToSend(outFrame);

    drivetrain.getControl(pid_controls);
    outFrame.id = 51;
    outFrame.len = 4;
    memcpy(outFrame.data, &pid_controls, 4);
    can.tryToSend(outFrame);
  }

}
void printCanMsg(CANMessage frame) {
  Serial.print("  id: ");
  Serial.println(frame.id, HEX);
  Serial.println(frame.len);
  Serial.print("  data: ");
  for (int x = 0; x < frame.len; x++) {
    Serial.print(frame.data[x], HEX);
    Serial.print(":");
  }
  Serial.println("");
}

void updateLeft(){
  drivetrain.pulseLeftEncoder();
}

void updateRight(){
  drivetrain.pulseRightEncoder();
}

void resetDelta(){
  motorDistances.xn = 0;
  motorDistances.yn = 0;
  motorDistances.on = 0;
  delta_x = 0;
  delta_y = 0;
  delta_theta = 0;
}

void updateTimers() {
  motor_update_timer.update();
}
