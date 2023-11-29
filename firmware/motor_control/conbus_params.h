#ifndef CONBUS_PARAMS
#define CONBUS_PARAMS

#include <I2C_8Bit.h>
#include <Wire.h>

class ConbusParams {

public:
    ConbusParams(int SDA, int SCL, int WP);

    void update_variable(float id, float value);
    void refresh();

    float getUpdatePeriod();
    float getPulsesPerRadian();
    float getWheelRadius();
    float getWheelBaseLength();
    float getSlewRateLimit();
    float getLeftEncoderFactor();
    float getRightEncoderFactor();
    float getVelocitykP();
    float getVelocitykI();
    float getVelocitykD();
    float getVelocitykF();
    float getAngularkP();
    float getAngularkI();
    float getAngularkD();
    float getAngularkF();
    float getUseObstacleAvoidence();
    float getCollisionDist();
    float getSendStatistics();
    float getMotorUpdateBetweenDelatOdom();

private:
    void refreshVariables();
    void setUpdatePeriod(float period);
    void setPulsesPerRadian(float ppr);
    void setWheelRadius(float radius);
    void setWheelBaseLength(float length);
    void setSlewRateLimit(float slewRate);
    void setLeftEncoderFactor(float factor);
    void setRightEncoderFactor(float factor);
    void setVelocitykP(float vkp);
    void setVelocitykI(float vki);
    void setVelocitykD(float vkd);
    void setVelocitykF(float vkf);
    void setAngularkP(float akp);
    void setAngularkI(float aki);
    void setAngularkD(float akd);
    void setAngularkF(float akf);
    void setUseObstacleAvoidence(bool use);
    void setCollisionDist(int distance);
    void setSendStatistics(bool send);
    void setMotorUpdateBetweenDeltaOdom(bool update);

    float updatePeriod;
    float pulsesPerRadian;
    float wheelRadius;
    float wheelBaseLength;
    float slewRateLimit;
    float leftEncoderFactor;
    float rightEncoderFactor;
    float velocitykP;
    float velocitykI;
    float velocitykD;
    float velocitykF;
    float angularkP;
    float angularkI;
    float angularkD;
    float angularkF;
    bool useObstacleAvoidance;
    bool sendStatisics;
    bool MotorUpdaatesBetweenDeltaOdom;
    int collisionDist;

    int addr_updatePeriod = 0x00;
    int addr_pulsesPerRadian = 0x04;
    int addr_wheelRadius = 0x08;
    int addr_wheelBaseLength = 0xC;
    int addr_slewRateLimit = 0x10;
    int addr_leftEncoderFactor = 0x14;
    int addr_rightEncoderFactor = 0x18;
    int addr_velocitykP = 0x1C;
    int addr_velocitykI = 0x20;
    int addr_velocitykD = 0x24;
    int addr_velocitykF = 0x28;
    int addr_angularkP = 0x2C;
    int addr_angularkI = 0x30;
    int addr_angularkD = 0x34;
    int addr_angularkF = 0x38;
    int addr_useObstacleAvoidance = 0x3C;
    int addr_sendStatisics = 0x40;
    int addr_MotorUpdaatesBetweenDeltaOdom = 0x44;
    int addr_updatePeriod = 0x48;
    int addr_collisionDist = 0x4C;


    int EEPROM_SDA;
    int EEPROM_SCL;
    int EEPROM_WP;


};

inline ConbusParams::ConbusParams(int SDA, int SCL, int WP) {
    Wire.setSDA(SDA);
    Wire.setSCL(SCL);
    pinMode(WP, OUTPUT);

}