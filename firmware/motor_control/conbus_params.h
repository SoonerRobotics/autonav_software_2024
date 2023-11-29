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

    int addr_updatePeriod;
    int addr_pulsesPerRadian;
    int addr_wheelRadius;
    int addr_wheelBaseLength;
    int addr_slewRateLimit;
    int addr_leftEncoderFactor;
    int addr_rightEncoderFactor;
    int addr_velocitykP;
    int addr_velocitykI;
    int addr_velocitykD;
    int addr_velocitykF;
    int addr_angularkP;
    int addr_angularkI;
    int addr_angularkD;
    int addr_angularkF;
    int addr_useObstacleAvoidance;
    int addr_sendStatisics;
    int addr_MotorUpdaatesBetweenDeltaOdom;
    int addr_updatePeriod;
    int addr_collisionDist;


    int EEPROM_SDA;
    int EEPROM_SCL;
    int EEPROM_WP;


};

inline ConbusParams::ConbusParams(int SDA, int SCL, int WP) {
    Wire.setSDA(SDA);
    Wire.setSCL(SCL);
    pinMode(WP, OUTPUT);
    
}