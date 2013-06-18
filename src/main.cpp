#include <Wire.h>
#include <Arduino.h>
#include "Actuator.h"

/* IMU Data */
int16_t accX, accY, accZ;
int16_t tempRaw;
int16_t gyroX, gyroY, gyroZ;

double accXangle, accYangle; // Angle calculate using the accelerometer
double temp; // Temperature
double gyroXangle, gyroYangle; // Angle calculate using the gyro
double compAngleX, compAngleY; // Calculate the angle using a complementary filter
double kalAngleX, kalAngleY; // Calculate the angle using a Kalman filter

uint32_t timer;
uint8_t i2cData[14]; // Buffer for I2C data

Actuator m1( 10, 7, 6 );
Actuator m2( 9, 4, 5 );

KalmanFilter kalmanX; // Create the Kalman instances
KalmanFilter kalmanY;

int FREQ = 10;
int lastLoopTime = 0;
int lastLoopUsefulTime = 0;
unsigned long loopRefTime = 0;
int prevCmd = 0;


const uint8_t IMUAddress = 0x68; // AD0 is logic low on the PCB
const uint16_t I2C_TIMEOUT = 1000; // Used to check for errors in I2C communication

//-----------------------------------------------------------------------------
//
uint8_t i2cWrite( uint8_t registerAddress, uint8_t* data, uint8_t length,
                  bool sendStop )
{
    Wire.beginTransmission( IMUAddress );
    Wire.write( registerAddress );
    Wire.write( data, length );
    return Wire.endTransmission( sendStop ); // Returns 0 on success
}

//-----------------------------------------------------------------------------
//
uint8_t i2cWrite( uint8_t registerAddress, uint8_t data, bool sendStop )
{
    return i2cWrite( registerAddress, &data, 1, sendStop ); // Returns 0 on success
}

//-----------------------------------------------------------------------------
//
uint8_t i2cRead( uint8_t registerAddress, uint8_t* data, uint8_t nbytes )
{
    uint32_t timeOutTimer;
    Wire.beginTransmission( IMUAddress );
    Wire.write( registerAddress );

    if( Wire.endTransmission(false) ) // Don't release the bus
        return 1; // Error in communication

    Wire.requestFrom( IMUAddress, nbytes, (uint8_t)true ); // Send a repeated start and then release the bus after reading
    for( uint8_t i = 0; i < nbytes; ++i )
    {
        if( Wire.available() )
            data[i] = Wire.read();
        else
        {
            timeOutTimer = micros();
            while( ((micros() - timeOutTimer) < I2C_TIMEOUT) &&
                   !Wire.available() );
            if( Wire.available() )
                data[i] = Wire.read();
            else
                return 2; // Error in communication
        }
    }
    return 0; // Success
}

//-----------------------------------------------------------------------------
//
#define round(x) ((x)>=0?(long)((x)+0.5):(long)((x)-0.5))
#define GUARD_GAIN  40

float K = 1;
float Kp = 4; // Proportional control Gain
float Kd = 7; // Derivative control gain
float Ki = 2.2; // Integral control gain

float lastErr = 0;
float integratedErr = 0;
float pTerm = 0, iTerm = 0, dTerm = 0;

int UpdatePid( int targetPosition, int currentPosition )
{
    int error = targetPosition - currentPosition;
    integratedErr += error;

    pTerm = Kp * error;
    iTerm = Ki * constrain( integratedErr, -GUARD_GAIN, GUARD_GAIN );
    dTerm = Kd * (error - lastErr);
    float res = -constrain( K*(pTerm + iTerm + dTerm), -100, 100 );

    lastErr = error;

    Serial.print( "  pTerm: " );
    Serial.print( pTerm );
    Serial.print( "  dTerm: " );
    Serial.print( dTerm );
    Serial.print( "  iTerm: ");
    Serial.print( iTerm );

    return round(res);
}

//-----------------------------------------------------------------------------
//
void setup()
{
    Serial.begin(9600);
    Wire.begin();
    i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
    i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
    i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
    i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
    while(i2cWrite(0x19,i2cData,4,false)); // Write to all four registers at once
    while(i2cWrite(0x6B,0x01,true)); // PLL with X axis gyroscope reference and disable sleep mode

    while(i2cRead(0x75,i2cData,1));
    if(i2cData[0] != 0x68)   // Read "WHO_AM_I" register
    {
        Serial.print(F("Error reading sensor"));
        while(1);
    }

    delay(100); // Wait for sensor to stabilize

    /* Set kalman and gyro starting angle */
    while(i2cRead(0x3B,i2cData,6));
    accX = ((i2cData[0] << 8) | i2cData[1]);
    accY = ((i2cData[2] << 8) | i2cData[3]);
    accZ = ((i2cData[4] << 8) | i2cData[5]);
    // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
    // We then convert it to 0 to 2π and then from radians to degrees
    accYangle = (atan2(accX,accZ)+PI)*RAD_TO_DEG;
    accXangle = (atan2(accY,accZ)+PI)*RAD_TO_DEG;

    kalmanX.setAngle(accXangle); // Set starting angle
    kalmanY.setAngle(accYangle);
    gyroXangle = accXangle;
    gyroYangle = accYangle;
    compAngleX = accXangle;
    compAngleY = accYangle;

    TCCR2B = TCCR2B & 0b11111000 | 0x05;

    m1.Initialize();
    m2.Initialize();

    m1.Activate();
    m2.Activate();

    timer = micros();
}

//-----------------------------------------------------------------------------
//
void loop()
{
    /* Update all the values */
    while(i2cRead(0x3B,i2cData,14));
    accX = ((i2cData[0] << 8) | i2cData[1]);
    accY = ((i2cData[2] << 8) | i2cData[3]);
    accZ = ((i2cData[4] << 8) | i2cData[5]);
    tempRaw = ((i2cData[6] << 8) | i2cData[7]);
    gyroX = ((i2cData[8] << 8) | i2cData[9]);
    gyroY = ((i2cData[10] << 8) | i2cData[11]);
    gyroZ = ((i2cData[12] << 8) | i2cData[13]);

    // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
    // We then convert it to 0 to 2π and then from radians to degrees
    accXangle = (atan2(accY,accZ)+PI)*RAD_TO_DEG;
    accYangle = (atan2(accX,accZ)+PI)*RAD_TO_DEG;

    double gyroXrate = (double)gyroX/131.0;
    double gyroYrate = -((double)gyroY/131.0);
    gyroXangle += gyroXrate*((double)(micros()-timer)/1000000); // Calculate gyro angle without any filter
    gyroYangle += gyroYrate*((double)(micros()-timer)/1000000);
    //gyroXangle += kalmanX.getRate()*((double)(micros()-timer)/1000000); // Calculate gyro angle using the unbiased rate
    //gyroYangle += kalmanY.getRate()*((double)(micros()-timer)/1000000);

    compAngleX = (0.93*(compAngleX+(gyroXrate*(double)(micros()-timer)/1000000)))+(0.07*accXangle); // Calculate the angle using a Complimentary filter
    compAngleY = (0.93*(compAngleY+(gyroYrate*(double)(micros()-timer)/1000000)))+(0.07*accYangle);

    kalAngleX = kalmanX.getAngle(accXangle, gyroXrate, (double)(micros()-timer)/1000000); // Calculate the angle using a Kalman filter
    kalAngleY = kalmanY.getAngle(accYangle, gyroYrate, (double)(micros()-timer)/1000000);
    timer = micros();

    temp = ((double)tempRaw + 12412.0) / 340.0;

    // Print Data
    Serial.print("accX: "); Serial.print(accX); Serial.print("\t");
    Serial.print("accY: "); Serial.print(accY); Serial.print("\t");
    Serial.print("accZ: "); Serial.print(accZ); Serial.print("\t");

    Serial.print("gyroX: "); Serial.print(gyroX); Serial.print("\t");
    Serial.print("gyroY: "); Serial.print(gyroY); Serial.print("\t");
    Serial.print("gyroZ: "); Serial.print(gyroZ); Serial.print("\t");

    Serial.print(accXangle); Serial.print("\t");
    Serial.print(gyroXangle); Serial.print("\t");
    Serial.print(compAngleX); Serial.print("\t");
    Serial.print("kalAngleY: "); Serial.print(kalAngleX); Serial.print("\t");

    Serial.print("\t");

    Serial.print(accYangle); Serial.print("\t");
    Serial.print(gyroYangle); Serial.print("\t");
    Serial.print(compAngleY); Serial.print("\t");
    Serial.print("kalAngleY: "); Serial.print(kalAngleY); Serial.print("\t");

    Serial.print(temp); Serial.print("\t");

    int corr = UpdatePid( 178, kalAngleX );
    Serial.print("  corr:");
    Serial.print(corr);

    if( (corr > 0 && prevCmd < 0) ||
        (corr < 0 && prevCmd > 0) )
    {
        m1.Brake();
        m2.Brake();
    }

    prevCmd = corr;

    if( corr > 0 )
    {
        m1.Backward( corr );
        m2.Backward( corr );
    }
    else if( corr < 0 )
    {
        m1.Forward( -corr );
        m2.Forward( -corr );
    }
    else
    {
        m1.Coast();
        m2.Coast();
    }

    // LOOP CONTROL
    lastLoopUsefulTime = millis() - loopRefTime;
    if( lastLoopUsefulTime < FREQ )
    {
        delay( FREQ - lastLoopUsefulTime );
    }
    loopRefTime = millis();

    Serial.print("\r\n");
}
