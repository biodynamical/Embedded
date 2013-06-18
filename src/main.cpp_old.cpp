// #include "MMA7361.h"
// #include "Actuator.h"
// #include <Arduino.h>
// #include "Wire.h"
// #include "I2Cdev.h"
// #include "MPU6050_6Axis_MotionApps20.h"

// int FREQ = 20;
// int lastLoopTime = 0;
// int lastLoopUsefulTime = 0;
// unsigned long loopRefTime = 0;

// Actuator m1( 13, 12, 11 );
// Actuator m2( 10, 9, 8 );

// // AcceleroMMA7361 accelero;
// // int x;
// // int y;
// // int z;

// MPU6050 mpu;

// // MPU control/status vars
// bool dmpReady = false;  // set true if DMP init was successful
// uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
// uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
// uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
// uint16_t fifoCount;     // count of all bytes currently in FIFO
// uint8_t fifoBuffer[64]; // FIFO storage buffer

// // orientation/motion vars
// Quaternion q;           // [w, x, y, z]         quaternion container
// VectorInt16 aa;         // [x, y, z]            accel sensor measurements
// VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
// VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
// VectorFloat gravity;    // [x, y, z]            gravity vector
// float euler[3];         // [psi, theta, phi]    Euler angle container
// float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector


// volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
// void dmpDataReady()
// {
//     mpuInterrupt = true;
// }



// //-----------------------------------------------------------------------------
// //
// void setup()
// {
//     Wire.begin();
//     Serial.begin( 9600 );

//     // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3v or Ardunio
//     // Pro Mini running at 3.3v, cannot handle this baud rate reliably due to
//     // the baud timing being too misaligned with processor ticks. You must use
//     // 38400 or slower in these cases, or use some kind of external separate
//     // crystal solution for the UART timer.

//     // initialize device
//     Serial.println(F("Initializing I2C devices..."));
//     mpu.initialize();

//     // verify connection
//     Serial.println(F("Testing device connections..."));
//     Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

//     // // wait for ready
//     // Serial.println(F("\nSend any character to begin DMP programming and demo: "));
//     // while (Serial.available() && Serial.read()); // empty buffer
//     // while (!Serial.available());                 // wait for data
//     // while (Serial.available() && Serial.read()); // empty buffer again

//     // load and configure the DMP
//     Serial.println(F("Initializing DMP..."));
//     devStatus = mpu.dmpInitialize();

//     // make sure it worked (returns 0 if so)
//     if (devStatus == 0)
//     {
//         // turn on the DMP, now that it's ready
//         Serial.println(F("Enabling DMP..."));
//         mpu.setDMPEnabled(true);

//         // enable Arduino interrupt detection
//         Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
//         attachInterrupt(0, dmpDataReady, RISING);
//         mpuIntStatus = mpu.getIntStatus();

//         // set our DMP Ready flag so the main loop() function knows it's okay to use it
//         Serial.println(F("DMP ready! Waiting for first interrupt..."));
//         dmpReady = true;

//         // get expected DMP packet size for later comparison
//         packetSize = mpu.dmpGetFIFOPacketSize();
//     }
//     else
//     {
//         // ERROR!
//         // 1 = initial memory load failed
//         // 2 = DMP configuration updates failed
//         // (if it's going to break, usually the code will be 1)
//         Serial.print(F("DMP Initialization failed (code "));
//         Serial.print(devStatus);
//         Serial.println(F(")"));
//     }

//     m1.Initialize();
//     m2.Initialize();

//     m1.Activate();
//     m2.Activate();


//     // accelero.begin( 13, 12, 11, 10, A0, A1, A2 );
//     // accelero.setARefVoltage( 3.3 );
//     // accelero.setSensitivity( HIGH );
//     // accelero.setOffSets( 0, 0, 0 );
//     // accelero.calibrate();
// }

// //-----------------------------------------------------------------------------
// //
// #define round(x) ((x)>=0?(long)((x)+0.5):(long)((x)-0.5))
// #define GUARD_GAIN  100

// float K = 1;
// float Kp = 1; // Proportional control Gain
// float Kd = 1.7; // Derivative control gain
// float Ki = 0.1; // Integral control gain

// float lastErr = 0;
// float integratedErr = 0;
// float pTerm = 0, iTerm = 0, dTerm = 0;

// int UpdatePid( int targetPosition, int currentPosition )
// {
//     int error = targetPosition - currentPosition;
//     integratedErr += error;

//     pTerm = Kp * error;
//     iTerm = Ki * constrain( integratedErr, -GUARD_GAIN, GUARD_GAIN );
//     dTerm = Kd * (error - lastErr);
//     float res = -constrain( K*(pTerm + iTerm + dTerm), -100, 100 );

//     lastErr = error;

//     // Serial.print( "  pTerm: " );
//     // Serial.print( pTerm );
//     // Serial.print( "  dTerm: " );
//     // Serial.print( dTerm );
//     // Serial.print( "  iTerm: ");
//     // Serial.print( iTerm );

//     return round(res);
// }

// //-----------------------------------------------------------------------------
// //
// void DebugTimming()
// {
//     static int skip = 0;

//     // display every 500 ms (at 100 Hz)
//     if( skip ++== 5 )
//     {
//         skip = 0;
//         Serial.print( lastLoopUsefulTime );
//         Serial.print(",");
//         Serial.print( lastLoopTime );
//         Serial.print("\n");
//     }
// }

// //-----------------------------------------------------------------------------
// //
// // void TestMotorReactivity( const int x )
// // {
// //     if( test%2 == 0 )
// //     {
// //         m1.Forward( 100 );
// //         m2.Forward( 100 );
// //         test += 2;
// //         if( test > 20 ) test = 1;
// //     }
// //     else
// //     {
// //         m1.Backward( 100 );
// //         m2.Backward( 100 );
// //         test += 2;
// //         if( test > 20 ) test = 2;
// //     }
// // }

// int AccelArray[1];
// int Smoothing( int value, int* Data )
// {
//     static int ndx = 0;
//     static int count = 0;
//     static int total = 0;
//     total -= Data[ndx];
//     Data[ndx] = value;
//     total += Data[ndx];
//     ndx = (ndx+1) % 1;
//     if( count < 1 ) count++;
//     return total/count;
// }

// float Aold;
// float beta = 0.6;

// int kalman1D( int Ameasured )
// {
//     float Anew = (1-beta)*Aold + beta*Ameasured;
//     Aold = Anew;
//     return round(Anew);
// }

// //-----------------------------------------------------------------------------
// //
// void loop()
// {
//     if (!dmpReady) return;

//     // wait for MPU interrupt or extra packet(s) available
//     while (!mpuInterrupt && fifoCount < packetSize)
//     {
//         // other program behavior stuff here
//         // .
//         // .
//         // .
//         // if you are really paranoid you can frequently test in between other
//         // stuff to see if mpuInterrupt is true, and if so, "break;" from the
//         // while() loop to immediately process the MPU data
//         // .
//         // .
//         // .
//     }

//     // reset interrupt flag and get INT_STATUS byte
//     mpuInterrupt = false;
//     mpuIntStatus = mpu.getIntStatus();

//     // get current FIFO count
//     fifoCount = mpu.getFIFOCount();

//     // check for overflow (this should never happen unless our code is too inefficient)
//     if ((mpuIntStatus & 0x10) || fifoCount == 1024)
//     {
//         // reset so we can continue cleanly
//         mpu.resetFIFO();
//         // Serial.println(F("FIFO overflow!"));

//         // otherwise, check for DMP data ready interrupt (this should happen frequently)
//     }
//     else if (mpuIntStatus & 0x02)
//     {
//         // wait for correct available data length, should be a VERY short wait
//         while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

//         // read a packet from FIFO
//         mpu.getFIFOBytes(fifoBuffer, packetSize);

//         // track FIFO count here in case there is > 1 packet available
//         // (this lets us immediately read more without waiting for an interrupt)
//         fifoCount -= packetSize;

//         // display Euler angles in degrees
//         mpu.dmpGetQuaternion(&q, fifoBuffer);
//         mpu.dmpGetEuler(euler, &q);
//         Serial.print("euler\tx:");
//         Serial.print(euler[0] * 180/M_PI);
//         Serial.print("\ty:");
//         Serial.print(euler[1] * 180/M_PI);
//         Serial.print("\tz:");
//         Serial.print(euler[2] * 180/M_PI);

//         Serial.print("\t");

//         // display real acceleration, adjusted to remove gravity
//         mpu.dmpGetQuaternion(&q, fifoBuffer);
//         mpu.dmpGetAccel(&aa, fifoBuffer);
//         mpu.dmpGetGravity(&gravity, &q);
//         mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
//         Serial.print("areal\tx:");
//         Serial.print(aaReal.x);
//         Serial.print("\ty:");
//         Serial.print(aaReal.y);
//         Serial.print("\tz:");
//         Serial.println(aaReal.z);

//     }
//     // Serial.print( "y: " );
//     // Serial.print(y);
//     // Serial.print( " z: " );
//     // Serial.print(z);

//     // y = kalman1D( y );
//     // float a = sqrt( (y*y) + (z*z) - 1 );
//     // if( y < 0 )
//     // {
//     //     a = -a;
//     // }

//     // float tilt = 0;
//     // if( a > 0 )
//     //     tilt = asin((y-z*a)/((y*y)+(z*z)));
//     // else if( a < 0 )
//     //     tilt = asin((y+z*a)/((y*y)+(z*z)));
//     // else
//     //     tilt = asin(y);

//     // if( y < 0 )
//     // {
//     //     tilt = -tilt;
//     // }

//     // Serial.print( " a: " );
//     // Serial.print(a);

//     // Serial.print( " tilt: " );
//     // Serial.print(tilt);

//     // int corr = UpdatePid( 0, y );
//     // Serial.print("  corr:");
//     // Serial.println( corr );

//     // if( corr > 0 )
//     // {
//     //     m1.Backward( corr );
//     //     m2.Backward( corr );
//     // }
//     // else if( corr < 0 )
//     // {
//     //     m1.Forward( -corr );
//     //     m2.Forward( -corr );
//     // }
//     // else
//     // {
//     //     m1.Brake();
//     //     m2.Brake();
//     // }

//     // LOOP CONTROL
//     lastLoopUsefulTime = millis() - loopRefTime;
//     if( lastLoopUsefulTime < FREQ )
//     {
//         delay( FREQ - lastLoopUsefulTime );
//     }
//     // lastLoopTime = millis() - loopRefTime;
//     // DebugTimming();

//     loopRefTime = millis();
// }
