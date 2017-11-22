#include "mbed.h"
#include "QEI.h"

// wait function has an input unit of seconds, with precision of float.

void forward(int);
void rest(float);
void turnLeft(int);
void turnRight(int);
void resetEncoders();
void printGyro();
void printSensors();
void printSensorsCont();
void calibration(float& cL, float& cR, float& cFL, float& cFR);
void PID( float InputL, float InputR, float& speedL, float& speedR, const float kp , const float ki, const float kd);
float map ( float& var, float& imin, float& imax, float& omin, float& omax);
void move();

const float SPEED = 0.4f;
const float TURNSPEED = 0.4f;
const float WHEEL_RATIO = 0.95f;
const float SLOW = SPEED * WHEEL_RATIO;
const float CELL = 18;      // Length for 1 Cell.
const float TURN = 1.333815f;      // DISTANCE for a TURN of 45 deg.
const float TURNERRORL = 1.94740f;
const float TURNERRORR = 1.97644f; // 1.75655// 1.84668
const float HOLD = 0.001f;
const float dPerCountR = 0.010279f;
const float dPerCountL = 0.010272f;
const int samples = 1000;

    // For Moving
  float SetpointL; // run calibration function 
  float SetpointR; // run calibration function 
  float SetpointFL; // run calibration function 
  float SetpointFR; // run calibration function 
  

  float distanceR, distanceL, distanceFR, distanceFL;
  float calibratorR, calibratorL, calibratorFR, calibratorFL;
  float InputR, InputL;
  float defaultSpeed = SPEED;
  float speedL = SPEED, speedR = SPEED;
  float pError=0;
    

const float Kp = 5;
const float Kd = 8;
const float Ki = 0;

DigitalOut A (PC_10);
DigitalOut B (PC_11);
DigitalOut C (PB_0);
DigitalOut D (PB_7);

PwmOut LMF (PA_7);
PwmOut RMF (PB_10);
PwmOut LMB (PB_6);
PwmOut RMB (PC_7);
AnalogIn gyro(PC_2);
AnalogIn frontLeft(PC_1);
AnalogIn frontRight(PA_4);
AnalogIn sideLeft(PC_0);
AnalogIn sideRight(PA_0);

Serial pc(PA_2, PA_3);
QEI leftWheel(PA_1, PC_4, NC, 624, QEI::X4_ENCODING);
QEI rightWheel(PA_15, PB_3, NC, 624, QEI::X4_ENCODING);

int main()
{
    pc.printf("\nStart!\n");
    A = 1;
    B = 1;
    C = 1;
    D = 1;
    
    //printSensorsCont();
    
    wait(2);    
    calibration(calibratorL, calibratorR, calibratorFL, calibratorFR);
    SetpointL = calibratorL;
    SetpointR = calibratorR;
    SetpointFL = calibratorFL;
    SetpointFR = calibratorFR;
    
    for(;;)
    {
        distanceR = sideRight.read(); 
        distanceL = sideLeft.read();
        distanceFR = frontRight.read();
        distanceFL = frontLeft.read();
        printSensorsCont();
        /*
        if (distanceR >= 0.3 && distanceL <= 0.3)
            {
                rest(1);
                turnLeft(2);
            }
          */ 
        if (distanceFR >= 0.6 && distanceFL >= 0.6 && distanceR > distanceL )
            {
                rest(1);
                turnLeft(1);
            }
          /*
        if (distanceR >= 0.3 && distanceL <= 0.3)
            {
                rest(1);
                turnRight(2);
            }
          */
        if (distanceFR >= 0.6 && distanceFL >= 0.6 && distanceL > distanceR )
            {
                rest(1);
                turnRight(1);
            }
            
        if (distanceFR >= 0.6 && distanceFL >= 0.6 && distanceR >= 0.85 && distanceL >= 0.85 )
            {
                rest(1);
                turnLeft(2);
            }
            
            
        // Map raw sensor values to between 0 and 255
        // Set values to input to PID
         InputL = distanceL-SetpointL;
         InputR = distanceR-SetpointR;
         // mapping, cuz the max speed motor can have is 0.4f
         InputL  = SPEED * InputL;
         InputR  = SPEED * InputR;
         
        PID( InputL, InputR, speedL, speedR, Kp , Ki, Kd);
        
        // move
        move();
        
       
    }
    
    pc.printf("End!\n");
    
}

// move forward number of cells
// WORKS!
void forward(int cells)
{
    resetEncoders();
    pc.printf("Moving forward %i cells\n", cells);
    float distance = CELL*cells;
    int   leftPulses = 0;
    int   rightPulses = 0;
        LMB = 0;
        RMF = SPEED;
        LMF = SPEED;
        RMB = 0;
    // Old way of doing things (without serial):
    // for (float s = 0; s < count; s+= HOLD) {
        
    // Misheel's code starts here (with serial):
    // 1 revolution is approximated to be ~400 counts.
    for (;;){
        

        leftPulses = leftWheel.getPulses();
        rightPulses = rightWheel.getPulses(); 
        // Somehow the signs were random each time I do things
        // sometimes plus sometimes negative
        // so just made everything plus
        
        if (leftPulses < 0 )
            leftPulses = (-1) * leftPulses;
        if (rightPulses < 0 )
            rightPulses = (-1) * rightPulses;
            
        pc.printf("Left: %i\tRight: %i\n", leftPulses, rightPulses);
        // pc.printf("Gyro: %f\n", gyro.read() * 100.0f);
        // If sufficient distance traveled, break.
        if ((leftPulses * dPerCountL) >= distance || rightPulses * dPerCountR >= distance)
            break;
        wait(HOLD);
    }
    pc.printf("\n");
    resetEncoders();
}

// Rest for some seconds
void rest(float sec)
{
    LMF = 0;
    RMF = 0;
    LMB = 0;
    RMB = 0;
    wait(sec);
}

void turnLeft(int times)
{
    resetEncoders();
    pc.printf("Turning left %i times\n", times);
    float distance;
    // this is for 180 deg tuning:
    if (times == 4)
        distance = TURN * times * TURNERRORL;
    else 
        distance = TURN * times;
    
    int leftPulses = 0;
    int rightPulses = 0;
    
    LMB = TURNSPEED;
    RMF = TURNSPEED;
    LMF = 0;
    RMB = 0;
    
    for (;;) {
        leftPulses = leftWheel.getPulses();
        rightPulses = rightWheel.getPulses(); 
        
        // Somehow the signs were random each time I do things
        // sometimes plus sometimes negative
        // so just made everything plus
        
       if (leftPulses < 0 )
            leftPulses = (-1) * leftPulses;
        if (rightPulses < 0 )
            rightPulses = (-1) * rightPulses;
           
        pc.printf("Left: %i\tRight: %i\n", leftPulses, rightPulses);
        pc.printf("Gyro: %f\n", gyro.read() * 100.0f);
        if ((leftPulses * dPerCountL) >= distance || rightPulses * dPerCountR >= distance)
            break;  
    }
    pc.printf("\n");
    resetEncoders();
}

void turnRight(int times)
{
    resetEncoders();
    pc.printf("Turning right %i times\n", times);
    
    float distance;
    // for 180 deg tuning
    if (times == 4)
        distance = TURN * times * TURNERRORR;
    else 
        distance = TURN * times;
    int leftPulses = 0;
    int rightPulses = 0;
    
        LMB = 0;
        RMF = 0;
        LMF = TURNSPEED;
        RMB = TURNSPEED;
    for (;;) {
        
        leftPulses = leftWheel.getPulses();
        rightPulses = rightWheel.getPulses(); 
        
        // Somehow the signs were random each time I do things
        // sometimes plus sometimes negative
        // so just made everything plus
        
       if (leftPulses < 0 )
            leftPulses = (-1) * leftPulses;
        if (rightPulses < 0 )
            rightPulses = (-1) * rightPulses;
            
        pc.printf("Left: %i\tRight: %i\n", leftPulses, rightPulses);
        pc.printf("Gyro: %f\n", gyro.read() * 100.0f);
        if ((leftPulses * dPerCountL) >= distance || rightPulses * dPerCountR >= distance)
            break;
        
    }
    pc.printf("\n");
    resetEncoders();
}

// resets encoders
// reseting at the beginning and the end
void resetEncoders()
{
    rightWheel.reset();
    leftWheel.reset();
}

void printGyro()
{
    for(;;)
    {
        pc.printf("Gyro: %f\n", gyro.read() * 1.0f);
        wait(0.1);
    }
}


void printSensors()
{
    float Left = sideLeft.read() ;
    float FLeft = frontLeft.read() ;
    float FRight = frontRight.read() ;
    float Right = sideRight.read();
    
    pc.printf("Left: %f\tFLeft: %f\t\tFRight: %f\tRight: %f\n", 
                Left, FLeft, FRight, Right);
   
    
}

void printSensorsCont()
{
    for(;;)
    {
        printSensors();
    }
}

int P_Controller (int error)
{
    int correction = Kp * error;
    return correction;
}
void calibration(float& cL, float& cR, float& cFL, float& cFR)
{
  float TcR = 0;
  float TcL = 0;
  float TcFR = 0;
  float TcFL = 0;
  
  for(int i= 0; i < samples; i++)
  {
    TcR += sideRight.read();
    TcL += sideLeft.read();
    TcFR += frontRight.read();
    TcFL += frontLeft.read();
  }
  TcR /= samples;
  TcL /= samples;
  TcFR /= samples;
  TcFL /= samples;
  cR = TcR;
  cL = TcL;
  cFR = TcR;
  cFL = TcL;
}

void PID( float InputL, float InputR, float& speedL, float& speedR, const float kp , const float ki, const float kd)
{
      /*Compute all the working error variables*/
      float tspeedL = speedL;
      float tspeedR = speedR;
 
      float tError = InputR - InputL;
      float derivation = tError - pError;
      float correction = kp*tError + kd*derivation;
      tspeedR = defaultSpeed + correction;
      tspeedL = defaultSpeed - correction;
      if (tspeedR > SPEED)
        speedR = SPEED;
      if (tspeedL > SPEED)
        speedL = SPEED;
      if (tspeedL < 0)
        speedR = 0;
      if (tspeedL < 0)
        speedL = 0;
      speedL = tspeedL;
      speedR = tspeedR;
      pError = tError;               
}

float map (float& var, float& imin, float& imax, float& omin, float& omax)
{
    return var = omax / imax * var;

}

void move()
{
    LMB = 0;
    RMF = speedR;
    LMF = speedL;
    RMB = 0; 
}

void reverse(int cells)
{
    resetEncoders();
    pc.printf("Moving forward %i cells\n", cells);
    float distance = CELL*cells;
    int   leftPulses = 0;
    int   rightPulses = 0;
        LMB = 0;
        RMF = SPEED;
        LMF = SPEED;
        RMB = 0;
    // Old way of doing things (without serial):
    // for (float s = 0; s < count; s+= HOLD) {
        
    // Misheel's code starts here (with serial):
    // 1 revolution is approximated to be ~400 counts.
    for (;;){
        

        leftPulses = leftWheel.getPulses();
        rightPulses = rightWheel.getPulses(); 
        // Somehow the signs were random each time I do things
        // sometimes plus sometimes negative
        // so just made everything plus
        
        if (leftPulses < 0 )
            leftPulses = (-1) * leftPulses;
        if (rightPulses < 0 )
            rightPulses = (-1) * rightPulses;
            
        pc.printf("Left: %i\tRight: %i\n", leftPulses, rightPulses);
        pc.printf("Gyro: %f\n", gyro.read() * 100.0f);
        // If sufficient distance traveled, break.
        if ((leftPulses * dPerCountL) >= distance || rightPulses * dPerCountR >= distance)
            break;
        wait(HOLD);
    }
    pc.printf("\n");
    resetEncoders();
}
 
