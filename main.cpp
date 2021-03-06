#include "mbed.h"
#include "QEI.h"
#define ENC_PER_REV 360
#define SAMPLES 20
#define MAX_SPEED 0.2f
float defaultSpeed = MAX_SPEED * 0.75f;

///////////////////////
// Function Declarations
///////////////////////
void turn(bool left);
float P_Controller(float error, float Kp);
float I_Controller(float error, float Ki, float& integrator, float decayFactor);
float D_Controller(float error, float Kd, float& prevError );
void update_enc();
void update_IR();
void setup_IR(float& cL, float& cR);
void setup_enc();
void systick_forward_default(float error, float correction_factor, float Kp, float Kd, float Ki, float& integrator, float decayFactor, float& prevError, Timer& timer);
void systick_forward();
void systick_forward_enc();
bool detect_wall_front();
bool detect_missing_wall();     // side walls
void forward(int n);
void stop();


///////////////////////
// flags
///////////////////////
bool missing_left_wall = false, missing_right_wall = false;
int wall_state = 0;         // 1 for missing no walls, 2 for missing left wall,
                            // 3 for missing right wall, 4 for missing both walls

///////////////////////
// K Values
///////////////////////
float Kp_enc = 6,
      Ki_enc = 2,
      Kd_enc = 20,
      Kp_IR = 0.5f,
      Ki_IR = 0,
      Kd_IR = 15;

///////////////////////
// Speed Variables
///////////////////////
float LSpeed = defaultSpeed,
      RSpeed = defaultSpeed;

////////////////////////
// PID miscellanies
////////////////////////
float integrator_enc = 0;     // I_Controller() variables
float integrator_IR = 0;
float decayFactor_enc = 2;    // I_Controller() variable
float decayFactor_IR = 2;
float prevError_enc = 0;      // D_Controller() variable
float prevError_IR = 0;
float correction_factor_enc = 1/(36000*(LSpeed+RSpeed));
float correction_factor_IR = 1;

//////////////////////
// Pin definitions
/////////////////////
DigitalOut IR_L(PB_5);
DigitalOut IR_LF(PB_4);
DigitalOut IR_LS(PB_15);
DigitalOut IR_R(PB_10);
//DigitalOut IR_RF(PB_11);
DigitalOut IR_RS(PB_14);

AnalogIn REC_L(PC_0);
AnalogIn REC_LF(PC_1);
AnalogIn REC_LS(PB_1);
AnalogIn REC_R(PA_4);
AnalogIn REC_RF(PA_5);
AnalogIn REC_RS(PB_0);

PwmOut  MLF(PB_7),
        MRF(PB_9),
        MLB(PB_6),
        MRB(PB_8);

/////////////////////////
// Encoder Variables
/////////////////////////
int enc_leftD, enc_rightD,
    enc_left, enc_right,
    enc_left_prev, enc_right_prev;         // enc_leftD = encoder value difference of left motor

/////////////////////////
// IR Variables/Used in PID
/////////////////////////
float
    IR_leftD, IR_rightD,
    IR_frontLeft, IR_frontRight,
    IR_left, IR_right,
    IR_left_prev,
    IR_right_prev;              // IR_leftD = IR value difference of left motor

////////////////////////
// REC Setup Variables
////////////////////////
float
    REC_threshold,         // not used atm, for hardcoding/testing
    REC_left_baseline = 0,
    REC_right_baseline = 0,
    REC_front_threshold = 0;

///////////////////////
// Timer/Serial
///////////////////////
Ticker Systicker;
Timer timer_enc, timer_IR;
Serial pc(PA_9, PA_10);

///////////////////////
// Motor init
///////////////////////
QEI wheelL(PA_15, PB_3, NC, 624, QEI::X4_ENCODING);
QEI wheelR(PA_0, PA_1, NC, 624, QEI::X4_ENCODING);

//////////////////////
// FUNCTIONS
//////////////////////

// Turn immediately to direction dir (pos right, left neg)
// Shouldn't the dir mapping in the comment be: (1: for left, 0 for right)... code is right tho
void turn(bool left){
    int left_pulse = wheelL.getPulses();
    int right_pulse = wheelR.getPulses();
    while(abs(wheelL.getPulses()-left_pulse) + abs(wheelR.getPulses()-right_pulse) <= 365){
        //turn right
        if (left){
            MLF.write(defaultSpeed*0.9f); //-Kp_turn*diff);
            MLB.write(0);
            MRB.write(defaultSpeed); //-Kp_turn*diff);
            MRF.write(0);
        }
        //turn left
        else{
            MLB.write(defaultSpeed*0.9f); //-Kp_turn*diff);
            MLF.write(0);
            MRF.write(defaultSpeed); //-Kp_turn*diff);
            MRB.write(0);
        }
        //diff = wheelR.getPulses() - wheelL.getPulses();
        //pc.printf("diff: %d\n", diff); //to debug and determine proper Kp
    }

    MLB.write(0);
    MLF.write(0);
    MRB.write(0);
    MRF.write(0);
    wait(0.1f);
}

float P_Controller(float error, float Kp)
{
    float correction = Kp*error;          // Calculate Correction
    return correction;
}

float I_Controller(float error, float Ki, float& integrator, float decayFactor)
{
    integrator += error;                  // Add error to running total
    float correction = Ki*integrator;     // Calculate Correction
    integrator /= decayFactor;            // Need to make sure running total
                                          // doesn't grow too large
    return correction;
}

float D_Controller(float error, float Kd, float& prevError, Timer& timer)
{
    float dError = error - prevError;     // Get change in error
    int dt = timer.read_us();              // Get change in time, may not be
    timer.reset();                        // Reset Time for next cycle
    prevError = error;                    // Update previous error
    float correction = Kd*dError/dt;      // Calculate Correction
    return correction;
}
//This is to update encoder values even when the rat is not moving.
//Previously this update function and forward function was one.
void update_enc()
{
    //update EL and ER
    enc_left = -wheelL.getPulses();
    enc_right = wheelR.getPulses();
    //find difference from prev values
    enc_leftD = enc_left - enc_left_prev;
    enc_rightD = enc_right - enc_right_prev;
    //update EL_prev and ER_prev
    enc_left_prev = enc_left;
    enc_right_prev = enc_right;
}

//This is to update IR values even when the rat is not moving.
//Previously this update function and forward function was one.
void update_IR()
{
    //update left and right IR
    IR_L = REC_L.read();
    //IR_RF = REC_RF.read();
    IR_LF = REC_LF.read();
    IR_R = REC_R.read();
    //find difference compared to calibrated/setup threshold
    IR_leftD = IR_left - REC_left_baseline;
    IR_rightD = IR_right - REC_right_baseline;
}
void systick_forward_enc()
{
    //find error (can optimize later)
    //NOTE: ELD and ERD are both positive if spinning forward.
    int error = enc_rightD - enc_leftD;
    //find correction
    float speedDiff = MAX_SPEED* correction_factor_enc * (P_Controller(error, Kp_enc) + I_Controller(error,Ki_enc,integrator_enc,decayFactor_enc) + D_Controller(error, Kd_enc, prevError_enc, timer_enc));
    //update speeds
      if (RSpeed > MAX_SPEED)
        RSpeed = MAX_SPEED;
      if (LSpeed > MAX_SPEED)
        LSpeed = MAX_SPEED;
      if (LSpeed < 0)
        LSpeed = 0;
      if (RSpeed < 0)
        RSpeed = 0;
    LSpeed = LSpeed + speedDiff;
    RSpeed = RSpeed - speedDiff; // right wheel runs little faster than left
}
void systick_forward_default(float error, float correction_factor, float Kp, float Kd, float Ki, float& integrator, float decayFactor, float& prevError, Timer& timer)
{
    //NOTE: Multiplied by MAX_SPEED because the speed can only be in range 0 - MAX_SPEED, whereas
    //IR values are from 0 - 1.
    float speedDiff = MAX_SPEED * correction_factor * (P_Controller(error, Kp) + I_Controller(error,Ki,integrator,decayFactor) + D_Controller(error, Kd, prevError, timer));
    //update speeds
    LSpeed = defaultSpeed + speedDiff;
    RSpeed = defaultSpeed - speedDiff;
    if (RSpeed  > MAX_SPEED)
     RSpeed = MAX_SPEED;
    if (LSpeed  > MAX_SPEED)
     LSpeed = MAX_SPEED;
    if (LSpeed  < 0)
     LSpeed = 0;
    if (RSpeed  < 0)
     RSpeed = 0;
}

void systick_forward()
{
    //update IR variables
    update_IR();
    update_enc();
    //missing no walls
    if (!detect_missing_wall())
    {

        if(wall_state != 1)
        {
            wall_state = 1;
            integrator_IR = 0;
            prevError_IR = 0;
            timer_IR.reset();
        }
        systick_forward_default(IR_rightD - IR_leftD, correction_factor_IR, Kp_IR, Kd_IR, Ki_IR, integrator_IR, decayFactor_IR, prevError_IR, timer_IR);
    }
    else if(missing_left_wall)
    {
        if(wall_state != 2)
        {
            wall_state = 4;
            integrator_enc = 0;
            prevError_enc = 0;
            timer_enc.reset();
        }
        systick_forward_enc();
    }
    else if(missing_right_wall)
    {
        if(wall_state != 3)
        {
            wall_state = 4;
            integrator_enc = 0;
            prevError_enc = 0;
            timer_enc.reset();
        }
        systick_forward_enc();
    }
    else // missing both walls
    {
        //update encoder variables
        if(wall_state != 4)
        {
            wall_state = 4;
            integrator_enc = 0;
            prevError_enc = 0;
            timer_enc.reset();
        }
        systick_forward_enc();
    }

}

void forward(int n)
{
    MLF.write(LSpeed);
    MLB.write(0);
    MRF.write(RSpeed);
    MRB.write(0);
    wait(n*0.001);
}

void setup_IR(float& cL, float& cR)
{
    IR_L = 1;
    //IR_RF = 1;
    IR_LF = 1;
    IR_R = 1;
    IR_RS = 1;
    IR_LS = 1;
    
    wait(2);

    float TcR = 0;
    float TcL = 0;
    float TcFR = 0;
    float TcFL = 0;

    for(int i= 0; i < SAMPLES; i++)
    {
       TcR += REC_R.read();
       TcL += REC_L.read();
       TcFR += REC_RF.read();
       TcFL += REC_LF.read();
    }
    TcR /= SAMPLES;
    TcL /= SAMPLES;
    TcFR /= SAMPLES;
    TcFL /= SAMPLES;
    cR = TcR;
    cL = TcL;
    REC_front_threshold = TcFR + TcFL;
    timer_IR.start();

}

bool detect_wall_front(){
    float reading = IR_frontLeft + IR_frontRight;
    //pc.printf("%f\t| %f\n", reading, REC_front_threshold);
    return (reading >= REC_front_threshold* 0.95f);
}

//0 is false; 1 is true
bool detect_missing_wall(){
    //Derek had .7f here
    //Misheel made it to 1f on Thu
    missing_left_wall = (IR_left <= .35f);
    missing_right_wall = (IR_right <= .35f);
    return (missing_left_wall || missing_right_wall);
}

void stop()
{
    Timer temp;
    temp.start();
    int baseline = temp.read_ms();
    int stop_time = 10;
    while (temp.read_ms() - baseline < stop_time){
        MLB.write(0.5f);
        MLF.write(0);
        MRB.write(0.5f);
        MRF.write(0);
    }
    MLB.write(0);
    MRB.write(0);
    wait(0.5f);
    temp.stop();
}

void setup_enc()
{
    enc_left_prev = -wheelL.getPulses();
    enc_right_prev = wheelR.getPulses();
    timer_enc.start();
}

void turn_180(){
    int left_pulse = wheelL.getPulses();
    int right_pulse = wheelR.getPulses();
    int left_pulse_prev = left_pulse;
    int right_pulse_prev = right_pulse;
    int diff = 0;
    while(abs(wheelL.getPulses()-left_pulse) + abs(wheelR.getPulses()-right_pulse) <= 780){
        //turn right
        MRF.write(defaultSpeed+diff*0.0001f); //-Kp_turn*diff);
        MRB.write(0);
        MLB.write(defaultSpeed-diff*0.0001f); //-Kp_turn*diff);
        MLF.write(0);
        wait(0.01f);
        diff = abs(wheelR.getPulses()-right_pulse_prev) - abs(wheelL.getPulses()-left_pulse_prev);
        right_pulse_prev = wheelR.getPulses();
        left_pulse_prev = wheelL.getPulses();
        //pc.printf("diff: %d\n", diff); //to debug and determine proper Kp
    }

    MLB.write(0);
    MLF.write(0);
    MRB.write(0);
    MRF.write(0);
    wait(0.1f);
}
int main()
{

    
    // ==general setup
    
    setup_IR(REC_left_baseline, REC_right_baseline);
    
    setup_enc();
    turn(true);
    turn(true);
    Systicker.attach(&systick_forward,0.01f);
    wait(1);
   // int left_pulse = -wheelL.getPulses();
   // int right_pulse = wheelR.getPulses();
   // while(-wheelL.getPulses()-left_pulse < 180 && wheelR.getPulses()-right_pulse < 180)
   // { pc.printf("%d\t%d\n", -wheelL.getPulses()-left_pulse, wheelR.getPulses()-right_pulse); }

   while(1){
        //move
        //pc.printf("iteration\n");
        //pc.printf("Wall state: %d\n", wall_state);

        if (detect_wall_front()){
            //Systicker.attach(NULL,0.01f);
            stop();
            turn(IR_left < IR_right);    // will turn left if true.
            wait(0.5f);

            //timer_IR.reset();
            //Systicker.attach(&systick_forward,0.01f);
        }
        forward(1);
    }
    timer_enc.stop();
    timer_IR.stop();
    
}
