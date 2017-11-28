#include "mbed.h"
#include "QEI.h"
#define RECEIVER_THRESHOLD 0.1 //not used atm
#define SAMPLES 20
#define MAX_SPEED 0.1f
float defaultSpeed = MAX_SPEED;

////MODIFICATIONS FROM MAIN_REVISED.CPP////
// 1. added/revised turning and stopping code (not tested)
// 2. removed unused code
// 3. minor cleanup
/////////////TO DO/////////////////
// 1. (Optional) Tune encoder so that it goes straight with IR enabled
// 2. Check if dt in D_controller needs 10^(-6) multiplier because us.
// 3. Tune all K values
// 4. Tune stopping
// 5. Naming conventions becoming ambiguous again (thresholds in IR setup is actually BASELINEs)
///////////////////////////////////

///////////////////////
// K Values
///////////////////////
float Kp_enc = 3,
      Ki_enc = 2,
      Kd_enc = 20,
      Kp_IR = 0.5,
      Ki_IR = 2,
      Kd_IR = 20;

///////////////////////
// Speed Variables
///////////////////////
float LSpeed = MAX_SPEED,
      RSpeed = MAX_SPEED,
      enc_correction = 0.8;   // Right wheel turns slightly faster with IR enc both enabled.

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
float correction_factor_IR = 0.2;

//////////////////////
// Pin definitions
/////////////////////
DigitalOut LED_left(PB_7);
DigitalOut LED_frontLeft(PB_0);
DigitalOut LED_frontRight(PC_11);
DigitalOut LED_right(PC_10);

AnalogIn REC_left(PC_0);
AnalogIn REC_frontLeft(PC_1);
AnalogIn REC_frontRight(PA_4);
AnalogIn REC_right(PA_0);

PinName mLencA = PA_15,
        mLencB = PB_3,
        mRencA = PA_1,
        mRencB = PC_4,
        usbTX = PA_2,
        usbRX = PA_3,
        mLB = PC_7,
        mLF = PB_10,
        mRF = PA_7,
        mRB = PB_6;
PwmOut  MLF(mLF),
        MRF(mRF),
        MLB(mLB),
        MRB(mRB);

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
    IR_left, IR_right,
    IR_left_prev,
    IR_right_prev;              // IR_leftD = IR value difference of left motor

////////////////////////
// IR Setup Variables
////////////////////////
float
    IR_threshold,         // not used atm, for hardcoding/testing
//TODO: THESE ARE NOT THRESHOLDS; THESE ARE BASELINES!
    IR_left_threshold,
    IR_right_threshold;

////////////////////////
// IR Values/Readings
////////////////////////
float
    REC_val_left,
    REC_val_right,
    REC_val_frontLeft,
    REC_val_frontRight;         // IR readings

///////////////////////
// Timer/Serial
///////////////////////
Ticker Systicker;
Timer timer;
Serial pc(usbTX, usbRX);

///////////////////////
// Motor init
///////////////////////
QEI wheelL(mLencA, mLencB, NC, 624, QEI::X4_ENCODING);
QEI wheelR(mRencA, mRencB, NC, 624, QEI::X4_ENCODING);

//////////////////////
// FUNCTIONS
//////////////////////

// Turn immediately to direction dir (pos right, left neg)
void turn(bool isRight){
    //right now turning with time not encoder or IR
    Timer temp;
    temp.start();
    int baseline = temp.read_ms();
    int turn_time = 2000;
    int diff;
    while (temp.read_ms() - baseline < turn_time){
        //turn right
        if (isRight){
            MLF.write(MAX_SPEED); //-Kp_turn*diff);
            MLB.write(0);
            MRB.write(MAX_SPEED); //-Kp_turn*diff);
            MRF.write(0);
        }
        //turn left
        else{
            MLB.write(MAX_SPEED); //-Kp_turn*diff);
            MLF.write(0);
            MRF.write(MAX_SPEED); //-Kp_turn*diff);
            MRB.write(0);
        }
        diff = wheelR.getPulses() - wheelL.getPulses();
        pc.printf("diff: %d\n", diff); //to debug and determine proper Kp
    }
    temp.stop();
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

float D_Controller(float error, float Kd, float& prevError )
{
    float dError = error - prevError;     // Get change in error
    float dt = timer.read_us()*10^(-6);   // Get change in time, may not be
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
void systick_forward_enc()
{
    //update encoder variables
    update_enc();
    //find error (can optimize later)
    //NOTE: ELD and ERD are both positive if spinning forward.
    int error = enc_rightD - enc_leftD;
    //find correction
    float speedDiff = MAX_SPEED* correction_factor_enc * (P_Controller(error, Kp_enc) + I_Controller(error,Ki_enc,integrator_enc,decayFactor_enc) + D_Controller(error, Kd_enc, prevError_enc));
    //update speeds
      if ( enc_correction* (RSpeed - speedDiff) > MAX_SPEED)
        RSpeed = MAX_SPEED;
      if (LSpeed + speedDiff > MAX_SPEED)
        LSpeed = MAX_SPEED;
      if (LSpeed + speedDiff < 0)
        LSpeed = 0;
      if (RSpeed - speedDiff < 0)
        RSpeed = 0;
    LSpeed = LSpeed + speedDiff;
    RSpeed = enc_correction * (RSpeed - speedDiff); // right wheel runs little faster than left
}
//This is to update IR values even when the rat is not moving.
//Previously this update function and forward function was one.
void update_IR()
{
    //update left and right IR
    IR_left = REC_val_left;
    IR_right = REC_val_right;
    //find difference compared to calibrated/setup threshold
    IR_leftD = IR_left - IR_left_threshold;
    IR_rightD = IR_right - IR_right_threshold;
}

void systick_forward_IR()
{
    //update IR variables
    update_IR();
    //find error
    float error = (IR_rightD - IR_leftD); // in a way correction factor;
    //NOTE: Multiplied by MAX_SPEED because the speed can only be in range 0 - MAX_SPEED, whereas
    //IR values are from 0 - 1.
    float speedDiff = MAX_SPEED * correction_factor_IR * (P_Controller(error, Kp_IR) + I_Controller(error,Ki_IR,integrator_IR,decayFactor_IR) + D_Controller(error, Kd_IR, prevError_IR));
    //update speeds
    //pc.printf("%f\n", speedDiff);
    if (RSpeed - speedDiff > MAX_SPEED)
     RSpeed = MAX_SPEED;
    if (LSpeed + speedDiff > MAX_SPEED)
     LSpeed = MAX_SPEED;
    if (LSpeed + speedDiff < 0)
     LSpeed = 0;
    if (RSpeed - speedDiff < 0)
     RSpeed = 0;
    LSpeed = defaultSpeed + speedDiff;
    RSpeed = defaultSpeed - speedDiff;
}
//attached to systicker var in PID setup func
void systick_forward()
{
    systick_forward_enc();
    systick_forward_IR();
}

void update_sensors()
{
    //update_enc();
    update_IR();
}

void forward(int n)
{
    MLF.write(LSpeed);
    MRF.write(RSpeed);
    wait(n*0.001);
}

void setup_IR(float& cL, float& cR)
{
  LED_left = 1;
  LED_frontRight = 1;
  LED_frontLeft = 1;
  LED_right = 1;
  wait(2);

  float TcR = 0;
  float TcL = 0;
  float TcFR = 0;
  float TcFL = 0;

  for(int i= 0; i < SAMPLES; i++)
  {
    TcR += REC_right.read();
    TcL += REC_left.read();
    TcFR += REC_frontRight.read();
    TcFL += REC_frontLeft.read();
  }
  TcR /= SAMPLES;
  TcL /= SAMPLES;
  TcFR /= SAMPLES;
  TcFL /= SAMPLES;
  cR = TcR;
  cL = TcL;
  IR_threshold = TcFR + TcFL;
}

void read_IR()
{
    REC_val_left = REC_left.read() - IR_left_threshold;
    REC_val_frontRight = REC_frontRight.read();
    REC_val_frontLeft = REC_frontLeft.read();
    REC_val_right = REC_right.read() - IR_right_threshold;
}

bool detect_wall(){
    float reading = REC_val_frontRight + REC_val_frontRight;
    return (reading < IR_threshold) ? false : true;
}

//0 is false; 1 is true
bool detect_isRight(){
    return (REC_val_left - IR_left_threshold) > (REC_val_right - IR_right_threshold) ? true : false;
}

void stop()
{
    Timer temp;
    temp.start();
    int baseline = temp.read_ms();
    int stop_time = 10;
    while (temp.read_ms() - baseline < stop_time){
        MLB.write(0.5f); //-Kp_turn*diff);
        MLF.write(0);
        MRB.write(0.5f); //-Kp_turn*diff);
        MRF.write(0);
    }
    temp.stop();
}

int main()
{
    // general setup
    setup_IR(IR_left_threshold, IR_right_threshold);
    //setup_PID();

   while(1){
        //read values from IR
        read_IR();
        wait(0.1f);
        //move
        if (detect_wall()){
            stop();
            turn(detect_isRight());
        }
        else
            forward(1);
    }
    timer.stop();
}
