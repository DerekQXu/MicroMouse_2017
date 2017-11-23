#include "mbed.h"
#include "QEI.h"
//#define ENC_PER_REV 360

//IR LED init
DigitalOut
    LED_left(PB_7),
    LED_frontLeft(PB_0),
    LED_frontRight(PC_11),
    LED_right(PC_10);

//IR RECEIVER init
AnalogIn
    REC_left(PC_0),
    REC_frontLeft(PA_4),
    REC_frontRight(PC_1),
    REC_right(PA_0);
    
float
    REC_left_val,
    REC_frontLeft_val,
    REC_frontRight_val,
    REC_right_val;
    
//NEW CONSTANTS
float threshold = 0.5; // TODO: find optimal threshold to detect wall (b/w 0 and 1)
float baseline_front_left = 0;
float baseline_front_right = 0;
float baseline_left = 0;
float baseline_right = 0;

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

int ELD, ERD, EL, ER, EL_prev, ER_prev;         // ELD = encoder value difference of left motor

float LSpeed = 0.2f,
      RSpeed = 0.16f;

float Kp = 6,
      Ki = 2,
      Kd = 20;

float integrator = 0;     // I_Controller() variable
float decayFactor = 2;    // I_Controller() variable
float prevError = 0;      // D_Controller() variable
float correction_factor = 1/(3600*(LSpeed+RSpeed));

Ticker Systicker;
Timer timer;
Serial pc(usbTX, usbRX);

PwmOut MLF(mLF), MRF(mRF), MLB(mLB), MRB(mRB);

QEI wheelL(mLencA, mLencB, NC, 624, QEI::X4_ENCODING);
QEI wheelR(mRencA, mRencB, NC, 624, QEI::X4_ENCODING);

float P_Controller(int error)
{
    float correction = Kp*error;          // Calculate Correction
    //FOR DEBUGGING:
    //pc.printf("Perror: %d\t", error);
    //pc.printf("Pcorr: %f\t", correction);
    return correction;
}

float I_Controller(int error)
{
    integrator += error;                  // Add error to running total
    float correction = Ki*integrator;     // Calculate Correction
    integrator /= decayFactor;            // Need to make sure running total
    // doesn't grow too large
    //FOR DEBUGGING:
    //pc.printf("Ierror: %d\t", error);
    //pc.printf("Icorr: %f\t", correction);
    return correction;
}

float D_Controller(int error)
{
    float dError = error - prevError;     // Get change in error
    int dt = timer.read_us();             // Get change in time
    timer.reset();                        // Reset Time for next cycle
    prevError = error;                    // Update previous error
    float correction = Kd*dError/dt;      // Calculate Correction
    //FOR DEBUGGING:
    //pc.printf("dError: %f\t", dError);
    //pc.printf("dcorr: %f\t", correction);
    //pc.printf("dt: %d\n", dt);
    return correction;
}


void systick_forward()
{
    //update EL and ER
    EL = -wheelL.getPulses();
    ER = wheelR.getPulses();
    //update EL_prev and ER_prev
    ELD = EL - EL_prev;
    ERD = ER - ER_prev;
    EL_prev = EL;
    ER_prev = ER;
    //find error (can optimize later)
    //NOTE: ELD and ERD are both positive if spinning forward.
    int error = ERD - ELD;
    //find correction
    float speedDiff = correction_factor * (P_Controller(error) + I_Controller(error) + D_Controller(error));
    //update speeds
    LSpeed = LSpeed + speedDiff;
    RSpeed = RSpeed - speedDiff;
}


void stop()
{
    MLF.write(0);
    MLB.write(0);
    MRF.write(0);
    MRB.write(0);
}

void forward(int n)
{
    //pc.printf("wheelL: %i\t", -wheelL.getPulses());
    //pc.printf("wheelR: %i\n", wheelR.getPulses());
    //continuously write to the motor
    //int fwdLTemp = -wheelL.getPulses();
    //int fwdRTemp = wheelR.getPulses();
    MLF.write(LSpeed);
    MRF.write(RSpeed);
    //pc.printf("%f, ", LSpeed);
    //pc.printf("%f, ", RSpeed);
    //pc.printf("%d, ", fwdLTemp - fwdL);
    //pc.printf("%d\n", fwdRTemp - fwdR);
    //fwdL = fwdLTemp;
    //fwdR = fwdRTemp;
    wait(n*0.001);
}

void setup_PID()
{
    //variable initiation
    timer.start();
    //use interrupt to apply PID
    Systicker.attach(&systick_forward, 0.01f);
    EL_prev = -wheelL.getPulses();
    ER_prev = wheelR.getPulses();
}
/*
void setup_IR()
{
    //INITIALIZE BASELINES
    int count = 20;
    for (int i = 0; i < count; ++i){
        baseline_front_left += input_receiver_front_left.read();
        baseline_left += input_slant_IRl.read();
        baseline_front_right += input_receiver_front_left.read();
        baseline_right += input_slant_IRl.read();
    }
    baseline_front_left /= count;
    baseline_left /= count;
    baseline_front_right /= count;
    baseline_right /= count;
}
*/


int main()
{
/*    setup_PID();
   while(1){
        //check if this wait() is necessary
        wait(0.1f);
        forward(1);
    }
    timer.stop();
*/
    LED_left = 1;
    LED_frontRight = 1;
    LED_frontLeft = 1;
    LED_right = 1;
    while(1)
    {
        REC_left_val = REC_left.read();
        REC_frontRight_val = REC_frontRight.read();
        REC_frontLeft_val = REC_frontLeft.read();
        REC_right_val = REC_right.read();
        pc.printf("left: %f\tfront left: %f\tfront right: %f\tright: %f\n",
                REC_left_val,
                REC_frontRight_val,
                REC_frontLeft_val,
                REC_right_val);
    }
}
