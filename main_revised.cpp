#include "mbed.h"
#include "QEI.h"

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

float LSpeed = 0.02f,
      RSpeed = 0.05f;

float Kp = 1.1,
      Ki = 0,
      Kd = 0.9;

float integrator = 0;     // I_Controller() variable
float decayFactor = 2;    // I_Controller() variable
float prevError = 0;      // D_Controller() variable

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
    pc.printf("error: %d\n", error);
    return correction;
}

float I_Controller(int error)
{
    integrator += error;                  // Add error to running total
    float correction = Ki*integrator;     // Calculate Correction
    integrator /= decayFactor;            // Need to make sure running total
    // doesn't grow too large
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
    pc.printf("dError: %d\t", dError);
    pc.printf("dt: %d\n", dt);
    return correction;
}


void systick_forward()
{
    //update EL_prev and ER_prev
    ELD = EL - EL_prev;
    ERD = ER - EL_prev;
    EL = EL_prev;
    ER = ER_prev;
    //find error (can optimize later)
    //NOTE: ELD and ERD are of opposite signs, impl. in hardware
    int error = ELD + ERD;
    //find correction
    float speedDiff = correction_factor * (P_Controller(error) + I_Controller(error) + D_Controller(error));
    //update speeds
//    LSpeed += speedDiff;
//    RSpeed -= speedDiff;
    //FOR DEBUGGING:
    pc.printf("ELD: %d\t", ELD);
    pc.printf("ERD: %d\n", ERD);
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
    //use interrupt to apply PID
    Systicker.attach(&systick_forward, 0.01f);
    wait(2);
    stop();
    //update EL and ER
    EL = wheelL.getPulses();
    ER = wheelR.getPulses();
    //continuously write to the motor
    MLF.write(LSpeed);
    MRF.write(RSpeed);
}

void setup()
{
    //variable initiation
    EL_prev = wheelL.getPulses();
    ER_prev = wheelR.getPulses();
    EL = wheelL.getPulses();
    ER = wheelR.getPulses();
}

int main()
{
    setup();
    while(1){
        //check if this wait() is necessary
        wait(0.1f);
        forward(1);
    }
}
