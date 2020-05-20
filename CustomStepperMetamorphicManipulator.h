 /*
  * CustomStepperMetamorphicManipulator.h - Library for controlling Steppers of a Metamorphic Manipulator
  * Created by N.A. Stravopodis, February , 2020.
  */

#ifndef CustomStepperMetamorphicManipulator_h
#define CustomStepperMetamorphicManipulator_h

// OPENCR EEPROM AREA ADDRESSES FOR STEPPER NEMA34[0~255]
#define CP_JOINT1_STEPPER_EEPROM_ADDR    1      // float    
#define CD_JOINT1_STEPPER_EEPROM_ADDR    10     // uint32_t
#define VL_JOINT1_STEPPER_EEPROM_ADDR    20     // double
#define AL_JOINT1_STEPPER_EEPROM_ADDR    30     // double    

#include "Arduino.h"
#include <vector>
#include <fstream>

using namespace std;

const float pi              = 3.14159265359;

extern bool return_function_state;
extern vector<double> TrajAssignedDuration;
extern vector<double> StpTrapzProfParams;
extern vector<unsigned long> PROFILE_STEPS;
extern vector<double> vector_for_trajectoryVelocity;

extern unsigned long time_now_micros;
extern unsigned long time_now_millis;

extern unsigned long currentAbsPos;
extern double currentAbsPos_double;
extern long currentMoveRel;
extern byte currentDirStatus;
extern bool segmentExists;
extern bool positionReached;
extern bool unlockStepper;
extern double VelocityLimitStp;
extern double AccelerationLimitStp;

class CustomStepperMetamorphicManipulator
{
    public:

        CustomStepperMetamorphicManipulator(int stepID, int stepPin, int dirPin, int enblPin, int ledPin, int hallHomePin, int limitSwitchMinPin, int limitSwitchMaxPin, int lockPin, int spr, int GEAR_FACTOR, int ft );
        
        // read EEPROM settings for stepper Motion Profiles 
        void read_STP_EEPROM_settings( byte * currentDirStatus, double * currentAbsPos_double, double * VelocityLimitStp, double * AccelerationLimitStp);
        void save_STP_EEPROM_settings( byte * currentDirStatus, double * currentAbsPos_double, double * VelocityLimitStp, double * AccelerationLimitStp);

        // User gives Texec, hAbs

        // Returns hRel
        double setStepperGoalPositionAssignedDuration(double Texec, double hAbs);                                                       

        // Returns: StpTrapzProfParams
        vector<double> returnTrajAssignedDurationProperties(double Texec, double hRel);

            // Returns: segmentExists
            bool segmentExists_TrapzVelProfile(vector<double> StpTrapzProfParams);
            
            // Returns: PROFILE_STEPS
            vector<unsigned long> returnTrapzVelProfileSteps(vector<double> StpTrapzProfParams, bool segmentExists);

            // Returns: delta_t
            double calculateInitialStepDelay(vector<double> StpTrapzProfParams);

        // Moves motor to home position - Hall Sensor and Limit switches Needed
        bool setStepperHomePositionSlow();
        
        // Moves motor to home position - Hall sensor only for evaluation, No Limit switches Needed - currentAbsPos is read from EEPROM
        bool setStepperHomePositionFast(double * currentAbsPos_double, unsigned long * currentAbsPos,  byte *currentDirStatus);

        // Executes Trajectory - sets value to PositionReached
        bool executeStepperTrapzProfile(bool segmentExists, vector<unsigned long> PROFILE_STEPS, double Texec, double delta_t);

        // Locks Motor - sets value to ManipulatorMode = {true = Action, false = Metamorphosis}
        bool lockMotor(bool positionReached);

        // Unlocks Motor - sets value to ManipulatorMode = {true = Metamorphosis, false = Action}
        bool unlockMotor(bool unlockStepper);

        void printDoubleVector2csvFile(vector<double> vector_for_trajectoryVelocity);

    private:
        int _stepID;
        int _stepPin;
        int _dirPin;
        int _enblPin;
        int _ledPin;
        int _hallHomePin;
        int _limitSwitchMinPin;
        int _limitSwitchMaxPin;
        int _lockPin;
        int _spr;
        int _GEAR_FACTOR;
        int _ft;
        float _a;
        float _ag;
        float _accel_width;
        double _step_delay_time;
        double _simultaneous_velocity;

        void singleStepVarDelay(unsigned long delayTime);

        // Returns _simultaneous_velocity : This vaue is pushed back to a global storage vector
        double trajectorySimultaneousVelocity(double _step_delay_time);

        // Returns vector of Angular Acceleration Values during Trajectory execution
        //vector<double> trajectoryAcceleration(double _simultaneous_velocity);

};

 #endif