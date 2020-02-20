 /*
  * CustomStepperMetamorphicManipulator.h - Library for controlling Steppers of a Metamorphic Manipulator
  * Created by N.A. Stravopodis, February , 2020.
  */

#ifndef CustomStepperMetamorphicManipulator_h
#define CustomStepperMetamorphicManipulator_h

#include "Arduino.h"
#include <Streaming.h>
#include <vector>

using namespace std;

const float pi              = 3.14159265359;

extern bool return_function_state;
extern vector<double> TrajAssignedDuration;
extern vector<double> StpTrapzProfParams;
extern vector<unsigned long> PROFILE_STEPS;

class CustomStepperMetamorphicManipulator
{
    public:
        long currentAbsPos;
        long currentMoveRel;
        int currentDirStatus;
        bool segmentExists;
        bool positionReached;
        bool unlockStepper;

        CustomStepperMetamorphicManipulator(int stepID, int stepPin, int dirPin, int enblPin, int ledPin, int hallSwitchPin, int lockPin, int spr, int GEAR_FACTOR, int ft );
        
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

        // Moves motor to home position - Hall Sensor Needed
        bool setStepperHomePosition();

        // Executes Trajectory - sets value to PositionReached
        bool executeStepperTrapzProfile(bool segmentExists, vector<unsigned long> PROFILE_STEPS, double Texec, double delta_t);

        // Locks Motor - sets value to ManipulatorMode = {true = Action, false = Metamorphosis}
        bool lockMotor(bool positionReached);

        // Unlocks Motor - sets value to ManipulatorMode = {true = Metamorphosis, false = Action}
        bool unlockMotor(bool unlockStepper);

    private:
        int _stepID;
        int _stepPin;
        int _dirPin;
        int _enblPin;
        int _ledPin;
        int _hallSwitchPin;
        int _lockPin;
        int _spr;
        int _GEAR_FACTOR;
        int _ft;
        float _a;
        float _ag;
        float _accel_width;
        
        void singleStepVarDelay(unsigned long delayTime);
};

 #endif