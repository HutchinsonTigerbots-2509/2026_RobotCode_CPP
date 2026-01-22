#include "ctre/Phoenix.h"

class launcher{
//    static const int launch_motor_ID=5;
    public:
        launcher();
        void fireLaunch(double trigger);
    private:
        ctre::phoenix::motorcontrol::can::VictorSPX firing;
};