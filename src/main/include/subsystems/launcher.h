#include "ctre/Phoenix.h"

class launcher {
    public:
        launcher();
        void Launch(double trigger);
    private:
        ctre::phoenix::motorcontrol::can::VictorSPX launchMotor;
};