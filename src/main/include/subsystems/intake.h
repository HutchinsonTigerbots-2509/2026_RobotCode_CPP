#include "ctre/Phoenix.h"
#include "Robot.h"
class intake {
    public:
        intake();
            void takingIn(double trigger);
            void stop();
            ctre::phoenix::motorcontrol::can::VictorSPX intakeMotor;
};