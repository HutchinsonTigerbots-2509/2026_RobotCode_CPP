#include <subsystems/intake.h>
    intake::intake()
        :intakeMotor(6){}
        void intake::takingIn(double trigger){
            if(trigger>=0.1){
                intakeMotor.Set(ControlMode::PercentOutput,1);
            }else {
                stop();
            }
        }
            void intake::stop(){
                intakeMotor.Set(ControlMode::PercentOutput,0);
            };