#include <subsystems/launcher.h>
#include<math.h>

launcher::launcher()
  : launchMotor(5)
    {}

void launcher::Launch(double trigger) {
    // double velocity;
    // if(trigger>=0.75){
    //     velocity=4500;
    // }else if(trigger>=0.5){
    //     velocity=3000;
    // }else{
    //     velocity=1500;
    // }
    launchMotor.Set(ControlMode::PercentOutput,trigger);
};
