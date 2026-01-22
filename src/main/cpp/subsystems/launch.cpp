#include <subsystems/launch.h>
launcher::launcher()
    :firing(5)
    {}
void launcher::fireLaunch(double triggerInput){
    double velocity;
    if (triggerInput>=.75){
        velocity=5600;
    }else if (triggerInput>=.5){
        velocity=4000;
    }else if (triggerInput>=.25){
        velocity=3000;
    }else{
        velocity=2000;
    }
    firing.Set(ControlMode::Velocity,velocity);
    /*This is commented because we don't have a camera set up so we can't use this.
    double distance = camera.GetDistance()
    velocity=(distance*10)+1000;
    We have the plus 1000 because that is the velocity needed to score when we are right up next to the goal or as we can be.
    if (triggerInput>=0.1&& aprilTag detected){
        firing.Set(ControlMode::Velocity,velocity);
    }
    */
}