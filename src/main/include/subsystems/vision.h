#include <stdlib.h>
#include "LimelightHelpers.h"

/*---- April Tag Libraries ----*/
#include <frc/apriltag/AprilTag.h>
#include <frc/apriltag/AprilTagDetection.h>
#include <frc/apriltag/AprilTagDetector.h>
#include <frc/apriltag/AprilTagDetector_cv.h>
#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/apriltag/AprilTagFields.h>
#include <frc/apriltag/AprilTagPoseEstimate.h>
#include <frc/apriltag/AprilTagPoseEstimator.h>
#include <frc/MathUtil.h>
#include <frc/geometry/Pose2d.h>
#include <frc/DriverStation.h>
#include <Robot.h>
#include <vector>
/*
THIS IS A DRAWING NOT DELETING. I'M TALKING TO YOU ACESON
                            ---------------------


                                    Tench
                                FRONT   BACK


                            -      LEFT         -
            RIGHT           -F                 B-
Driver      Tower           -R       hub       A-  
Station     LEFT            -O                 C-
                            -N                 K-
                            -T                  -
                            -      RIGHT        -

                                    
    RIGHT                            Tench
    Outpost                       FRONT   BACK
    LEFT
                            ---------------------
*/


enum {
    RED_HUB_FRONT_RIGHT = 10,
    RED_HUB_FRONT_LEFT = 9,
    RED_HUB_LEFT_LEFT = 5, 
    RED_HUB_LEFT_RIGHT = 8, 
    RED_HUB_RIGHT_LEFT = 11, 
    RED_HUB_RIGHT_RIGHT = 2, 
    RED_HUB_BACK_LEFT = 3, 
    RED_HUB_BACK_RIGHT = 4, 
    RED_RIGHT_TRENCH_CLOSE = 12, 
    RED_RIGHT_TRENCH_FAR = 1, 
    RED_LEFT_TRENCH_CLOSE = 7, 
    RED_LEFT_TRENCH_FAR = 6, 
    RED_TOWER_WALL_LEFT = 15, 
    RED_TOWER_WALL_RIGHT = 16,
    RED_OUTPOST_LEFT = 14, 
    RED_OUTPOST_RIGHT = 13
}(RED_APRIL_TAG_IDS);
enum {
    BLUE_HUB_FRONT_RIGHT = 26,
    BLUE_HUB_FRONT_LEFT = 25,
    BLUE_HUB_LEFT_LEFT = 21,
    BLUE_HUB_LEFT_RIGHT = 24,
    BLUE_HUB_RIGHT_LEFT = 27,
    BLUE_HUB_RIGHT_RIGHT = 18,
    BLUE_HUB_BACK_LEFT = 19,
    BLUE_HUB_BACK_RIGHT = 20,
    BLUE_RIGHT_TRENCH_CLOSE = 28,
    BLUE_RIGHT_TRENCH_FAR = 17, 
    BLUE_LEFT_TRENCH_CLOSE = 23, 
    BLUE_LEFT_TRENCH_FAR = 22, 
    BLUE_TOWER_WALL_LEFT = 32, 
    BLUE_TOWER_WALL_RIGHT = 31,
    BLUE_OUTPOST_LEFT = 30, 
    BLUE_OUTPOST_RIGHT = 29
}(BLUE_APRIL_TAG_IDS);

double getDistanceFromHub(std::string limelightId){
    double aprilTagId=LimelightHelpers::getFiducialID(limelightId);    // this isn't giving me an error but not what the estimate distance docs says what  to do
    double mountAngle=12;   //The angle of the mount in degrees
    double lensHeight=6.75;  //This is in inches.Change this to get the corrct distance
    double goalHeight=11.5; //In inches
    double angleToGoal;
    double radianAngleToGoal;
    double distance;
    if(frc::DriverStation::GetAlliance()==frc::DriverStation::Alliance::kRed){
        if((LimelightHelpers::getTargetCount(limelightId)>0)&&(aprilTagId==RED_APRIL_TAG_IDS)){	     
            LimelightHelpers::SetFiducialIDFiltersOverride(limelightId,std::vector<int>{1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16});
            //tl = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumberArray("botpose",std::vector<double>(6));
            distance = (nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumberArray("botpose",std::vector<double>(6)))[2];
            return distance;
        }else{
            return -1;
        }
    }else if(frc::DriverStation::GetAlliance()==frc::DriverStation::Alliance::kBlue){
        if((LimelightHelpers::getTargetCount(limelightId)>0)&&(aprilTagId==BLUE_APRIL_TAG_IDS)){
            LimelightHelpers::SetFiducialIDFiltersOverride(limelightId,std::vector<int>{17,18,19,20,21,22,23,24,25,26,27,28,29,30,31});
            radianAngleToGoal=angleToGoal*(3.14159 / 180.0);
            distance=(goalHeight-lensHeight)/tan(radianAngleToGoal);
            distance=distance*0.035;
            return distance;
        }else{
            return -1;
        }               
    }else{
        return -1;
    }  
}	
    //Determine If April Tags Are Visible  DONE


    //Determine If Visible April Tags Hub Tags That Match Your Alliance Color DONE

    //Get Distance From Relevant Tag And Return Value (Return -1 if no relevent target is found)            Needs to be checked

    //Bonus points if you can determine your rotational relationship to the hub by using both the right and left april tags on the structure
