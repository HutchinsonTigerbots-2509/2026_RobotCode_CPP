#include <stdlib.h>
#include "LimelightHelpers.h"
#include <math.h>

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
#include <frc/smartdashboard/SmartDashboard.h>
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

const int RED_APRIL_TAGS[16] = {
    RED_HUB_FRONT_RIGHT, RED_HUB_FRONT_LEFT, RED_HUB_LEFT_LEFT, RED_HUB_LEFT_RIGHT, RED_HUB_RIGHT_LEFT, RED_HUB_RIGHT_RIGHT, RED_HUB_BACK_LEFT, RED_HUB_BACK_RIGHT, RED_RIGHT_TRENCH_CLOSE, RED_RIGHT_TRENCH_FAR, RED_LEFT_TRENCH_CLOSE, RED_LEFT_TRENCH_FAR, RED_TOWER_WALL_LEFT, RED_TOWER_WALL_RIGHT, RED_OUTPOST_LEFT, RED_OUTPOST_RIGHT
};
const int RED_HUB_APRIL_TAGS[16] = {
    RED_HUB_FRONT_RIGHT, RED_HUB_FRONT_LEFT, RED_HUB_LEFT_LEFT, RED_HUB_LEFT_RIGHT, RED_HUB_RIGHT_LEFT, RED_HUB_RIGHT_RIGHT, RED_HUB_BACK_LEFT, RED_HUB_BACK_RIGHT
};
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
const int BLUE_APRIL_TAGS[16] = {
    BLUE_HUB_FRONT_RIGHT, BLUE_HUB_FRONT_LEFT, BLUE_HUB_LEFT_LEFT, BLUE_HUB_LEFT_RIGHT, BLUE_HUB_RIGHT_LEFT, BLUE_HUB_RIGHT_RIGHT, BLUE_HUB_BACK_LEFT, BLUE_HUB_BACK_RIGHT, BLUE_RIGHT_TRENCH_CLOSE, BLUE_RIGHT_TRENCH_FAR, BLUE_LEFT_TRENCH_CLOSE, BLUE_LEFT_TRENCH_FAR, BLUE_TOWER_WALL_LEFT, BLUE_TOWER_WALL_RIGHT, BLUE_OUTPOST_LEFT, BLUE_OUTPOST_RIGHT
};
const int BLUE_HUB_APRIL_TAGS[16] = {
    BLUE_HUB_FRONT_RIGHT, BLUE_HUB_FRONT_LEFT, BLUE_HUB_LEFT_LEFT, BLUE_HUB_LEFT_RIGHT, BLUE_HUB_RIGHT_LEFT, BLUE_HUB_RIGHT_RIGHT, BLUE_HUB_BACK_LEFT, BLUE_HUB_BACK_RIGHT
};


/*
    Function: getRawFiducials
    Parameters:     std::string limelight_id            :   The name of the limelight camera you are using
    Output:         return vector<FiducialResultClass>  :   Return a vector of FiducialResults that are currently being given by 
                                                            the limelight network table
                                                            (what April Tags can be seen and what their values are)
    Notes: LimelightResultsClass is the top-level container for all Limelight results parsed from JSON. FiducialResultsClass 
    represents an AprilTag/fiducial target result from the Limelight's JSON output 
*/
std::vector<LimelightHelpers::RawFiducial> getRawFiducials(std::string limelight_id){
    LimelightHelpers::PoseEstimate BotPoseEstimate = LimelightHelpers::getBotPoseEstimate("limelight-b", "AprilTagInfo", false);
    return BotPoseEstimate.rawFiducials;
}

double getDistanceFromHub(std::string limelightId){
    double aprilTagId=LimelightHelpers::getFiducialID("limelight-b");
    // double mountAngle=12;   //The angle of the mount in degrees
    // double lensHeight=6.75;  //This is in inches.Change this to get the corrct distance
    // double goalHeight=11.5; //In inches
    // double angleToGoal;
    // double radianAngleToGoal;
    int ClosestAprilTag_id;
    int partnerTag;
    int parterTagChecker;
    const double tagSize=6.5;
    double closestCurrentArea;
    double furtherCurrentArea;
    double cornerY1;
    double cornerY2;
    double cornerY0;
    double cornerY3;
    double distance1;
    double distance2;
    if(frc::DriverStation::GetAlliance()==frc::DriverStation::Alliance::kRed){
        /*frc::SmartDashboard::PutNumber("power",aprilTagId);
        frc::SmartDashboard::PutBoolean("TV", LimelightHelpers::getTV("limelight-b"));
        */
       if(LimelightHelpers::getTV("limelight-b")){	  //Get past here   
            /* Limelight Notes:
            *   FiducialResultClass represents an AprilTag/fiducial target result from JSON output. FiducialResultClass inherits 
                from the SingleTargetingResultClass so contains the following public attributes:
                double m_TargetXPixels
                double m_TargetYPixels
                double m_TargetXNormalized
                double m_TargetYNormalized
                double m_TargetXNormalizedCrosshairAdjusted
                double m_TargetYNormalizedCrosshairAdjusted
                double m_TargetXDegreesCrosshairAjusted
                double m_TargetYDegreesCrosshairAdjusted
                double m_TargetAreaPixels
                double m_TargetAreaNormalized
                double m_TargedAreaNormalizedPercentage
                double m_timeStamp
                double m_latency
                double m_pipelineIndex
                std::vector<std::vector<double>> m_TargetCorners   -> x, y corner coordinates
                std::vector<double> m_CAMERATransform6DTARGETSPACE
                std::vector<double> m_TargetTransform6DCAMERASPACE
                std::vector<double> m_TargetTransform6DROBOTSPACE
                std::vector<double> m_RobotTransform6DTARGETSPACE
                std::vector<double> m_RobotTransform6DFIELDSPACE
                std::vector<double> m_CAMERTransform6DROBOTSPACE

                **6D being x, y, z, roll, pitch, yaw
            */
            //std::vector<LimelightHelpers::FiducialResultClass> aprilTagResults = getRawFiducials("limelight-b"); //Get data for all visible April Tags from limelight
            std::vector<LimelightHelpers::RawFiducial> aprilTagResults = getRawFiducials("limelight-b");
            if(aprilTagResults.empty()){return -42;}
            for(LimelightHelpers::RawFiducial aprilTag : aprilTagResults){ //Iterate through all visible april tags
                if((std::find(std::begin(RED_HUB_APRIL_TAGS), std::end(RED_APRIL_TAGS), aprilTag.ta)) != std::end(RED_APRIL_TAGS)){ //check to see if the aprilTag is a RED tag
                    if(aprilTag.ta  > closestCurrentArea){
                        closestCurrentArea=aprilTag.ta;
                        ClosestAprilTag_id=aprilTag.id;
                    }else{
                    return -7;
                }
                }else{
                    return -4;
                }  
            }
           for(LimelightHelpers::RawFiducial closestTag : aprilTagResults){
                if(closestTag.id == ClosestAprilTag_id){    
                    if(closestTag.id==LimelightHelpers::INVALID_TARGET){   //NO hub tag was found         //This is where the error is
                        return-99;
                    }else{
                        switch (closestTag.id){
                        case RED_HUB_BACK_LEFT:
                            parterTagChecker=RED_HUB_BACK_RIGHT;
                            break;
                        case RED_HUB_BACK_RIGHT:
                            parterTagChecker=RED_HUB_BACK_LEFT;
                            break;
                        case RED_HUB_FRONT_LEFT:
                            parterTagChecker=RED_HUB_FRONT_RIGHT;
                            break;
                        case RED_HUB_FRONT_RIGHT:
                            parterTagChecker=RED_HUB_FRONT_LEFT;
                            break;
                        case RED_HUB_LEFT_LEFT:
                            parterTagChecker=RED_HUB_LEFT_RIGHT;
                            break;
                        case RED_HUB_LEFT_RIGHT:
                            parterTagChecker=RED_HUB_LEFT_LEFT;
                            break;
                        case RED_HUB_RIGHT_LEFT:
                            parterTagChecker=RED_HUB_RIGHT_RIGHT;
                            break;
                        case RED_HUB_RIGHT_RIGHT:
                            parterTagChecker=RED_HUB_RIGHT_LEFT;
                            break;
                        };
                        for(LimelightHelpers::RawFiducial aprilTag : aprilTagResults){ //Iterate through all visible april tags
                            if(aprilTag.id==parterTagChecker){
                                partnerTag = aprilTag.id;
                                furtherCurrentArea=aprilTag.ta;
                                break;
                            }
                        }
                        
                    }
                    break;
                }
            }
            //return distance;
        }else{
            return -2;
        }
    }else if(frc::DriverStation::GetAlliance()==frc::DriverStation::Alliance::kBlue){
        if(LimelightHelpers::getTargetCount("limelight-b")>0){
            std::vector<LimelightHelpers::RawFiducial> aprilTagResults = getRawFiducials("limelight-b"); //Get data for all visible April Tags from limelight
            frc::SmartDashboard::PutNumber("test",67);
            for(LimelightHelpers::RawFiducial aprilTag : aprilTagResults){ //Iterate through all visible april tags
                if((std::find(std::begin(BLUE_HUB_APRIL_TAGS), std::end(BLUE_APRIL_TAGS), aprilTag.id)) != std::end(BLUE_APRIL_TAGS)){ //check to see if the aprilTag is a RED tag
                    if(aprilTag.ta > closestCurrentArea){
                        closestCurrentArea = aprilTag.ta;
                        ClosestAprilTag_id = aprilTag.id;
                    }

                }
            }
            //Check if a hub tag was found
        for (LimelightHelpers::RawFiducial closestTag : aprilTagResults) {
            if(closestTag.id == LimelightHelpers::INVALID_TARGET){return-2;}
            else{
                switch (closestTag.id){
                case BLUE_HUB_BACK_LEFT:
                    parterTagChecker=BLUE_HUB_BACK_RIGHT;
                    break;
                case BLUE_HUB_BACK_RIGHT:
                    parterTagChecker=BLUE_HUB_BACK_LEFT;
                    break;
                case BLUE_HUB_FRONT_LEFT:
                    parterTagChecker=BLUE_HUB_FRONT_RIGHT;
                    break;
                case BLUE_HUB_FRONT_RIGHT:
                parterTagChecker=BLUE_HUB_FRONT_LEFT;
                    break;
                case BLUE_HUB_LEFT_LEFT:
                parterTagChecker=BLUE_HUB_LEFT_RIGHT;
                    break;
                case BLUE_HUB_LEFT_RIGHT:
                parterTagChecker=BLUE_HUB_LEFT_LEFT;
                    break;
                case BLUE_HUB_RIGHT_LEFT:
                parterTagChecker=BLUE_HUB_RIGHT_RIGHT;
                    break;
                case BLUE_HUB_RIGHT_RIGHT:
                parterTagChecker=BLUE_HUB_RIGHT_LEFT;
                    break;
                };
                for(LimelightHelpers::RawFiducial aprilTag : aprilTagResults){ //Iterate through all visible april tags
                    if(aprilTag.id==parterTagChecker){
                        partnerTag=aprilTag.id;
                        furtherCurrentArea = aprilTag.ta;
                        break;
                    }
                }
            }
        }
        }else{
            return -1;
        }   
              
    }else{
        return -1;
    }  
    distance1=closestCurrentArea/tagSize; //Get distance ratio to determine how far the robot is from the target
    distance2=furtherCurrentArea/tagSize; //Get distance ratio to determine how far the robot is from the target
    double y =acos((abs(pow(distance1,2)+pow(distance2,2)-pow(14,2)))/(2*distance1*distance2));
    distance1=pow((distance1*0.00008),2);
    return y;
}


    //Determine If April Tags Are Visible  DONE


    //Determine If Visible April Tags Hub Tags That Match Your Alliance Color DONE

    //Get Distance From Relevant Tag And Return Value (Return -1 if no relevent target is found)            Needs to be checked

    //Bonus points if you can determine your rotational relationship to the hub by using both the right and left april tags on the structure
