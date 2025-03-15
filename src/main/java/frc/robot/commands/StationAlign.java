package frc.robot.commands;

import java.util.List;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.VisionConstants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Drive.DriveSubsystem;

@Logged
public class StationAlign {

    PIDController xPID = new PIDController(
        VisionConstants.STATION_TRANSLATION_P, 
        VisionConstants.STATION_TRANSLATION_I, 
        VisionConstants.STATION_TRANSLATION_D);
    PIDController yPID = new PIDController(
        VisionConstants.STATION_TRANSLATION_P, 
        VisionConstants.STATION_TRANSLATION_I, 
        VisionConstants.STATION_TRANSLATION_D
    );
    PIDController rotPID = new PIDController(
        VisionConstants.STATION_ROTATION_P, 
        VisionConstants.STATION_ROTATION_I, 
        VisionConstants.STATION_ROTATION_D
    );

    PIDController dumbRotPID = new PIDController(
        VisionConstants.DUMB_STATION_ROTATION_P, 
        VisionConstants.DUMB_STATION_ROTATION_I, 
        VisionConstants.DUMB_STATION_ROTATION_D
    );

    PIDController dumbXPID = new PIDController(
        VisionConstants.DUMB_STATION_TRANSLATION_P, 
        VisionConstants.DUMB_STATION_TRANSLATION_I, 
        VisionConstants.DUMB_STATION_TRANSLATION_D);

    Pose3d currentPose;
    Pose3d nearestStationPose;

    Timer alignTimer = new Timer();
    boolean timerReset = false;

    boolean biasCenter = false;
    TimeOfFlight tof = new TimeOfFlight(2);

    double shuffleOutput = 0;

    public StationAlign(){
        tof.setRangingMode(RangingMode.Long, 24);
    }
    
    public Command alignNearestStation(DriveSubsystem drive){

        biasCenter = false;
        updatePoses();

        return Commands.run(() -> {
            if(checkStationTag()){
                drive.drive(getXOutput(), getYOutput(), getRotOutput(), false, 0);
            }
            else{
                drive.drive(0, 0, 0, false, 0);
            }
        }, drive).until(() -> checkAligned(drive));
    }

    public Command alignCenterStation(DriveSubsystem drive){

        biasCenter = true;
        updatePoses();

        return Commands.run(() -> {
            if(checkStationTag()){
                drive.drive(getXOutput(), getYOutput(), getRotOutput(), false, 0);
            }
            else{
                drive.drive(0, 0, 0, false, 0);
            }
        }, drive).until(() -> checkAligned(drive));
    }

    public Command dumbStationAlign(DriveSubsystem drive){
        return Commands.run(() -> {
            drive.drive(getDumbXOutput(), 0, getDumbRotOutput(drive), false, 0);
        }).until(() -> dumbCheckAligned(drive)).andThen(Commands.run(() -> {
            drive.drive(0, shuffleYOutput(drive), 0, false, 0);
        }));
    }

    public double getDumbXDiff(){
        return tof.getRange()/1000 - 0.33;
    }

    public double getDumbRotDiff(DriveSubsystem drive){
        double difference = 0;
        double modifiedHeading = MathUtil.inputModulus(drive.getHeading(), -180, 180);
        if(modifiedHeading < 0){
            difference = -120 - modifiedHeading;
        }else{
            difference = 120 - modifiedHeading;
        }
        return difference;
    }

    public double getDumbXOutput(){

        double clampedOutput = MathUtil.clamp(
            -xPID.calculate(getDumbXDiff()), 
            -.3, 
            .3);
        
        return clampedOutput;
    }

    public double getDumbRotOutput(DriveSubsystem drive){

        return -dumbRotPID.calculate(getDumbRotDiff(drive));
    }

    public double shuffleRotOutput(DriveSubsystem drive){
        double output = getDumbRotOutput(drive) * 17;
        return MathUtil.clamp(output, -0.2, .2);
    }

    public double shuffleYOutput(DriveSubsystem drive){
        shuffleOutput += .01;
        shuffleOutput = MathUtil.inputModulus(shuffleOutput, -0.1, 0.1);
        return MathUtil.clamp(shuffleOutput, -0.1, 0.1);
    }

    //FORWARD: 
    //ROBOT SPACE: +X 
    //LL TARGET SPACE: +Z
    public double getXOutput(){

        double clampedOutput = MathUtil.clamp(
            -xPID.calculate(getXDiff()), 
            -VisionConstants.MAX_VISION_SPEED, 
            VisionConstants.MAX_VISION_SPEED);

        return clampedOutput;
    }

    //LEFT:
    //ROBOT SPACE: +Y
    //LL TARGET SPACE: +X
    public double getYOutput(){

        double clampedOutput = MathUtil.clamp(
            -yPID.calculate(getYDiff()), 
            -VisionConstants.MAX_VISION_SPEED, 
            VisionConstants.MAX_VISION_SPEED);
        
        return clampedOutput;
    }

    public double getRotOutput(){

        return rotPID.calculate(getRotDiff());
    }

    public double getXDiff(){
        
        double robotX = currentPose.getZ();
        double targetX = nearestStationPose.getZ();
        //return -(targetX - robotX);
        return targetX - robotX;
    }

    public double getYDiff(){
        double robotY = currentPose.getX();
        double targetY = nearestStationPose.getX();
        //return -(targetY - robotY);
        return robotY - targetY;
    }

    public double getRotDiff(){
        double robotRot = Units.radiansToDegrees(currentPose.getRotation().getQuaternion().getY());
        double targetRot = Units.radiansToDegrees(nearestStationPose.getRotation().getQuaternion().getY());
        return targetRot - robotRot;
    }

    public void updatePoses(){
        // if(!LimelightHelpers.getTV(VisionConstants.STATION_LIMELIGHT_NAME) && !timerReset){
        //     timerReset = true;
        //     alignTimer.reset();
        // }
        // if((alignTimer.get() >= VisionConstants.STATION_DROP_TOLERANCE && timerReset) || LimelightHelpers.getTV(VisionConstants.STATION_LIMELIGHT_NAME)){
        //     timerReset = false;
        //     currentPose = getCurrentPose();
        //     nearestStationPose = getNearestPose();
        // }
        // if (currentPose == null || nearestStationPose == null){
        //     currentPose = new Pose3d();
        //     nearestStationPose = new Pose3d();
        // }
        currentPose = getCurrentPose();
        if(biasCenter){
            nearestStationPose = VisionConstants.MID_STATION;
        }else{
            nearestStationPose = getNearestPose();
        }
        
    }

    public Pose3d getCurrentPose(){
        return LimelightHelpers.getBotPose3d_TargetSpace(VisionConstants.STATION_LIMELIGHT_NAME);
    }

    public Pose3d getNearestPose(){
        return findNearestPose(currentPose, VisionConstants.STATION_POSES);

    }

    //MINIMIZES TARGET SPACE X: LEFT/RIGHT
    public Pose3d findNearestPose(Pose3d current, List<Pose3d> poses){
        Pose3d nearestPose = null;
        double minDistance = Double.MAX_VALUE;

        for (Pose3d pose : poses){
            double distance = pose.getX()-currentPose.getX();

            if (Math.abs(distance) < Math.abs(minDistance)){
                minDistance = distance;
                nearestPose = pose;
            }
        }

        return nearestPose;
    }

    public boolean checkStationTag(){
        boolean valid = false;
        if(
            LimelightHelpers.getTV(VisionConstants.STATION_LIMELIGHT_NAME)
            && VisionConstants.STATION_TAG_IDS.contains(LimelightHelpers.getFiducialID(VisionConstants.STATION_LIMELIGHT_NAME))
            ){
            valid = true;
        }
        return valid;
    }

    // end condition for command. Finishes when close to target position and speed is low
    public boolean checkAligned(DriveSubsystem drive){
        boolean aligned = Math.abs(getXDiff()) <= VisionConstants.STATION_TRANSLATION_TOLERANCE
        && Math.abs(getYDiff()) <= VisionConstants.STATION_TRANSLATION_TOLERANCE
        && Math.abs(getRotDiff()) <= VisionConstants.ROTATION_TOLERANCE
        && drive.getLinearSpeed() <= VisionConstants.SPEED_TOLERANCE;

        return aligned;
    }

    public boolean dumbCheckAligned(DriveSubsystem drive){
        boolean aligned = Math.abs(getDumbXDiff()) <= VisionConstants.STATION_TRANSLATION_TOLERANCE
        && Math.abs(getDumbRotDiff(drive)) <= VisionConstants.ROTATION_TOLERANCE;

        return aligned;
    }

    public void periodic(){
        updatePoses();
    }

    //DEBUGGING LOGGING FUNCTIONS

    public String stationPose(){
        return nearestStationPose.toString();
    }

    public String robotPose(){
        return currentPose.toString();
    }

}
