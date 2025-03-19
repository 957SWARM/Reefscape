package frc.robot.commands;

import java.util.List;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.VisionConstants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Drive.DriveSubsystem;

@Logged
public class ReefAlign {

    PIDController xPID = new PIDController(
        VisionConstants.REEF_TOWARD_TRANSLATION_P, 
        VisionConstants.REEF_TOWARD_TRANSLATION_I, 
        VisionConstants.REEF_TOWARD_TRANSLATION_D);
    PIDController yPID = new PIDController(
        VisionConstants.REEF_HORIZONTAL_TRANSLATION_P, 
        VisionConstants.REEF_HORIZONTAL_TRANSLATION_I, 
        VisionConstants.REEF_HORIZONTAL_TRANSLATION_D
    );
    PIDController rotPID = new PIDController(
        VisionConstants.REEF_ROTATION_P, 
        VisionConstants.REEF_ROTATION_I, 
        VisionConstants.REEF_ROTATION_D
    );

    Pose3d currentPose;
    Pose3d nearestReefPose;

    boolean biasLeft;
    boolean biasRight;

    public ReefAlign(){
        xPID.setIZone(.5);
    }
    
    public Command alignNearestReef(DriveSubsystem drive){
        
        return Commands.run(() -> {
            
            biasLeft = false;
            biasRight = false;
            updatePoses();

            if(checkReefTag()){
                drive.drive(getXOutput(), getYOutput(), getRotOutput(), false, 0);
            }
            else{
                drive.drive(0, 0, 0, false, 0);
            }
        }, drive).until(() -> checkAligned(drive))
        .andThen(Commands.runOnce(() -> drive.drive(0, 0, 0, false, 0)));
    }

    public Command alignRightReef(DriveSubsystem drive){

        return Commands.run(() -> {

            biasRight = true;
            biasLeft = false;
            updatePoses();

            if(checkReefTag()){
                drive.drive(getXOutput(), getYOutput(), getRotOutput(), false, 0);
            }
            else{
                drive.drive(0, 0, 0, false, 0);
            }
        }, drive)
        .until(() -> checkAligned(drive))
        .andThen(Commands.runOnce(() -> drive.drive(0, 0, 0, false, 0)));
    }

    public Command alignLeftReef(DriveSubsystem drive){

        return Commands.run(() -> {

            biasLeft = true;
            biasRight = false;
            updatePoses();

            if(checkReefTag()){

                drive.drive(getXOutput(), getYOutput(), getRotOutput(), false, 0);
            }
            else if(StationAlign.tof.getRange()/1000 < .5){
                drive.drive(0, -0.1, 0, false, 0);
            }
            else{
                drive.drive(0, 0, 0, false, 0);
            }
        }, drive)
        .until(() -> checkAligned(drive))
        .andThen(Commands.runOnce(() -> drive.drive(0, 0, 0, false, 0)));
    }

    public void setTagPriority(){
        LimelightHelpers.setPriorityTagID(
            VisionConstants.REEF_LIMELIGHT_NAME, 
            (int)LimelightHelpers.getFiducialID(VisionConstants.REEF_LIMELIGHT_NAME));
    }

    public void removeTagPriority(){
        LimelightHelpers.setPriorityTagID(
            VisionConstants.REEF_LIMELIGHT_NAME, 
            -1);
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
            -0.4, 
            0.4);
        
        return clampedOutput;
    }

    public double getRotOutput(){

        return rotPID.calculate(getRotDiff());
    }

    // public Pose2d getPoseDifferential(){
    //     Pose2d currentPose = LimelightHelpers.getBotPose3d_TargetSpace(VisionConstants.REEF_LIMELIGHT_NAME).toPose2d();

    //     return new Pose2d(
    //         findNearestReef().minus(currentPose).getTranslation(),
    //         findNearestReef().getRotation().minus(currentPose.getRotation())
    //     );
    // }

    public double getXDiff(){
        double robotX = currentPose.getZ();
        double targetX = nearestReefPose.getZ();
        //return -(targetX - robotX);
        return targetX - robotX;
    }

    public double getYDiff(){
        double robotY = currentPose.getX();
        double targetY = nearestReefPose.getX();
        //return -(targetY - robotY);
        return robotY - targetY;
    }

    public double getRotDiff(){
        double robotRot = Units.radiansToDegrees(currentPose.getRotation().getQuaternion().getY());
        double targetRot = Units.radiansToDegrees(nearestReefPose.getRotation().getQuaternion().getY());
        return targetRot - robotRot;
    }

    public void updatePoses(){
        currentPose = getCurrentPose();
        if(biasRight){
            biasLeft = false;
            nearestReefPose = VisionConstants.RIGHT_REEF;
        }
        else if(biasLeft){
            biasRight = false;
            nearestReefPose = VisionConstants.LEFT_REEF;
        }else{
            nearestReefPose = getNearestPose();
        }
        
    }

    public Pose3d getCurrentPose(){
        return LimelightHelpers.getBotPose3d_TargetSpace(VisionConstants.REEF_LIMELIGHT_NAME);
    }

    public Pose3d getNearestPose(){
        return findNearestPose(currentPose, VisionConstants.REEF_POSES);

    }

    //MINIMIZES TARGET SPACE X: LEFT/RIGHT
    public Pose3d findNearestPose(Pose3d current, List<Pose3d> poses){
        Pose3d nearestPose = null;

        if(currentPose.getX() > VisionConstants.REEF_CENTER_OFFSET){
            //System.out.println("RIGHTT");
            nearestPose = VisionConstants.RIGHT_REEF;
        }else{
            //System.out.println("LEFTT");
            nearestPose = VisionConstants.LEFT_REEF;
        }
    
        return nearestPose;
    }

    public boolean checkReefTag(){
        boolean valid = false;
        if(
            LimelightHelpers.getTV(VisionConstants.REEF_LIMELIGHT_NAME)
            && VisionConstants.REEF_TAG_IDS.contains(LimelightHelpers.getFiducialID(VisionConstants.REEF_LIMELIGHT_NAME))
            && Math.abs(currentPose.getZ()) < VisionConstants.MAX_TRACKING_DISTANCE
            ){
            valid = true;
        }
        return valid;
    }

    public boolean checkAligned(DriveSubsystem drive){
        boolean aligned = Math.abs(getXDiff()) <= VisionConstants.REEF_TRANSLATION_TOLERANCE
        && Math.abs(getYDiff()) <= VisionConstants.REEF_TRANSLATION_TOLERANCE
        && Math.abs(getRotDiff()) <= VisionConstants.ROTATION_TOLERANCE
        && drive.getLinearSpeed() <= VisionConstants.SPEED_TOLERANCE;

        return aligned;
    }
 
    public boolean xCondition(){
        return Math.abs(getXDiff()) <= VisionConstants.REEF_TRANSLATION_TOLERANCE;
    }

    public boolean yCondition(){
        return Math.abs(getYDiff()) <= VisionConstants.REEF_TRANSLATION_TOLERANCE;
    }

    public boolean rotCondition(){
        return Math.abs(getRotDiff()) <= VisionConstants.ROTATION_TOLERANCE;
    }

    public void periodic(){
        updatePoses();
    }

    //DEBUGGING LOGGING FUNCTIONS

    public String reefPose(){
        return nearestReefPose.toString();
    }

    public String robotPose(){
        return currentPose.toString();
    }

}
