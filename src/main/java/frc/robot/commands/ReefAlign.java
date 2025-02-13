package frc.robot.commands;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.VisionConstants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Drive.DriveSubsystem;

@Logged
public class ReefAlign {

    PIDController xPID = new PIDController(
        VisionConstants.TRANSLATION_P, 
        VisionConstants.TRANSLATION_I, 
        VisionConstants.TRANSLATION_D);
    PIDController yPID = new PIDController(
        VisionConstants.TRANSLATION_P, 
        VisionConstants.TRANSLATION_I, 
        VisionConstants.TRANSLATION_D
    );
    PIDController rotPID = new PIDController(
        VisionConstants.ROTATION_P, 
        VisionConstants.ROTATION_I, 
        VisionConstants.ROTATION_D
    );

    public ReefAlign(){}
    
    public Command alignNearestReef(DriveSubsystem drive){

        return Commands.run(() -> {
            drive.drive(getXOutput(), getYOutput(), getRotOutput(), false, 0);
        }, drive);
    }

    public double getXOutput(){
        return xPID.calculate(getPoseDifferential().getX());
    }

    public double getYOutput(){
        return yPID.calculate(getPoseDifferential().getY());
    }

    public double getRotOutput(){
        return rotPID.calculate(getPoseDifferential().getRotation().getDegrees());
    }

    public Pose2d getPoseDifferential(){
        Pose2d currentPose = LimelightHelpers.getBotPose3d_TargetSpace(VisionConstants.REEF_LIMELIGHT_NAME).toPose2d();

        return new Pose2d(
            findNearestReef().minus(currentPose).getTranslation(),
            findNearestReef().getRotation().minus(currentPose.getRotation())
        );


    }

    public Pose2d findNearestReef(){
        Pose2d currentPose = LimelightHelpers.getBotPose3d_TargetSpace(VisionConstants.REEF_LIMELIGHT_NAME).toPose2d();

        return currentPose.nearest(VisionConstants.REEF_POSES);
    }

    public boolean checkReefTag(){
        boolean valid = false;
        if(LimelightHelpers.getTV(VisionConstants.REEF_LIMELIGHT_NAME)){
            valid = true;
        }
        return valid;
    }

}
