package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Drive.DriveSubsystem;

public class ReefAlign {

    PIDController xPID = new PIDController(0, 0, 0);
    PIDController yPID = new PIDController(0, 0, 0);
    PIDController rotPID = new PIDController(0, 0, 0);
    
    public static Command alignNearestReef(DriveSubsystem drive){



        return Commands.run(() -> {
            drive.drive(0, 0, 0, false, 0);
        }, drive);
    }

    public Pose2d findNearestReef(){
        return new Pose2d();
    }

}
