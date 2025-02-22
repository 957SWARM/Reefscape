package frc.robot.commands;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Drive.DriveSubsystem;

@Logged
public class StationAlignTOF {

    TimeOfFlight stationTOF = new TimeOfFlight(0);

    PIDController xPID = new PIDController(
        VisionConstants.TRANSLATION_P, 
        VisionConstants.TRANSLATION_I, 
        VisionConstants.TRANSLATION_D);

    PIDController rotPID = new PIDController(
        VisionConstants.ROTATION_P, 
        VisionConstants.ROTATION_I, 
        VisionConstants.ROTATION_D
    );

    public StationAlignTOF(){
        stationTOF.setRangingMode(RangingMode.Short, 20);
        xPID.setSetpoint(VisionConstants.STATION_DISTANCE);
    }
    
    public Command alignStation(DriveSubsystem drive){
        return Commands.run(() -> {
            drive.drive(getXOutput(), 0, 0, false, 0);
        }, drive).until(this::checkAligned);
    }

    //FORWARD: 
    //ROBOT SPACE: +X 
    //LL TARGET SPACE: +Z
    public double getXOutput(){

        double clampedOutput = MathUtil.clamp(
            xPID.calculate(getXDiff()), 
            -VisionConstants.MAX_VISION_SPEED, 
            VisionConstants.MAX_VISION_SPEED);

        return clampedOutput;
    }

    public double getRotOutput(){
        return rotPID.calculate(getRotDiff());
    }

    public double getXDiff(){
        // converts millimeters to meters, then gets horizontal distance
        return (stationTOF.getRange() / 1000) * Math.cos(Units.degreesToRadians(30));
    }

    public double getRotDiff(){
        return 0;
    }

    public boolean checkAligned(){
        return Math.abs(getXDiff() - VisionConstants.STATION_DISTANCE) <= .01;
    }
}
