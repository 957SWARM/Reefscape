package frc.robot.subsystems;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.IntakeConstants;

public class ClimbSubsystem extends SubsystemBase{

    SparkMax neo;
    double appliedVoltage;

    public ClimbSubsystem(){
        neo = new SparkMax(ClimberConstants.MOTOR_CAN_ID, MotorType.kBrushless);
        appliedVoltage = 0;
    }

    public void periodic(){
        neo.setVoltage(appliedVoltage);
    }

    public Command extend(){
        return Commands.runOnce(() -> {
            appliedVoltage = ClimberConstants.CLIMB_PULL_STRENGTH_VOLTS;
        });
    }

    public Command retract(){
        return Commands.runOnce(() -> {
            appliedVoltage = -ClimberConstants.CLIMB_PULL_STRENGTH_VOLTS;
        });
    }    
}
