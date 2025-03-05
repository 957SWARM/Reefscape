package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

@Logged
public class ClimbSubsystem extends SubsystemBase{

    private SparkMax neo;
    private double appliedVoltage;
    public final SparkMaxConfig motorConfig = new SparkMaxConfig();
    private final RelativeEncoder encoder;
    private DigitalInput limitSwitch = new DigitalInput(1);

    public ClimbSubsystem(){
        neo = new SparkMax(ClimberConstants.MOTOR_CAN_ID, MotorType.kBrushless);

        // Current Limit
        motorConfig.smartCurrentLimit(ClimberConstants.CURRENT_LIMIT);
        motorConfig.idleMode(IdleMode.kBrake);
        neo.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        encoder = neo.getEncoder();
        

        appliedVoltage = 0;
    }

    public void periodic(){
        if(appliedVoltage <= 0){    // if extending
            if(getPosition() > ClimberConstants.MAX_EXTENSION){
                neo.setVoltage(appliedVoltage);
            }else{
                neo.setVoltage(0);
            }
        }else if(appliedVoltage >= 0){  // if retracting
            if(!limitSwitch.get() && getPosition() < ClimberConstants.MAX_RETRACTION){
                neo.setVoltage(appliedVoltage);
            }else{
                neo.setVoltage(0);
            }
        }
    }

    public Command stopCommand(){
        return Commands.runOnce(() -> {
            appliedVoltage = 0;
        });
    }
    
    public Command extend(){
        return Commands.runOnce(() -> {
            appliedVoltage = -ClimberConstants.CLIMB_PULL_STRENGTH_VOLTS;
        });
    }

    public Command retract(){
        return Commands.runOnce(() -> {
            appliedVoltage = ClimberConstants.CLIMB_PULL_STRENGTH_VOLTS;
        });
    } 

    public double getPosition(){
        return encoder.getPosition();
    }

    public boolean getLimitSwitch(){
        return limitSwitch.get();
    }

    public double getAppliedVoltage(){
        return neo.getAppliedOutput();
    }

    public double getCommandedVoltage(){
        return appliedVoltage;
    }
}
