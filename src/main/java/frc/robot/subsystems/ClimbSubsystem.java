package frc.robot.subsystems;

import com.reduxrobotics.sensors.canandmag.Canandmag;
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
import frc.robot.Constants.WristConstants;

@Logged
public class ClimbSubsystem extends SubsystemBase{

    private SparkMax neo;
    public final SparkMaxConfig motorConfig = new SparkMaxConfig();
    private DigitalInput limitSwitch = new DigitalInput(1);
    private ClimbState currentState = ClimbState.STOP;
    private Canandmag encoder = new Canandmag(ClimberConstants.CLIMB_ENCODER_CAN_ID);

    public ClimbSubsystem(){
        neo = new SparkMax(ClimberConstants.MOTOR_CAN_ID, MotorType.kBrushless);

        // Current Limit
        motorConfig.smartCurrentLimit(ClimberConstants.CURRENT_LIMIT);
        motorConfig.idleMode(IdleMode.kBrake);
        neo.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);  
    }

    public void periodic(){

        double angle = encoder.getAbsPosition();
        double modifiedAngle = (angle > 0.5)? angle - 1 : angle;
        // climber angle set to range (0.5, -0.5) (retracted, extended)

        switch(currentState){
            case CLIMBING:
                if(modifiedAngle < 0.25 || limitSwitch.get()){
                    neo.setVoltage(ClimberConstants.CLIMB_PULL_STRENGTH_VOLTS);
                } else {
                    neo.setVoltage(0);
                }
                break;
            case LATCHING:
                if(modifiedAngle > -0.01){
                    neo.setVoltage(-ClimberConstants.CLIMB_PULL_STRENGTH_VOLTS);
                } else {
                    neo.setVoltage(0);
                }
                break;
            case STOP:
            default:
                neo.setVoltage(0);
                break;
            
        }

    }
    
    public Command setState(ClimbState targetState){
        return Commands.runOnce(() -> {
            currentState = targetState;
        });
    }

    public Command stopClimber(){
        return setState(ClimbState.STOP);
    }

    public Command retractClimber(){
        return setState(ClimbState.CLIMBING);
    }

    public Command latchClimber(){
        return setState(ClimbState.LATCHING);
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

    public enum ClimbState {
        LATCHING, CLIMBING, STOP
    }
}
