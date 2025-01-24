package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase{
    TalonFX kraken;
    final MotionMagicVoltage request;
    double targetSetpoint;

    public ElevatorSubsystem(){
        kraken = new TalonFX(ElevatorConstants.MOTOR_ID);
        
        TalonFXConfiguration configs = new TalonFXConfiguration();

        Slot0Configs slot0 = configs.Slot0;
        slot0.kS = ElevatorConstants.kS;
        slot0.kV = ElevatorConstants.kV; 
        slot0.kA = ElevatorConstants.kA;
        slot0.kP = ElevatorConstants.kP; 
        slot0.kI = ElevatorConstants.kI; 
        slot0.kD = ElevatorConstants.kD;

        MotionMagicConfigs mmConfigs = configs.MotionMagic;
        mmConfigs.MotionMagicCruiseVelocity = 80; // Target cruise velocity of 80 rps
        mmConfigs.MotionMagicAcceleration = 160; // Target acceleration of 160 rps/s (0.5 seconds)
        mmConfigs.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)

        kraken.getConfigurator().apply(configs);

        request = new MotionMagicVoltage(ElevatorConstants.kG);
    }

    public void periodic(){
        kraken.setControl(request.withPosition(getAsRotations(targetSetpoint)));
    }

    public static double getAsRotations(double meters){
        double rotations = meters * ElevatorConstants.metersToRotations;
        return rotations;
    }

    private void assignSetpoint(double assignSetpoint){
        // make sure setpoint is within safe range
        targetSetpoint = MathUtil.clamp(
            assignSetpoint, 
            ElevatorConstants.MIN_HEIGHT, 
            ElevatorConstants.MAX_HEIGHT
        );
    }

    public Command toL1(){
        return Commands.runOnce(() -> {
            assignSetpoint(ElevatorConstants.POSITION_L1);
        });
    }
    
    public Command toL2(){
        return Commands.runOnce(() -> {
            assignSetpoint(ElevatorConstants.POSITION_L2);
        });
    }

    public Command toL3(){
        return Commands.runOnce(() -> {
            assignSetpoint(ElevatorConstants.POSITION_L3);
        });
    }

    public Command toL4(){
        return Commands.runOnce(() -> {
            assignSetpoint(ElevatorConstants.POSITION_L4);
        });
    }

    public Command toIntake(){
        return Commands.runOnce(() -> {
            assignSetpoint(ElevatorConstants.POSITION_INTAKE);
        });
    }

    public Command slowRise(){
        return Commands.run(() -> {
            assignSetpoint(targetSetpoint += ElevatorConstants.SETPOINT_INCREMENT);
        });
    }

    public Command slowFall(){
        return Commands.run(() -> {
            assignSetpoint(targetSetpoint -= ElevatorConstants.SETPOINT_INCREMENT);
        });
    }

}
