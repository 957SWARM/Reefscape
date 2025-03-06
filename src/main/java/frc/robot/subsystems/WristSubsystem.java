package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.reduxrobotics.sensors.canandmag.Canandmag;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.WristConstants;

@Logged
public class WristSubsystem extends SubsystemBase{

    // Hardware
    private TalonFX motor = new TalonFX(WristConstants.MOTOR_CAN_ID);
    private Canandmag encoder = new Canandmag(WristConstants.ENCODER_CAN_ID);
    
    private PIDController pid = new PIDController(
        WristConstants.kP, 
        WristConstants.kI, 
        WristConstants.kD
    );
    private double targetSetpoint = WristConstants.STOW_ANGLE;
    private double output;

    public WristSubsystem(){ 
        // Current Limit
        var limitConfigs = new CurrentLimitsConfigs();
        limitConfigs.SupplyCurrentLimit = WristConstants.CURRENT_LIMIT;
        motor.getConfigurator().apply(limitConfigs);
        output = 0;
    }

    public void periodic(){
        double modifiedSetpoint = targetSetpoint;
        modifiedSetpoint = (modifiedSetpoint >= 0.5) ? modifiedSetpoint - 1: targetSetpoint;
        pid.setSetpoint(modifiedSetpoint);

        // calculations of feedback and feedforward
        double feedback = pid.calculate(getModifiedSetpoint(getPosition()));  //pid feedback on current position
        double feedforward = getFeedForward();  // feedforwad that counteracts gravity

        // clamping of output to minimum/maximum voltage
        output = MathUtil.clamp(
            feedback + feedforward,
            WristConstants.MINIMUM_VOLTAGE,
            WristConstants.MAXIMUM_VOLTAGE
        );

        motor.setVoltage(-output);
    }

    // angle measured in rotations
    public Command setSetpoint(double setpoint) {
        return runOnce(
                () -> {
                    targetSetpoint = setpoint;
                });
    }

    public double getTargetSetpoint(){
        return targetSetpoint;
    }

    public Command toL1(){
        return setSetpoint(WristConstants.L1_ANGLE);
    }

    public Command toL2(){
        return setSetpoint(WristConstants.L2_ANGLE);
    }

    public Command toL3(){
        return setSetpoint(WristConstants.L3_ANGLE);
    }

    public Command toL4(){
        return setSetpoint(WristConstants.L4_ANGLE);
    }

    public Command toIntake(){
        return setSetpoint(WristConstants.INTAKE_ANGLE);
    }

    public Command toStow(){
        return setSetpoint(WristConstants.STOW_ANGLE);
    }

    // gets encoder position
    public double getPosition(){
        return encoder.getAbsPosition();
    }

    // purely for checking if at setpoint
    public boolean atSetpoint(){
        return Math.abs(getPosition() - targetSetpoint) < WristConstants.TOLERANCE;
    }
    
    // Helper function. feedforward = voltage to remain at current position
    private double getFeedForward(){
        // angle measured between straight down and current position
        // converted to radians for sine function
        double angle = Units.rotationsToRadians(encoder.getAbsPosition());
        return Math.cos(angle) * WristConstants.kG;
    }

    public double getModifiedSetpoint(double setpoint){
        if (setpoint >= 0.5) 
            setpoint = setpoint - 1;
        return setpoint;
    }

    public double getAppliedVoltage(){
        return motor.getMotorVoltage().getValueAsDouble();
    }

    public double getCommandedVoltage(){
        return output;
    }
}
