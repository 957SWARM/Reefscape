package frc.robot.subsystems;

import com.reduxrobotics.sensors.canandmag.Canandmag;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.WristConstants;

@Logged
public class WristSubsystem extends SubsystemBase{

    SparkMax motor = new SparkMax(WristConstants.MOTOR_CAN_ID, MotorType.kBrushless);
    Canandmag encoder = new Canandmag(WristConstants.ENCODER_CAN_ID);
    PIDController pid = new PIDController(
        WristConstants.kP, 
        WristConstants.kI, 
        WristConstants.kD
    );

    public WristSubsystem(){
        // assumes wrist starts out straight up. Zero point is defined as straight down
        encoder.setAbsPosition(.5);
    }

    // angle measured in rotations
    public Command goToSetpoint(double setpoint) {
        return run(
                () -> {
                    // makes sure angle is within a reasonable range
                    double safeSetpoint = MathUtil.clamp(
                        setpoint,
                        WristConstants.MINIMUM_ANGLE,
                        WristConstants.MAXIMUM_ANGLE
                    );

                    pid.setSetpoint(safeSetpoint);

                    // calculations of feedback and feedforward
                    double feedback = pid.calculate(getPosition());  //pid feedback on current position
                    double feedforward = getFeedForward();  // feedforwad that counteracts gravity

                    // clamping of output to minimum/maximum voltage
                    double output = MathUtil.clamp(
                        feedback + feedforward,
                        WristConstants.MINIMUM_VOLTAGE,
                        WristConstants.MAXIMUM_VOLTAGE
                    );

                    motor.setVoltage(output);
                });
    }

    public Command toL1(){
        return goToSetpoint(WristConstants.L1_ANGLE);
    }

    public Command toL2(){
        return goToSetpoint(WristConstants.L2_ANGLE);
    }

    public Command toL3(){
        return goToSetpoint(WristConstants.L3_ANGLE);
    }

    public Command toL4(){
        return goToSetpoint(WristConstants.L4_ANGLE);
    }

    public Command toIntake(){
        return goToSetpoint(WristConstants.INTAKE_ANGLE);
    }

    public Command toStow(){
        return goToSetpoint(WristConstants.STOW_ANGLE);
    }

    // gets encoder position
    public double getPosition(){
        return encoder.getAbsPosition();
    }

    // Helper function. feedforward = voltage to remain at current position
    private double getFeedForward(){
        // angle measured between straight down and current position
        // converted to radians for sine function
        double angle = Units.rotationsToRadians(encoder.getAbsPosition());
        return Math.sin(angle) * WristConstants.kG;
    }

}
