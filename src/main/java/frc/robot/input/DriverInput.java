package frc.robot.input;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.IOConstants;

public class DriverInput {

    XboxController controller = new XboxController(IOConstants.DRIVER_PORT);

    // Slew Rate Limiters. limit how fast joystick values can change
    SlewRateLimiter xLimiter = new SlewRateLimiter(20);
    SlewRateLimiter yLimiter = new SlewRateLimiter(20);
    SlewRateLimiter turnLimiter = new SlewRateLimiter(20);

    Debouncer removalDebouncer = new Debouncer(IOConstants.DEBOUNCE_TIME);

    // input squared to improve fine movement at slow speeds
    public double driveX(){
        return xLimiter.calculate(
            Math.signum(controller.getLeftY()) * Math.pow(controller.getLeftY(), 2)
        );
    }

    //input squared to improve fine movement at slow speeds
    public double driveY(){
        return yLimiter.calculate(
            Math.signum(controller.getLeftX()) * Math.pow(controller.getLeftX(), 2)
        );
    }

    public double driveTurn(){
        return turnLimiter.calculate(controller.getRightX());
    }

    public boolean resetGyro(){
        return controller.getRawButton(IOConstants.LEFT_CENTER_BUTTON);
    }

    public boolean L1(){
        return controller.getAButton();
    }

    public boolean L2(){
        return controller.getXButton();
    }

    public boolean L3(){
        return controller.getYButton();
    }

    public boolean L4(){
        return controller.getBButton();
    }

    public boolean stow(){
        return controller.getLeftBumperButton();
    }

    public boolean score(){
        return controller.getRightTriggerAxis() > IOConstants.TRIGGER_THRESHOLD;
    }

    public boolean intake(){
        return controller.getLeftTriggerAxis() > IOConstants.TRIGGER_THRESHOLD;
    }

    public boolean deployClimb(){
        return controller.getPOV() == 0;
    }

    public boolean retractClimb(){
        return controller.getPOV() == 180;
    }

    public void setRumble(boolean on) {
        controller.setRumble(RumbleType.kBothRumble, on ? 1 : 0);
    }

    public boolean visionAlign(){
        return controller.getRightBumperButton();
    }

    public boolean lowRemove(){
        return removalDebouncer.calculate(controller.getPOV() == 270);
    }

    public boolean highRemove(){
        return controller.getPOV() == 90;
    }


}
