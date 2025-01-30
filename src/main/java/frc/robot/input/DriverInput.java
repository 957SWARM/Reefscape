package frc.robot.input;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.IOConstants;

public class DriverInput {

    XboxController controller = new XboxController(IOConstants.DRIVER_PORT);

    // input cubed to improve fine movement at slow speeds
    public double driveX(){
        return Math.pow(controller.getLeftY(), 3);
    }

    //input cubed to improve fine movement at slow speeds
    public double driveY(){
        return Math.pow(controller.getLeftX(), 3);
    }

    //input cubed to improve fine movement at slow speeds
    public double driveTurn(){
        return Math.pow(controller.getRightX(), 3);
    }

    public boolean resetGyro(){
        return controller.getRawButton(IOConstants.LEFT_CENTER_BUTTON);
    }

    public boolean L1(){
        return controller.getAButton();
    }

    public boolean L2(){
        return controller.getBButton();
    }

    public boolean L3(){
        return controller.getXButton();
    }

    public boolean L4(){
        return controller.getYButton();
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

    public boolean slowRise(){
        return controller.getPOV() == 0;
    }

    public boolean slowFall(){
        return controller.getPOV() == 180;
    }

}
