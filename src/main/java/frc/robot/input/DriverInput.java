package frc.robot.input;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.IOConstants;

public class DriverInput {

    XboxController controller = new XboxController(IOConstants.DRIVER_PORT);

    // input cubed to improve fine movement at slow speeds
    public double driveX(){
        return controller.getLeftY();
    }

    //input cubed to improve fine movement at slow speeds
    public double driveY(){
        return controller.getLeftX();
    }

    //input cubed to improve fine movement at slow speeds
    public double driveTurn(){
        return controller.getRightX();
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
