package frc.robot.input;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.IOConstants;

public class OperatorInput {

    XboxController controller = new XboxController(IOConstants.OPERATOR_PORT);

    public boolean fall(){
        return controller.getPOV() == 270;
    }

    public boolean deployClimb(){
        return controller.getPOV() == 0;
    }

    public boolean retractClimb(){
        return controller.getPOV() == 180;
    }

}
