package frc.robot.input;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.IOConstants;

public class OperatorInput {

    XboxController controller = new XboxController(IOConstants.OPERATOR_PORT);

    public boolean stopClimb(){
        return controller.getXButton();
    }
    
    public boolean deployClimb(){
        return controller.getYButton();
    }

    public boolean retractClimb(){
        return controller.getAButton();
    }

}
