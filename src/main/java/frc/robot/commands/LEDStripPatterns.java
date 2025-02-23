package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.LEDConstants;
import frc.robot.subsystems.LED;

public class LEDStripPatterns {

    LED led;
    int frame;
    Timer timer;

    public LEDStripPatterns() {
        led = new LED(LEDConstants.TOTAL_PIXELS);
        frame = 0;
        timer = new Timer();
        
        timer.restart();
    }

    public void scheduleDefaultCommand(Command command) {
        CommandScheduler.getInstance().setDefaultCommand(led, command);
    }

    public Command getBlankPatternCommand(int start, int length) {
        return Commands.run(
                () -> {
                    for (int i = start; i < start + length; i++) {
                        led.setPixel(i - 1, 0, 0, 0);
                    }
                },
                led);
    }

    public Command constantColorAnimation(int start, int length, int r, int g, int b) {
        return Commands.run(
                () -> {
                    // Sets LEDs to a specific color constantly
                    for (int i = start; i < start + length; i++) {
                        led.setPixel(i - 1, r, g, b);
                    }
                },
                led);
    }

    public Command flashingColorCommand(int start, int length, int r, int g, int b){
        return Commands.run(
                () -> {
                    // Sets LEDs to flash with the RSL light
                    boolean rslState = RobotController.getRSLState();
                    if (rslState) {
                        for (int i = start; i < start + length; i++) {
                            led.setPixel(i - 1, r, g, b);
                         }
                        } else {
                            for (int i = start; i < start + length; i++) {
                                led.setPixel(i - 1, 0, 0, 0);
                            }
                        }
                },
                led);
    }

    public Command chasingPatternAnimation(int start, int length, double frameTime, boolean isInverted, int r, int g, int b) {
        return Commands.run(
                () -> {
                    int currentFrame = frame;
                    if (isInverted) currentFrame = 2 - currentFrame;

                    for (int i = start; i < start + length; i++) {
                        if ((i - 1) % 3 == currentFrame) {
                            led.setPixel(i, r, g, b);
                        } else {
                            led.setPixel(i, 0, 0, 0);
                        }
                    }

                    if (frameTime <= timer.get()) {
                        timer.reset();
                        if (frame == 2) {
                            frame = 0;
                        } else {
                            frame++;
                        }
                    }
                },
                led);
    }

    // TODO: Create new command with single chasing pixel

    public Command chasingAlernatingColorAnimation(
        int start, int length, 
        int r1, int g1, int b1, int r2, int g2, int b2, double frameTime, boolean isInverted) {
        return Commands.run(
                () -> {
                    int currentFrame = frame;
                    if (isInverted) currentFrame = 2 - currentFrame;

                    for (int i = start; i < start + length; i++) {
                        if ((i - 1) % 3 == currentFrame) {
                            led.setPixel(i, r1, g1, b1);
                        } else {
                            led.setPixel(i, r2, g2, b2);
                        }
                    }

                    if (frameTime <= timer.get()) {
                        timer.reset();
                        if (frame == 2) {
                            frame = 0;
                        } else {
                            frame++;
                        }
                    }
                },
                led);
    }

    public Command chasingBlocksAnimation( int start, int length, double frameTime, boolean isInverted, int r, int g, int b) {
        return Commands.run(
                () -> {
                    int currentFrame = frame;
                    if (isInverted) currentFrame = 2 - currentFrame;

                    for (int i = start; i < start + length; i++) {
                        if ((i - 1) % 2 == currentFrame || (i - 1) % 3 == currentFrame) {
                            led.setPixel(i, r, g, b);
                        } else {
                            led.setPixel(i, 0, 0, 0);
                        }
                    }

                    if (frameTime <= timer.get()) {
                        timer.reset();
                        if (frame == 2) {
                            frame = 0;
                        } else {
                            frame++;
                        }
                    }
                },
                led);
    }

    public Command chasingChainAnimation(int start, int length, double frameTime, boolean isInverted, int r, int g, int b) {
        return Commands.run(
                () -> {
                    int currentFrame = frame;
                    if (isInverted) currentFrame = 2 - currentFrame;

                    boolean on = (currentFrame == 0);

                    for (int i = start; i < start + length; i += 2) {

                        if (on) {
                            led.setPixel(i, r, g, b);
                            if (i + 1 < start + length) {
                                led.setPixel(i + 1, r, g, b);
                            }
                        } else {
                            led.setPixel(i, 255, 255, 255);
                            if (i + 1 < start + length) {
                                led.setPixel(i + 1, 255, 255, 255);
                            }
                        }

                        on = !on;
                    }

                    if (frameTime <= timer.get()) {
                        timer.reset();
                        if (frame > 0) {
                            frame = 0;
                        } else {
                            frame++;
                        }
                    }
                },
                led);
    }

    public Command fillThenEmptyCommand(int start, int length, double frameTime, boolean isInverted, int r, int g, int b) {
        return Commands.run(
                () -> {
                    int currentFrame = frame;
                    if (isInverted) currentFrame = 2 * length - currentFrame;
                    boolean isFirstHalf;
                    
                    for (int i = start; i < start + length + 1; i++) {
                        isFirstHalf = currentFrame < (length);
                        if ((i - 1) == currentFrame % length) {
                            if (isFirstHalf){
                                led.setPixel(i, r, g, b);
                            } else {
                                led.setPixel(i, 0, 0, 0);
                            }
                        }
                    }    

                    if (frameTime <= timer.get()) {
                        timer.reset();
                        if (frame == 2 * length) {
                            frame = 0;
                        } else {
                            frame++;
                        }
                    }
                },
                led);
    }

    public Command blueWavesLightCommand(int start, int length, double frameTime, boolean isInverted) {
        return chasingAlernatingColorAnimation(
            0, LEDConstants.TOTAL_PIXELS, 0, 118, 147, 0, 0, LEDConstants.FULL_BLUE_RGB / 2, 
            0.1, false);
    }

    public Command fullBlueCommand(int start, int length) {
        return constantColorAnimation(start, length, 0, 5, 25);
    }

    public Command chasingBlueCommand(int start, int length, double frameTime, boolean isInverted){
        return chasingPatternAnimation(start, length, frameTime, isInverted, 0, 5, 25);
    }

    public Command fillEmptyBlueCommand(int start, int length, double frameTime, boolean isInverted){
        return fillThenEmptyCommand(start, length, frameTime, isInverted, 0, 5, 25);
    }

    public Command allianceColor(int start, int length) {
        if (DriverStation.getAlliance().get() == Alliance.Red) {
            return constantColorAnimation(start, length, LEDConstants.FULL_RED_RGB, 0, 0);
        } else {
            return constantColorAnimation(start, length, 23, 29, 79);
        }
    }

    public Command boltAllianceColor(int start, int length, double frameTime, boolean isInverted) {
        if (DriverStation.getAlliance().get() == Alliance.Red) {
            return chasingPatternAnimation(
                    start, length, frameTime, isInverted, LEDConstants.FULL_RED_RGB, 0, 0);
        } else {
            return chasingPatternAnimation(start, length, frameTime, isInverted, 23, 29, 79);
        }
    }

    public Command blockAllianceColor(int start, int length, double frameTime, boolean isInverted) {
        if (DriverStation.getAlliance().get() == Alliance.Red) {
            return chasingBlocksAnimation(
                    start, length, frameTime, isInverted, LEDConstants.FULL_RED_RGB, 0, 0);
        } else {
            return chasingBlocksAnimation(start, length, frameTime, isInverted, 23, 29, 79);
        }
    }

    public Command noteInRobotCommand(int start, int length, double frameTime, boolean isInverted) {
        return chasingBlocksAnimation(
                start, length, frameTime, isInverted, LEDConstants.FULL_RED_RGB, 40, 0);
    }

    public Command notePickupCommand(int start, int length) {
        return constantColorAnimation(start, length, LEDConstants.FULL_RED_RGB, 40, 0);
    }

    public Command noteShootingCommand(
            int start, int length, double frameTime, boolean isInverted) {
        return chasingPatternAnimation(
                start, length, frameTime, isInverted, LEDConstants.FULL_RED_RGB, 40, 0);
    }

    public Command fullGreenCommand(int start, int length) {
        return constantColorAnimation(start, length, 0, LEDConstants.FULL_GREEN_RGB, 0);
    }

    public Command blockGreenCommand(int start, int length, double frameTime, boolean isInverted) {
        return chasingBlocksAnimation(
                start, length, frameTime, isInverted, 0, LEDConstants.FULL_GREEN_RGB, 0);
    }

    public Command endGameCommand(int start, int length, double frameTime, boolean isInverted) {
        return chasingChainAnimation(
                0,
                LEDConstants.TOTAL_PIXELS,
                frameTime,
                isInverted,
                LEDConstants.FULL_RED_RGB,
                0,
                0);
    }
}