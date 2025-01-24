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

    LED led = new LED(LEDConstants.TOTAL_PIXELS);
    int frame = 0;
    Timer timer = new Timer();

    public LEDStripPatterns() {
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

    public Command movingForwardAnimation(
            int start, int length, double frameTime, boolean isInverted, int r, int g, int b) {

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

    public Command movingForwardBlocksAnimation(
            int start, int length, double frameTime, boolean isInverted, int r, int g, int b) {

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

    public Command movingForwardChainAnimation(
            int start, int length, double frameTime, boolean isInverted, int r, int g, int b) {

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

    public Command yellowBoltLightCommand(
            int start, int length, double frameTime, boolean isInverted) {
        return movingForwardAnimation(start, length, frameTime, isInverted, 254, 150, 0);
    }

    public Command fullYellowCommand(int start, int length) {
        return constantColorAnimation(start, length, 254, 150, 0).ignoringDisable(true);
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
            return movingForwardAnimation(
                    start, length, frameTime, isInverted, LEDConstants.FULL_RED_RGB, 0, 0);
        } else {
            return movingForwardAnimation(start, length, frameTime, isInverted, 23, 29, 79);
        }
    }

    public Command blockAllianceColor(int start, int length, double frameTime, boolean isInverted) {
        if (DriverStation.getAlliance().get() == Alliance.Red) {
            return movingForwardBlocksAnimation(
                    start, length, frameTime, isInverted, LEDConstants.FULL_RED_RGB, 0, 0);
        } else {
            return movingForwardBlocksAnimation(start, length, frameTime, isInverted, 23, 29, 79);
        }
    }

    public Command noteInRobotCommand(int start, int length, double frameTime, boolean isInverted) {
        return movingForwardBlocksAnimation(
                start, length, frameTime, isInverted, LEDConstants.FULL_RED_RGB, 40, 0);
    }

    public Command notePickupCommand(int start, int length) {
        return constantColorAnimation(start, length, LEDConstants.FULL_RED_RGB, 40, 0);
    }

    public Command noteShootingCommand(
            int start, int length, double frameTime, boolean isInverted) {
        return movingForwardAnimation(
                start, length, frameTime, isInverted, LEDConstants.FULL_RED_RGB, 40, 0);
    }

    public Command fullGreenCommand(int start, int length) {
        return constantColorAnimation(start, length, 0, LEDConstants.FULL_GREEN_RGB, 0);
    }

    public Command blockGreenCommand(int start, int length, double frameTime, boolean isInverted) {
        return movingForwardBlocksAnimation(
                start, length, frameTime, isInverted, 0, LEDConstants.FULL_GREEN_RGB, 0);
    }

    public Command endGameCommand(int start, int length, double frameTime, boolean isInverted) {
        return movingForwardChainAnimation(
                0,
                LEDConstants.TOTAL_PIXELS,
                frameTime,
                isInverted,
                LEDConstants.FULL_RED_RGB,
                0,
                0);
    }
}