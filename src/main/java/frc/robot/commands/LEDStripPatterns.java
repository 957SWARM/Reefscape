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
    Timer timer;
    
    int frame;
    double time;

    public LEDStripPatterns() {
        led = new LED(LEDConstants.TOTAL_PIXELS);
        frame = 0;
        timer = new Timer();
        
        timer.restart();
    }

    public void scheduleDefaultCommand(Command command) {
        CommandScheduler.getInstance().setDefaultCommand(led, command);
    }

    public boolean isSelectedColor(int selectedPixel, int r, int g, int b){
        boolean isRed = (led.getRed(selectedPixel) == r);
        boolean isGreen = (led.getGreen(selectedPixel) == g);
        boolean isBlue = (led.getGreen(selectedPixel) == b);

        if (isRed && isGreen && isBlue) return true;
        return false;
    }

    public boolean isBlank(int selectedPixel){
        boolean isRedEmpty = (led.getRed(selectedPixel) == 0);
        boolean isGreenEmpty = (led.getGreen(selectedPixel) == 0);
        boolean isBlueEmpty = (led.getBlue(selectedPixel) == 0);
        
        if (isRedEmpty && isGreenEmpty && isBlueEmpty) return true;
        return false;
    }

    public boolean isFull(int selectedPixel){
        boolean isRedFull = (led.getRed(selectedPixel) == LEDConstants.FULL_RED_RGB);
        boolean isGreenFull = (led.getGreen(selectedPixel) == LEDConstants.FULL_GREEN_RGB);
        boolean isBlueFull = (led.getBlue(selectedPixel) == LEDConstants.FULL_BLUE_RGB);
        
        if (isRedFull && isGreenFull && isBlueFull) return true;
        return false;
    }

    public void getBlankPattern(int start, int length){
        for (int i = start; i < start + length; i++) {
            led.setPixel(i, 0, 0, 0);
        }
    }

    public void getWhitePattern(int start, int length){
        for (int i = start; i < start + length; i++) {
            led.setPixel(i, LEDConstants.FULL_RED_RGB, LEDConstants.FULL_GREEN_RGB, LEDConstants.FULL_BLUE_RGB);
        }
    }

    public Command blankPatternAnimation(int start, int length) {
        return Commands.run(
                () -> {
                    for (int i = start; i < start + length; i++) {
                        led.setPixel(i, 0, 0, 0);
                    }
                },
                led);
    }

    public Command whitePatternAnimation(int start, int length) {
        return Commands.run(
            () -> {
                for (int i = start; i < start + length; i++) {
                    led.setPixel(i, LEDConstants.FULL_RED_RGB, LEDConstants.FULL_GREEN_RGB, LEDConstants.FULL_BLUE_RGB);
                }
        }, 
        led);
    }

    public Command constantColorAnimation(int start, int length, int r, int g, int b) {
        return Commands.run(
                () -> {
                    // Sets LEDs to a specific color constantly
                    for (int i = start; i < start + length; i++) {
                        led.setPixel(i, r, g, b);
                    }
                },
                led);
    }

    public Command flashingColorAnimation(int start, int length, int r, int g, int b){
        return Commands.run(
                () -> {
                    // Sets LEDs to flash with the RSL light
                    boolean rslState = RobotController.getRSLState();
                    if (rslState) {
                        for (int i = start; i < start + length; i++) {
                            led.setPixel(i, r, g, b);
                         }
                        } else {
                            for (int i = start; i < start + length; i++) {
                                led.setPixel(i, 0, 0, 0);
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
                        if (i % 3 == currentFrame) {
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

    // Fast command; run with 0.0333 frameTime
    public Command chasingSinglePatternAnimation(int start, int length, double frameTime, boolean isInverted, int r, int g, int b) {
        return Commands.run(
                () -> {
                    int currentFrame = frame;
                    if (isInverted) currentFrame = length - currentFrame;
                    
                    for (int i = start; i < start + length + 1; i++) {
                        if (i == currentFrame % length) {
                            getBlankPattern(start, length);
                            led.setPixel(i, r, g, b);
                        }
                    }    

                    if (frameTime <= timer.get()) {
                        timer.reset();
                        if (frame == length) {
                            frame = 0;
                        } else {
                            frame++;
                        }
                    }
                },
                led);
    }

    public Command chasingAlernatingColorAnimation(
        int start, int length, 
        int r1, int g1, int b1, int r2, int g2, int b2, double frameTime, boolean isInverted) {
        return Commands.run(
                () -> {
                    int currentFrame = frame;
                    if (isInverted) currentFrame = 2 - currentFrame;

                    for (int i = start; i < start + length; i++) {
                        if (i % 3 == currentFrame) {
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
                        if (i % 2 == currentFrame || i % 3 == currentFrame) {
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

    // Fast command; run with 0.01 frameTime 
    public Command fillThenEmptyAnimation(int start, int length, double frameTime, boolean isInverted, int r, int g, int b) {
        return Commands.run(
                () -> {
                    int currentFrame = frame;
                    if (isInverted) currentFrame = 2 * length - currentFrame - 1;
                    boolean isFirstHalf;

                    for (int i = start; i < start + length; i++) {
                        isFirstHalf = currentFrame < (length);
                        if (i == currentFrame % length) {
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

    // Fast command; run with 0.0333 breathRate
    public Command breathingPatternAnimation(int start, int length, double breathRate, boolean isInverted, int r, int g, int b){
        return Commands.run(
            () -> {
                time++;
                double brightness = (!isInverted) ? ((Math.cos(breathRate * time) + 1) / 2) : ((Math.sin(breathRate * time) + 1) / 2);
                
                for (int i = start; i < start + length; i++) {
                    led.setPixel(i, (int) (r * brightness), (int) (g * brightness), (int) (b * brightness));
                }
        }, 
        led);
    }

    public Command defaultBlueWavesLightCommand(int start, int length, double frameTime, boolean isInverted) {
        return chasingAlernatingColorAnimation(
            0, LEDConstants.TOTAL_PIXELS, 0, 118, 147, 0, 0, LEDConstants.FULL_BLUE_RGB / 2, 
            0.1, isInverted);
    }

    public Command intakeBreatheBlueCommand(int start, int length, double breathRate, boolean isInverted){
        return breathingPatternAnimation(start, length, breathRate, isInverted, 0, 45, 225);
    }

    public Command shootingFillEmptyBlueCommand(int start, int length, double frameTime, boolean isInverted){
        return fillThenEmptyAnimation(start, length, frameTime, isInverted, 0, 5, LEDConstants.FULL_BLUE_RGB / 2);
    }

    public Command coralReceivedFlashingBlueCommand(int start, int length){
        return flashingColorAnimation(start, length, 0, 118, 147);
    }

    public Command coralOutChasingBlueCommand(int start, int length, double frameTime, boolean isInverted){
        return chasingPatternAnimation(start, length, frameTime, isInverted, 0, 5, LEDConstants.FULL_BLUE_RGB / 2);
    }

    public Command fullBlueCommand(int start, int length) {
        return constantColorAnimation(start, length, 0, 5, LEDConstants.FULL_BLUE_RGB / 2);
    }

    public Command fullWhiteCommand(int start, int length) {
        return constantColorAnimation(start, length, LEDConstants.FULL_RED_RGB, LEDConstants.FULL_GREEN_RGB, LEDConstants.FULL_BLUE_RGB);
    }

    public Command autoPatternChasingSingleBlueCommand(int start, int length, double frameTime, boolean isInverted){
        return chasingSinglePatternAnimation(start, length, frameTime, isInverted, 0, 5, 25);
    }

    public Command allianceColorCommand(int start, int length) {
        if (DriverStation.getAlliance().get() == Alliance.Red) {
            return constantColorAnimation(start, length, LEDConstants.FULL_RED_RGB, 0, 0);
        } else {
            return constantColorAnimation(start, length, 23, 29, 79);
        }
    }

    public Command boltAllianceColorCommand(int start, int length, double frameTime, boolean isInverted) {
        if (DriverStation.getAlliance().get() == Alliance.Red) {
            return chasingPatternAnimation(
                    start, length, frameTime, isInverted, LEDConstants.FULL_RED_RGB, 0, 0);
        } else {
            return chasingPatternAnimation(start, length, frameTime, isInverted, 23, 29, 79);
        }
    }

    public Command blockAllianceColorCommand(int start, int length, double frameTime, boolean isInverted) {
        if (DriverStation.getAlliance().get() == Alliance.Red) {
            return chasingBlocksAnimation(
                    start, length, frameTime, isInverted, LEDConstants.FULL_RED_RGB, 0, 0);
        } else {
            return chasingBlocksAnimation(start, length, frameTime, isInverted, 23, 29, 79);
        }
    }

    public Command allianceColorBreatheCommand(int start, int length, double breathRate, boolean isInverted) {
        if (DriverStation.getAlliance().get() == Alliance.Red) {
            return breathingPatternAnimation(start, length, breathRate, isInverted, LEDConstants.FULL_RED_RGB, 0, 0);
        } else {
            return breathingPatternAnimation(start, length, breathRate, isInverted, 0, 0, LEDConstants.FULL_BLUE_RGB);
        }
    }

    public Command fullGreenCommand(int start, int length) {
        return constantColorAnimation(start, length, 0, LEDConstants.FULL_GREEN_RGB, 10);
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
                0,
                0,
                LEDConstants.FULL_BLUE_RGB);
    }

}