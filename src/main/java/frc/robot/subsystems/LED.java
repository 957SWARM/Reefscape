package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class LED implements Subsystem {

    private AddressableLED led;
    private AddressableLEDBuffer ledBuffer;

    int totalPixels;

    public LED(int totalPixels) {
        register();

        this.totalPixels = totalPixels;

        led = new AddressableLED(0);

        ledBuffer = new AddressableLEDBuffer(totalPixels);
        led.setLength(totalPixels);

        // Set the data
        led.setData(ledBuffer);
        led.start();
    }

    public void setPixel(int selectedPixel, int r, int g, int b) {
        if (selectedPixel < 0 || selectedPixel > totalPixels - 1) return;
        ledBuffer.setRGB(selectedPixel, r, g, b);
    }

    public int getRed(int selectedPixel) {
        return ledBuffer.getRed(selectedPixel);
    }

    public int getGreen(int selectedPixel) {
        return ledBuffer.getGreen(selectedPixel);
    }

    public int getBlue(int selectedPixel) {
        return ledBuffer.getBlue(selectedPixel);
    }

    @Override
    public void periodic() {
        led.setData(ledBuffer);
    }
}