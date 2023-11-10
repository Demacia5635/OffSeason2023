package frc.robot;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

public class LedController {
    private AddressableLED led;
    private AddressableLEDBuffer buffer;
    private boolean enabled;
    private Color color;

    public LedController(int port, int count) {
        led = new AddressableLED(port);
        buffer = new AddressableLEDBuffer(count);
        led.setLength(buffer.getLength());
    }

    public void changeColor(Color color) {
        this.color = color;
        for (int i = 0; i < buffer.getLength(); i++)
            buffer.setLED(i, color);
        led.setData(buffer);
    }

    public void setEnabled(boolean enable) {
        if (enable) start();
        else stop();
    }

    public void toggleEnabled() {
        enabled = !enabled;
        if (enabled) start();
        else stop();
    }

    private void start() {
        changeColor(color);
    }

    private void stop() {
        for (int i = 0; i < buffer.getLength(); i++)
            buffer.setRGB(i, 0, 0, 0);
    }
}
