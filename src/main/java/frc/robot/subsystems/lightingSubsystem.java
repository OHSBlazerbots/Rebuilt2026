package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class lightingSubsystem extends SubsystemBase {
    AddressableLED m_led;
    AddressableLEDBuffer m_ledBuffer;
    private int rainbowPixelHue = 0;

    public lightingSubsystem() {
        m_led = new AddressableLED(0);
        m_ledBuffer = new AddressableLEDBuffer(300);
        m_led.setLength(m_ledBuffer.getLength());
        
        m_led.setData(m_ledBuffer);
        m_led.start();
        // this.setPink();
    }

    public void setPink() {
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setRGB(i, 255, 0, 127);
        }

        m_led.setData(m_ledBuffer);

    }

    @Override
    public void periodic() {
        rainbow();
        m_led.setData(m_ledBuffer);
    }

    public void pinkWhiteStrobe() {
        LEDPattern gradient = LEDPattern.gradient(LEDPattern.GradientType.kContinuous, Color.kPink, Color.kWhite);
        gradient.applyTo(m_ledBuffer);
        m_led.setData(m_ledBuffer);
    }

     public void rainbow() {
        for (int i = 0; i < m_ledBuffer.getLength(); i++) {
            final int hue = (rainbowPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180;
            m_ledBuffer.setHSV(i, hue, 255, 128);
        }

        rainbowPixelHue += 3;
        rainbowPixelHue %= 180; 
    }
}