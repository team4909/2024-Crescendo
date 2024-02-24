package frc.robot.LEDs;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.CANdle.LEDStripType;

public class Led{
    private final CANdle m_led;

    public Led(){
        m_led = new CANdle(1);
        CANdleConfiguration config = new CANdleConfiguration();
        config.stripType = LEDStripType.RGB; // set the strip type to RGB
        config.brightnessScalar = 1; // dim the LEDs to half brightness
        m_led.configAllSettings(config);
    }

    public void setLEDColor(int red, int green, int blue){
        m_led.setLEDs(red, green, blue);
    }

    public void setLEDWhite(){
        m_led.setLEDs(255, 255, 255);
    }
}