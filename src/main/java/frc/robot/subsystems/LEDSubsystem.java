// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
  /** Creates a new LEDSubsystem. */
  private AddressableLED m_led;
  private AddressableLEDBuffer m_ledBuffer;
  private int rainbowHue;

  public LEDSubsystem() {
    m_led = new AddressableLED(9);

    m_ledBuffer = new AddressableLEDBuffer(105);
    m_led.setLength(105);

    m_led.setData(m_ledBuffer);
    m_led.start();

    rainbowHue = 0;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void turnOrange() {
    for (int i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setRGB(i, 255, 17, 0);
    }
    m_led.setData(m_ledBuffer);
  }

    public void turnGreen() {
    for (int i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setRGB(i, 60, 255, 0);
    }
    m_led.setData(m_ledBuffer);
  }

  public void rainbow() {
    // For every pixel
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
      final var hue = (rainbowHue + (i * 180 / m_ledBuffer.getLength())) % 180;
      // Set the value
      m_ledBuffer.setHSV(i, hue, 255, 128);
    }
    // Increase by to make the rainbow "move"
    rainbowHue += 3;
    // Check bounds
    rainbowHue %= 180;
    m_led.setData(m_ledBuffer);
  }
}
