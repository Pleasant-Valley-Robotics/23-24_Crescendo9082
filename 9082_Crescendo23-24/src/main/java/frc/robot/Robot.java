// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

/** This is a demo program showing how to use Mecanum control with the MecanumDrive class. */
public class Robot extends TimedRobot {
  //Define global public variables (Generally should be avoided with this)

  //Define local private variables
  private static final int kFrontLeftChannel = 0;
  private static final int kFrontRightChannel = 1;
  private static final int kBackLeftChannel = 2;
  private static final int kBackRightChannel = 3;
  private static final int kJoystickChannel = 0;
  private static AddressableLED m_Led;
  private static AddressableLEDBuffer m_LedBuffer;
  private int m_rainbowFirstPixelHue = 0;
  private MecanumDrive m_robotDrive;
  private Joystick m_stick;


  //Method to allow for a rainbow cascade effect on our LEDs
  private void rainbow() {
    
    //For every pixel
    for (var i = 0; i < m_LedBuffer.getLength(); i++) {
      //Calculate the hue - hue is easier for rainbows because the color
      //shape is a circle so only one value needs to precess
      final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_LedBuffer.getLength())) % 180;
    
      //Set the value
      m_LedBuffer.setHSV(i, hue, 255, 128);
    }
    
    //Increase by to make the rainbow "move"
    m_rainbowFirstPixelHue += 3;
    
    //Check bounds
    m_rainbowFirstPixelHue %= 180;

  } //End of rainbow

  @Override
  public void robotInit() {
    //Define each motor controller
    CANSparkMax frontLeft = new CANSparkMax(kFrontLeftChannel, MotorType.kBrushless);
    CANSparkMax frontRight = new CANSparkMax(kFrontRightChannel, MotorType.kBrushless);
    CANSparkMax backLeft = new CANSparkMax(kBackLeftChannel, MotorType.kBrushless);
    CANSparkMax backRight = new CANSparkMax(kBackRightChannel, MotorType.kBrushless);
    
    //Define Motor Controller Groups for future usage
    MotorControllerGroup leftMotors = new MotorControllerGroup(frontLeft, backLeft);
    MotorControllerGroup rightMotors = new MotorControllerGroup(frontRight, backRight);
    MotorControllerGroup backMotors = new MotorControllerGroup(backLeft, backRight);
    MotorControllerGroup frontMotors = new MotorControllerGroup(frontLeft, frontRight);
    
    //Invert the right side motors
    //You may need to change or remove this to match your robot
    frontRight.setInverted(true);
    backRight.setInverted(true);

    //Define the mecanum drive
    m_robotDrive = new MecanumDrive(frontLeft, backLeft, frontRight, backRight);

    //Define the joystick controlling the robot chassis
    m_stick = new Joystick(kJoystickChannel);

    //LED Code Initialization
    //PWM port 0
    //Must be a PWM header, not MXP or DIO
    m_Led = new AddressableLED(0);

    //Reuse buffer
    //Default to a length of 60, start empty output
    //Length is expensive to set, so only set it once, then just update data
    m_LedBuffer = new AddressableLEDBuffer(60);
    m_Led.setLength(m_LedBuffer.getLength());

    //Set the data
    m_Led.setData(m_LedBuffer);
    m_Led.start();
  } //End of robotInit

  @Override
  public void teleopPeriodic() {
    //Use the joystick Y axis for forward movement, X axis for lateral
    //movement, and Z axis for rotation.
    m_robotDrive.driveCartesian(-m_stick.getY(), -m_stick.getX(), -m_stick.getZ());
    
    //Fill the buffer with a rainbow
    rainbow();
    
    //Set the LEDs
    m_Led.setData(m_LedBuffer);
  
  } //End of teleopPeriodic

} //End of Class
