// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class HangingSubsystem extends SubsystemBase {
  private final CANSparkMax m_leftHangingMotor = new CANSparkMax(DriveConstants.kLeftHangingMotorPort, MotorType.kBrushless);
  private final CANSparkMax m_rightHangingMotor = new CANSparkMax(DriveConstants.kRightHangingMotorPort, MotorType.kBrushless);
  private final RelativeEncoder m_leftHangingEncoder = m_leftHangingMotor.getEncoder();
  private final RelativeEncoder m_rightHangingEncoder = m_rightHangingMotor.getEncoder();
  /** Creates a new HangingSubsystem. */
  public HangingSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  
 /**
   * Drives the hanging subsystem at given speed. Speeds range from [-1, 1].
   *
   * @param speed speed of the hanging motors.
   */
  public void hang(double speed) 
  {
    m_leftHangingMotor.set(speed);
    m_rightHangingMotor.set(speed);
  }

  /** Sets the hanging MotorControllers to a voltage. */
  public void hangVoltage(double volts) {
    m_leftHangingMotor.setVoltage(volts);
    m_rightHangingMotor.setVoltage(volts);
  }

  /** Resets the hanging encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_leftHangingEncoder.setPosition(0);
    m_rightHangingEncoder.setPosition(0);
  }

  /**
   * Gets the front hanging drive encoder.
   *
   * @return the front hanging drive encoder
   */
  public RelativeEncoder getLeftHangingEncoder() {
    return m_leftHangingEncoder;
  }

  /**
   * Gets the rear hanging drive encoder.
   *
   * @return the rear hanging drive encoder
   */
  public RelativeEncoder getRightHangingEncoder() {
    return m_rightHangingEncoder;
  }
}
