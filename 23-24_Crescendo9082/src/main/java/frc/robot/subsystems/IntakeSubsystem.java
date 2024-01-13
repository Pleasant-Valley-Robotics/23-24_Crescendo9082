// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class IntakeSubsystem extends SubsystemBase {
  private final CANSparkMax m_intakeArm = new CANSparkMax(DriveConstants.kIntakeFeedMotorPort, MotorType.kBrushless);
  private final CANSparkMax m_intakeFeed = new CANSparkMax(DriveConstants.kIntakeArmMotorPort, MotorType.kBrushless);
  private final RelativeEncoder m_intakeArmEncoder = m_intakeArm.getEncoder();
  private final RelativeEncoder m_intakeFeedEncoder = m_intakeFeed.getEncoder();
  
  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  /**
   * Drives the intake feed motors at given speed. Speeds range from [-1, 1].
   *
   * @param speed speed of the intake feed motors.
   */
  public void feed(double speed) 
  {
    m_intakeFeed.set(speed);
  }

  /** Sets the intake feed MotorControllers to a voltage. */
  public void feedVoltage(double volts) {
    m_intakeFeed.setVoltage(volts);
  }

  /** Resets the intake feed encoders to currently read a position of 0. */
  public void resetFeedEncoders() {
    m_intakeFeedEncoder.setPosition(0);
  }

  /**
   * Gets the front shooter drive encoder.
   *
   * @return the intake feed drive encoder
   */
  public RelativeEncoder getIntakeFeedEncoder() {
    return m_intakeFeedEncoder;
  }
/**
   * Drives the intake arm at given speed. Speeds range from [-1, 1].
   *
   * @param speed speed of the intake arm motors.
   */
  public void arm(double speed) 
  {
    m_intakeFeed.set(speed);
  }

  /** Sets the intake arm MotorControllers to a voltage. */
  public void armVoltage(double volts) {
    m_intakeFeed.setVoltage(volts);
  }

  /** Resets the intake arm encoders to currently read a position of 0. */
  public void resetArmEncoders() {
    m_intakeArmEncoder.setPosition(0);
  }

  /**
   * Gets the intake arm encoder.
   *
   * @return the intake arm encoder
   */
  public RelativeEncoder getArmEncoder() {
    return m_intakeFeedEncoder;
  }
}
