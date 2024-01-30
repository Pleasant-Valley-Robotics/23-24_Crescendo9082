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
  private final CANSparkMax intakeArm = new CANSparkMax(DriveConstants.INTAKE_FEED_MOTOR_PORT, MotorType.kBrushless);
  private final CANSparkMax intakeFeed = new CANSparkMax(DriveConstants.INTAKE_ARM_MOTOR_PORT, MotorType.kBrushless);
  private final RelativeEncoder intakeArmEncoder = intakeArm.getEncoder();
  private final RelativeEncoder intakeFeedEncoder = intakeFeed.getEncoder();
  
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
    intakeFeed.set(speed);
  }

  /** Sets the intake feed MotorControllers to a voltage. */
  public void feedVoltage(double volts) {
    intakeFeed.setVoltage(volts);
  }

  /** Resets the intake feed encoders to currently read a position of 0. */
  public void resetFeedEncoders() {
    intakeFeedEncoder.setPosition(0);
  }

  /**
   * Gets the front shooter drive encoder.
   *
   * @return the intake feed drive encoder
   */
  public RelativeEncoder getIntakeFeedEncoder() {
    return intakeFeedEncoder;
  }
/**
   * Drives the intake arm at given speed. Speeds range from [-1, 1].
   *
   * @param speed speed of the intake arm motors.
   */
  public void arm(double speed) 
  {
    intakeFeed.set(speed);
  }

  /** Sets the intake arm MotorControllers to a voltage. */
  public void armVoltage(double volts) {
    intakeFeed.setVoltage(volts);
  }

  /** Resets the intake arm encoders to currently read a position of 0. */
  public void resetArmEncoders() {
    intakeArmEncoder.setPosition(0);
  }

  /**
   * Gets the intake arm encoder.
   *
   * @return the intake arm encoder
   */
  public RelativeEncoder getArmEncoder() {
    return intakeFeedEncoder;
  }
}
