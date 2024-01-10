// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class ShooterSubsystem extends SubsystemBase {
  private final CANSparkMax m_frontShooter = new CANSparkMax(DriveConstants.kFrontShooterMotorPort, MotorType.kBrushless);
  private final CANSparkMax m_rearShooter = new CANSparkMax(DriveConstants.kRearShooterMotorPort, MotorType.kBrushless);
  private final RelativeEncoder m_frontShooterEncoder = m_frontShooter.getEncoder();
  private final RelativeEncoder m_rearShooterEncoder = m_rearShooter.getEncoder();
  

  /** Creates a new DriveSubsystem. */
  public ShooterSubsystem() {
    //Put any default values for the subsystem into this constructor.
  }

  @Override
  public void periodic() {
    //Consider putting shooter motor turn rates from the subsystem into telemetry here
  }

   /**
   * Drives the shooter subsystem at given speed. Speeds range from [-1, 1].
   *
   * @param speed speed of the shooter motors.
   */
  public void shoot(double speed) 
  {
    m_frontShooter.set(speed);
    m_rearShooter.set(speed);
  }

  /** Sets the shooter MotorControllers to a voltage. */
  public void shootVoltage(double volts) {
    m_frontShooter.setVoltage(volts);
    m_rearShooter.setVoltage(volts);
  }

  /** Resets the shooter encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontShooterEncoder.setPosition(0);
    m_rearShooterEncoder.setPosition(0);
  }

  /**
   * Gets the front shooter drive encoder.
   *
   * @return the front shooter drive encoder
   */
  public RelativeEncoder getFrontShooterEncoder() {
    return m_frontShooterEncoder;
  }

  /**
   * Gets the rear shooter drive encoder.
   *
   * @return the rear shooter drive encoder
   */
  public RelativeEncoder getRearShooterEncoder() {
    return m_rearShooterEncoder;
  }
}
