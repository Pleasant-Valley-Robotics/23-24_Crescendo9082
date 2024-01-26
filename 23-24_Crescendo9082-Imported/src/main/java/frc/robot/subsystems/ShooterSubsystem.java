// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class ShooterSubsystem extends SubsystemBase {
  private final CANSparkMax m_upperLeftShooter = new CANSparkMax(DriveConstants.kUpperLeftShooterMotorPort, MotorType.kBrushless);
  private final CANSparkMax m_lowerLeftShooter = new CANSparkMax(DriveConstants.kLowerLeftShooterMotorPort, MotorType.kBrushless);
  private final CANSparkMax m_upperRightShooter = new CANSparkMax(DriveConstants.kUpperRightShooterMotorPort, MotorType.kBrushless);
  private final CANSparkMax m_lowerRightShooter = new CANSparkMax(DriveConstants.kLowerRightShooterMotorPort, MotorType.kBrushless);
  private final CANSparkMax m_shooterArm = new CANSparkMax(DriveConstants.kShooterPivotPort, MotorType.kBrushless);
  private final RelativeEncoder m_upperLeftShooterEncoder = m_upperLeftShooter.getEncoder();
  private final RelativeEncoder m_lowerLeftShooterEncoder = m_lowerLeftShooter.getEncoder();
  private final RelativeEncoder m_upperRightShooterEncoder = m_upperRightShooter.getEncoder();
  private final RelativeEncoder m_lowerRightShooterEncoder = m_lowerRightShooter.getEncoder();
  

  /** Creates a new DriveSubsystem. */
  public ShooterSubsystem() {
    //Put any default values for the subsystem into this constructor.
  }

  @Override
  public void periodic() {
    //Consider putting shooter motor turn rates from the subsystem into telemetry here
  }

  public void shooterArmSpeed(double speed)
  {
    m_shooterArm.set(speed*.1);
  }

  public void shooterArmVoltage(double volts)
  {
    m_shooterArm.setVoltage(volts);
    
  }

   /**
   * Drives the shooter subsystem at given speed. Speeds range from [-1, 1].
   *
   * @param speed speed of the shooter motors.
   */
  public void shootSpeaker(double speed) 
  {
    m_upperLeftShooter.set(speed);
    m_lowerLeftShooter.set(speed);
    m_upperRightShooter.set(speed);
    m_lowerRightShooter.set(speed);
  }

  /** Sets the shooter MotorControllers to a voltage. */
  public void shootVoltageSpeaker(double volts) {
    m_upperLeftShooter.setVoltage(volts);
    m_lowerLeftShooter.setVoltage(volts);
    m_upperRightShooter.setVoltage(volts);
    m_lowerRightShooter.setVoltage(volts);
  }

  public void shootAmp(double speed) 
  {
    m_upperLeftShooter.set(-speed);
    m_lowerLeftShooter.set(speed);
    m_upperRightShooter.set(speed);
    m_lowerRightShooter.set(-speed);
  }

  /** Sets the shooter MotorControllers to a voltage. */
  public void shootVoltageAmp(double volts) {
    m_upperLeftShooter.setVoltage(-volts);
    m_lowerLeftShooter.setVoltage(volts);
    m_upperRightShooter.setVoltage(volts);
    m_lowerRightShooter.setVoltage(-volts);
  }

  /** Resets the shooter encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_upperLeftShooterEncoder.setPosition(0);
    m_lowerLeftShooterEncoder.setPosition(0);
    m_upperRightShooterEncoder.setPosition(0);
    m_lowerRightShooterEncoder.setPosition(0);
  }

  public RelativeEncoder getUpperLeftShooterEncoder() {
    return m_upperLeftShooterEncoder;
  }
    public RelativeEncoder getLowerLeftShooterEncoder() {
    return m_lowerLeftShooterEncoder;
  }
    public RelativeEncoder getUpperRightShooterEncoder() {
    return m_upperRightShooterEncoder;
  }
    public RelativeEncoder getLowerRightShooterEncoder() {
    return m_lowerRightShooterEncoder;
  }

}
