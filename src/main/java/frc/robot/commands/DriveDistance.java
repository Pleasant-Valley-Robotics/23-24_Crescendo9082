// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class DriveDistance extends Command {
  private final DriveSubsystem m_drive;
  private final double m_distance;
  private final double m_speed;

  /**
   * Creates a new DriveDistance.
   *
   * @param inches The number of inches the robot will drive
   * @param speed The speed at which the robot will drive
   * @param drive The drive subsystem on which this command will run
   */
  public DriveDistance(double meters, double speed, DriveSubsystem drive) {
    m_distance = meters;
    m_speed = speed;
    m_drive = drive;
    addRequirements(m_drive);
  }

  @Override
  public void initialize() {
    m_drive.resetEncoders();
    m_drive.drive(0,0,0, false);
  }

  @Override
  public void execute() {
    m_drive.drive(m_speed,0, 0, false);
    SmartDashboard.putNumber("FLEncoderPos", m_drive.getFrontLeftEncoder().getPosition());
    SmartDashboard.putNumber("AverageEncoderDistance", m_drive.getAverageEncoders());
  }

  @Override
  public void end(boolean interrupted) {
  m_drive.drive(0,0,0, false);
  }

  @Override
  public boolean isFinished() {
    return m_drive.getAverageEncoders() >= m_distance;
  }
}