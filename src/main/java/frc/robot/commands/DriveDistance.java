// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class DriveDistance extends Command {
    private final DriveSubsystem drive;
    private final double distance;
    private final double speed;

    /**
     * Creates a new DriveDistance.
     *
     * @param meters Meters to drive forward
     * @param speed  The speed at which the robot will drive
     * @param drive  The drive subsystem on which this command will run
     */
    public DriveDistance(double meters, double speed, DriveSubsystem drive) {
        this.distance = meters;
        this.speed = speed;
        this.drive = drive;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        drive.resetEncoders();
        drive.drive(0, 0, 0, false);
    }

    @Override
    public void execute() {
        drive.drive(speed, 0, 0, false);
        SmartDashboard.putNumber("FLEncoderPos", drive.getFrontLeftEncoder().getPosition());
        SmartDashboard.putNumber("AverageEncoderDistance", drive.getAverageEncoders());
    }

    @Override
    public void end(boolean interrupted) {
        drive.drive(0, 0, 0, false);
    }

    @Override
    public boolean isFinished() {
        return drive.getAverageEncoders() >= distance;
    }
}