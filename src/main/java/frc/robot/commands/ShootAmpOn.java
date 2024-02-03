package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ShootAmpOn {
    public static Command turnAmpShooterOn (ShooterSubsystem robotShooter) {
        return new InstantCommand(() -> robotShooter.shootVoltageAmp(12), robotShooter);
    }
}
