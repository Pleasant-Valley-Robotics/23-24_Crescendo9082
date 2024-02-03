package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootAmpOff {
    public static Command turnAmpShooterOff(ShooterSubsystem robotShooter) {
        return new InstantCommand(() -> robotShooter.shootVoltageAmp(0), robotShooter);
    }
}
