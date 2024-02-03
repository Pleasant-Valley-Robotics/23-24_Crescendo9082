package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ShootSpeakerOff {
    public static Command turnSpeakerShooterOff(ShooterSubsystem robotShooter) {
        return new InstantCommand(() -> robotShooter.shootVoltageSpeaker(0), robotShooter);
    }
}
