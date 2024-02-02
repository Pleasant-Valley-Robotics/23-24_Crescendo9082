package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootAmpOff {
    public static Command turnRobotShooterOff(ShooterSubsystem robotShooter) {
        return new InstantCommand(() -> robotShooter.shootVoltageAmp(0), robotShooter);
    }
}
