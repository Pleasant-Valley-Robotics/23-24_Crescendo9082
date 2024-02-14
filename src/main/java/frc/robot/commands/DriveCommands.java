package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class DriveCommands {
    public static double applyDeadzone(double input, double deadzone) {
        if (input > deadzone) return (input - deadzone) / (1 - deadzone);
        else if (input < -deadzone) return (input + deadzone) / (1 - deadzone);
        else return 0.0;
    }

    public static Command getDrivebaseTeleCommand(Joystick joystick, DriveSubsystem robotDrive) {
        return new RunCommand(() -> {
            // Scale throttle from [1.0, -1.0] to [0.0, 1.0].
            double throttle = joystick.getThrottle() / -2 + 0.5;

            // The y input is inverted on the controllers.
            // The reason that the x and y inputs seem to be flipped is that
            // the robot uses a different coordinate system.
            double xInput = applyDeadzone(-joystick.getY(), 0.15) * throttle;
            double yInput = applyDeadzone(joystick.getX(), 0.15) * throttle;
            double turnInput = applyDeadzone(joystick.getTwist(), 0.15) * throttle;

            robotDrive.drive(xInput, yInput, turnInput, false);
        }, robotDrive);
    }

    public static Command getIntakeTeleCommand(Joystick joystick, IntakeSubsystem robotIntake) {
        return new RunCommand(() -> robotIntake.setIntakeSpeed(joystick.getY()), robotIntake);
    }
}
