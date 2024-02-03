package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.IntakeSubsystem;

public class DefaultDrivingSettings {
    public static Command SetDriveDefaults(Joystick driverJoystick, Joystick driverJoystick2, RobotContainer robotContainer, DriveSubsystem robotDrive, IntakeSubsystem robotIntake) {

        // Scale throttle from [1.0, -1.0] to [0.0, 1.0].
        double throttle = driverJoystick.getThrottle() / -2 + 0.5;

        // The y input is inverted on the controllers.
        // The reason that the x and y inputs seem to be flipped is that
        // the robot uses a different coordinate system.
        double xInput = robotContainer.applyDeadzone(-driverJoystick.getY(), 0.1) * throttle;
        double yInput = robotContainer.applyDeadzone(driverJoystick.getX(), 0.1) * throttle;
        double turnInput = robotContainer.applyDeadzone(driverJoystick.getTwist(), 0.1) * throttle;

        robotDrive.drive(xInput, yInput, turnInput, true);

        return new InstantCommand(() -> robotIntake.arm(driverJoystick2.getY()), robotIntake);
    }
}
