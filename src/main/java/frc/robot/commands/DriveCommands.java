package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HangingSubsystem;
import frc.robot.subsystems.NoteMoverSubsystem;

import static edu.wpi.first.math.MathUtil.clamp;
import static frc.robot.Constants.HangingConstants.ARM_SPEED;

public class DriveCommands {
    public static double applyDeadzone(double input, double deadzone) {
        if (input > deadzone) return (input - deadzone) / (1 - deadzone);
        else if (input < -deadzone) return (input + deadzone) / (1 - deadzone);
        else return 0.0;
    }

    public static Command getDrivebaseTeleCommand(XboxController joystick, DriveSubsystem robotDrive) {
        return new RunCommand(() -> {
            // Scale throttle from [1.0, -1.0] to [0.0, 1.0].
            double throttle = 1 - joystick.getRightTriggerAxis();
            // The y input is inverted on the controllers.
            // The reason that the x and y inputs seem to be flipped is that
            // the robot uses a different coordinate system.
            double xInput = applyDeadzone(-joystick.getLeftY(), 0.15) * throttle;
            double yInput = applyDeadzone(joystick.getLeftX(), 0.15) * throttle;
            double turnInput = applyDeadzone(joystick.getRightX(), 0.15) * throttle;

            robotDrive.drive(xInput, yInput, turnInput, false);
        }, robotDrive);
    }

    public static Command getNoteMoverTeleCommand(Joystick joystick, NoteMoverSubsystem robotNoteMover) {
        return new RunCommand(() -> {
            double intakeA = joystick.getRawButton(11) ? 1 : 0;
            double intakeB = joystick.getRawButton(12) ? -1 : 0;
            robotNoteMover.setIntakeSpeed(clamp(intakeA + intakeB, -1, 1));

            double shooterA = joystick.getRawButton(9) ? 1 : 0;
            double shooterB = joystick.getRawButton(10) ? -1 : 0;
            robotNoteMover.setShooterSpeed(clamp(shooterA + shooterB, -1, 1));
        }, robotNoteMover);
    }

    public static Command getArmTeleCommand(Joystick joystick, ArmSubsystem robotArm) {
        return new RunCommand(() -> robotArm.setArmSpeed(joystick.getY()), robotArm);
    }

    public static Command getHangingTeleCommand(Joystick joystick, HangingSubsystem robotHanging) {
        return new RunCommand(() -> {
            double leftArmUp = joystick.getRawButton(5) ? ARM_SPEED : 0;
            double leftArmDown = joystick.getRawButton(3) ? ARM_SPEED : 0;
            double rightArmUp = joystick.getRawButton(6) ? ARM_SPEED : 0;
            double rightArmDown = joystick.getRawButton(4) ? ARM_SPEED : 0;
            robotHanging.hangMotors(leftArmUp + leftArmDown, rightArmUp + rightArmDown);
        }, robotHanging);
    }
}
