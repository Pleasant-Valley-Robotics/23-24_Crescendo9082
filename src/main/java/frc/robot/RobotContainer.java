// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AutoCommands;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HangingSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems
    public final DriveSubsystem robotDrive = new DriveSubsystem();
    public final ShooterSubsystem robotShooter = new ShooterSubsystem();
    public final HangingSubsystem robotHanging = new HangingSubsystem();
    public final IntakeSubsystem robotIntake = new IntakeSubsystem();
    // The driver's joystick
    Joystick driverJoystick = new Joystick(OIConstants.DRIVER_CONTROLLER_PORT);
    Joystick driverJoystick2 = new Joystick(OIConstants.DRIVER_CONTROLLER_PORT_2);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        configureButtonBindings();
        configureTeleOpCommands();
    }

    public double applyDeadzone(double input, double deadzone) {
        if (input > deadzone) return (input - deadzone) / (1 - deadzone);
        else if (input < -deadzone) return (input + deadzone) / (1 - deadzone);
        else return 0.0;
    }

    private void configureTeleOpCommands() {
        robotDrive.setDefaultCommand(new RunCommand(() -> {
            // Scale throttle from [1.0, -1.0] to [0.0, 1.0].
            double throttle = driverJoystick.getThrottle() / -2 + 0.5;

            // The y input is inverted on the controllers.
            // The reason that the x and y inputs seem to be flipped is that
            // the robot uses a different coordinate system.
            double xInput = applyDeadzone(-driverJoystick.getY(), 0.15) * throttle;
            double yInput = applyDeadzone(driverJoystick.getX(), 0.15) * throttle;
            double turnInput = applyDeadzone(driverJoystick.getTwist(), 0.15) * throttle;

            robotDrive.drive(xInput, yInput, turnInput, false);
        }, robotDrive));

        robotIntake.setDefaultCommand(new RunCommand(() -> robotIntake.setIntakeSpeed(driverJoystick2.getY()), robotIntake));
    }

    private void configureButtonBindings() {
//        // Shoot amp when button 1 is pressed.
//        new JoystickButton(driverJoystick, 1)
//                .onTrue(new ShootAmp(12, robotShooter));
//        // Shoot when the 2 button is held.
//        new JoystickButton(driverJoystick, 2)
//                .onTrue(new ShootSpeaker(12, robotShooter));
//        // Hang when the 3 button is held.
//        new JoystickButton(driverJoystick, 3)
//                .onTrue(new Hang(12, robotHanging));
//        // Intake Feed when the 4 button is held.
//        new JoystickButton(driverJoystick, 4)
//                .onTrue(new RobotIntake(12, robotIntake));
//
//        new JoystickButton(driverJoystick, 12)
//                .onTrue(new InstantCommand(() -> robotDrive.resetOdometry(new Pose2d(0, 0, new Rotation2d(0)))));

        new JoystickButton(driverJoystick, 7).whileTrue(robotDrive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        new JoystickButton(driverJoystick, 8).whileTrue(robotDrive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        new JoystickButton(driverJoystick, 9).whileTrue(robotDrive.sysIdDynamic(SysIdRoutine.Direction.kForward));
        new JoystickButton(driverJoystick, 10).whileTrue(robotDrive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    }

     /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
       return AutoCommands.simpleAuto(robotDrive);
        // return new DriveDistance(2,.2, robotDrive);
    }
}
