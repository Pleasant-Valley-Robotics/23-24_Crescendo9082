// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.MecanumControllerCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HangingSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

import java.util.List;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems
    private final DriveSubsystem robotDrive = new DriveSubsystem();
    private final ShooterSubsystem robotShooter = new ShooterSubsystem();
    private final HangingSubsystem robotHanging = new HangingSubsystem();
    private final IntakeSubsystem robotIntake = new IntakeSubsystem();
    // The driver's joystick
    Joystick driverJoystick = new Joystick(OIConstants.DRIVER_CONTROLLER_PORT);
    Joystick driverJoystick2 = new Joystick(OIConstants.DRIVER_CONTROLLER_PORT_2);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the button bindings
        configureButtonBindings();

        // Configure default commands
        // Set the default drive command to drive with single joystick on field-oriented chassis movement, with shooter controlled by a second joystick's trigger.
        robotDrive.setDefaultCommand(
                // A split-stick arcade command, with forward/backward controlled by the left
                // hand, and turning controlled by the right.
                new RunCommand(
                        () ->
                                robotDrive.drive(driverJoystick.getY(), driverJoystick.getX(), driverJoystick.getTwist(), true), robotDrive));

        robotIntake.setDefaultCommand(
                new RunCommand(
                        () ->
                                robotIntake.arm(driverJoystick2.getY()), robotIntake));
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
     * {@link JoystickButton}.
     */
    private void configureButtonBindings() {
        // Drive at half speed when the 1 button is held.
        new JoystickButton(driverJoystick, 1)
                .onTrue(new InstantCommand(() -> robotShooter.shootVoltageAmp(12), robotShooter))
                .onFalse(new InstantCommand(() -> robotShooter.shootVoltageAmp(0)));
        // Shoot when the 2 button is held.
        new JoystickButton(driverJoystick, 2)
                .onTrue(new InstantCommand(() -> robotShooter.shootVoltageSpeaker(12), robotShooter))
                .onFalse(new InstantCommand(() -> robotShooter.shootVoltageSpeaker(0)));
        // Hang when the 3 button is held.
        new JoystickButton(driverJoystick, 3)
                .onTrue(new InstantCommand(() -> robotHanging.hangVoltage(12), robotHanging))
                .onFalse(new InstantCommand(() -> robotHanging.hangVoltage(0)));
        // Intake Feed when the 4 button is held.
        new JoystickButton(driverJoystick, 4)
                .onTrue(new InstantCommand(() -> robotIntake.feedVoltage(12), robotIntake))
                .onFalse(new InstantCommand(() -> robotIntake.feedVoltage(0), robotIntake));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // Create config for trajectory
        TrajectoryConfig config =
                new TrajectoryConfig(
                        AutoConstants.MAX_SPEED_METERS_PER_SECOND,
                        AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED)
                        // Add kinematics to ensure max speed is actually obeyed
                        .setKinematics(DriveConstants.DRIVE_KINEMATICS);

        // An example trajectory to follow.  All units in meters.
        Trajectory exampleTrajectory =
                TrajectoryGenerator.generateTrajectory(
                        // Start at the origin facing the +X direction
                        new Pose2d(0, 0, new Rotation2d(0)),
                        // Pass through these two interior waypoints, making an 's' curve path
                        List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
                        // End 3 meters straight ahead of where we started, facing forward
                        new Pose2d(3, 0, new Rotation2d(0)),
                        config);

        MecanumControllerCommand mecanumControllerCommand =
                new MecanumControllerCommand(
                        exampleTrajectory,
                        robotDrive::getPose,
                        DriveConstants.FEEDFORWARD,
                        DriveConstants.DRIVE_KINEMATICS,

                        // Position controllers
                        new PIDController(AutoConstants.PX_CONTROLLER, 0, 0),
                        new PIDController(AutoConstants.PY_CONTROLLER, 0, 0),
                        new ProfiledPIDController(
                                AutoConstants.P_THETA_CONTROLLER, 0, 0, AutoConstants.THETA_CONTROLLER_CONSTRAINTS),

                        // Needed for normalizing wheel speeds
                        AutoConstants.MAX_SPEED_METERS_PER_SECOND,

                        // Velocity PID's
                        new PIDController(DriveConstants.P_FRONT_LEFT_VEL, 0, 0),
                        new PIDController(DriveConstants.P_REAR_LEFT_VEL, 0, 0),
                        new PIDController(DriveConstants.P_FRONT_RIGHT_VEL, 0, 0),
                        new PIDController(DriveConstants.P_REAR_RIGHT_VEL, 0, 0),
                        robotDrive::getCurrentWheelSpeeds,
                        robotDrive::setDriveMotorControllersVolts, // Consumer for the output motor voltages
                        robotDrive);

        // Reset odometry to the starting pose of the trajectory.
        robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

        // Run path following command, then stop at the end.
        return mecanumControllerCommand.andThen(() -> robotDrive.drive(0, 0, 0, false));
    }
}
