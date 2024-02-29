// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AutoCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HangingSubsystem;
import frc.robot.subsystems.NoteMoverSubsystem;
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
    public final HangingSubsystem robotHanging = new HangingSubsystem();
    public final NoteMoverSubsystem robotNoteMover = new NoteMoverSubsystem();
    // The driver's joystick
    XboxController driverJoystick = new XboxController(OIConstants.DRIVER_CONTROLLER_PORT);
    Joystick driverJoystick2 = new Joystick(OIConstants.DRIVER_CONTROLLER_PORT_2);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        configureButtonBindings();
        configureTeleOpCommands();
    }


    private void configureTeleOpCommands() {
        robotDrive.setDefaultCommand(DriveCommands.getDrivebaseTeleCommand(driverJoystick, robotDrive));

//        robotIntake.setDefaultCommand(DriveCommands.getIntakeTeleCommand(driverJoystick2, robotIntake));
        robotHanging.setDefaultCommand(DriveCommands.getHangingTeleCommand(driverJoystick2, robotHanging));
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

        new JoystickButton(driverJoystick, 7).whileTrue(robotDrive.routine.quasistatic(SysIdRoutine.Direction.kForward));
        new JoystickButton(driverJoystick, 8).whileTrue(robotDrive.routine.quasistatic(SysIdRoutine.Direction.kReverse));
        new JoystickButton(driverJoystick, 9).whileTrue(robotDrive.routine.dynamic(SysIdRoutine.Direction.kForward));
        new JoystickButton(driverJoystick, 10).whileTrue(robotDrive.routine.dynamic(SysIdRoutine.Direction.kReverse));

//        new JoystickButton(driverJoystick, 7).whileTrue(robotNoteMover.routine.quasistatic(SysIdRoutine.Direction.kForward));
//        new JoystickButton(driverJoystick, 8).whileTrue(robotNoteMover.routine.quasistatic(SysIdRoutine.Direction.kReverse));
//        new JoystickButton(driverJoystick, 9).whileTrue(robotNoteMover.routine.dynamic(SysIdRoutine.Direction.kForward));
//        new JoystickButton(driverJoystick, 10).whileTrue(robotNoteMover.routine.dynamic(SysIdRoutine.Direction.kReverse));

//        new JoystickButton(driverJoystick, 7).whileTrue(robotHanging.routine.quasistatic(SysIdRoutine.Direction.kForward));
//        new JoystickButton(driverJoystick, 8).whileTrue(robotHanging.routine.quasistatic(SysIdRoutine.Direction.kReverse));
//        new JoystickButton(driverJoystick, 9).whileTrue(robotHanging.routine.dynamic(SysIdRoutine.Direction.kForward));
//        new JoystickButton(driverJoystick, 10).whileTrue(robotHanging.routine.dynamic(SysIdRoutine.Direction.kReverse));
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
