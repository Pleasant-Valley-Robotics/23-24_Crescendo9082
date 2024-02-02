package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.MecanumControllerCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

import java.util.List;

public class Autos {
    public static Command simpleAuto(DriveSubsystem robotDrive) {
        TrajectoryConfig config =
                new TrajectoryConfig(
                        Constants.AutoConstants.MAX_SPEED_METERS_PER_SECOND,
                        Constants.AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED)
                        // Add kinematics to ensure max speed is actually obeyed
                        .setKinematics(Constants.DriveConstants.DRIVE_KINEMATICS);

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

        var mecanumCommand = new MecanumControllerCommand(
                exampleTrajectory,
                robotDrive::getPose,
                Constants.DriveConstants.FEEDFORWARD,
                Constants.DriveConstants.DRIVE_KINEMATICS,

                // Position controllers
                new PIDController(Constants.AutoConstants.PX_CONTROLLER, 0, 0),
                new PIDController(Constants.AutoConstants.PY_CONTROLLER, 0, 0),
                new ProfiledPIDController(
                        Constants.AutoConstants.P_THETA_CONTROLLER, 0, 0, Constants.AutoConstants.THETA_CONTROLLER_CONSTRAINTS),

                // Needed for normalizing wheel speeds
                Constants.AutoConstants.MAX_SPEED_METERS_PER_SECOND,

                // Velocity PID's
                new PIDController(Constants.DriveConstants.P_FRONT_LEFT_VEL, 0, 0),
                new PIDController(Constants.DriveConstants.P_REAR_LEFT_VEL, 0, 0),
                new PIDController(Constants.DriveConstants.P_FRONT_RIGHT_VEL, 0, 0),
                new PIDController(Constants.DriveConstants.P_REAR_RIGHT_VEL, 0, 0),
                robotDrive::getCurrentWheelSpeeds,
                robotDrive::setDriveMotorControllersVolts, // Consumer for the output motor voltages
                robotDrive);

        robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

        return mecanumCommand.andThen(() -> robotDrive.drive(0, 0, 0, false));
    }
}
