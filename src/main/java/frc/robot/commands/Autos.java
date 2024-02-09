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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.MecanumControllerCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import java.util.List;

public class Autos {
        public static Command simpleAuto(DriveSubsystem robotDrive) {
                // Define a PID Controller for controlling our X position
                PIDController positionXController = new PIDController(Constants.AutoConstants.PX_CONTROLLER, 0, Constants.AutoConstants.DX_CONTROLLER);
                positionXController.setTolerance(0.05); //Tolerance in meters, 0.01 being one centimeter

                // Define a PID Controller for controlling our Y position
                PIDController positionYController = new PIDController(Constants.AutoConstants.PY_CONTROLLER, 0, Constants.AutoConstants.DY_CONTROLLER);
                positionYController.setTolerance(0.05); //Tolerance in meters, 0.01 being one centimeter

                // Define a profiled PID controller for controlling the theta (rotation) of our robot
                ProfiledPIDController positionThetaController = new ProfiledPIDController(
                                                Constants.AutoConstants.P_THETA_CONTROLLER, 0, Constants.AutoConstants.D_THETA_CONTROLLER,
                                                Constants.AutoConstants.THETA_CONTROLLER_CONSTRAINTS);
                positionThetaController.setTolerance(0.1); // <----- This will need to be adjusted, Brandon doesn't really understand exactly what this translates to on the physical robot yet.
                
                PIDController velocityFLController = new PIDController(Constants.DriveConstants.P_FRONT_LEFT_VEL, 0, 0);
                PIDController velocityBLController = new PIDController(Constants.DriveConstants.P_REAR_LEFT_VEL, 0, 0);               
                PIDController velocityFRController = new PIDController(Constants.DriveConstants.P_FRONT_RIGHT_VEL, 0, 0);
                PIDController velocityBRController = new PIDController(Constants.DriveConstants.P_REAR_RIGHT_VEL, 0, 0);                

                // Create the configuration for our generated trajectories to follow
                TrajectoryConfig config = new TrajectoryConfig(
                                Constants.AutoConstants.MAX_SPEED_METERS_PER_SECOND,
                                Constants.AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED)
                                // Add kinematics to ensure max speed is actually obeyed
                                .setKinematics(Constants.DriveConstants.DRIVE_KINEMATICS);
                // An example trajectory to follow. All units in meters.
                Trajectory backwardsSCurveTrajectory = TrajectoryGenerator.generateTrajectory(
                        // Define start point
                        new Pose2d(0, 0, new Rotation2d(0)),
                        
                        // Define mid points
                        List.of(new Translation2d(1, 0), new Translation2d(1, -1), 
                        new Translation2d(2,-1), new Translation2d(2,0), new Translation2d(2,1)),

                        //Define end point
                        new Pose2d(3, 1, new Rotation2d(Math.PI)),
                        config);

                var mecanumCommand = new MecanumControllerCommand(
                                backwardsSCurveTrajectory,
                                robotDrive::getPose,
                                Constants.DriveConstants.FEEDFORWARD,
                                Constants.DriveConstants.DRIVE_KINEMATICS,

                                // Position controllers (Meters)
                                positionXController,
                                positionYController,
                                positionThetaController,

                                // Needed for normalizing wheel speeds
                                Constants.AutoConstants.MAX_SPEED_METERS_PER_SECOND,

                                // Velocity PID's (Meters/Second)
                                velocityFLController,
                                velocityBLController,
                                velocityFRController,
                                velocityBRController,
                                robotDrive::getCurrentWheelSpeeds,
                                robotDrive::setDriveMotorControllersVolts, // Consumer for the output motor voltages
                                robotDrive);

                robotDrive.resetOdometry(backwardsSCurveTrajectory.getInitialPose());

                return new SequentialCommandGroup(
                                new InstantCommand(() -> robotDrive.setMotorSafetyTimeout(30)),
                                mecanumCommand,
                                new InstantCommand(() -> robotDrive.drive(0, 0, 0, false)),
                                new InstantCommand(() -> robotDrive.setMotorSafetyTimeout(.1)));
        }
}
