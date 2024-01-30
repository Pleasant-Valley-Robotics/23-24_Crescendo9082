// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    //Motor Controller Ports
    public static final int FRONT_LEFT_MOTOR_PORT = 0;
    public static final int FRONT_RIGHT_MOTOR_PORT = 1;
    public static final int REAR_LEFT_MOTOR_PORT = 2;
    public static final int REAR_RIGHT_MOTOR_PORT = 3;
    public static final int INTAKE_FEED_MOTOR_PORT = 4;
    public static final int INTAKE_ARM_MOTOR_PORT =5;
    public static final int LEFT_HANGING_MOTOR_PORT = 6;
    public static final int RIGHT_HANGING_MOTOR_PORT = 7;
    public static final int UPPER_LEFT_SHOOTER_MOTOR_PORT = 8;
    public static final int LOWER_LEFT_SHOOTER_MOTOR_PORT = 9;
    public static final int UPPER_RIGHT_SHOOTER_MOTOR_PORT = 10;
    public static final int LOWER_RIGHT_SHOOTER_MOTOR_PORT = 11;
    public static final int SHOOTER_PIVOT_PORT = 12;

    // Distance between centers of right and left wheels on robot
    public static final double TRACK_WIDTH = 0.5;
    
    // Distance between centers of front and back wheels on robot
    public static final double WHEEL_BASE = 0.7;

    //Mecanum Drive Kinematics Constant Calculations
    public static final MecanumDriveKinematics DRIVE_KINEMATICS =
        new MecanumDriveKinematics(
            new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),
            new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),
            new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2),
            new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2));

    public static final int ENCODER_CPR = 42; //This value might need divided by 4 if this is meant to be "pulses per revolution" and not "Counts per revolution" https://www.andymark.com/products/neo-1-1-brushless-motor?via=Z2lkOi8vYW5keW1hcmsvV29ya2FyZWE6OkNhdGFsb2c6OkNhdGVnb3J5LzViYjYxOGI0YmM2ZjZkNmRlMWU2OWZkZg
    public static final double WHEEL_DIAMETER_INCHES = 5.97;
    public static final double WHEEL_DIAMETER_METERS = WHEEL_DIAMETER_INCHES * 0.0254;
    public static final double DRIVE_GEAR_REDUCTION = 5.95; //https://www.andymark.com/products/toughbox-micro-classic
    public static final double ENCODER_DISTANCE_PER_PULSE =
        // Assumes the encoders are directly mounted on the wheel shafts
        (WHEEL_DIAMETER_METERS * Math.PI) / ((double) ENCODER_CPR * DRIVE_GEAR_REDUCTION);

    // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
    // These characterization values MUST be determined either experimentally or theoretically
    // for *your* robot's drive.
    // The SysId tool provides a convenient method for obtaining these values for your robot.
    public static final SimpleMotorFeedforward FEEDFORWARD =
        new SimpleMotorFeedforward(1, 0.8, 0.15);

    // Example value only - as above, this must be tuned for your drive!
    public static final double P_FRONT_LEFT_VEL = 0.5;
    public static final double P_REAR_LEFT_VEL = 0.5;
    public static final double P_FRONT_RIGHT_VEL = 0.5;
    public static final double P_REAR_RIGHT_VEL = 0.5;
  }

  public static final class OIConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int DRIVER_CONTROLLER_PORT_2 = 1;
  }

  public static final class AutoConstants {
    public static final double MAX_SPEED_METERS_PER_SECOND = 3;
    public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 3;
    public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = Math.PI;
    public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED = Math.PI;

    public static final double PX_CONTROLLER = 0.5;
    public static final double PY_CONTROLLER = 0.5;
    public static final double P_THETA_CONTROLLER = 0.5;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints THETA_CONTROLLER_CONSTRAINTS =
        new TrapezoidProfile.Constraints(
                MAX_ANGULAR_SPEED_RADIANS_PER_SECOND, MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED);
  }
}
