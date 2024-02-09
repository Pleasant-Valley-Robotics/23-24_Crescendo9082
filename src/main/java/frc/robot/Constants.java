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
        public static final int FRONT_LEFT_MOTOR_PORT = 5;
        public static final int FRONT_RIGHT_MOTOR_PORT = 2;
        public static final int REAR_LEFT_MOTOR_PORT = 3;
        public static final int REAR_RIGHT_MOTOR_PORT = 4;
        public static final int INTAKE_FEED_MOTOR_PORT = 14;
        public static final int INTAKE_ARM_MOTOR_PORT = 6;
        public static final int LEFT_HANGING_MOTOR_PORT = 7;
        public static final int RIGHT_HANGING_MOTOR_PORT = 8;
        public static final int UPPER_LEFT_SHOOTER_MOTOR_PORT = 9;
        public static final int LOWER_LEFT_SHOOTER_MOTOR_PORT = 10;
        public static final int UPPER_RIGHT_SHOOTER_MOTOR_PORT = 11;
        public static final int LOWER_RIGHT_SHOOTER_MOTOR_PORT = 12;
        public static final int SHOOTER_PIVOT_PORT = 13;

        // Distance between centers of right and left wheels on robot
        public static final double TRACK_WIDTH = 23 * .0254;

        // Distance between centers of front and back wheels on robzot
        public static final double WHEEL_BASE = 20.25 * .0254;

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
        // Assumes the encoders are directly mounted on the wheel shafts
        public static final double ENCODER_DISTANCE_PER_PULSE = (WHEEL_DIAMETER_METERS * Math.PI) / (DRIVE_GEAR_REDUCTION);
        public static final double ENCODER_DISTANCE_PER_PULSE_VEL = (WHEEL_DIAMETER_METERS * Math.PI) / (DRIVE_GEAR_REDUCTION * 60);

        // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
        // These characterization values MUST be determined either experimentally or theoretically
        // for *your* robot's drive.
        // The SysId tool provides a convenient method for obtaining these values for your robot.
        public static final SimpleMotorFeedforward FEEDFORWARD = new SimpleMotorFeedforward(0, 0.13872, 0.025702); //Tuned by Brandon Seamer via SysID on tests 2/8/2024

        // Example value only - as above, this must be tuned for your drive!
        public static final double P_FRONT_LEFT_VEL = 0;   //Tuned by Brandon Seamer via SysID on tests 2/8/2024 values approached 0 when vel error < 1m/s
        public static final double P_REAR_LEFT_VEL = 0;    //Tuned by Brandon Seamer via SysID on tests 2/8/2024 values approached 0 when vel error < 1m/s
        public static final double P_FRONT_RIGHT_VEL = 0;  //Tuned by Brandon Seamer via SysID on tests 2/8/2024 values approached 0 when vel error < 1m/s
        public static final double P_REAR_RIGHT_VEL = 0;   //Tuned by Brandon Seamer via SysID on tests 2/8/2024 values approached 0 when vel error < 1m/s
    }

    public static final class OIConstants {
        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final int DRIVER_CONTROLLER_PORT_2 = 1;
    }

    public static final class AutoConstants {
        public static final double kAutoDriveDistanceInches = 12;
        public static final double kAutoDriveSpeed = .1;
        public static final double MAX_SPEED_METERS_PER_SECOND = 2;
        public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 1;
        public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = Math.PI;
        public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED = Math.PI;

        public static final double PX_CONTROLLER = 11.201; // Volts per Meter    //Tuned by Brandon Seamer via SysID on tests 2/8/2024 with (0.1 pos error | 1 vel error)
        public static final double PY_CONTROLLER = 11.201; // Volts per Meter    //Tuned by Brandon Seamer via SysID on tests 2/8/2024 with (0.1 pos error | 1 vel error)
        public static final double P_THETA_CONTROLLER = 11.201;    //Volts per radian    //Tuned by Brandon Seamer via SysID on tests 2/8/2024 this parts fuzzy
        public static final double DX_CONTROLLER = 1.2116;   //Tuned by Brandon Seamer via SysID on tests 2/8/2024 with (0.1 pos error | 1 vel error)
        public static final double DY_CONTROLLER = 1.2116;   //Tuned by Brandon Seamer via SysID on tests 2/8/2024 with (0.1 pos error | 1 vel error)
        public static final double D_THETA_CONTROLLER = 1.2116;  //Tuned by Brandon Seamer via SysID on tests 2/8/2024 this parts fuzzy

        // Constraint for the motion profiled robot angle controller
        public static final TrapezoidProfile.Constraints THETA_CONTROLLER_CONSTRAINTS =
                new TrapezoidProfile.Constraints(
                        MAX_ANGULAR_SPEED_RADIANS_PER_SECOND, MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED);
    }

    public static final class AprilTagCords {
        public static final double aprilTag1X = 593.68; // Cords for april tag 1's X measured in inches
        public static final double aprilTag1Y = 9.68; // Cords for april tag 1's Y measured in inches
        public static final double aprilTag1Z = 53.38; // Cords for april tag 1's Z measured in inches
        public static final double aprilTag1Rotation = 120; // Cords for april tag 1's Rotation measured in degrees

        public static final double aprilTag2X = 637.21; // Cords for april tag 2's X measured in inches
        public static final double aprilTag2Y = 34.79; // Cords for april tag 2's Y measured in inches
        public static final double aprilTag2Z = 53.38; // Cords for april tag 2's Z measured in inches
        public static final double aprilTag2Rotation = 120; // Cords for april tag 2's Rotation measured in degrees

        public static final double aprilTag3X = 652.73; // Cords for april tag 3's X measured in inches
        public static final double aprilTag3Y = 196.17; // Cords for april tag 3's Y measured in inches
        public static final double aprilTag3Z = 57.13; // Cords for april tag 3's Z measured in inches
        public static final double aprilTag3Rotation = 180; // Cords for april tag 3's Rotation measured in degrees

        public static final double aprilTag4X = 652.73; // Cords for april tag 4's X measured in inches
        public static final double aprilTag4Y = 218.42; // Cords for april tag 4's Y measured in inches
        public static final double aprilTag4Z = 57.13; // Cords for april tag 4's Z measured in inches
        public static final double aprilTag4Rotation = 180; // Cords for april tag 4's Rotation measured in degrees

        public static final double aprilTag5X = 578.77; // Cords for april tag 5's X measured in inches
        public static final double aprilTag5Y = 323.00; // Cords for april tag 5's Y measured in inches
        public static final double aprilTag5Z = 53.38; // Cords for april tag 5's Z measured in inches
        public static final double aprilTag5Rotation = 270; // Cords for april tag 5's Rotation measured in degrees

        public static final double aprilTag6X = 72.5; // Cords for april tag 6's X measured in inches
        public static final double aprilTag6Y = 323.00; // Cords for april tag 6's Y measured in inches
        public static final double aprilTag6Z = 53.38; // Cords for april tag 6's Z measured in inches
        public static final double aprilTag6Rotation = 270; // Cords for april tag 6's Rotation measured in degrees

        public static final double aprilTag7X = -1.50; // Cords for april tag 7's X measured in inches
        public static final double aprilTag7Y = 218.42; // Cords for april tag 7's Y measured in inches
        public static final double aprilTag7Z = 57.13; // Cords for april tag 7's Z measured in inches
        public static final double aprilTag7Rotation = 0; // Cords for april tag 7's Rotation measured in degrees

        public static final double aprilTag8X = -1.50; // Cords for april tag 8's X measured in inches
        public static final double aprilTag8Y = 196.17; // Cords for april tag 8's Y measured in inches
        public static final double aprilTag8Z = 57.13; // Cords for april tag 8's Z measured in inches
        public static final double aprilTag8Rotation = 0; // Cords for april tag 8's Rotation measured in degrees

        public static final double aprilTag9X = 14.02; // Cords for april tag 9's X measured in inches
        public static final double aprilTag9Y = 34.79; // Cords for april tag 9's Y measured in inches
        public static final double aprilTag9Z = 53.38; // Cords for april tag 9's Z measured in inches
        public static final double aprilTag9Rotation = 60; // Cords for april tag 9's Rotation measured in degrees

        public static final double aprilTag10X = 57.54; // Cords for april tag 10's X measured in inches
        public static final double aprilTag10Y = 9.68; // Cords for april tag 10's Y measured in inches
        public static final double aprilTag10Z = 53.38; // Cords for april tag 10's Z measured in inches
        public static final double aprilTag10Rotation = 60; // Cords for april tag 10's Rotation measured in degrees

        public static final double aprilTag11X = 468.69; // Cords for april tag 11's X measured in inches
        public static final double aprilTag11Y = 146.19; // Cords for april tag 11's Y measured in inches
        public static final double aprilTag11Z = 52.00; // Cords for april tag 11's Z measured in inches
        public static final double aprilTag11Rotation = 300; // Cords for april tag 11's Rotation measured in degrees

        public static final double aprilTag12X = 468.69; // Cords for april tag 12's X measured in inches
        public static final double aprilTag12Y = 177.10; // Cords for april tag 12's Y measured in inches
        public static final double aprilTag12Z = 52.00; // Cords for april tag 12's Z measured in inches
        public static final double aprilTag12Rotation = 60; // Cords for april tag 12's Rotation measured in degrees

        public static final double aprilTag13X = 441.74; // Cords for april tag 13's X measured in inches
        public static final double aprilTag13Y = 161.62; // Cords for april tag 13's Y measured in inches
        public static final double aprilTag13Z = 52.00; // Cords for april tag 13's Z measured in inches
        public static final double aprilTag13Rotation = 180; // Cords for april tag 13's Rotation measured in degrees

        public static final double aprilTag14X = 209.48; // Cords for april tag 14's X measured in inches
        public static final double aprilTag14Y = 161.62; // Cords for april tag 14's Y measured in inches
        public static final double aprilTag14Z = 52.00; // Cords for april tag 14's Z measured in inches
        public static final double aprilTag14Rotation = 0; // Cords for april tag 14's Rotation measured in degrees

        public static final double aprilTag15X = 182.73; // Cords for april tag 15's X measured in inches
        public static final double aprilTag15Y = 177.10; // Cords for april tag 15's Y measured in inches
        public static final double aprilTag15Z = 52.00; // Cords for april tag 15's Z measured in inches
        public static final double aprilTag15Rotation = 120; // Cords for april tag 15's Rotation measured in degrees

        public static final double aprilTag16X = 182.73; // Cords for april tag 16's X measured in inches
        public static final double aprilTag16Y = 146.19; // Cords for april tag 16's Y measured in inches
        public static final double aprilTag16Z = 52.00; // Cords for april tag 16's Z measured in inches
        public static final double aprilTag16Rotation = 240; // Cords for april tag 16's Rotation measured in degrees
    }

    public static final class IntakeConstants{
        public static final double lowerLimit = 0;
        public static final double upperLimit = 100000;
    }

    public static final class ShooterConstants{
        public static final double lowerLimit = 0;
        public static final double upperLimit = 100000;
    }
}
