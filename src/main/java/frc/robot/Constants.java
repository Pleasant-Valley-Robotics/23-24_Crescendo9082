// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
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
    public static final class MotorPorts {
        // Motor ports
        public static final int FRONT_LEFT_MOTOR_PORT = 1;
        public static final int FRONT_RIGHT_MOTOR_PORT = 2;

        public static final int REAR_LEFT_MOTOR_PORT = 3;
        public static final int REAR_RIGHT_MOTOR_PORT = 4;

        public static final int INTAKE_TOP_MOTOR_PORT = 5;
        public static final int INTAKE_BOTTOM_MOTOR_PORT = 6;

        public static final int SHOOTER_TOP_MOTOR_PORT = 7;
        public static final int SHOOTER_BOTTOM_MOTOR_PORT = 8;

        public static final int ARM_LEFT_MOTOR_PORT = 9;
        public static final int ARM_RIGHT_MOTOR_PORT = 10;

        public static final int LEFT_HANGING_MOTOR_PORT = 11;
        public static final int RIGHT_HANGING_MOTOR_PORT = 12;
    }

    public static final class PhysicalConstants {
        // Physical specifications

        // Multiplying by .0254: inch -> meter
        public static final double WHEEL_SEP_X_METER = 20.25 * .0254; // Distance between centers of front and back wheels on robot
        public static final double WHEEL_SEP_Y_METER = 23 * .0254; // Distance between centers of right and left wheels on robot

        public static final MecanumDriveKinematics DRIVE_KINEMATICS = new MecanumDriveKinematics(new Translation2d(WHEEL_SEP_X_METER / 2, WHEEL_SEP_Y_METER / 2), new Translation2d(WHEEL_SEP_X_METER / 2, -WHEEL_SEP_Y_METER / 2), new Translation2d(-WHEEL_SEP_X_METER / 2, WHEEL_SEP_Y_METER / 2), new Translation2d(-WHEEL_SEP_X_METER / 2, -WHEEL_SEP_Y_METER / 2));

        // This value might need divided by 4 if this is meant to be "pulses per revolution" and not "Counts per revolution"
        public static final int ENCODER_CPR = 42; // https://www.andymark.com/products/neo-1-1-brushless-motor
        public static final double WHEEL_DIAMETER_INCHES = 5.97;
        public static final double WHEEL_DIAMETER_METERS = WHEEL_DIAMETER_INCHES * 0.0254;
        public static final double DRIVE_GEAR_REDUCTION = 5.95; //https://www.andymark.com/products/toughbox-micro-classic
        // Assumes the encoders are directly mounted on the wheel shafts
        public static final double ENCODER_DISTANCE_PER_PULSE = (WHEEL_DIAMETER_METERS * Math.PI) / (DRIVE_GEAR_REDUCTION);
    }

    public static final class OIConstants {
        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final int DRIVER_CONTROLLER_PORT_2 = 1;
    }

    public static final class AutoConstants {
        public static final SimpleMotorFeedforward FEEDFORWARD = new SimpleMotorFeedforward(0, 0.13872, 0.025702); //Tuned by Brandon Seamer via SysID on tests 2/8/2024

        // Example value only - as above, this must be tuned for your drive!
        public static final double P_FRONT_LEFT_VEL = 0;   //Tuned by Brandon Seamer via SysID on tests 2/8/2024 values approached 0 when vel error < 1m/s
        public static final double P_REAR_LEFT_VEL = 0;    //Tuned by Brandon Seamer via SysID on tests 2/8/2024 values approached 0 when vel error < 1m/s
        public static final double P_FRONT_RIGHT_VEL = 0;  //Tuned by Brandon Seamer via SysID on tests 2/8/2024 values approached 0 when vel error < 1m/s
        public static final double P_REAR_RIGHT_VEL = 0;   //Tuned by Brandon Seamer via SysID on tests 2/8/2024 values approached 0 when vel error < 1m/s

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
        public static final TrapezoidProfile.Constraints THETA_CONTROLLER_CONSTRAINTS = new TrapezoidProfile.Constraints(MAX_ANGULAR_SPEED_RADIANS_PER_SECOND, MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED);
    }

    // as for apriltags, this is all you need
    public static final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

    public static final class IntakeConstants {
        // PID controller constants
        public static final double K_P = 0, K_I = 0, K_D = 0;
        // Feedforward controller constants
        public static final double K_S = 0, K_V = 0;

        public static final PIDController INTAKE_PID = new PIDController(K_P, K_I, K_D);
        public static final SimpleMotorFeedforward INTAKE_FEEDFORWARD = new SimpleMotorFeedforward(K_S, K_V);

        public static final double INTAKE_RPM_CONVERSION = 26.0 / 18;
    }

    public static final class ShooterConstants {
        // PID controller constants
        public static final double K_P = 0, K_I = 0, K_D = 0;

        // Feedforward controller constants
        public static final double K_S = 0, K_V = 0;

        public static final PIDController SHOOTER_PID = new PIDController(K_P, K_I, K_D);
        public static final SimpleMotorFeedforward SHOOTER_FEEDFORWARD = new SimpleMotorFeedforward(K_S, K_V);

        public static final double SHOOTER_RPM_CONVERSION = 26.0 / 18;
    }

    public static final class ArmConstants {
        // nothing here yet...
    }

    public static final class HangingConstants {
        public static final double ARM_SPEED = 0.3;
    }
}
