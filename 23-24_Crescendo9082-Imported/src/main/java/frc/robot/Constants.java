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
    public static final int kFrontLeftMotorPort = 1;
    public static final int kFrontRightMotorPort = 2;
    public static final int kRearLeftMotorPort = 3;
    public static final int kRearRightMotorPort = 4;
    public static final int kIntakeFeedMotorPort = 5;
    public static final int kIntakeArmMotorPort = 6;
    public static final int kLeftHangingMotorPort = 7;
    public static final int kRightHangingMotorPort = 8;
    public static final int kUpperLeftShooterMotorPort = 9;
    public static final int kLowerLeftShooterMotorPort = 10;
    public static final int kUpperRightShooterMotorPort = 11;
    public static final int kLowerRightShooterMotorPort = 12;
    public static final int kShooterPivotPort = 13;

    // Distance between centers of right and left wheels on robot
    public static final double kTrackWidth = 22.98 * 0.0254;
    
    // Distance between centers of front and back wheels on robot
    public static final double kWheelBase = 20.75 * 0.0254;

    //Mecanum Drive Kinematics Constant Calculations
    public static final MecanumDriveKinematics kDriveKinematics =
        new MecanumDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    public static final int kEncoderCPR = 42; //This value might need divided by 4 if this is meant to be "pulses per revolution" and not "Counts per revolution" https://www.andymark.com/products/neo-1-1-brushless-motor?via=Z2lkOi8vYW5keW1hcmsvV29ya2FyZWE6OkNhdGFsb2c6OkNhdGVnb3J5LzViYjYxOGI0YmM2ZjZkNmRlMWU2OWZkZg
    public static final double kWheelDiameterInches = 5.97;
    public static final double kWheelDiameterMeters = kWheelDiameterInches * 0.0254;
    public static final double kdriveGearReduction = 5.95; //https://www.andymark.com/products/toughbox-micro-classic
    public static final double kEncoderDistancePerPulse =
        // Assumes the encoders are directly mounted on the wheel shafts
        (kWheelDiameterMeters * Math.PI) / ((double) kEncoderCPR * kdriveGearReduction);

    // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
    // These characterization values MUST be determined either experimentally or theoretically
    // for *your* robot's drive.
    // The SysId tool provides a convenient method for obtaining these values for your robot.
    public static final SimpleMotorFeedforward kFeedforward =
        new SimpleMotorFeedforward(1, 0.8, 0.15);

    // Example value only - as above, this must be tuned for your drive!
    public static final double kPFrontLeftVel = 0.5;
    public static final double kPRearLeftVel = 0.5;
    public static final double kPFrontRightVel = 0.5;
    public static final double kPRearRightVel = 0.5;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kDriverControllerPort2 = 1;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 0.5;
    public static final double kPYController = 0.5;
    public static final double kPThetaController = 0.5;

    // Constraint for the motion profilied robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }
}
