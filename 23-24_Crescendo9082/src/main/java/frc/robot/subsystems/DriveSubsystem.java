// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.MecanumDriveMotorVoltages;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {
    private final CANSparkMax frontLeft = new CANSparkMax(DriveConstants.FRONT_LEFT_MOTOR_PORT, MotorType.kBrushless);
    private final CANSparkMax frontRight = new CANSparkMax(DriveConstants.FRONT_RIGHT_MOTOR_PORT, MotorType.kBrushless);
    private final CANSparkMax rearLeft = new CANSparkMax(DriveConstants.REAR_LEFT_MOTOR_PORT, MotorType.kBrushless);
    private final CANSparkMax rearRight = new CANSparkMax(DriveConstants.REAR_RIGHT_MOTOR_PORT, MotorType.kBrushless);
    private final RelativeEncoder frontLeftEncoder = frontLeft.getEncoder();
    private final RelativeEncoder frontRightEncoder = frontRight.getEncoder();
    private final RelativeEncoder rearLeftEncoder = rearLeft.getEncoder();
    private final RelativeEncoder rearRightEncoder = rearRight.getEncoder();
    private final MecanumDrive drive =
            new MecanumDrive(frontLeft, rearLeft, frontRight, rearRight);


    // The gyro sensor
    private final Gyro gyro = new ADXRS450_Gyro();

    // Odometry class for tracking robot pose
    MecanumDriveOdometry odometry =
            new MecanumDriveOdometry(
                    DriveConstants.DRIVE_KINEMATICS,
                    gyro.getRotation2d(),
                    new MecanumDriveWheelPositions());

    /**
     * Creates a new DriveSubsystem.
     */
    public DriveSubsystem() {
        // Sets the distance per pulse for the encoders
        frontLeftEncoder.setPositionConversionFactor(DriveConstants.ENCODER_DISTANCE_PER_PULSE);
        frontRightEncoder.setPositionConversionFactor(DriveConstants.ENCODER_DISTANCE_PER_PULSE);
        rearLeftEncoder.setPositionConversionFactor(DriveConstants.ENCODER_DISTANCE_PER_PULSE);
        rearRightEncoder.setPositionConversionFactor(DriveConstants.ENCODER_DISTANCE_PER_PULSE);
        // We need to invert one side of the drivetrain so that positive voltages
        // result in both sides moving forward. Depending on how your robot's
        // gearbox is constructed, you might have to invert the left side instead.
        frontRight.setInverted(true);
        rearRight.setInverted(true);
    }

    @Override
    public void periodic() {
        // Update the odometry in the periodic block
        odometry.update(gyro.getRotation2d(), getCurrentWheelDistances());
    }

    /**
     * Returns the currently-estimated pose of the robot.
     *
     * @return The pose.
     */
    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(gyro.getRotation2d(), getCurrentWheelDistances(), pose);
    }

    /**
     * Drives the robot at given x, y and theta speeds. Speeds range from [-1, 1] and the linear
     * speeds have no effect on the angular speed.
     *
     * @param xSpeed        Speed of the robot in the x direction (forward/backwards).
     * @param ySpeed        Speed of the robot in the y direction (sideways).
     * @param rot           Angular rate of the robot.
     * @param fieldRelative Whether the provided x and y speeds are relative to the field.
     */
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        if (fieldRelative) {
            drive.driveCartesian(xSpeed, ySpeed, rot, gyro.getRotation2d());
        } else {
            drive.driveCartesian(xSpeed, ySpeed, rot);
        }
    }

    /**
     * Sets the front left drive MotorController to a voltage.
     */
    public void setDriveMotorControllersVolts(MecanumDriveMotorVoltages volts) {
        frontLeft.setVoltage(volts.frontLeftVoltage);
        frontRight.setVoltage(volts.frontRightVoltage);
        rearLeft.setVoltage(volts.rearLeftVoltage);
        rearRight.setVoltage(volts.rearRightVoltage);
    }

    /**
     * Resets the drive encoders to currently read a position of 0.
     */
    public void resetEncoders() {
        frontLeftEncoder.setPosition(0);
        frontRightEncoder.setPosition(0);
        rearLeftEncoder.setPosition(0);
        rearRightEncoder.setPosition(0);
    }

    /**
     * Gets the front left drive encoder.
     *
     * @return the front left drive encoder
     */
    public RelativeEncoder getFrontLeftEncoder() {
        return frontLeftEncoder;
    }

    /**
     * Gets the rear left drive encoder.
     *
     * @return the rear left drive encoder
     */
    public RelativeEncoder getRearLeftEncoder() {
        return rearLeftEncoder;
    }

    /**
     * Gets the front right drive encoder.
     *
     * @return the front right drive encoder
     */
    public RelativeEncoder getFrontRightEncoder() {
        return frontRightEncoder;
    }

    /**
     * Gets the rear right drive encoder.
     *
     * @return the rear right encoder
     */
    public RelativeEncoder getRearRightEncoder() {
        return rearRightEncoder;
    }

    /**
     * Gets the current wheel speeds.
     *
     * @return the current wheel speeds in a MecanumDriveWheelSpeeds object.
     */
    public MecanumDriveWheelSpeeds getCurrentWheelSpeeds() {
        return new MecanumDriveWheelSpeeds(
                frontLeftEncoder.getVelocity(),
                frontRightEncoder.getVelocity(),
                rearLeftEncoder.getVelocity(),
                rearRightEncoder.getVelocity());
    }

    /**
     * Gets the current wheel distance measurements.
     *
     * @return the current wheel distance measurements in a MecanumDriveWheelPositions object.
     */
    public MecanumDriveWheelPositions getCurrentWheelDistances() {
        return new MecanumDriveWheelPositions(
                frontLeftEncoder.getPosition(),
                rearLeftEncoder.getPosition(),
                frontRightEncoder.getPosition(),
                rearRightEncoder.getPosition());
    }

    /**
     * Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
     *
     * @param maxOutput the maximum output to which the drive will be constrained
     */
    public void setMaxOutput(double maxOutput) {
        drive.setMaxOutput(maxOutput);
    }

    /**
     * Zeroes the heading of the robot.
     */
    public void zeroHeading() {
        gyro.reset();
    }

    /**
     * Returns the heading of the robot.
     *
     * @return the robot's heading in degrees, from -180 to 180
     */
    public double getHeading() {
        return gyro.getRotation2d().getDegrees();
    }

    /**
     * Returns the turn rate of the robot.
     *
     * @return The turn rate of the robot, in degrees per second
     */
    public double getTurnRate() {
        return -gyro.getRate();
    }
}
