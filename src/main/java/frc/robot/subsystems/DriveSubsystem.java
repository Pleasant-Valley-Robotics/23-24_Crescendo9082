// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkRelativeEncoder.Type;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.MecanumDriveMotorVoltages;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.MotorPorts.*;
import static frc.robot.Constants.PhysicalConstants.*;
import static frc.robot.ShuffleboardContainer.robotData;

public class DriveSubsystem extends SubsystemBase {
    private final CANSparkMax FLDrive = new CANSparkMax(FRONT_LEFT_MOTOR_PORT, MotorType.kBrushless);
    private final CANSparkMax FRDrive = new CANSparkMax(FRONT_RIGHT_MOTOR_PORT, MotorType.kBrushless);
    private final CANSparkMax BLDrive = new CANSparkMax(REAR_LEFT_MOTOR_PORT, MotorType.kBrushless);
    private final CANSparkMax BRDrive = new CANSparkMax(REAR_RIGHT_MOTOR_PORT, MotorType.kBrushless);

    private final RelativeEncoder frontLeftEncoder = FLDrive.getEncoder(Type.kHallSensor, ENCODER_CPR);
    private final RelativeEncoder frontRightEncoder = FRDrive.getEncoder(Type.kHallSensor, ENCODER_CPR);
    private final RelativeEncoder backLeftEncoder = BLDrive.getEncoder(Type.kHallSensor, ENCODER_CPR);
    private final RelativeEncoder backRightEncoder = BRDrive.getEncoder(Type.kHallSensor, ENCODER_CPR);

    private final MecanumDrive drive = new MecanumDrive(FLDrive, BLDrive, FRDrive, BRDrive);


    // The gyro sensor
    private final AHRS gyro = new AHRS();

    // Odometry class for tracking robot pose
    MecanumDriveOdometry odometry = new MecanumDriveOdometry(DRIVE_KINEMATICS, gyro.getRotation2d(), new MecanumDriveWheelPositions());

    /**
     * Creates a new DriveSubsystem.
     */
    public DriveSubsystem() {
        // Sets the distance per pulse for the encoders
        frontLeftEncoder.setPositionConversionFactor(DRIVE_ENCODER_DISTANCE_PER_PULSE);
        frontRightEncoder.setPositionConversionFactor(DRIVE_ENCODER_DISTANCE_PER_PULSE);
        backLeftEncoder.setPositionConversionFactor(DRIVE_ENCODER_DISTANCE_PER_PULSE);
        backRightEncoder.setPositionConversionFactor(DRIVE_ENCODER_DISTANCE_PER_PULSE);

        frontLeftEncoder.setVelocityConversionFactor(DRIVE_ENCODER_DISTANCE_PER_PULSE / 60);
        frontRightEncoder.setVelocityConversionFactor(DRIVE_ENCODER_DISTANCE_PER_PULSE / 60);
        backLeftEncoder.setVelocityConversionFactor(DRIVE_ENCODER_DISTANCE_PER_PULSE / 60);
        backRightEncoder.setVelocityConversionFactor(DRIVE_ENCODER_DISTANCE_PER_PULSE / 60);

        FLDrive.setIdleMode(CANSparkBase.IdleMode.kBrake);
        FRDrive.setIdleMode(CANSparkBase.IdleMode.kBrake);
        BLDrive.setIdleMode(CANSparkBase.IdleMode.kBrake);
        BRDrive.setIdleMode(CANSparkBase.IdleMode.kBrake);
        // We need to invert one side of the drivetrain so that positive voltages
        // result in both sides moving forward. Depending on how your robot's
        // gearbox is constructed, you might have to invert the left side instead.
        FRDrive.setInverted(true);
        BRDrive.setInverted(true);
        // BLDrive.setInverted(true);

        robotData.add(drive);
        robotData.add(gyro);
        robotData.addDouble("robot x", () -> getPose().getX());
        robotData.addDouble("robot y", () -> getPose().getY());
        robotData.addDouble("robot rotation", () -> getPose().getRotation().getDegrees());

//        robotData.addDouble("fl velocity", () -> getCurrentWheelSpeeds().frontLeftMetersPerSecond);
//        robotData.addDouble("fr velocity", () -> getCurrentWheelSpeeds().frontRightMetersPerSecond);
//        robotData.addDouble("bl velocity", () -> getCurrentWheelSpeeds().rearLeftMetersPerSecond);
//        robotData.addDouble("br velocity", () -> getCurrentWheelSpeeds().rearRightMetersPerSecond);
//        robotData.addDoubleArray("wheel voltages", () );
    }

    public MecanumDriveMotorVoltages getAppliedVoltages() {
        return new MecanumDriveMotorVoltages(
                FLDrive.getAppliedOutput(),
                FRDrive.getAppliedOutput(),
                BLDrive.getAppliedOutput(),
                BRDrive.getAppliedOutput()
        );
    }

    public void disableMotorSafetyTimeout() {
        drive.setSafetyEnabled(false);
    }

    public void enableMotorSafetyTimeout() {
        drive.setSafetyEnabled(true);
    }

    public void setMotorSafetyTimeout(double seconds) {
        drive.setExpiration(seconds);
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
        FLDrive.setVoltage(volts.frontLeftVoltage);
        FRDrive.setVoltage(volts.frontRightVoltage);
        BLDrive.setVoltage(volts.rearLeftVoltage);
        BRDrive.setVoltage(volts.rearRightVoltage);
    }

    /**
     * Resets the drive encoders to currently read a position of 0.
     */
    public void resetEncoders() {
        frontLeftEncoder.setPosition(0);
        frontRightEncoder.setPosition(0);
        backLeftEncoder.setPosition(0);
        backRightEncoder.setPosition(0);
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
    public RelativeEncoder getBackLeftEncoder() {
        return backLeftEncoder;
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
    public RelativeEncoder getBackRightEncoder() {
        return backRightEncoder;
    }

    /**
     * Gets the current wheel speeds.
     *
     * @return the current wheel speeds in a MecanumDriveWheelSpeeds object.
     */
    public MecanumDriveWheelSpeeds getCurrentWheelSpeeds() {
        return new MecanumDriveWheelSpeeds(frontLeftEncoder.getVelocity(), frontRightEncoder.getVelocity(), backLeftEncoder.getVelocity(), backRightEncoder.getVelocity());
    }

    /**
     * Gets the current wheel distance measurements.
     *
     * @return the current wheel distance measurements in a MecanumDriveWheelPositions object.
     */
    public MecanumDriveWheelPositions getCurrentWheelDistances() {
        return new MecanumDriveWheelPositions(frontLeftEncoder.getPosition(), backLeftEncoder.getPosition(), frontRightEncoder.getPosition(), backRightEncoder.getPosition());
    }

    /**
     * Calculates the average encoder value.
     *
     * @return the average encoder value.
     */
    public double getAverageEncoders() {
        double AverageEncoders = ((Math.abs(frontLeftEncoder.getPosition()) + Math.abs(backLeftEncoder.getPosition()) + Math.abs(frontRightEncoder.getPosition()) + Math.abs(backRightEncoder.getPosition())) / 4);
        return AverageEncoders;
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

    private final MutableMeasure<Voltage> volts = MutableMeasure.mutable(Volts.of(0));

    private final MutableMeasure<Distance> distance = MutableMeasure.mutable(Meters.of(0));

    private final MutableMeasure<Velocity<Distance>> velocity = MutableMeasure.mutable(MetersPerSecond.of(0));

    public final SysIdRoutine routine = new SysIdRoutine(new SysIdRoutine.Config(), new SysIdRoutine.Mechanism(
            volts -> this.setDriveMotorControllersVolts(new MecanumDriveMotorVoltages(volts.in(Volts), volts.in(Volts), volts.in(Volts), volts.in(Volts))),
            log -> {
                var voltages = this.getAppliedVoltages();
                log.motor("FLDrive")
                        .voltage(volts.mut_replace(voltages.frontLeftVoltage, Volts))
                        .linearPosition(distance.mut_replace(this.getFrontLeftEncoder().getPosition(), Meters))
                        .linearVelocity(velocity.mut_replace(this.getFrontLeftEncoder().getVelocity(), MetersPerSecond));
                log.motor("FRDrive")
                        .voltage(volts.mut_replace(voltages.frontRightVoltage, Volts))
                        .linearPosition(distance.mut_replace(this.getFrontRightEncoder().getPosition(), Meters))
                        .linearVelocity(velocity.mut_replace(this.getFrontRightEncoder().getVelocity(), MetersPerSecond));
                log.motor("BLDrive")
                        .voltage(volts.mut_replace(voltages.rearLeftVoltage, Volts))
                        .linearPosition(distance.mut_replace(this.getBackLeftEncoder().getPosition(), Meters))
                        .linearVelocity(velocity.mut_replace(this.getBackLeftEncoder().getVelocity(), MetersPerSecond));
                log.motor("BRDrive")
                        .voltage(volts.mut_replace(voltages.rearRightVoltage, Volts))
                        .linearPosition(distance.mut_replace(this.getBackRightEncoder().getPosition(), Meters))
                        .linearVelocity(velocity.mut_replace(this.getBackRightEncoder().getVelocity(), MetersPerSecond));
            }, this));
}
