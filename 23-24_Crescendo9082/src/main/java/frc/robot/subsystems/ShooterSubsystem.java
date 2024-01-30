// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class ShooterSubsystem extends SubsystemBase {
    private final CANSparkMax upperLeftShooter = new CANSparkMax(DriveConstants.UPPER_LEFT_SHOOTER_MOTOR_PORT, MotorType.kBrushless);
    private final CANSparkMax lowerLeftShooter = new CANSparkMax(DriveConstants.LOWER_LEFT_SHOOTER_MOTOR_PORT, MotorType.kBrushless);
    private final CANSparkMax upperRightShooter = new CANSparkMax(DriveConstants.UPPER_RIGHT_SHOOTER_MOTOR_PORT, MotorType.kBrushless);
    private final CANSparkMax lowerRightShooter = new CANSparkMax(DriveConstants.LOWER_RIGHT_SHOOTER_MOTOR_PORT, MotorType.kBrushless);
    private final CANSparkMax shooterArm = new CANSparkMax(DriveConstants.SHOOTER_PIVOT_PORT, MotorType.kBrushless);
    private final RelativeEncoder upperLeftShooterEncoder = upperLeftShooter.getEncoder();
    private final RelativeEncoder lowerLeftShooterEncoder = lowerLeftShooter.getEncoder();
    private final RelativeEncoder upperRightShooterEncoder = upperRightShooter.getEncoder();
    private final RelativeEncoder lowerRightShooterEncoder = lowerRightShooter.getEncoder();


    /**
     * Creates a new DriveSubsystem.
     */
    public ShooterSubsystem() {
        //Put any default values for the subsystem into this constructor.
    }

    @Override
    public void periodic() {
        //Consider putting shooter motor turn rates from the subsystem into telemetry here
    }

    public void shooterArmSpeed(double speed) {
        shooterArm.set(speed * .1);
    }

    public void shooterArmVoltage(double volts) {
        shooterArm.setVoltage(volts);

    }

    /**
     * Drives the shooter subsystem at given speed. Speeds range from [-1, 1].
     *
     * @param speed speed of the shooter motors.
     */
    public void shootSpeaker(double speed) {
        upperLeftShooter.set(speed);
        lowerLeftShooter.set(speed);
        upperRightShooter.set(speed);
        lowerRightShooter.set(speed);
    }

    /**
     * Sets the shooter MotorControllers to a voltage.
     */
    public void shootVoltageSpeaker(double volts) {
        upperLeftShooter.setVoltage(volts);
        lowerLeftShooter.setVoltage(volts);
        upperRightShooter.setVoltage(volts);
        lowerRightShooter.setVoltage(volts);
    }

    public void shootAmp(double speed) {
        upperLeftShooter.set(-speed);
        lowerLeftShooter.set(speed);
        upperRightShooter.set(speed);
        lowerRightShooter.set(-speed);
    }

    /**
     * Sets the shooter MotorControllers to a voltage.
     */
    public void shootVoltageAmp(double volts) {
        upperLeftShooter.setVoltage(-volts);
        lowerLeftShooter.setVoltage(volts);
        upperRightShooter.setVoltage(volts);
        lowerRightShooter.setVoltage(-volts);
    }

    /**
     * Resets the shooter encoders to currently read a position of 0.
     */
    public void resetEncoders() {
        upperLeftShooterEncoder.setPosition(0);
        lowerLeftShooterEncoder.setPosition(0);
        upperRightShooterEncoder.setPosition(0);
        lowerRightShooterEncoder.setPosition(0);
    }

    public RelativeEncoder getUpperLeftShooterEncoder() {
        return upperLeftShooterEncoder;
    }

    public RelativeEncoder getLowerLeftShooterEncoder() {
        return lowerLeftShooterEncoder;
    }

    public RelativeEncoder getUpperRightShooterEncoder() {
        return upperRightShooterEncoder;
    }

    public RelativeEncoder getLowerRightShooterEncoder() {
        return lowerRightShooterEncoder;
    }

}
