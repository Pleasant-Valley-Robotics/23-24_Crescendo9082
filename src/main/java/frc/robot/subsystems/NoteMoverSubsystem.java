// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.MotorPorts.*;
import static frc.robot.Constants.PhysicalConstants.REVS_TO_RADIANS;
import static frc.robot.Constants.PhysicalConstants.REV_PER_MIN_TO_RADIAN_PER_SEC;

public class NoteMoverSubsystem extends SubsystemBase {
    public final CANSparkMax intakeTopMotor = new CANSparkMax(INTAKE_TOP_MOTOR_PORT, MotorType.kBrushless);
    public final CANSparkMax intakeBottomMotor = new CANSparkMax(INTAKE_BOTTOM_MOTOR_PORT, MotorType.kBrushless);
    public final RelativeEncoder intakeTopEncoder = intakeTopMotor.getEncoder();
    public final RelativeEncoder intakeBottomEncoder = intakeBottomMotor.getEncoder();
    // just add together the outputs from pidController and feedforward

    public final CANSparkMax shooterTopMotor = new CANSparkMax(SHOOTER_TOP_MOTOR_PORT, MotorType.kBrushless);
    public final CANSparkMax shooterBottomMotor = new CANSparkMax(SHOOTER_BOTTOM_MOTOR_PORT, MotorType.kBrushless);
    public final RelativeEncoder shooterTopEncoder = shooterTopMotor.getEncoder();
    public final RelativeEncoder shooterBottomEncoder = shooterBottomMotor.getEncoder();

    public final CANSparkMax armLeftMotor = new CANSparkMax(ARM_LEFT_MOTOR_PORT, MotorType.kBrushless);
    public final CANSparkMax armRightMotor = new CANSparkMax(ARM_RIGHT_MOTOR_PORT, MotorType.kBrushless);
    public final RelativeEncoder armLeftEncoder = armLeftMotor.getEncoder();
    public final RelativeEncoder armRightEncoder = armRightMotor.getEncoder();


    public NoteMoverSubsystem() {
        shooterTopEncoder.setPositionConversionFactor(REVS_TO_RADIANS);
        shooterTopEncoder.setVelocityConversionFactor(REV_PER_MIN_TO_RADIAN_PER_SEC);

        shooterBottomEncoder.setPositionConversionFactor(REVS_TO_RADIANS);
        shooterBottomEncoder.setVelocityConversionFactor(REV_PER_MIN_TO_RADIAN_PER_SEC);

        armLeftEncoder.setPositionConversionFactor(REVS_TO_RADIANS);
        armLeftEncoder.setVelocityConversionFactor(REV_PER_MIN_TO_RADIAN_PER_SEC);

        armRightEncoder.setPositionConversionFactor(REVS_TO_RADIANS);
        armRightEncoder.setVelocityConversionFactor(REV_PER_MIN_TO_RADIAN_PER_SEC);
    }

    @Override
    public void periodic() {
        // this function doesn't do anything right now
    }

    /**
     * Drives the intake at given speed.
     *
     * @param speed Speed of the intake arm motors. [-1, 1], negative is consumption, positive is barfing.
     */
    public void setIntakeSpeed(double speed) {
        intakeTopMotor.set(speed);
        intakeBottomMotor.set(speed);
    }


    /**
     * Drives the shooter arm at given speed.
     *
     * @param speed Speed of the shooter arm motors. [-1, 1], negative is consumption, positive is barfing.
     */
    public void setShooterSpeed(double speed) {
        shooterTopMotor.set(speed);
        shooterBottomMotor.set(speed);
    }

    public void setArmSpeed(double speed) {
        armLeftMotor.set(speed);
        armRightMotor.set(speed);
    }

    private final MutableMeasure<Voltage> volts = MutableMeasure.mutable(Volts.of(0));

    private final MutableMeasure<Angle> distance = MutableMeasure.mutable(Radians.of(0));

    private final MutableMeasure<Velocity<Angle>> velocity = MutableMeasure.mutable(RadiansPerSecond.of(0));

    public final SysIdRoutine routine = new SysIdRoutine(new SysIdRoutine.Config(), new SysIdRoutine.Mechanism(
            volts -> {
                intakeTopMotor.setVoltage(volts.in(Volts));
                intakeBottomMotor.setVoltage(volts.in(Volts));
                shooterTopMotor.setVoltage(volts.in(Volts));
                shooterBottomMotor.setVoltage(volts.in(Volts));
            },
            log -> {
                log.motor("shooterTopMotor")
                        .voltage(volts.mut_replace(intakeTopMotor.getBusVoltage(), Volts))
                        .angularPosition(distance.mut_replace(intakeTopEncoder.getPosition(), Radians))
                        .angularVelocity(velocity.mut_replace(intakeBottomEncoder.getVelocity(), RadiansPerSecond));
                log.motor("intakeBottomMotor")
                        .voltage(volts.mut_replace(intakeBottomMotor.getBusVoltage(), Volts))
                        .angularPosition(distance.mut_replace(shooterTopEncoder.getPosition(), Radians))
                        .angularVelocity(velocity.mut_replace(shooterBottomEncoder.getVelocity(), RadiansPerSecond));
                log.motor("shooterBottomMotor")
                        .voltage(volts.mut_replace(shooterTopMotor.getBusVoltage(), Volts))
                        .angularPosition(distance.mut_replace(shooterBottomEncoder.getPosition(), Radians))
                        .angularVelocity(velocity.mut_replace(shooterBottomEncoder.getVelocity(), RadiansPerSecond));
                log.motor("intakeTopMotor")
                        .voltage(volts.mut_replace(shooterBottomMotor.getBusVoltage(), Volts))
                        .angularPosition(distance.mut_replace(shooterBottomEncoder.getPosition(), Radians))
                        .angularVelocity(velocity.mut_replace(shooterBottomEncoder.getVelocity(), RadiansPerSecond));
            }, this));
}
