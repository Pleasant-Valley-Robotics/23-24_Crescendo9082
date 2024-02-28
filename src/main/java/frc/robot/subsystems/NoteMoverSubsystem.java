// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.MotorPorts.*;

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
}
