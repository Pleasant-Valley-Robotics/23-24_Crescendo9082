// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.MotorPorts.LEFT_HANGING_MOTOR_PORT;
import static frc.robot.Constants.MotorPorts.RIGHT_HANGING_MOTOR_PORT;

public class HangingSubsystem extends SubsystemBase {
    private final CANSparkMax leftHangingMotor = new CANSparkMax(LEFT_HANGING_MOTOR_PORT, MotorType.kBrushless);
    private final CANSparkMax rightHangingMotor = new CANSparkMax(RIGHT_HANGING_MOTOR_PORT, MotorType.kBrushless);
    private final RelativeEncoder leftHangingEncoder = leftHangingMotor.getEncoder();
    private final RelativeEncoder rightHangingEncoder = rightHangingMotor.getEncoder();

    /**
     * Creates a new HangingSubsystem.
     */
    public HangingSubsystem() {
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    /**
     * Drives the hanging subsystem at given speed. Speeds range from [-1, 1].
     *
     * @param speed speed of the hanging motors.
     */
    public void hang(double speed) {
        leftHangingMotor.set(speed);
        rightHangingMotor.set(speed);
    }

    /**
     * Sets the hanging MotorControllers to a voltage.
     */
    public void hangVoltage(double volts) {
        leftHangingMotor.setVoltage(volts);
        rightHangingMotor.setVoltage(volts);
    }

    /**
     * Resets the hanging encoders to currently read a position of 0.
     */
    public void resetEncoders() {
        leftHangingEncoder.setPosition(0);
        rightHangingEncoder.setPosition(0);
    }

    /**
     * Gets the front hanging drive encoder.
     *
     * @return the front hanging drive encoder
     */
    public RelativeEncoder getLeftHangingEncoder() {
        return leftHangingEncoder;
    }

    /**
     * Gets the rear hanging drive encoder.
     *
     * @return the rear hanging drive encoder
     */
    public RelativeEncoder getRightHangingEncoder() {
        return rightHangingEncoder;
    }
}
