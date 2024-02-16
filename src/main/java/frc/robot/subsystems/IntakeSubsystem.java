// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.IntakeConstants.*;
import static frc.robot.Constants.MotorPorts.INTAKE_BOTTOM_MOTOR_PORT;
import static frc.robot.Constants.MotorPorts.INTAKE_TOP_MOTOR_PORT;

public class IntakeSubsystem extends SubsystemBase {
    public final CANSparkMax topMotor = new CANSparkMax(INTAKE_TOP_MOTOR_PORT, MotorType.kBrushless);
    public final CANSparkMax bottomMotor = new CANSparkMax(INTAKE_BOTTOM_MOTOR_PORT, MotorType.kBrushless);

    public final RelativeEncoder topEncoder = topMotor.getEncoder();
    public final RelativeEncoder bottomEncoder = bottomMotor.getEncoder();

    // just add together the outputs from pidController and feedforward
    public final PIDController pidController = new PIDController(K_P, K_I, K_D);
    public final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(K_S, K_V);

    public IntakeSubsystem() {
    }

    @Override
    public void periodic() {
        // this function doesn't do anything right now
    }

    /**
     * Drives the intake arm at given speed.
     *
     * @param speed Speed of the intake arm motors. [-1, 1], negative is consumption, positive is barfing.
     */
    public void setIntakeSpeed(double speed) {
        topMotor.set(speed);
        bottomMotor.set(speed);
    }
}
