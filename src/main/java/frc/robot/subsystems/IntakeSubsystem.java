// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.MotorPorts.INTAKE_BOTTOM_MOTOR_PORT;
import static frc.robot.Constants.MotorPorts.INTAKE_TOP_MOTOR_PORT;

public class IntakeSubsystem extends SubsystemBase {
    private final CANSparkMax intakeTop = new CANSparkMax(INTAKE_TOP_MOTOR_PORT, MotorType.kBrushless);
    private final CANSparkMax intakeBottom = new CANSparkMax(INTAKE_BOTTOM_MOTOR_PORT, MotorType.kBrushless);

    public final RelativeEncoder intakeTopEncoder = intakeTop.getEncoder();
    public final RelativeEncoder intakeBottomEncoder = intakeBottom.getEncoder();

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
        intakeTop.set(speed);
        intakeBottom.set(speed);
    }
}
