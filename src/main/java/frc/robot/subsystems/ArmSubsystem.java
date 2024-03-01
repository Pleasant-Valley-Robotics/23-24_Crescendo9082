package frc.robot.subsystems;


import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.MotorPorts.ARM_LEFT_MOTOR_PORT;
import static frc.robot.Constants.MotorPorts.ARM_RIGHT_MOTOR_PORT;
import static frc.robot.Constants.PhysicalConstants.REVS_TO_RADIANS;
import static frc.robot.Constants.PhysicalConstants.REV_PER_MIN_TO_RADIAN_PER_SEC;

public class ArmSubsystem extends SubsystemBase {
    public final CANSparkMax armLeftMotor = new CANSparkMax(ARM_LEFT_MOTOR_PORT, CANSparkLowLevel.MotorType.kBrushless);
    public final CANSparkMax armRightMotor = new CANSparkMax(ARM_RIGHT_MOTOR_PORT, CANSparkLowLevel.MotorType.kBrushless);
    public final RelativeEncoder armLeftEncoder = armLeftMotor.getEncoder();
    public final RelativeEncoder armRightEncoder = armRightMotor.getEncoder();

    public ArmSubsystem() {
        // TODO: Set the default command, if any, for this subsystem by calling setDefaultCommand(command)
        //       in the constructor or in the robot coordination class, such as RobotContainer.
        //       Also, you can call addChild(name, sendableChild) to associate sendables with the subsystem
        //       such as SpeedControllers, Encoders, DigitalInputs, etc.

        armLeftEncoder.setPositionConversionFactor(REVS_TO_RADIANS);
        armLeftEncoder.setVelocityConversionFactor(REV_PER_MIN_TO_RADIAN_PER_SEC);

        armRightEncoder.setPositionConversionFactor(REVS_TO_RADIANS);
        armRightEncoder.setVelocityConversionFactor(REV_PER_MIN_TO_RADIAN_PER_SEC);

    }


    public void setArmSpeed(double speed) {
        armLeftMotor.set(speed);
        armRightMotor.set(speed);
    }
}

