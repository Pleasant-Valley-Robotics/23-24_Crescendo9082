package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;


public class IntakeSpeedCommand extends Command {
    private final IntakeSubsystem intake;
    private final double speed;

    public IntakeSpeedCommand(IntakeSubsystem intake, double speed) {
        this.intake = intake;
        addRequirements(this.intake);

        this.speed = speed;
    }

    @Override
    public void initialize() {
        intake.pidController.setSetpoint(speed);
    }

    @Override
    public void execute() {
        double averageVelocity = (intake.topEncoder.getVelocity() + intake.bottomEncoder.getVelocity()) / 2;
        double voltage = intake.pidController.calculate(averageVelocity, speed) + intake.feedforward.calculate(speed);

        intake.topMotor.setVoltage(voltage);
        intake.bottomMotor.setVoltage(voltage);
    }


    @Override
    public void end(boolean interrupted) {
        intake.topMotor.setVoltage(0);
        intake.bottomMotor.setVoltage(0);
    }
}
