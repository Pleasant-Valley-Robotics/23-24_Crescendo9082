package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.NoteMoverSubsystem;

import static frc.robot.Constants.IntakeConstants.INTAKE_FEEDFORWARD;
import static frc.robot.Constants.IntakeConstants.INTAKE_PID;


public class IntakeSpeedCommand extends Command {
    private final NoteMoverSubsystem noteMover;
    private final double speed;

    public IntakeSpeedCommand(NoteMoverSubsystem noteMover, double speed) {
        this.noteMover = noteMover;
        addRequirements(this.noteMover);

        this.speed = speed;
    }

    @Override
    public void initialize() {
        INTAKE_PID.setSetpoint(speed);
    }

    @Override
    public void execute() {
        double averageVelocity = (noteMover.intakeTopEncoder.getVelocity() + noteMover.intakeBottomEncoder.getVelocity()) / 2;
        double voltage = INTAKE_PID.calculate(averageVelocity, speed) + INTAKE_FEEDFORWARD.calculate(speed);

        noteMover.intakeTopMotor.setVoltage(voltage);
        noteMover.intakeBottomMotor.setVoltage(voltage);
    }


    @Override
    public void end(boolean interrupted) {
        noteMover.intakeTopMotor.setVoltage(0);
        noteMover.intakeBottomMotor.setVoltage(0);
    }
}
