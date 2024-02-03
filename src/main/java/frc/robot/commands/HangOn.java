package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.HangingSubsystem;

public class HangOn {
    public static Command turnHangOn(HangingSubsystem robotHanging){
        return new InstantCommand(() -> robotHanging.hangVoltage(12), robotHanging);
    }
}
