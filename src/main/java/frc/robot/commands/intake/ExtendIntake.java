package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class ExtendIntake extends Command {
    private final IntakeSubsystem intake;

    public ExtendIntake(IntakeSubsystem intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.extend();
    }

    @Override
    public boolean isFinished() {
        return false; 
    }
}