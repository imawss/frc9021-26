package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class RetractIntake extends Command {
    private final IntakeSubsystem intake;

    public RetractIntake(IntakeSubsystem intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.retract();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}