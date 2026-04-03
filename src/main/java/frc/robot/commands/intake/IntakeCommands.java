package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intake.ArmSubsystem;
import frc.robot.subsystems.intake.IntakeRollerSubsystem;

public class IntakeCommands {

    /** Arm indir + roller aç */
    public static Command intakeDown(ArmSubsystem arm, IntakeRollerSubsystem roller) {
        return Commands.sequence(
            Commands.run(() -> arm.manualDrive(1), arm)
                    .until(() -> arm.getAngleDeg() >= 118.0)
                    .withTimeout(3.0),
            Commands.runOnce(() -> {
                arm.manualDrive(0);
                roller.intake();
            }, arm, roller)
        ).finallyDo(interrupted -> {
            if (interrupted) {
                arm.stop();
                roller.stop();
            }
        }).withName("IntakeDown");
    }

    /** Arm kaldır + roller kapat */
    public static Command intakeUp(ArmSubsystem arm, IntakeRollerSubsystem roller) {
        return Commands.sequence(
            Commands.runOnce(() -> roller.stop(), roller),
            Commands.run(() -> arm.manualDrive(-1), arm)
                    .until(() -> arm.getAngleDeg() <= 5.0)
                    .withTimeout(3.0),
            Commands.runOnce(() -> arm.manualDrive(0), arm)
        ).finallyDo(interrupted -> {
            if (interrupted) {
                arm.stop();
                roller.stop();
            }
        }).withName("IntakeUp");
    }
}