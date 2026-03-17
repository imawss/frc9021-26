package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.hooper.HopperSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

/**
 * Spins the shooter to a fixed RPM, then feeds once ready.
 * All three subsystems stop when the command ends (button released).
 *
 * Usage in RobotContainer:
 *   joystick.rightBumper().whileTrue(new ShootAtRPMCommand(shooter, hopper, feeder, 3500));
 */
public class ShootAtRPMCommand extends Command {

    private final ShooterSubsystem shooter;
    private final HopperSubsystem  hopper;
    private final FeederSubsystem  feeder;
    private final double           targetRPM;

    public ShootAtRPMCommand(
            ShooterSubsystem shooter,
            HopperSubsystem  hopper,
            FeederSubsystem  feeder,
            double           targetRPM) {

        this.shooter   = shooter;
        this.hopper    = hopper;
        this.feeder    = feeder;
        this.targetRPM = targetRPM;

        addRequirements(shooter, hopper, feeder);
    }

    @Override
    public void initialize() {
        shooter.setVelocityRPM(targetRPM);
        hopper.stop();
        feeder.stop();
    }

    @Override
    public void execute() {
        if (shooter.isReadyToShoot()) {
            hopper.feed();
            feeder.feed();
        }
        // else: keep waiting — shooter is still spinning up
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
        hopper.stop();
        feeder.stop();
    }

    @Override
    public boolean isFinished() {
        return false; // runs until button is released
    }
}