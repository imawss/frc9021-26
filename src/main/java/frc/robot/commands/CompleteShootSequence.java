package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.hooper.HopperSubsystem;
import frc.robot.subsystems.feeder.FeederSubsystem;

/**
 * Complete shooting sequence for your robot:
 * Ground → IntakeRoller → Hopper → Feeder → Shooter
 * 
 * SEQUENCE:
 * 1. Spin up shooter to correct velocity
 * 2. Run hopper to stage game piece
 * 3. Run feeder to push into shooter
 * 4. Hold for shot to complete
 * 5. Stop everything
 */
public class CompleteShootSequence extends SequentialCommandGroup {
    
    public CompleteShootSequence(
        ShooterSubsystem shooter,
        HopperSubsystem hopper,
        FeederSubsystem feeder,
        double distanceMeters
    ) {
        addCommands(
            Commands.print("[Shoot] Spinning up for " + distanceMeters + "m"),
            Commands.runOnce(() -> shooter.setVelocityForDistance(distanceMeters), shooter),
            
            Commands.waitUntil(shooter::atTargetVelocity).withTimeout(2.0),
            Commands.print("[Shoot] Shooter ready"),
            
            Commands.parallel(
                Commands.run(hopper::feed, hopper),
                Commands.run(feeder::feed, feeder)
            ).withTimeout(0.75), //until can be added when we have a sensor to detect game piece position
            
            Commands.runOnce(() -> {
                shooter.stop();
                hopper.stop();
                feeder.stop();
            }, shooter, hopper, feeder),
            
            Commands.print("[Shoot] Complete")
        );
    }
}