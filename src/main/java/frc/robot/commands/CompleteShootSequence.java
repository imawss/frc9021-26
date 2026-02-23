package frc.robot.commands;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.hooper.HopperSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.util.FuelSim;

public class CompleteShootSequence extends SequentialCommandGroup {

    /**
     * @param shooter        ShooterSubsystem — controls the flywheel
     * @param hopper         HopperSubsystem — stages the game piece
     * @param feeder         FeederSubsystem — pushes game piece into shooter
     * @param distanceMeters Horizontal distance to the target in meters
     * @param fuelSim        FuelSim instance (pass null if not in simulation)
     */
    public CompleteShootSequence(
            ShooterSubsystem shooter,
            HopperSubsystem hopper,
            FeederSubsystem feeder,
            double distanceMeters,
            FuelSim fuelSim) {
        addCommands(

                Commands.print("[Shoot] Spinning up for " + distanceMeters + "m"),
                Commands.runOnce(() -> shooter.setVelocityForDistance(distanceMeters), shooter),

                Commands.waitUntil(shooter::atTargetVelocity).withTimeout(2.0),
                Commands.print("[Shoot] At speed — feeding"),

                Commands.parallel(
                        Commands.run(hopper::feed, hopper),
                        Commands.run(feeder::feed, feeder)).withTimeout(0.75),

                Commands.runOnce(() -> {
                    if (RobotBase.isSimulation() && fuelSim != null) {
                        double wheelRPM = shooter.getWheelRPM();
                        double launchSpeedMPS = Math.min(
                                (wheelRPM / 60.0) * 2.0 * Math.PI * ShooterConstants.WHEEL_RADIUS_METERS,
                                15.0 // max realistic FRC launch speed
                        );

                        double hoodAngleDeg = shooter.getActiveProfileAngle();

                        fuelSim.launchFuel(
                                Units.MetersPerSecond.of(launchSpeedMPS), // linear launch speed
                                Units.Degrees.of(hoodAngleDeg), // hood elevation angle
                                Units.Degrees.of(0.0), // turret yaw (0 = straight ahead)
                                Units.Meters.of(0.5) // launch height above the floor
                        );

                        System.out.printf("[FuelSim] Launched: %.1f m/s at %.1f°%n",
                                launchSpeedMPS, hoodAngleDeg);
                    }
                }),

                Commands.runOnce(() -> {
                    shooter.stop();
                    hopper.stop();
                    feeder.stop();
                }, shooter, hopper, feeder),

                Commands.print("[Shoot] Complete"));
    }

    public CompleteShootSequence(
            ShooterSubsystem shooter,
            HopperSubsystem hopper,
            FeederSubsystem feeder,
            double distanceMeters) {
        this(shooter, hopper, feeder, distanceMeters, null);
    }
}