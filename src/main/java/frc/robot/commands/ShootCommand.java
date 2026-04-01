package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.hooper.HopperSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class ShootCommand extends Command {

    private static final String LIMELIGHT_NAME = "limelight";

    private static final Translation2d BLUE_HUB = new Translation2d(4.552, 4.021);
    private static final Translation2d RED_HUB = new Translation2d(11.961, 4.021);

    private static final double LL_MAX_AMBIGUITY = 0.2;
    private static final double LL_MIN_TAG_AREA = 0.1;

    private final ShooterSubsystem shooter;
    private final HopperSubsystem hopper;
    private final FeederSubsystem feeder;
    private final CommandSwerveDrivetrain drivetrain;

    private enum Phase {
        SPINNING_UP, FEEDING
    }

    private Phase phase;
    private double lastCommandedDistance = -1.0;

    public ShootCommand(
            ShooterSubsystem shooter,
            HopperSubsystem hopper,
            FeederSubsystem feeder,
            CommandSwerveDrivetrain drivetrain) {

        this.shooter = shooter;
        this.hopper = hopper;
        this.feeder = feeder;
        this.drivetrain = drivetrain;

        addRequirements(shooter, hopper, feeder);
    }

    @Override
    public void initialize() {
        phase = Phase.SPINNING_UP;
        lastCommandedDistance = -1.0;

        double distance = getDistanceToHub();
        shooter.setVelocityForDistance(distance);
        lastCommandedDistance = distance;

        SmartDashboard.putString("ShootCommand/Phase", phase.toString());
        SmartDashboard.putNumber("ShootCommand/Distance (m)", distance);
    }

    @Override
    public void execute() {
        double distance = getDistanceToHub();
        SmartDashboard.putNumber("ShootCommand/Distance (m)", distance);
        SmartDashboard.putBoolean("ShootCommand/Shooter Ready", shooter.isReadyToShoot());

        boolean distanceChanged = Math.abs(distance - lastCommandedDistance) > 0.05;

        switch (phase) {

            case SPINNING_UP:
                if (distanceChanged) {
                    shooter.setVelocityForDistance(distance);
                    lastCommandedDistance = distance;
                }
                if (shooter.isReadyToShoot()) {
                    phase = Phase.FEEDING;
                    SmartDashboard.putString("ShootCommand/Phase", phase.toString());
                }
                break;

            case FEEDING:
                if (distanceChanged) {
                    shooter.setVelocityForDistance(distance);
                    lastCommandedDistance = distance;
                }

                hopper.feed();
                feeder.feed();

                // Only pause if RPM crashes badly (e.g., below 60% of target)
                if (shooter.getWheelRPM() < shooter.targetWheelRPM * 0.6) {
                    phase = Phase.SPINNING_UP;
                    hopper.stop();
                    feeder.stop();
                }
                break;
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
        hopper.stop();
        feeder.stop();
        SmartDashboard.putString("ShootCommand/Phase", "IDLE");
        SmartDashboard.putBoolean("ShootCommand/Shooter Ready", false);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    public double getDistanceToHub() {
        if (LimelightHelpers.getTV(LIMELIGHT_NAME)) {
            LimelightHelpers.RawFiducial[] fiducials = LimelightHelpers.getRawFiducials(LIMELIGHT_NAME);

            if (fiducials != null && fiducials.length > 0) {
                LimelightHelpers.RawFiducial best = null;
                for (LimelightHelpers.RawFiducial f : fiducials) {
                    if (f.ambiguity > LL_MAX_AMBIGUITY)
                        continue;
                    if (f.ta < LL_MIN_TAG_AREA)
                        continue;
                    if (best == null || f.ambiguity < best.ambiguity) {
                        best = f;
                    }
                }

                if (best != null) {
                    SmartDashboard.putBoolean("ShootCommand/Using Limelight", true);
                    SmartDashboard.putNumber("ShootCommand/LL Tag ID", best.id);
                    SmartDashboard.putNumber("ShootCommand/LL Dist To Robot (m)", best.distToRobot);
                    return best.distToRobot;
                }
            }
        }

        SmartDashboard.putBoolean("ShootCommand/Using Limelight", true);
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        Translation2d hub = (alliance == Alliance.Blue) ? BLUE_HUB : RED_HUB;
        return hub.minus(drivetrain.getState().Pose.getTranslation()).getNorm();
    }
}