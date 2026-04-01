package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.hooper.HopperSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

import java.util.function.DoubleSupplier;

/**
 * AllianceZoneShootCommand
 *
 * Tek komutta üç şey yapar:
 *   1. Robotu kendi alliance zone merkezine döndürür (ContinuousAim gibi)
 *   2. Mesafeye göre shooter RPM ayarlar
 *   3. Hizalanınca + RPM hazır olunca feed başlatır
 *
 * Basılı tut → döner + spin-up → hizalanınca + hazır olunca atar.
 * Bırakınca her şey durur.
 */
public class AllianceZoneShootCommand extends Command {

    // ── Alliance zone merkezleri (WPILib Blue origin) ────────────────────────
    private static final Translation2d BLUE_ZONE_CENTER = new Translation2d(2.5, 4.0);
    private static final Translation2d RED_ZONE_CENTER  = new Translation2d(14.0, 4.0);

    // ── Rotation P-controller (ContinuousAim ile aynı yapı) ─────────────────
    private static final double KP_ROTATION           = 0.065;
    private static final double MIN_COMMAND_RADPS     = 0.03;
    private static final double ERROR_DEADZONE_DEG    = 1.5;
    private static final double ALIGNED_TOLERANCE_DEG = 2.5;
    private static final double MAX_ROT_RADPS         = Units.rotationsToRadians(2.0);

    // ── Drive input ──────────────────────────────────────────────────────────
    private static final double JOYSTICK_DEADBAND = 0.10;

    // ── Limelight filtreleme ─────────────────────────────────────────────────
    private static final String LIMELIGHT_NAME   = "limelight";
    private static final double LL_MAX_AMBIGUITY = 0.2;
    private static final double LL_MIN_TAG_AREA  = 0.1;

    // ── RPM crash threshold ──────────────────────────────────────────────────
    private static final double RPM_CRASH_RATIO = 0.6;

    // ── Dependencies ────────────────────────────────────────────────────────
    private final CommandSwerveDrivetrain drivetrain;
    private final ShooterSubsystem shooter;
    private final HopperSubsystem hopper;
    private final FeederSubsystem feeder;
    private final DoubleSupplier leftYSupplier;
    private final DoubleSupplier leftXSupplier;
    private final double maxSpeed;

    // ── State ────────────────────────────────────────────────────────────────
    private enum Phase {
        AIMING_AND_SPINNING,   // döner + shooter spin-up
        FEEDING                // hizalı + hazır → topları at
    }

    private Phase phase;
    private double lastCommandedDistance = -1.0;
    private boolean isAligned = false;

    // ── Swerve request ──────────────────────────────────────────────────────
    private final SwerveRequest.FieldCentric driveRequest =
            new SwerveRequest.FieldCentric()
                    .withDeadband(0.0)
                    .withRotationalDeadband(0.0)
                    .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    public AllianceZoneShootCommand(
            ShooterSubsystem shooter,
            HopperSubsystem hopper,
            FeederSubsystem feeder,
            CommandSwerveDrivetrain drivetrain,
            DoubleSupplier leftYSupplier,
            DoubleSupplier leftXSupplier,
            double maxSpeed) {

        this.shooter       = shooter;
        this.hopper        = hopper;
        this.feeder        = feeder;
        this.drivetrain    = drivetrain;
        this.leftYSupplier = leftYSupplier;
        this.leftXSupplier = leftXSupplier;
        this.maxSpeed      = maxSpeed;

        addRequirements(shooter, hopper, feeder, drivetrain);
    }

    // ── Lifecycle ────────────────────────────────────────────────────────────

    @Override
    public void initialize() {
        phase = Phase.AIMING_AND_SPINNING;
        lastCommandedDistance = -1.0;
        isAligned = false;

        double distance = getDistanceToZone();
        shooter.setVelocityForDistance(distance);
        lastCommandedDistance = distance;

        SmartDashboard.putString("AllianceShoot/Phase", phase.toString());
        SmartDashboard.putNumber("AllianceShoot/Distance (m)", distance);
    }

    @Override
    public void execute() {
        // ── 1. Joystick okuma (sürerken dönebilsin) ──
        double forwardMps = applyDeadband(leftYSupplier.getAsDouble()) * -maxSpeed;
        double strafeMps  = applyDeadband(leftXSupplier.getAsDouble()) * -maxSpeed;

        // ── 2. Alliance zone'a dönüş hesaplama ──
        Pose2d robotPose = drivetrain.getState().Pose;
        Translation2d zoneCenter = getZoneCenter();
        Translation2d toZone = zoneCenter.minus(robotPose.getTranslation());
        Rotation2d targetHeading = new Rotation2d(toZone.getX(), toZone.getY());

        double errorDeg = wrapDegrees(
                targetHeading.getDegrees() - robotPose.getRotation().getDegrees());

        // P controller + static friction boost
        double rotOutput;
        if (Math.abs(errorDeg) > ERROR_DEADZONE_DEG) {
            rotOutput  = KP_ROTATION * errorDeg;
            rotOutput += (errorDeg > 0) ? MIN_COMMAND_RADPS : -MIN_COMMAND_RADPS;
        } else {
            rotOutput = KP_ROTATION * errorDeg;
        }
        rotOutput = Math.max(-MAX_ROT_RADPS, Math.min(MAX_ROT_RADPS, rotOutput));

        isAligned = Math.abs(errorDeg) <= ALIGNED_TOLERANCE_DEG;

        // ── 3. Swerve komut (her fazda — sürekli döner) ──
        drivetrain.setControl(
                driveRequest
                        .withVelocityX(forwardMps)
                        .withVelocityY(strafeMps)
                        .withRotationalRate(rotOutput));

        // ── 4. Mesafe güncelleme ──
        double distance = getDistanceToZone();
        boolean distanceChanged = Math.abs(distance - lastCommandedDistance) > 0.05;

        SmartDashboard.putNumber("AllianceShoot/Distance (m)", distance);
        SmartDashboard.putNumber("AllianceShoot/ErrorDeg", errorDeg);
        SmartDashboard.putBoolean("AllianceShoot/Aligned", isAligned);
        SmartDashboard.putBoolean("AllianceShoot/ShooterReady", shooter.isReadyToShoot());

        // ── 5. Phase logic ──
        switch (phase) {

            case AIMING_AND_SPINNING:
                if (distanceChanged) {
                    shooter.setVelocityForDistance(distance);
                    lastCommandedDistance = distance;
                }

                // Hizalı + shooter hazır → feed başlat
                if (isAligned && shooter.isReadyToShoot()) {
                    phase = Phase.FEEDING;
                    SmartDashboard.putString("AllianceShoot/Phase", phase.toString());
                }
                break;

            case FEEDING:
                if (distanceChanged) {
                    shooter.setVelocityForDistance(distance);
                    lastCommandedDistance = distance;
                }

                hopper.feed();
                feeder.feed();

                // RPM çöktüyse → tekrar spin-up
                if (shooter.getWheelRPM() < shooter.targetWheelRPM * RPM_CRASH_RATIO) {
                    phase = Phase.AIMING_AND_SPINNING;
                    hopper.stop();
                    feeder.stop();
                    SmartDashboard.putString("AllianceShoot/Phase", phase.toString());
                }

                // Hizadan çıktıysa → feed durdur, tekrar hizalan
                if (!isAligned) {
                    phase = Phase.AIMING_AND_SPINNING;
                    hopper.stop();
                    feeder.stop();
                    SmartDashboard.putString("AllianceShoot/Phase", phase.toString());
                }
                break;
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
        hopper.stop();
        feeder.stop();
        SmartDashboard.putString("AllianceShoot/Phase", "IDLE");
        SmartDashboard.putBoolean("AllianceShoot/Aligned", false);
        SmartDashboard.putBoolean("AllianceShoot/ShooterReady", false);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    // ── Yardımcı metodlar ────────────────────────────────────────────────────

    public boolean isAligned() {
        return isAligned;
    }

    private Translation2d getZoneCenter() {
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        return (alliance == Alliance.Blue) ? BLUE_ZONE_CENTER : RED_ZONE_CENTER;
    }

    private double getDistanceToZone() {
        if (LimelightHelpers.getTV(LIMELIGHT_NAME)) {
            LimelightHelpers.RawFiducial[] fiducials =
                    LimelightHelpers.getRawFiducials(LIMELIGHT_NAME);

            if (fiducials != null && fiducials.length > 0) {
                LimelightHelpers.RawFiducial best = null;
                for (LimelightHelpers.RawFiducial f : fiducials) {
                    if (f.ambiguity > LL_MAX_AMBIGUITY) continue;
                    if (f.ta < LL_MIN_TAG_AREA) continue;
                    if (best == null || f.ambiguity < best.ambiguity) {
                        best = f;
                    }
                }
                if (best != null) {
                    SmartDashboard.putBoolean("AllianceShoot/UsingLL", true);
                    return best.distToRobot;
                }
            }
        }

        SmartDashboard.putBoolean("AllianceShoot/UsingLL", false);
        Pose2d robotPose = drivetrain.getState().Pose;
        return getZoneCenter().minus(robotPose.getTranslation()).getNorm();
    }

    private static double applyDeadband(double value) {
        return Math.abs(value) > JOYSTICK_DEADBAND ? value : 0.0;
    }

    private static double wrapDegrees(double deg) {
        double mod = deg % 360.0;
        if (mod >  180.0) mod -= 360.0;
        if (mod <= -180.0) mod += 360.0;
        return mod;
    }
}