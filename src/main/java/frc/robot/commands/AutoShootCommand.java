package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.hooper.HopperSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

/**
 * AutoShootCommand — Otonom için atış komutu.
 *
 * Mesafeye göre shooter RPM ayarlar, hazır olunca feed başlatır.
 * Süre bitince veya otonom kesilince durur.
 *
 * ShootCommand'dan farkı:
 *   - Sabit süre ile biter (otonom için)
 *   - Limelight kullanmaz, sadece odometri
 *
 * PathPlanner'da kullanım:
 *   NamedCommands.registerCommand("AutoShoot", new AutoShootCommand(...));
 *   → Event marker adı: "AutoShoot"
 */
public class AutoShootCommand extends Command {

    // Otonom süresi kısıtlı — 4 top için 5 saniye yeterli
    private static final double SHOOT_DURATION_SECONDS = 5.0;

    // RPM bu oranın altına düşerse feed'i durdur, tekrar spin-up
    // %60 = ciddi düşüş (top sıkıştı). Küçük dalgalanmalarda feed devam eder.
    private static final double RPM_CRASH_RATIO = 0.6;

    // ── Hub pozisyonları (WPILib Blue origin) ────────────────────────────────
    private static final Translation2d BLUE_HUB = new Translation2d(4.552, 4.021);
    private static final Translation2d RED_HUB  = new Translation2d(11.961, 4.021);

    // ── Subsystems ───────────────────────────────────────────────────────────
    private final ShooterSubsystem       shooter;
    private final HopperSubsystem        hopper;
    private final FeederSubsystem        feeder;
    private final CommandSwerveDrivetrain drivetrain;

    // ── State ────────────────────────────────────────────────────────────────
    private enum Phase { SPINNING_UP, FEEDING }
    private Phase  phase;
    private double lastCommandedDistance = -1.0;
    private double startTime = -1.0;

    public AutoShootCommand(
            ShooterSubsystem shooter,
            HopperSubsystem hopper,
            FeederSubsystem feeder,
            CommandSwerveDrivetrain drivetrain) {

        this.shooter    = shooter;
        this.hopper     = hopper;
        this.feeder     = feeder;
        this.drivetrain = drivetrain;

        addRequirements(shooter, hopper, feeder);
    }

    @Override
    public void initialize() {
        phase = Phase.SPINNING_UP;
        lastCommandedDistance = -1.0;
        startTime = Timer.getFPGATimestamp();

        double distance = getDistanceToHub();
        shooter.setVelocityForDistance(distance);
        lastCommandedDistance = distance;

        SmartDashboard.putString("AutoShoot/Phase", phase.toString());
        SmartDashboard.putNumber("AutoShoot/Distance (m)", distance);
    }

    @Override
    public void execute() {
        double distance = getDistanceToHub();
        double elapsed  = Timer.getFPGATimestamp() - startTime;
        boolean distanceChanged = Math.abs(distance - lastCommandedDistance) > 0.05;

        SmartDashboard.putNumber("AutoShoot/Distance (m)", distance);
        SmartDashboard.putNumber("AutoShoot/Elapsed (s)", elapsed);
        SmartDashboard.putNumber("AutoShoot/Remaining (s)",
                Math.max(0, SHOOT_DURATION_SECONDS - elapsed));
        SmartDashboard.putBoolean("AutoShoot/ShooterReady", shooter.isReadyToShoot());
        SmartDashboard.putString("AutoShoot/Phase", phase.toString());

        switch (phase) {

            case SPINNING_UP:
                if (distanceChanged) {
                    shooter.setVelocityForDistance(distance);
                    lastCommandedDistance = distance;
                }
                // Hazır olunca feed'e geç
                if (shooter.isReadyToShoot()) {
                    phase = Phase.FEEDING;
                    SmartDashboard.putString("AutoShoot/Phase", phase.toString());
                }
                break;

            case FEEDING:
                if (distanceChanged) {
                    shooter.setVelocityForDistance(distance);
                    lastCommandedDistance = distance;
                }

                hopper.feed();
                feeder.feed();

                // Sadece ciddi RPM düşüşünde dur (%60'ın altı)
                // Normal top geçişi RPM'i %10-20 düşürür — bu devam etmeli
                if (shooter.getWheelRPM() < shooter.targetWheelRPM * RPM_CRASH_RATIO) {
                    phase = Phase.SPINNING_UP;
                    hopper.stop();
                    feeder.stop();
                    SmartDashboard.putString("AutoShoot/Phase", phase.toString());
                }
                break;
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
        hopper.stop();
        feeder.stop();
        SmartDashboard.putString("AutoShoot/Phase", "IDLE");
    }

    @Override
    public boolean isFinished() {
        return (Timer.getFPGATimestamp() - startTime) >= SHOOT_DURATION_SECONDS;
    }

    private double getDistanceToHub() {
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        Translation2d hub = (alliance == Alliance.Blue) ? BLUE_HUB : RED_HUB;
        return hub.minus(drivetrain.getState().Pose.getTranslation()).getNorm();
    }
}