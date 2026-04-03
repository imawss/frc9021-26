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
 * Top hopper'da sıkışırsa (akım spike), geri çevirip tekrar dener.
 * Süre bitince veya otonom kesilince durur.
 *
 * PathPlanner'da kullanım:
 *   NamedCommands.registerCommand("AutoShoot", new AutoShootCommand(...));
 */
public class AutoShootCommand extends Command {

    // ── Zamanlama ────────────────────────────────────────────────────────────
    private static final double SHOOT_DURATION_SECONDS = 4.0;

    // ── Jam tespiti (sadece hopper akımı) ────────────────────────────────────
    // Sıkışma ~30A ama normal feed'de de bazen üstüne çıkabiliyor.
    // Eşiği biraz altına koy, asıl filtrelemeyi süre ile yap.
    private static final double JAM_CURRENT_THRESHOLD = 28.0;

    // Akım bu süre boyunca SÜREKLI eşiğin üstünde kalırsa → gerçek jam.
    // 250ms, anlık dalgalanmaları filtreler (~12 robot cycle).
    // Çok false positive olursa 0.3'e çık, geç yakalıyorsa 0.15'e in.
    private static final double JAM_CONFIRM_DURATION = 0.25;

    // Geri çevirme süresi (saniye)
    private static final double UNJAM_REVERSE_DURATION = 0.1;

    // Sonsuz döngüye girmesin — max deneme sayısı
    private static final int MAX_UNJAM_ATTEMPTS = 3;

    // ── Hub pozisyonları (WPILib Blue origin) ────────────────────────────────
    private static final Translation2d BLUE_HUB = new Translation2d(4.552, 4.021);
    private static final Translation2d RED_HUB  = new Translation2d(11.961, 4.021);

    // ── Subsystems ───────────────────────────────────────────────────────────
    private final ShooterSubsystem       shooter;
    private final HopperSubsystem        hopper;
    private final FeederSubsystem        feeder;
    private final CommandSwerveDrivetrain drivetrain;

    // ── State ────────────────────────────────────────────────────────────────
    private enum Phase { SPINNING_UP, FEEDING, UNJAMMING }

    private Phase  phase;
    private double lastCommandedDistance = -1.0;
    private double startTime = -1.0;

    // Jam tespiti state
    private double jamDetectStartTime = -1.0;   // akım ilk ne zaman eşiği aştı
    private double unjamStartTime     = -1.0;    // geri çevirme ne zaman başladı
    private int    unjamAttempts      = 0;

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
        jamDetectStartTime = -1.0;
        unjamStartTime     = -1.0;
        unjamAttempts      = 0;

        double distance = getDistanceToHub();
        shooter.setVelocityForDistance(distance);
        lastCommandedDistance = distance;

        logPhase();
        SmartDashboard.putNumber("AutoShoot/Distance (m)", distance);
    }

    @Override
    public void execute() {
        double now      = Timer.getFPGATimestamp();
        double distance = getDistanceToHub();
        double elapsed  = now - startTime;
        boolean distanceChanged = Math.abs(distance - lastCommandedDistance) > 0.05;

        // ── Telemetri ────────────────────────────────────────────────────────
        SmartDashboard.putNumber("AutoShoot/Distance (m)", distance);
        SmartDashboard.putNumber("AutoShoot/Elapsed (s)", elapsed);
        SmartDashboard.putNumber("AutoShoot/Remaining (s)",
                Math.max(0, SHOOT_DURATION_SECONDS - elapsed));
        SmartDashboard.putBoolean("AutoShoot/ShooterReady", shooter.isReadyToShoot());
        SmartDashboard.putNumber("AutoShoot/Hopper Current (A)", hopper.getCurrent());
        SmartDashboard.putNumber("AutoShoot/Unjam Attempts", unjamAttempts);

        switch (phase) {

            // ─────────────────────────────────────────────────────────────────
            case SPINNING_UP:
                if (distanceChanged) {
                    shooter.setVelocityForDistance(distance);
                    lastCommandedDistance = distance;
                }
                if (shooter.isReadyToShoot()) {
                    phase = Phase.FEEDING;
                    jamDetectStartTime = -1.0;  // yeni feed döngüsü, sıfırla
                    logPhase();
                }
                break;

            // ─────────────────────────────────────────────────────────────────
            case FEEDING:
                if (distanceChanged) {
                    shooter.setVelocityForDistance(distance);
                    lastCommandedDistance = distance;
                }

                hopper.feed();
                feeder.feed();

                // — Jam tespiti: hopper akımı eşiğin üstünde mi? —
                if (hopper.getCurrent() > JAM_CURRENT_THRESHOLD) {
                    // İlk spike anını kaydet
                    if (jamDetectStartTime < 0) {
                        jamDetectStartTime = now;
                    }
                    // Yeterince uzun sürdüyse → gerçek jam
                    if ((now - jamDetectStartTime) >= JAM_CONFIRM_DURATION
                            && unjamAttempts < MAX_UNJAM_ATTEMPTS) {
                        phase = Phase.UNJAMMING;
                        unjamStartTime = now;
                        unjamAttempts++;
                        hopper.eject();   // geri çevir
                        feeder.stop();
                        jamDetectStartTime = -1.0;
                        logPhase();
                        DriverStation.reportWarning(
                            String.format("[AutoShoot] Jam detected (%.1fA) — unjam attempt %d/%d",
                                hopper.getCurrent(), unjamAttempts, MAX_UNJAM_ATTEMPTS), false);
                    }
                } else {
                    // Akım normale döndü, sayacı sıfırla
                    jamDetectStartTime = -1.0;
                }
                break;

            // ─────────────────────────────────────────────────────────────────
            case UNJAMMING:
                hopper.eject();
                feeder.stop();

                // Geri çevirme süresi doldu → tekrar spin-up'a dön
                if ((now - unjamStartTime) >= UNJAM_REVERSE_DURATION) {
                    hopper.stop();
                    phase = Phase.SPINNING_UP;
                    logPhase();
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

        if (unjamAttempts > 0) {
            System.out.println("[AutoShoot] Finished — total unjam attempts: " + unjamAttempts);
        }
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

    private void logPhase() {
        SmartDashboard.putString("AutoShoot/Phase", phase.toString());
    }
}