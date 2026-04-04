package frc.robot.util;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cscore.HttpCamera.HttpCameraKind;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Optional;

import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.intake.ArmSubsystem;

/**
 * Elastic Dashboard yapılandırması — Team 9021, Rebuilt Sezonu.
 *
 * 6 widget:
 *   1. Field2d        — Rebuilt sahası + robot pozisyonu
 *   2. Match Time      — otomatik (DriverStation yayınlar)
 *   3. Hub Active      — boolean gösterge
 *   4. Intake Angle    — arm encoder açısı (derece)
 *   5. USB Camera      — intake kamerası stream
 *   6. Limelight       — hedefleme kamerası stream
 *
 * Kullanım:
 *   RobotContainer'da:  dashboard = new DashboardConfig(drivetrain, arm);
 *   Robot.java'da:      m_robotContainer.dashboard.update();  (robotPeriodic içinde)
 */
public class DashboardConfig {

    private final Field2d field = new Field2d();
    private final CommandSwerveDrivetrain drivetrain;
    private final ArmSubsystem arm;

    public DashboardConfig(CommandSwerveDrivetrain drivetrain, ArmSubsystem arm) {
        this.drivetrain = drivetrain;
        this.arm = arm;

        // ── 1. Field2d ──
        // Elastic'te sürükleyince "Field" widget olarak görünür
        // Properties → field_game = "Rebuilt" olarak seçin
        SmartDashboard.putData("Field", field);

        // ── 2. Match Time ──
        // DriverStation otomatik olarak /FMSInfo altında yayınlar.
        // Elastic bunu kendi Match Time widget'ıyla gösterir.
        // Ek bir şey yapmaya gerek yok.

        // ── 3. Hub Active ──
        SmartDashboard.putBoolean("Hub Active", false);

        // ── 4. Intake Angle ──
        SmartDashboard.putNumber("Intake Angle", 0.0);

        // ── 5. USB Camera (intake kamerası) ──
        setupUsbCamera();

    }

    /**
     * USB kamerayı CameraServer üzerinden başlatır.
     * Elastic'te NetworkTables → "CameraServer" altında görünür.
     * Sürükleyip Camera Stream widget olarak eklersiniz.
     *
     * ⚠ Kamera port numarasını (0) bağlantınıza göre değiştirin.
     *   Çözünürlüğü düşük tutun — yüksek çözünürlük bandwidth yer,
     *   Elastic performansını etkiler.
     */
    private void setupUsbCamera() {
        try {
            CameraServer.startAutomaticCapture("Intake Camera", 0);
        } catch (Exception e) {
            System.err.println("[Dashboard] USB kamera ayarlanamadı: " + e.getMessage());
        }
    }

    /**
     * Limelight'ın MJPEG stream'ini CameraServer'a HttpCamera olarak ekler.
     * Böylece Elastic'te Camera Stream widget olarak sürüklenebilir.
     *
     * ⚠ IP adresini kendi Limelight yapılandırmanıza göre düzenleyin.
     *   Varsayılan: http://10.90.21.11:5800
     *   (Team 9021 → 10.90.21.xx)
     */
    private void setupLimelightStream() {
        try {
            HttpCamera limelightCam = new HttpCamera(
                    "Limelight",
                    "http://10.90.21.11:5800/stream.mjpg",
                    HttpCameraKind.kMJPGStreamer);
            CameraServer.addCamera(limelightCam);

            var server = CameraServer.addSwitchedCamera("Limelight Stream");
            server.setSource(limelightCam);
        } catch (Exception e) {
            System.err.println("[Dashboard] Limelight stream ayarlanamadı: " + e.getMessage());
        }
    }

    /**
     * robotPeriodic() içinde her döngü çağırılır.
     */
    public void update() {
        // ── 1. Field pozisyon güncelle ──
        Pose2d pose = drivetrain.getState().Pose;
        if (pose != null) {
            field.setRobotPose(pose);
        }

        // ── 2. Match Time ──
        SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());

        // ── 3. Hub Active (shift mekaniği) ──
        SmartDashboard.putBoolean("Hub Active", isHubActive());

        // ── 4. Intake açısı ──
        SmartDashboard.putNumber("Intake Angle", getIntakeAngleDegrees());
    }

    // ── Hub shift mekaniği + arm açısı ──

    public boolean isHubActive() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isEmpty()) {
            return false;
        }
        if (DriverStation.isAutonomousEnabled()) {
            return true;
        }
        if (!DriverStation.isTeleopEnabled()) {
            return false;
        }

        double matchTime = DriverStation.getMatchTime();
        String gameData = DriverStation.getGameSpecificMessage();

        if (gameData.isEmpty()) {
            return true;
        }

        boolean redInactiveFirst = false;
        switch (gameData.charAt(0)) {
            case 'R' -> redInactiveFirst = true;
            case 'B' -> redInactiveFirst = false;
            default -> { return true; }
        }

        boolean shift1Active = switch (alliance.get()) {
            case Red -> !redInactiveFirst;
            case Blue -> redInactiveFirst;
        };

        if (matchTime > 130) {
            return true;              // Geçiş süresi
        } else if (matchTime > 105) {
            return shift1Active;      // Shift 1
        } else if (matchTime > 80) {
            return !shift1Active;     // Shift 2
        } else if (matchTime > 55) {
            return shift1Active;      // Shift 3
        } else if (matchTime > 30) {
            return !shift1Active;     // Shift 4
        } else {
            return true;              // Endgame
        }
    }

    private double getIntakeAngleDegrees() {
        return arm.getAngleDeg();
    }
}