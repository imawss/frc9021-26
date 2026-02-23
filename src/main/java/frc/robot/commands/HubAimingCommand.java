package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.hooper.HopperSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class HubAimingCommand extends Command {

    private static final Translation2d HUB_BLUE = new Translation2d(0.22, 5.55);
    private static final Translation2d HUB_RED  = new Translation2d(16.32, 5.55);

    private static final int    PIPELINE_INDEX        = 0;
    private static final double MAX_ANGULAR_VEL_DEG_S = 720.0;
    private static final double VISION_STD_XY         = 0.5;
    private static final double VISION_STD_THETA      = 9_999_999.0;

    private static final double HEADING_kP       = 7.0;
    private static final double HEADING_kI       = 0.0;
    private static final double HEADING_kD       = 0.2;
    private static final double AIM_DEADBAND_DEG = 2.0;
    private static final double SETTLE_TIME_S    = 0.10;

    private final CommandSwerveDrivetrain           drivetrain;
    private final ShooterSubsystem                  shooter;
    private final HopperSubsystem                   hopper;
    private final FeederSubsystem                   feeder;
    private final java.util.function.DoubleSupplier vxSupplier;
    private final java.util.function.DoubleSupplier vySupplier;
    private final String                            limelightName;

    private final SwerveRequest.FieldCentricFacingAngle aimRequest =
            new SwerveRequest.FieldCentricFacingAngle()
                    .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Timer   settleTimer  = new Timer();
    private       boolean firingActive = false;
    private Translation2d hubPosition;

    public HubAimingCommand(
            CommandSwerveDrivetrain drivetrain,
            ShooterSubsystem shooter,
            HopperSubsystem hopper,
            FeederSubsystem feeder,
            java.util.function.DoubleSupplier vxSupplier,
            java.util.function.DoubleSupplier vySupplier,
            String limelightName) {

        this.drivetrain    = drivetrain;
        this.shooter       = shooter;
        this.hopper        = hopper;
        this.feeder        = feeder;
        this.vxSupplier    = vxSupplier;
        this.vySupplier    = vySupplier;
        this.limelightName = limelightName;

        aimRequest.HeadingController.setPID(HEADING_kP, HEADING_kI, HEADING_kD);

        addRequirements(drivetrain, shooter, hopper, feeder);
    }

    @Override
    public void initialize() {
        LimelightHelpers.setPipelineIndex(limelightName, PIPELINE_INDEX);

        boolean isRed = DriverStation.getAlliance()
                .orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red;
        hubPosition = isRed ? HUB_RED : HUB_BLUE;

        firingActive = false;
        settleTimer.reset();
        settleTimer.stop();

        SmartDashboard.putBoolean("HubAim/Active", true);
        SmartDashboard.putString("HubAim/Alliance", isRed ? "RED" : "BLUE");
    }

    @Override
    public void execute() {
        updateVisionPose();

        Pose2d        robotPose      = drivetrain.getState().Pose;
        Translation2d toHub          = hubPosition.minus(robotPose.getTranslation());
        double        distanceMeters = toHub.getNorm();
        Rotation2d    desiredHeading = toHub.getAngle();

        drivetrain.setControl(
                aimRequest
                        .withVelocityX(vxSupplier.getAsDouble())
                        .withVelocityY(vySupplier.getAsDouble())
                        .withTargetDirection(desiredHeading));

        shooter.setVelocityForDistance(distanceMeters);

        double headingErrorDeg = Math.abs(
                robotPose.getRotation().minus(desiredHeading).getDegrees());
        if (headingErrorDeg > 180.0) headingErrorDeg = 360.0 - headingErrorDeg;

        boolean aimed        = headingErrorDeg < AIM_DEADBAND_DEG;
        boolean shooterReady = shooter.atTargetVelocity();

        if (aimed && shooterReady) {
            if (!firingActive) {
                settleTimer.restart();
                firingActive = true;
            }
            if (settleTimer.hasElapsed(SETTLE_TIME_S)) {
                hopper.feed();
                feeder.feed();
            }
        } else {
            firingActive = false;
            settleTimer.reset();
            settleTimer.stop();
            hopper.stop();
            feeder.stop();
        }

        SmartDashboard.putNumber("HubAim/Distance (m)",         distanceMeters);
        SmartDashboard.putNumber("HubAim/DesiredHeading (deg)", desiredHeading.getDegrees());
        SmartDashboard.putNumber("HubAim/HeadingError (deg)",   headingErrorDeg);
        SmartDashboard.putBoolean("HubAim/Aimed",               aimed);
        SmartDashboard.putBoolean("HubAim/ShooterReady",        shooterReady);
        SmartDashboard.putBoolean("HubAim/Firing",              firingActive && settleTimer.hasElapsed(SETTLE_TIME_S));
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
        hopper.stop();
        feeder.stop();
        settleTimer.stop();
        SmartDashboard.putBoolean("HubAim/Active", false);
        SmartDashboard.putBoolean("HubAim/Firing", false);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    private void updateVisionPose() {
        double yawDeg = drivetrain.getState().Pose.getRotation().getDegrees();
        LimelightHelpers.SetRobotOrientation(limelightName, yawDeg, 0, 0, 0, 0, 0);

        PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);

        if (mt2 == null || mt2.tagCount == 0) {
            SmartDashboard.putBoolean("HubAim/VisionRejected", true);
            SmartDashboard.putString("HubAim/VisionRejectReason", "no tags");
            return;
        }

        double omegaDegS = Math.abs(
                Units.radiansToDegrees(drivetrain.getState().Speeds.omegaRadiansPerSecond));
        if (omegaDegS > MAX_ANGULAR_VEL_DEG_S) {
            SmartDashboard.putBoolean("HubAim/VisionRejected", true);
            SmartDashboard.putString("HubAim/VisionRejectReason", String.format("omega %.0f deg/s", omegaDegS));
            return;
        }

        drivetrain.addVisionMeasurement(
                mt2.pose,
                mt2.timestampSeconds,
                VecBuilder.fill(VISION_STD_XY, VISION_STD_XY, VISION_STD_THETA));

        SmartDashboard.putBoolean("HubAim/VisionRejected", false);
        SmartDashboard.putString("HubAim/VisionRejectReason", "none");
        SmartDashboard.putNumber("HubAim/VisionTagCount",    mt2.tagCount);
        SmartDashboard.putNumber("HubAim/VisionAvgDist (m)", mt2.avgTagDist);
    }
}