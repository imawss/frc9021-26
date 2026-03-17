package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.math.geometry.Pose2d;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.commands.AutoShootCommand;
import frc.robot.commands.ContinuousAimCommand;
import frc.robot.commands.ShootAtRPMCommand;
import frc.robot.commands.ShootCommand;

import frc.robot.commands.intake.ExtendIntakeCommand;
import frc.robot.commands.intake.RetractIntakeCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.hooper.HopperSubsystem;
import frc.robot.subsystems.intake.IntakeRollerSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class RobotContainer {

        private double MaxSpeed = 0.5 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
        private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

        private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                        .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
        private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
        private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

        private final Telemetry logger = new Telemetry(MaxSpeed);

        private final CommandXboxController joystick = new CommandXboxController(0);

        public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
        public final IntakeSubsystem intake = new IntakeSubsystem();
        public final IntakeRollerSubsystem intakeRoller = new IntakeRollerSubsystem();
        public final ShooterSubsystem shooter = new ShooterSubsystem();
        public final HopperSubsystem hopper = new HopperSubsystem();
        public final FeederSubsystem feeder = new FeederSubsystem();

        private final SendableChooser<Command> autoChooser;

        private ContinuousAimCommand continuousAim;

        public RobotContainer() {
                NamedCommands.registerCommand("AutoShoot",
                                new AutoShootCommand(shooter, hopper, feeder, drivetrain));

                autoChooser = AutoBuilder.buildAutoChooser("Tests");

                configureBindings();

                FollowPathCommand.warmupCommand().schedule();
        }

        private void configureBindings() {

                // ── Default drive ─────────────────────────────────────────────────────
                drivetrain.setDefaultCommand(
                                drivetrain.applyRequest(() -> drive
                                                .withVelocityX(-joystick.getLeftY() * MaxSpeed)
                                                .withVelocityY(-joystick.getLeftX() * MaxSpeed)
                                                .withRotationalRate(-joystick.getRightX() * MaxAngularRate)));

                final var idle = new SwerveRequest.Idle();
                RobotModeTriggers.disabled().whileTrue(
                                drivetrain.applyRequest(() -> idle).ignoringDisable(true));

                // ── Drive utility ─────────────────────────────────────────────────────
                /*
                 * joystick.a().whileTrue(drivetrain.applyRequest(() -> brake)); //brake mod
                 * joystick.b().whileTrue(drivetrain.applyRequest(
                 * () -> point.withModuleDirection(
                 * new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));
                 * 
                 * joystick.leftTrigger().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric
                 * )); // reset field centric
                 * 
                 */

                // ── SysId ─────────────────────────────────────────────────────────────
                joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
                joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
                joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
                joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

                // ── Continuous Aim — hold left bumper ────────────────────────────────
                continuousAim = new ContinuousAimCommand(
                                drivetrain,
                                () -> joystick.getLeftY(),
                                () -> joystick.getLeftX(),
                                MaxSpeed);

                joystick.leftTrigger().onTrue(Commands.run(hopper::eject, hopper));

                // ── Shoot — hold right bumper ──────────────────────────────────────────
                // Spins up shooter to distance-based RPM, feeds when ready.
                // Pair with left trigger (ContinuousAim) to aim and shoot simultaneously.

                joystick.rightTrigger().whileTrue(Commands.run(hopper::feed, hopper));
                joystick.a().onTrue(Commands.run(hopper::stop, hopper));
                joystick.b().onTrue(Commands.run(intakeRoller::stop, intakeRoller));

                joystick.x().onTrue(
                                Commands.runOnce(intake::extend, intake)
                                                .alongWith(Commands.run(intakeRoller::intake, intakeRoller)));

                joystick.y().onTrue(
                                Commands.runOnce(intake::retract, intake)
                                                .alongWith(Commands.run(intakeRoller::stop, intakeRoller)));

                // joystick.x().onTrue(Commands.runOnce(intakeRoller::stop, intakeRoller));

                joystick.rightBumper().whileTrue(new ShootAtRPMCommand(shooter, hopper, feeder, 2500));

                drivetrain.registerTelemetry(logger::telemeterize);
                SmartDashboard.putData("Autonomous/Auto Chooser", autoChooser);

        }

        public Command getAutonomousCommand() {
                return autoChooser.getSelected();
        }
}