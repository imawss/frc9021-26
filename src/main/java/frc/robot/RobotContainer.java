package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.AllianceZoneShootCommand;
import frc.robot.commands.AutoShootCommand;
import frc.robot.commands.ContinuousAimCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.intake.ExtendIntakeCommand;
import frc.robot.commands.intake.IntakeCommands;
import frc.robot.commands.intake.RetractIntakeCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.hooper.HopperSubsystem;
import frc.robot.subsystems.intake.ArmSubsystem;
import frc.robot.subsystems.intake.IntakeRollerSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class RobotContainer {

        private final double MaxSpeed = 0.5 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
        private final double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

        private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                        .withDeadband(MaxSpeed * 0.1)
                        .withRotationalDeadband(MaxAngularRate * 0.1)
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        private final Telemetry logger = new Telemetry(MaxSpeed);

        private final CommandXboxController joystick = new CommandXboxController(0);

        public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
        // public final IntakeSubsystem intake = new IntakeSubsystem();
        public final IntakeRollerSubsystem intakeRoller = new IntakeRollerSubsystem();
        public final ShooterSubsystem shooter = new ShooterSubsystem();
        public final HopperSubsystem hopper = new HopperSubsystem();
        public final FeederSubsystem feeder = new FeederSubsystem();
        private final ArmSubsystem arm = new ArmSubsystem();

        private final SendableChooser<Command> autoChooser;

        private final ContinuousAimCommand continuousAim;
        private final ShootCommand shootCommand;

        public RobotContainer() {
                NamedCommands.registerCommand("AutoShoot",
                                new AutoShootCommand(shooter, hopper, feeder, drivetrain));

                NamedCommands.registerCommand("intakeDown",
                                IntakeCommands.intakeDown(arm, intakeRoller));
                NamedCommands.registerCommand("intakeUp",
                                IntakeCommands.intakeUp(arm, intakeRoller));

                autoChooser = AutoBuilder.buildAutoChooser("Tests");
                SmartDashboard.putData("AutoMode", autoChooser);

                continuousAim = new ContinuousAimCommand(
                                drivetrain,
                                joystick::getLeftY,
                                joystick::getLeftX,
                                MaxSpeed);

                shootCommand = new ShootCommand(shooter, hopper, feeder, drivetrain);

                configureBindings();

                FollowPathCommand.warmupCommand().schedule();
        }

        private void configureBindings() {

                drivetrain.setDefaultCommand(
                                drivetrain.applyRequest(() -> drive
                                                .withVelocityX(-joystick.getLeftY() * MaxSpeed)
                                                .withVelocityY(-joystick.getLeftX() * MaxSpeed)
                                                .withRotationalRate(-joystick.getRightX() * MaxAngularRate)));

                // Idle when disabled
                final var idle = new SwerveRequest.Idle();
                RobotModeTriggers.disabled().whileTrue(
                                drivetrain.applyRequest(() -> idle).ignoringDisable(true));

                joystick.leftTrigger().whileTrue(continuousAim);
                joystick.rightTrigger().whileTrue(shootCommand);

                // joystick.a().onTrue(new ExtendIntakeCommand(intake));
                // joystick.y().onTrue(new RetractIntakeCommand(intake));
                joystick.y().whileTrue(new AllianceZoneShootCommand(shooter, hopper, feeder, drivetrain,
                                joystick::getLeftY, joystick::getLeftX, MaxSpeed));

                joystick.x().onTrue(Commands.runOnce(intakeRoller::intake, intakeRoller));
                joystick.b().onTrue(Commands.runOnce(intakeRoller::stop, intakeRoller));

                joystick.leftBumper().whileTrue(
                                Commands.startEnd(intakeRoller::eject, intakeRoller::stop, intakeRoller));

                joystick.rightBumper().whileTrue(
                                Commands.startEnd(hopper::eject, hopper::stop, hopper));

                joystick.povDown().whileTrue(
                                Commands.run(() -> arm.manualDrive(+1), arm));

                joystick.povUp().whileTrue(
                                Commands.run(() -> arm.manualDrive(-1), arm));

                arm.setDefaultCommand(
                                Commands.run(() -> arm.manualDrive(0), arm)
                                                .withName("Arm Hold"));

                joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
                joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
                joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
                joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

                drivetrain.registerTelemetry(logger::telemeterize);
        }

        public Command getAutonomousCommand() {
                return autoChooser.getSelected();
        }
}