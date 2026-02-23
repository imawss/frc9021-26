// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.CompleteShootSequence;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.hooper.HopperSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.util.FuelSim;

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
    public final ShooterSubsystem shooter = new ShooterSubsystem();
    public final HopperSubsystem hopper = new HopperSubsystem();
    public final FeederSubsystem feeder = new FeederSubsystem();

    public final FuelSim fuelSim;

    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        fuelSim = RobotBase.isSimulation() ? configureFuelSim() : null;

        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);

        configureBindings();

        FollowPathCommand.warmupCommand().schedule();
    }

    private FuelSim configureFuelSim() {
        FuelSim sim = new FuelSim("FuelSim");

        sim.registerRobot(
                0.87, // robot width (left side to right side)
                0.87, // robot length (front bumper to back bumper)
                0.20, // bumper height (floor to top of bumpers)
                () -> drivetrain.getState().Pose,
                () -> drivetrain.getState().Speeds);

        sim.setSubticks(20); // default is 5

        SmartDashboard.putData("FuelSim/Reset Fuel",
                Commands.runOnce(() -> {
                    sim.clearFuel();
                    sim.spawnStartingFuel();
                }).ignoringDisable(true).withName("Reset Fuel"));

        sim.start();
        System.out.println("[FuelSim] Initialized. Balls spawned on field.");
        return sim;
    }

    private void configureBindings() {

        drivetrain.setDefaultCommand(
                drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed)
                        .withVelocityY(-joystick.getLeftX() * MaxSpeed)
                        .withRotationalRate(-joystick.getRightX() * MaxAngularRate)));

        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
                drivetrain.applyRequest(() -> idle).ignoringDisable(true));

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(
                () -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        joystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        SmartDashboard.putData("Test Shoot 2m",
                new CompleteShootSequence(shooter, hopper, feeder, 2.0, fuelSim));
        SmartDashboard.putData("Test Shoot 3m",
                new CompleteShootSequence(shooter, hopper, feeder, 3.0, fuelSim));
        SmartDashboard.putData("Test Shoot 4m",
                new CompleteShootSequence(shooter, hopper, feeder, 4.0, fuelSim));

        SmartDashboard.putData("Shooter Spin 3500 RPM",
                Commands.runOnce(() -> shooter.setVelocityRPM(3500), shooter));
        SmartDashboard.putData("Shooter Stop",
                Commands.runOnce(shooter::stop, shooter));

        SmartDashboard.putData("Burst Shoot 3x",
                Commands.sequence(
                        new CompleteShootSequence(shooter, hopper, feeder, 4.0, fuelSim),
                        new CompleteShootSequence(shooter, hopper, feeder, 4.0, fuelSim),
                        new CompleteShootSequence(shooter, hopper, feeder, 4.0, fuelSim)));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public void setInitialPoseForAlliance() {
        var alliance = edu.wpi.first.wpilibj.DriverStation.getAlliance()
                .orElse(edu.wpi.first.wpilibj.DriverStation.Alliance.Blue);
        Pose2d startPose;
        if (alliance == edu.wpi.first.wpilibj.DriverStation.Alliance.Blue) {
            startPose = new Pose2d(1, 1, Rotation2d.kZero);
            System.out.println("Blue Alliance - Pose: " + startPose);
        } else {
            startPose = new Pose2d(14, 1, Rotation2d.fromDegrees(180));
            System.out.println("Red Alliance - Pose: " + startPose);
        }
        drivetrain.resetPose(startPose);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}