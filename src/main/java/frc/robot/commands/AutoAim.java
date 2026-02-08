package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;

public class AutoAim extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final Limelight limelight;
    
    private final double kMaxSpeed = TunerConstants.kSpeedAt12Volts.in(edu.wpi.first.units.Units.MetersPerSecond);
    private final double kMaxAngularSpeed = 1.5 * Math.PI;
    
    private final SwerveRequest.RobotCentric driveRequest = new SwerveRequest.RobotCentric();

    public AutoAim(CommandSwerveDrivetrain drivetrain, Limelight limelight) {
        this.drivetrain = drivetrain;
        this.limelight = limelight;
        addRequirements(drivetrain, limelight);
    }

    @Override
    public void initialize() {
        System.out.println("========== AUTO AIM STARTED ==========");
        limelight.setPipeline(0);
        limelight.setLEDMode(3);
    }

    @Override
    public void execute() {
        final var rot_limelight = limelight_aim_proportional();
        final var forward_limelight = limelight_range_proportional();
        
        drivetrain.setControl(driveRequest
            .withVelocityX(forward_limelight)
            .withVelocityY(0)
            .withRotationalRate(rot_limelight)
        );
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(new SwerveRequest.Idle());
        limelight.setLEDMode(1);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
    double limelight_aim_proportional()
    {    
        // kP (constant of proportionality)
        // this is a hand-tuned number that determines the aggressiveness of our proportional control loop
        // if it is too high, the robot will oscillate around.
        // if it is too low, the robot will never reach its target
        // if the robot never turns in the correct direction, kP should be inverted.
        double kP = .035;
        // tx ranges from (-hfov/2) to (hfov/2) in degrees. If your target is on the rightmost edge of 
        // your limelight 3 feed, tx should return roughly 31 degrees.
        double targetingAngularVelocity = LimelightHelpers.getTX("limelight") * kP;
        // convert to radians per second for our drive method
        targetingAngularVelocity *= kMaxAngularSpeed;
        //invert since tx is positive when the target is to the right of the crosshair
        targetingAngularVelocity *= -1.0;
        return targetingAngularVelocity;
    }
    
    /**
     * simple proportional ranging control with Limelight's "ty" value
     * this works best if your Limelight's mount height and target mount height are different.
     * if your limelight and target are mounted at the same or similar heights, use "ta" (area) for target ranging rather than "ty"
     */
    double limelight_range_proportional()
    {    
        double kP = .1;
        double targetingForwardSpeed = LimelightHelpers.getTY("limelight") * kP;
        targetingForwardSpeed *= kMaxSpeed;
        targetingForwardSpeed *= -1.0;
        return targetingForwardSpeed;
    }
}