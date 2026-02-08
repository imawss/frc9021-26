package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class Limelight extends SubsystemBase {
    private final String limelightName = "limelight";
    
    public double getTx() {
        return LimelightHelpers.getTX(limelightName);
    }

    public double getTy() {
        return LimelightHelpers.getTY(limelightName);
    }

    public boolean hasTarget() {
        return LimelightHelpers.getTV(limelightName);
    }
    
    public void setPipeline(int pipeline) {
        LimelightHelpers.setPipelineIndex(limelightName, pipeline);
    }
    
    public void setLEDMode(int mode) {
        switch (mode) {
            case 0:
                LimelightHelpers.setLEDMode_PipelineControl(limelightName);
                break;
            case 1:
                LimelightHelpers.setLEDMode_ForceOff(limelightName);
                break;
            case 2:
                LimelightHelpers.setLEDMode_ForceBlink(limelightName);
                break;
            case 3:
                LimelightHelpers.setLEDMode_ForceOn(limelightName);
                break;
        }
    }
    
    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Limelight/HasTarget", hasTarget());
        SmartDashboard.putNumber("Limelight/TX", getTx());
        SmartDashboard.putNumber("Limelight/TY", getTy());
    }
}