package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import swervelib.SwerveDrive;

import limelight.Limelight;
import limelight.networktables.AngularVelocity3d;
import limelight.networktables.LimelightPoseEstimator;
import limelight.networktables.LimelightPoseEstimator.EstimationMode;
import limelight.networktables.LimelightSettings.ImuMode;
import limelight.networktables.LimelightSettings.LEDMode;
import limelight.networktables.Orientation3d;
import limelight.networktables.PoseEstimate;

public class SwerveSubsystem extends SubsystemBase{
    // swerve drive object (yagsl)
    //private final SwerveDrive swerveDrive;

    //private Limelight limelight;
   // private LimelightPoseEstimator poseEstimator;

    private double distanceToHub = 0.0;

    public double getDistanceToHub(){
        if(Constants.SwerveConstants.IS_LIMELIGHT_ENABLED){
            return distanceToHub;
        }
        return 0.0;
    }
}
