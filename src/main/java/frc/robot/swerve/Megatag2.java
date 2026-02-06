package frc.robot.swerve;
 
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import edu.wpi.first.math.VecBuilder;

//create class
class Megatag2 extends SubsystemBase{

    //initializer
    Megatag2(){

    }
   

    //function that updates your position using limelight megatag2
    //yawDegrees is current robot yaw in degrees
    //yaw Angular velocity is the angular velocity in degrees per second
    //pose estimator is your robots pose estimator
    public void updateVisionReading(double yawDegrees, double yawAngularVelocityDegreesPerSecond, 
            SwerveDrivePoseEstimator poseEstimator){
        //boolean for whether or not we should use the update
        boolean doRejectUpdate = false;
        //sets the limelight to use an outside gyro
        //make sure to set the limelight name to your limelights name
        LimelightHelpers.SetIMUMode("limelight", 0);

        //use gyro to set orientation
        LimelightHelpers.SetRobotOrientation("limelight", yawDegrees,0, 0, 0, 0, 0);
        //gets the pose with bottom blue corner at 0,0
        //make sure name of limelight is currect
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");

        //makes sure we have an estimate
        if(mt2 != null){
        // if our angular velocity is greater than 360 degrees per second, ignore vision updates
            if(Math.abs(yawAngularVelocityDegreesPerSecond) > 360)
            {
                doRejectUpdate = true;
            }
            //reject update if we don't have tags or a reading
            if(mt2.tagCount == 0 || mt2 == null)
            {
                doRejectUpdate = true;
            }
            //add measurement to pose estimator
            if(!doRejectUpdate)
            {
                poseEstimator.setVisionMeasurementStdDevs(new Matrix<N3,N1>(VecBuilder.fill(.7,.7,999999)));
                poseEstimator.addVisionMeasurement(mt2.pose, mt2.timestampSeconds);
            }
        }
    }
}