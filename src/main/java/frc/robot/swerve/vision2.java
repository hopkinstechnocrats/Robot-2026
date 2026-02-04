package frc.robot.swerve;

import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.RawFiducial;

public class vision2 {
    RawFiducial[] fiducials = LimelightHelpers.getRawFiducials("");

    for(RawFiducial fiducial : fiducials){
        int id = fiducial.id;
    }
}
