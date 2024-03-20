package frc.lib.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.lib.util.Median;

public class AprilTagFinder {
    private final int observations = 50;
    private String limelight;
    private Median medianX = new Median(observations);
    private Median medianY = new Median(observations);
    private Median medianZ = new Median(observations);
    private Median medianW = new Median(observations);
    private Median medianXQ = new Median(observations);
    private Median medianYQ = new Median(observations);
    private Median medianZQ = new Median(observations);


    public AprilTagFinder(String limelight){
        this.limelight = limelight;
    }

    public void update(Pose3d robotPose) {
        var results = LimelightHelpers.getLatestResults(limelight).targetingResults;
        if(results.targets_Fiducials.length == 0) return;
        Pose3d tagFromBot = results.targets_Fiducials[0].getTargetPose_RobotSpace();
        Pose3d tagFromField = robotPose.plus(new Transform3d(tagFromBot.getTranslation(), tagFromBot.getRotation()));
        medianX.add(tagFromField.getTranslation().getX());
        medianY.add(tagFromField.getTranslation().getY());
        medianZ.add(tagFromField.getTranslation().getZ());
        medianXQ.add(tagFromField.getRotation().getQuaternion().getX());
        medianYQ.add(tagFromField.getRotation().getQuaternion().getY());
        medianZQ.add(tagFromField.getRotation().getQuaternion().getZ());
        medianW.add(tagFromField.getRotation().getQuaternion().getW());
    }

    public void reset(){
        medianX = new Median(observations);
        medianY = new Median(observations);
        medianZ = new Median(observations);
        medianW = new Median(observations);
        medianXQ = new Median(observations);
        medianYQ = new Median(observations);
        medianZQ = new Median(observations);
    }

    public Pose3d getTagPose(){
        return new Pose3d(
            medianX.get(),
            medianY.get(),
            medianZ.get(),
            new Rotation3d(new Quaternion(
                medianW.get(),
                medianXQ.get(),
                medianYQ.get(),
                medianZQ.get()
            ))
        );
    }
}
