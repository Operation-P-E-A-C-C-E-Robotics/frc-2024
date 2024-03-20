package frc.lib.telemetry;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.StructArrayLogEntry;
import edu.wpi.first.util.datalog.StructLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import frc.lib.vision.LimelightHelpers;
import frc.robot.Robot;

/**
 * Telemetry for a limelight camera, intended for use with apriltag tracking.
 * Doesn't need to log too much, since the limelight's own networktables are
 * already being logged. Just reformats some data for advantagescope.
 * <ul>
 *    <li>Array of corners, formatted for advantagescope</li>
 *    <li>Primary tag pose3d from robot</li>
 *    <li>Primary tag pose3d from field</li>
 *    <li>Raw botpose as pose3d</li>
 * </ul>
 * 
 * A HOT MESS COURTESY OF PEACCY
 * I LEGINTIMATELY HAVE NO IDEA WHAT I'M DOING
 */
public class LimelightTelemetry {
    private static final DataLog log = DataLogManager.getLog();
    private static final StructArrayLogEntry<Pose3d> tagPosePublisher = StructArrayLogEntry.create(log, "Limelight/Tag Poses", Pose3d.struct);
    private static final StructArrayLogEntry<Pose3d> tagPoseFromFieldPublisher = StructArrayLogEntry.create(log, "Limelight/Tag Poses From Field", Pose3d.struct);
    private static final StructLogEntry<Pose3d> rawBotPosePublisher = StructLogEntry.create(log, "Limelight/Bot Pose", Pose3d.struct);

    public static void update (String llName, Pose3d robotPose){
        var results = LimelightHelpers.getLatestResults(llName).targetingResults;
        Pose3d[] tagPosesFromField = new Pose3d[results.targets_Fiducials.length];
        Pose3d[] tagPosesFromRobot = new Pose3d[results.targets_Fiducials.length];
        for(int i = 0; i < results.targets_Fiducials.length; i++){
            tagPosesFromField[i] = results.targets_Fiducials[i].getTargetPose_RobotSpace();
            tagPosesFromRobot[i] = robotPose.plus(new Transform3d(tagPosesFromField[i].getTranslation(), tagPosesFromField[i].getRotation()));
        }
        
        tagPosePublisher.append(tagPosesFromRobot);
        tagPoseFromFieldPublisher.append(tagPosesFromField);
        if(results.botpose.length == 6){
            rawBotPosePublisher.append(robotPose);
        }

        if(Robot.isSimulation()){
            tagPoseFromFieldPublisher.append(new Pose3d[] {
                new Pose3d(0, 0, 1, new Rotation3d(0, 0, 0)),
                new Pose3d(2, 2, 1, new Rotation3d(0, 0, 0)),
                new Pose3d(2, 0, 0.4, new Rotation3d(1, 0, 0)),
            });

            rawBotPosePublisher.append(new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0)));
        }
    }
}
