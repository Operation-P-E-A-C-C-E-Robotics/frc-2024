package frc.lib.telemetry;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain.SwerveDriveState;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleArrayLogEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.StructLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Robot;

/**
 * Handles all the telemetry for the swerve drive. Designed to be used with advantagescope.
 */
public class SwerveTelemetry {
    private static final NetworkTable swerveTable = NetworkTableInstance.getDefault().getTable("Swerve");
    private static final DataLog log = DataLogManager.getLog();

    private static final StructPublisher<Pose3d> swervePosePublisher = swerveTable.getStructTopic("Robot Pose", Pose3d.struct).publish();
    private static final StructPublisher<Pose3d> pathplannerTargetPosePublisher = swerveTable.getStructTopic("Target Pose", Pose3d.struct).publish();
    private static final StructArrayPublisher <Pose2d> pathplannerTrajectoryPublisher = swerveTable.getStructArrayTopic("Path", Pose2d.struct).publish();
    
    private static final DoubleArrayLogEntry swerveDataPublisher = new DoubleArrayLogEntry(log, "Swerve/Swerve Measured Data");
    private static final DoubleArrayLogEntry swerveRequestedData = new DoubleArrayLogEntry(log, "Swerve/Swerve Requested Data");

    private static final DoublePublisher measuredXVelocity = swerveTable.getDoubleTopic("Measured X Velocity").publish();
    private static final DoublePublisher measuredYVelocity = swerveTable.getDoubleTopic("Measured Y Velocity").publish();
    private static final DoublePublisher measuredAngularVelocity = swerveTable.getDoubleTopic("Measured Angular Velocity").publish();
    
    private static final DoubleLogEntry odometryPeriod = new DoubleLogEntry(log, "Swerve/Odometry Period");


    
    private static final DoubleLogEntry swerveRequestedXVelocity = new DoubleLogEntry(log, "Swerve/Requested X Velocity");
    private static final DoubleLogEntry swerveRequestedYVelocity = new DoubleLogEntry(log, "Swerve/Requested Y Velocity");
    private static final DoubleLogEntry swerveRequestedRawXVelocity = new DoubleLogEntry(log, "Swerve/Requested Raw X Velocity");
    private static final DoubleLogEntry swerveRequestedRawYVelocity = new DoubleLogEntry(log, "Swerve/Requested Raw Y Velocity");

    private static final DoubleLogEntry swerveRequestedAngularVelocity = new DoubleLogEntry(log, "Swerve/Requested Angular Velocity");
    private static final DoubleLogEntry swerveRequestedAutoHeadingAngle = new DoubleLogEntry(log, "Swerve/Requested Auto Heading Angle");
    private static final BooleanLogEntry requestFieldCentricPublisher = new BooleanLogEntry(log, "Swerve/Request Field Centric");
    private static final BooleanLogEntry requestAutoAnglePublisher = new BooleanLogEntry(log, "Swerve/Request Auto Angle");
    private static final BooleanLogEntry requestOpenLoopPublisher = new BooleanLogEntry(log, "Swerve/Request Open Loop");
    private static final BooleanLogEntry requestLockInPublisher = new BooleanLogEntry(log, "Swerve/Request Lock In");
    private static final BooleanLogEntry requestZeroOdometryPublisher = new BooleanLogEntry(log, "Swerve/Request Zero Odometry");

    private static final NetworkTable autoHeadingTable = swerveTable.getSubTable("Auto Heading");
    private static final DoublePublisher autoHeadingAngle = autoHeadingTable.getDoubleTopic("Target").publish();
    private static final DoublePublisher autoHeadingError = autoHeadingTable.getDoubleTopic("Error").publish();
    private static final DoubleLogEntry autoHeadingPComponent = new DoubleLogEntry(log, "Swerve/Auto Heading P Component");
    private static final DoubleLogEntry autoHeadingFeedForward = new DoubleLogEntry(log, "Swerve/Auto Heading Feed Forward");
    private static final DoubleLogEntry autoHeadingTrajectoryVelocity = new DoubleLogEntry(log, "Swerve/Auto Heading Trajectory Velocity");
    private static final DoubleLogEntry autoHeadingTrajectoryAcceleration = new DoubleLogEntry(log, "Swerve/Auto Heading Trajectory Acceleration");
    private static final DoubleLogEntry autoHeadingTrajectoryPosition = new DoubleLogEntry(log, "Swerve/Auto Heading Trajectory Position");
    private static final BooleanLogEntry autoHeadingCurrentLimited = new BooleanLogEntry(log, "Swerve/Auto Heading Current Limited");

    private static final StructLogEntry <Translation2d> positionCorrectionDeltaPublisher = StructLogEntry.create(log, "Swerve/Position Correction Delta", Translation2d.struct);
    private static final StructLogEntry <Translation2d> positionCorrectionMeasuredPublisher = StructLogEntry.create(log, "Swerve/Position Correction Measured", Translation2d.struct);

    
    private static final Mechanism2d swerve = new Mechanism2d(5, 5);
    private static final MechanismRoot2d frontLeftModule = swerve.getRoot("front left", 1, 1);
    private static final MechanismRoot2d frontRightModule = swerve.getRoot("front right", 1, 4);
    private static final MechanismRoot2d rearLeftModule = swerve.getRoot("rear left", 4, 1);
    private static final MechanismRoot2d rearRightModule = swerve.getRoot("rear right", 4, 4);

    private static final MechanismLigament2d frontLeftLigament = new MechanismLigament2d("front left ligament", 0, 0);
    private static final MechanismLigament2d frontRightLigament = new MechanismLigament2d("front right ligament", 0, 0);
    private static final MechanismLigament2d rearLeftLigament = new MechanismLigament2d("rear left ligament", 0, 0);
    private static final MechanismLigament2d rearRightLigament = new MechanismLigament2d("rear right ligament", 0, 0);

    private static final Field2d field = new Field2d();

    static {
        frontLeftModule.append(frontLeftLigament);
        frontRightModule.append(frontRightLigament);
        rearLeftModule.append(rearLeftLigament);
        rearRightModule.append(rearRightLigament);

        if(Robot.isSimulation()) SmartDashboard.putData("Swerve Mech", swerve);
        SmartDashboard.putData("Field", field);

        PathPlannerLogging.setLogActivePathCallback((poses) -> {
            var poseArray = poses.toArray(new Pose2d[0]);
            pathplannerTrajectoryPublisher.accept(poseArray);
            field.getObject("Path").setPoses(poseArray);
        });

        PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
            pathplannerTargetPosePublisher.accept(new Pose3d(pose));
            field.getObject("Target Pose").setPose(pose);
        });
    }

    public static void updateSwerveState(SwerveDriveState state, ChassisSpeeds measuredSpeeds, Pose3d pose) {
        swervePosePublisher.accept(pose);//TODO update once have 3d odometry stuff
        field.setRobotPose(state.Pose);

        frontLeftLigament.setAngle(state.ModuleStates[0].angle.getDegrees());
        frontRightLigament.setAngle(state.ModuleStates[1].angle.getDegrees());
        rearLeftLigament.setAngle(state.ModuleStates[2].angle.getDegrees());
        rearRightLigament.setAngle(state.ModuleStates[3].angle.getDegrees());

        frontLeftLigament.setLength(state.ModuleStates[0].speedMetersPerSecond / Constants.Swerve.pathfollowingMaxVelocity);
        frontRightLigament.setLength(state.ModuleStates[1].speedMetersPerSecond / Constants.Swerve.pathfollowingMaxVelocity);
        rearLeftLigament.setLength(state.ModuleStates[2].speedMetersPerSecond / Constants.Swerve.pathfollowingMaxVelocity);
        rearRightLigament.setLength(state.ModuleStates[3].speedMetersPerSecond / Constants.Swerve.pathfollowingMaxVelocity);

        /*
        format needed for advantagescope:
        [
            rotation_1, velocity_1,
            rotation_2, velocity_2,
            rotation_3, velocity_3,
            rotation_4, velocity_4
        ]
         */
        double[] swerveData = new double[]{
            state.ModuleStates[0].angle.getDegrees(), state.ModuleStates[0].speedMetersPerSecond,
            state.ModuleStates[1].angle.getDegrees(), state.ModuleStates[1].speedMetersPerSecond,
            state.ModuleStates[2].angle.getDegrees(), state.ModuleStates[2].speedMetersPerSecond,
            state.ModuleStates[3].angle.getDegrees(), state.ModuleStates[3].speedMetersPerSecond
        };
        swerveDataPublisher.append(swerveData);

        measuredXVelocity.accept(measuredSpeeds.vxMetersPerSecond);
        measuredYVelocity.accept(measuredSpeeds.vyMetersPerSecond);
        measuredAngularVelocity.accept(measuredSpeeds.omegaRadiansPerSecond);
        odometryPeriod.append(state.OdometryPeriod);
    }

    public static void updateSwerveCommand(double requestedXVelocity, 
                                            double requestedYVelocity, 
                                            double rawXVelocity,
                                            double rawYVelocity,
                                            double requestedAngularVelocity, 
                                            double requestedAutoHeading, 
                                            boolean isAutoHeading, 
                                            boolean isFieldRelative, 
                                            boolean isOpenLoop, 
                                            boolean isLockIn, 
                                            boolean isZeroOdometry) {
        swerveRequestedXVelocity.append(requestedXVelocity);
        swerveRequestedYVelocity.append(requestedYVelocity);
        swerveRequestedRawXVelocity.append(rawXVelocity);
        swerveRequestedRawYVelocity.append(rawYVelocity);
        swerveRequestedAngularVelocity.append(requestedAngularVelocity);
        swerveRequestedAutoHeadingAngle.append(requestedAutoHeading);
        requestFieldCentricPublisher.append(isFieldRelative);
        requestAutoAnglePublisher.append(isAutoHeading);
        requestOpenLoopPublisher.append(isOpenLoop);
        requestLockInPublisher.append(isLockIn);
        requestZeroOdometryPublisher.append(isZeroOdometry);
    }

    public static void updateAutoHeading(double targetAngle, 
                                        double error, 
                                        double pComponent, 
                                        double feedForward, 
                                        double trajectoryVelocity, 
                                        double trajectoryAcceleration, 
                                        double trajectoryPosition, 
                                        boolean isCurrentLimited) {
        autoHeadingAngle.accept(targetAngle);
        autoHeadingError.accept(error);
        autoHeadingPComponent.append(pComponent);
        autoHeadingFeedForward.append(feedForward);
        autoHeadingTrajectoryVelocity.append(trajectoryVelocity);
        autoHeadingTrajectoryAcceleration.append(trajectoryAcceleration);
        autoHeadingTrajectoryPosition.append(trajectoryPosition);
        autoHeadingCurrentLimited.append(isCurrentLimited);
    }

    public static void updateRequestedState(SwerveModuleState... states){
        //required for advantagescope:
        // [
        //     rotation_1, velocity_1,
        //     rotation_2, velocity_2,
        //     rotation_3, velocity_3,
        //     rotation_4, velocity_4
        // ]

        double[] swerveRequestedData = new double[]{
            states[0].angle.getDegrees(), states[0].speedMetersPerSecond,
            states[1].angle.getDegrees(), states[1].speedMetersPerSecond,
            states[2].angle.getDegrees(), states[2].speedMetersPerSecond,
            states[3].angle.getDegrees(), states[3].speedMetersPerSecond
        };
        SwerveTelemetry.swerveRequestedData.append(swerveRequestedData);
    }

    public static void updatePositionCorrection(Translation2d delta, Translation2d measured){
        positionCorrectionDeltaPublisher.append(delta);
        positionCorrectionMeasuredPublisher.append(measured);
    }
}
