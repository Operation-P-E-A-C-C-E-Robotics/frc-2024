package frc.robot.planners;

import java.util.function.Supplier;
import java.util.function.BooleanSupplier;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.lib.util.AllianceFlipUtil;
import frc.lib.util.LinearInterpolate;
import frc.lib.vision.LimelightHelpers;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

/**
 * In charge of calculating the correct angles for the pivot and drivetrain,
 * and the flywheel velocity, to shoot at.
 */
public class AimPlanner {
    private final Supplier<ChassisSpeeds> robotRelativeChassisSpeeds;
    private final BooleanSupplier shootWhileMoving; // enable correction for drivetrain velocity

    private ShotAngle correctedShotAngle = new ShotAngle(Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0), 0);
    private ShotAngle uncorrectedShotAngle = new ShotAngle(Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0), 0);
    private ShotAngle measuredShotAngle = new ShotAngle(Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0), 0);

    private LinearFilter llAngleFilter = LinearFilter.movingAverage(20);
    
    private boolean isSotm = false;
    private boolean isSimpleLocalizer = false;

    private final Translation2d targetCenterTranslation = new Translation2d(
        FieldConstants.Speaker.centerSpeakerOpening.getX(),
        FieldConstants.Speaker.centerSpeakerOpening.getY()
    );

    private final Translation2d apriltagTranslation = new Translation2d(
        FieldConstants.aprilTags.getTagPose(7).get().getTranslation().getX(), 
        FieldConstants.aprilTags.getTagPose(7).get().getTranslation().getY()
    );

    private final double TARGET_HEIGHT = Units.inchesToMeters(57.5);
    private final double LIMELIGHT_CAMERA_HEIGHT = Units.inchesToMeters(6.5);
    private final double LIMELIGHT_CAMERA_ANGLE = Units.degreesToRadians(34.311);

    //TODO
    private final double PHOTONVISION_CAMERA_HEIGHT = Units.inchesToMeters(6.5);
    private final double PHOTONVISION_CAMERA_ANGLE = Units.degreesToRadians(34.311);

    private final double SIMPLE_LOCALIZER_DISTANCE_FUDGE = -0.3;

    private final double[][] distanceCalibrationData = {
        {54, 41, 31, 27, 24}, // pivot angles (deg)
        {40, 43, 47, 50, 53}, // flywheel speed rps
        {1, 2, 3, 4, 5}  //distances (m)
    };

    // private final double[][] distanceCalibrationData = {
    //     {54, 42, 32.5, 29, 26}, // pivot angles (deg)
    //     {40, 43, 47, 50, 53}, // flywheel speed rps
    //     {1, 2, 3, 4, 5}  //distances (m)
    // };

    private final LinearInterpolate pivotInterpolator = new LinearInterpolate(distanceCalibrationData[2], distanceCalibrationData[0]);
    private final LinearInterpolate flywheelAngularVelocityInterpolater = new LinearInterpolate(distanceCalibrationData[2], distanceCalibrationData[1]);

    private Rotation2d drivetrainAngle = new Rotation2d();
    private Rotation2d pivotAngle = new Rotation2d();
    private double flywheelAngularVelocity = 0;

    //what the desired velocity of the different mechanisms are based off the robot's speed, 
    //so we can add a feedforward to them that will match the robots motion to track the target better
    private double drivetrainAngularVelocity = 0;
    private double pivotAngularVelocity = 0;
    private double shooterAngularAcceleration = 0;

    private double distanceToTarget = 0;

    private double limelighttXOffset = 0; //difference between tx and wanted rotation for target center

    /* TELEMETRY */ //note: SOTM = Shoot on the Move
    private final NetworkTable aimTable = NetworkTableInstance.getDefault().getTable("Aim Planner");
    private final DoublePublisher distanceToTargetPublisher = aimTable.getDoubleTopic("Distance to Target").publish();
    private final DoublePublisher angleToTargetPublisher = aimTable.getDoubleTopic("Angle to Target").publish();
    private final DoublePublisher pivotAnglePublisher = aimTable.getDoubleTopic("Pivot Angle Regression (deg)").publish();
    private final DoublePublisher shooterVelocityPublisher = aimTable.getDoubleTopic("Shooter Velocity Regression (rps)").publish();
    private final DoublePublisher exitVelocityPublisher = aimTable.getDoubleTopic("Exit Velocity Conversion (m/s)").publish();

    private final DoublePublisher sotmPivotAnglePublisher = aimTable.getDoubleTopic("SOTM Corrected Pivot Angle").publish();
    private final DoublePublisher sotmExitVelocityPublisher = aimTable.getDoubleTopic("SOTM Corrected Exit Velocity").publish();
    private final DoublePublisher sotmDrivetrainAnglePublisher = aimTable.getDoubleTopic("SOTM Corrected Drivetrain Angle").publish();

    private final DoublePublisher drivetrainAngularVelocityPublisher = aimTable.getDoubleTopic("SuperSOTM Drivetrain Angular Velocity").publish();
    private final DoublePublisher pivotAngularVelocityPublisher = aimTable.getDoubleTopic("SuperSOTM Pivot Angular Velocity").publish();
    private final DoublePublisher shooterAngularAccelerationPublisher = aimTable.getDoubleTopic("SuperSOTM Shooter Angular Acceleration").publish();

    public AimPlanner (Supplier<Pose2d> robotPoseSupplier, Supplier<ChassisSpeeds> robotRelativeChassisSpeeds, BooleanSupplier shootWhileMoving) {
        this.robotRelativeChassisSpeeds = robotRelativeChassisSpeeds;
        this.shootWhileMoving = shootWhileMoving;
    }

    public void update() {
        var blueOriginPose = Swerve.getInstance().getPose();
        var blueTargetTranslation = AllianceFlipUtil.apply(targetCenterTranslation);
        distanceToTarget = blueOriginPose.getTranslation().getDistance(blueTargetTranslation);

        Rotation2d angleToTarget = blueOriginPose.getTranslation().minus(blueTargetTranslation).getAngle();
        Rotation2d angleToTag = blueOriginPose.getTranslation().minus(apriltagTranslation).getAngle();
        isSimpleLocalizer = false;

        limelighttXOffset = angleToTag.getDegrees() - angleToTarget.getDegrees();

        //experimental: use the old-style limelight targeting to get the angle and distance to the target
        //to aim faster with accumulated odometry error
        var photonTargetingResults = Constants.Cameras.primaryPhotonvisionCamera.getCamera().getLatestResult();
        var hasPhotonvisionHeading = false;
        // for(var target : photonTargetingResults.getTargets()) {
        //     if(target.getFiducialId() == (AllianceFlipUtil.shouldFlip() ? 4 : 7)){
        //         angleToTarget = AllianceFlipUtil.apply(Swerve.getInstance().getPose()).getRotation().plus(Rotation2d.fromDegrees((AllianceFlipUtil.shouldFlip() ? 1 : -1) *(target.getYaw()*0.5) - 180 /*- limelighttXOffset*/));
        //         angleToTarget = new Rotation2d(llAngleFilter.calculate((AllianceFlipUtil.shouldFlip() ? -1 : 1) * angleToTarget.getRadians()));
        //         if(AllianceFlipUtil.shouldFlip()) angleToTarget = angleToTarget.minus(Rotation2d.fromDegrees(180));
        //         hasPhotonvisionHeading = true;
        //         // if(RobotContainer.getInstance().getOdometryError() > 5) {
        //         //     distanceToTarget = ((TARGET_HEIGHT - PHOTONVISION_CAMERA_HEIGHT) / Math.tan(PHOTONVISION_CAMERA_ANGLE + Units.degreesToRadians(target.getPitch()))) + SIMPLE_LOCALIZER_DISTANCE_FUDGE;
        //         //     isSimpleLocalizer = true;
        //         // }
        //     }
        // }

        // if(!hasPhotonvisionHeading) {
        //     var targetingResults = LimelightHelpers.getLatestResults(Constants.Cameras.frontLimelight);
        //     for(var result : targetingResults.targetingResults.targets_Fiducials) {
        //         if(result.fiducialID == (AllianceFlipUtil.shouldFlip() ? 4 : 7)) {
        //             angleToTarget = AllianceFlipUtil.apply(Swerve.getInstance().getPose()).getRotation().plus(Rotation2d.fromDegrees((AllianceFlipUtil.shouldFlip() ? 1 : -1) *(result.tx*0.8) - 180 /*- limelighttXOffset*/));
        //             angleToTarget = new Rotation2d(llAngleFilter.calculate((AllianceFlipUtil.shouldFlip() ? -1 : 1) * angleToTarget.getRadians()));
        //             if(AllianceFlipUtil.shouldFlip()) angleToTarget = angleToTarget.minus(Rotation2d.fromDegrees(180));
        //             if(RobotContainer.getInstance().getOdometryError() > 5) {
        //                 distanceToTarget = ((TARGET_HEIGHT - LIMELIGHT_CAMERA_HEIGHT) / Math.tan(LIMELIGHT_CAMERA_ANGLE + Units.degreesToRadians(result.ty))) + SIMPLE_LOCALIZER_DISTANCE_FUDGE;
        //                 isSimpleLocalizer = true;
        //             }
        //         }
        //     }
        // }

        if(!isSimpleLocalizer) {
            llAngleFilter.reset();
        }


        Rotation2d pivotAngle = Rotation2d.fromDegrees(pivotInterpolator.interpolate(distanceToTarget));
        double flywheelAngularVelocity = flywheelAngularVelocityInterpolater.interpolate(distanceToTarget);
        double exitVelocity = RPSToExitVelocity(flywheelAngularVelocity);

        distanceToTargetPublisher.accept(distanceToTarget);
        angleToTargetPublisher.accept(angleToTarget.getDegrees());
        pivotAnglePublisher.accept(pivotAngle.getDegrees());
        shooterVelocityPublisher.accept(flywheelAngularVelocity);
        exitVelocityPublisher.accept(exitVelocity);

        uncorrectedShotAngle = new ShotAngle(angleToTarget, pivotAngle, exitVelocity);
        measuredShotAngle = new ShotAngle(
            blueOriginPose.getRotation(), 
            Pivot.getInstance().getPivotPosition(), 
            RPSToExitVelocity(Shooter.getInstance().getFlywheelVelocity())
        );

        if(!shootWhileMoving.getAsBoolean()) {
            drivetrainAngularVelocity = 0;
            pivotAngularVelocity = 0;
            shooterAngularAcceleration = 0;
            this.pivotAngle = pivotAngle;
            this.flywheelAngularVelocity = flywheelAngularVelocity;
            this.drivetrainAngle = angleToTarget;
            isSotm = false;
        }
        if(!shootWhileMoving.getAsBoolean()) return;
        isSotm = true;

        ChassisSpeeds robotVelocity = ChassisSpeeds.fromRobotRelativeSpeeds(robotRelativeChassisSpeeds.get(), blueOriginPose.getRotation());
        if(AllianceFlipUtil.shouldFlip()) robotVelocity.vxMetersPerSecond *= -1;
        robotVelocity.vxMetersPerSecond *= 0.5; //avoid overcorrecting x velocity

        correctedShotAngle = ShotAngle.correctFromChassisSpeeds(uncorrectedShotAngle, robotVelocity, blueOriginPose.getRotation());
        sotmPivotAnglePublisher.accept(correctedShotAngle.getPivotAngle().getDegrees());
        sotmExitVelocityPublisher.accept(correctedShotAngle.getExitVelocity());
        sotmDrivetrainAnglePublisher.accept(correctedShotAngle.getDrivetrainAngle().getDegrees());
        this.pivotAngle = correctedShotAngle.getPivotAngle();
        this.flywheelAngularVelocity = exitVelocityToRPS(correctedShotAngle.getExitVelocity());
        this.drivetrainAngle = correctedShotAngle.getDrivetrainAngle();


        //calculate the angular velocities of the mechanisms
        //first get the robots velocity relative to the target
        Translation2d robotVelocityTranslation = new Translation2d(robotVelocity.vxMetersPerSecond, robotVelocity.vyMetersPerSecond);
        Translation2d targetRelativeVelocity = robotVelocityTranslation.rotateBy(angleToTarget);
        double deltaDistance = targetRelativeVelocity.getX();

        //use the distarce to the target to get the angular velocity
        drivetrainAngularVelocity = Units.radiansToDegrees((targetRelativeVelocity.getY() / distanceToTarget));
        pivotAngularVelocity = pivotInterpolator.derivative(distanceToTarget) * deltaDistance;
        shooterAngularAcceleration = flywheelAngularVelocityInterpolater.derivative(distanceToTarget) * deltaDistance;

        drivetrainAngularVelocityPublisher.accept(drivetrainAngularVelocity);
        pivotAngularVelocityPublisher.accept(pivotAngularVelocity);
        shooterAngularAccelerationPublisher.accept(shooterAngularAcceleration);
    }

    public Rotation2d getTargetDrivetrainAngle() {
        return drivetrainAngle.plus(Rotation2d.fromDegrees(AllianceFlipUtil.shouldFlip() ? 0 : 180));
    }

    public Rotation2d getLimelightTXOffset() {
        return Rotation2d.fromDegrees(limelighttXOffset);
    }

    public Rotation2d getTargetPivotAngle() {
        return pivotAngle;
    }

    public ShotAngle getWantedShotAngle() {
        return isSotm ? correctedShotAngle : uncorrectedShotAngle;
    }

    public ShotAngle getMeasuredShotAngle() {
        return measuredShotAngle;
    }

    public boolean isSotm() {
        return isSotm;
    }

    public boolean isSimpleLocalizer() {
        return isSimpleLocalizer;
    }

    public double getTargetFlywheelVelocityRPS() {
        return flywheelAngularVelocity;
    }

    public double getDrivetrainAngularVelocity() {
        return drivetrainAngularVelocity;
    }

    public double getPivotAngularVelocity() {
        return pivotAngularVelocity;
    }

    public double getShooterAngularAcceleration() {
        return shooterAngularAcceleration;
    }

    private double exitVelocityToRPS(double exitVelocity) {
        return (exitVelocity / (Constants.Shooter.flywheelDiameter * Math.PI) / Constants.Shooter.flywheelGearRatio) * Constants.Shooter.flywheelEfficiency;
    }

    public double getDistanceToTarget(){
        return distanceToTarget;
    }

    private double RPSToExitVelocity(double rps) {
        return (rps * Constants.Shooter.flywheelDiameter * Math.PI * Constants.Shooter.flywheelGearRatio) / Constants.Shooter.flywheelEfficiency;
    }

    public static class ShotAngle {
        private final Rotation2d drivetrainAngle;
        private final Rotation2d pivotAngle;
        private final double exitVelocity;

        public ShotAngle(Rotation2d drivetrainAngle, Rotation2d pivotAngle, double exitVelocity) {
            this.drivetrainAngle = drivetrainAngle;
            this.pivotAngle = pivotAngle;
            this.exitVelocity = exitVelocity;
        }

        public Rotation2d getDrivetrainAngle() {
            return drivetrainAngle;
        }

        public Rotation2d getPivotAngle() {
            return pivotAngle;
        }

        public double getExitVelocity() {
            return exitVelocity;
        }

        public Translation3d getRobotCentricVector () {
            var vector = new Translation3d(exitVelocity, new Rotation3d(0, -pivotAngle.getRadians(), AllianceFlipUtil.apply(drivetrainAngle).getRadians()));
            // System.out.println("Robot aiming vector: " + vector.toString());
            return vector;
        }

        public Translation3d getFieldCentricVector (Rotation2d robotAngle) {
            var vector =  getRobotCentricVector().rotateBy(new Rotation3d(0, 0, robotAngle.getRadians()));
            // System.out.println("field centric vector: " + vector.toString());
            return vector;
        } //57.5

        public static ShotAngle fromRobotCentricVector (Translation3d vector) {
            return new ShotAngle(
                AllianceFlipUtil.apply(new Rotation2d(vector.getX(), vector.getY())),
                new Rotation2d(vector.getX(), vector.getZ()),
                vector.getNorm()
            );
        }

        public static ShotAngle fromFieldCentricVector (Translation3d vector, Rotation2d robotAngle) {
            return fromRobotCentricVector(vector.rotateBy(new Rotation3d(0, 0, -robotAngle.getRadians())));
        }

        public static ShotAngle correctFromChassisSpeeds (ShotAngle shotAngle, ChassisSpeeds speeds, Rotation2d heading) {
            var robotVelocity = new Translation3d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, 0);
            var correctedVector = shotAngle.getFieldCentricVector(heading).plus(robotVelocity);
            return fromFieldCentricVector(correctedVector, heading);
        }

        public String toString () {
            return String.format("Drivetrain Angle: %f, Pivot Angle: %f, Exit Velocity: %f", drivetrainAngle.getDegrees(), pivotAngle.getDegrees(), exitVelocity);
        }
    }
}
