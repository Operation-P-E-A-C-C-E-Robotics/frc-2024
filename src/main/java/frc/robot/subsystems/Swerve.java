package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.safety.Inspiration;
import frc.lib.swerve.PeaccefulSwerve;
import frc.lib.swerve.SwerveDescription;
import frc.lib.swerve.SwerveDescription.PidGains;
import frc.lib.telemetry.SwerveTelemetry;
import frc.lib.util.AllianceFlipUtil;
import frc.lib.vision.ApriltagCamera;
import frc.lib.vision.LimelightHelpers;
import frc.lib.vision.PeaccyVision;
import frc.robot.Constants;
import frc.robot.auto.Autonomous;

import static frc.robot.Constants.Swerve.*;

import java.util.Optional;

public class Swerve extends SubsystemBase {
    protected final PeaccefulSwerve swerve;
    
    private final SwerveRequest.ApplyChassisSpeeds autonomousRequest = new SwerveRequest.ApplyChassisSpeeds()
    .withDriveRequestType(DriveRequestType.Velocity);
    private final SendableChooser<Pose2d> poseSeedChooser = new SendableChooser<>();

    private final NetworkTableEntry floorNoteWidth = LimelightHelpers.getLimelightNTTableEntry(Constants.Cameras.rearLimelight, "thor");
    private final NetworkTableEntry floorNoteHeight = LimelightHelpers.getLimelightNTTableEntry(Constants.Cameras.rearLimelight, "tvert");
    private final double NOTE_WIDTH = Units.inchesToMeters(14);

    private Transform2d visionDiscrepancy = new Transform2d();
    private Optional<Translation2d> noteFromRobot = Optional.empty();
    private Optional<Translation2d> noteFromField = Optional.empty();
    private Timer timeSinceFloorNoteUpdate = new Timer();
    // private LimelightHelper limelight;

    private static PeaccyVision eyes = new PeaccyVision(
        Constants.Cameras.primaryPhotonvisionCamera,
        // new ApriltagCamera.ApriltagPhotonvision(Constants.Cameras.secondaryPhotonvision, Constants.Cameras.robotToSecondaryPhotonvision, FieldConstants.aprilTags, 0.5),
        new ApriltagCamera.ApriltagLimelight(Constants.Cameras.frontLimelight, 0.1)
    );

    private Swerve() {
        swerve = SwerveDescription.generateDrivetrain(
            dimensions, 
            frontLeftIDs, 
            frontRightIDs, 
            rearLeftIDs, 
            rearRightIDs, 
            gearing, 
            offsets, 
            inversion, 
            physics, 
            driveGains, 
            angleGains, 
            pigeonCANId, 
            invertSteerMotors
        );

        swerve.setSteerCurrentLimit(steerMotorCurrentLimit);

        //pathplanner config
        AutoBuilder.configureHolonomic(this::getPose, this::resetOdometry, this::getChassisSpeeds, this::drive, pathFollowerConfig, AllianceFlipUtil::shouldFlip, this);

        //log swerve state data as fast as it comes in
        swerve.registerTelemetry((SwerveDriveState state) -> {
            SwerveTelemetry.updateSwerveState(state, ChassisSpeeds.fromRobotRelativeSpeeds(getChassisSpeeds(), getPose().getRotation()), swerve.getPose3d());
        });

        poseSeedChooser.setDefaultOption("zero", new Pose2d());
        poseSeedChooser.addOption("test", new Pose2d(1, 1, new Rotation2d()));
        poseSeedChooser.addOption("START 1", Autonomous.twoNoteStageSide.getStartPose());
        poseSeedChooser.addOption("START 2", Autonomous.twoNoteCenter.getStartPose());
        poseSeedChooser.addOption("START 3", Autonomous.twoNoteAmpSide.getStartPose());
        poseSeedChooser.addOption("START NEW AUTO", Autonomous.newAutoThing.getStartPose());
        poseSeedChooser.addOption("START DEFENCE", Autonomous.defence1.getStartPose());

        SmartDashboard.putData("POSE SEED", poseSeedChooser);
        SmartDashboard.putBoolean("seed pose", false);

        System.out.println("DriveTrain Initialized");
    }

    /**
     * make it go.
     * @param request the request to apply to the drivetrain.
     */
    public void drive(SwerveRequest request) {
        swerve.setControl(request);
    }

    /**
     * make it go in auto.
     * @param speeds the chassis speeds to apply to the drivetrain.
     */
    public void drive(ChassisSpeeds speeds) {
        // if(speeds.vxMetersPerSecond < 0.01 && speeds.vyMetersPerSecond < 0.01) {
        //     speeds = new ChassisSpeeds();
        // }
        drive(autonomousRequest.withSpeeds(speeds));
    }

    /**
     * the missile knows where it is at all times. it knows this because it knows where it isn't.
     * @return the pose of the robot.
     */
    public Pose2d getPose () {
        var pose = swerve.getState().Pose;
        if (pose == null) return new Pose2d();
        return pose;
    }

    public Transform2d getVisionDiscrepancy() {
        return visionDiscrepancy;
    }

    /**
     * this missile even knows how fast it's traveling. it knows this because it knows how fast it isn't traveling.
     * @return the chassis speeds of the robot.
     */
    public ChassisSpeeds getChassisSpeeds() {
        return swerve.getChassisSpeeds();
    }
    
    /**
     * sometimes, the missile forgets where it is, and it's not even where it's been.
     */
    public void resetOdometry() {
        swerve.seedFieldRelative();
    }

    public PeaccyVision getEyes(){
        return eyes;
    }

    /**
     * Hopefully a potential workaround for CTRE's moronic zeroing behavior.
     */
    public void attemptProperFieldCentricZeroing() {
        var cachedPose = getPose();
        resetOdometry(AllianceFlipUtil.apply(new Pose2d()));
        resetOdometry();
        resetOdometry(cachedPose);
    }

    /**
     * sometimes, we need to tell the missile where it is, and it's not even where it's been.
     * By subtracting where it's been from where it is, or where it's going from where it was, we get
     * where it should be.
     * @param pose the pose to set the robot to.
     */
    public void resetOdometry(Pose2d pose) {
        swerve.seedFieldRelative(pose);
    }

    public Rotation3d getGyroAngle() {
        return swerve.getRotation3d();
    }

    public double getTotalDriveCurrent(){
        return swerve.getTotalDriveCurrent();
    }

    public void updateDriveGains(PidGains gains){
        swerve.applyDriveConfigs(gains);
    }

    public void updateAngleGains(PidGains gains){
        swerve.applySteerConfigs(gains);
    }


    @Override
    public void periodic() {
        if(SmartDashboard.getBoolean("seed pose", false)) {
            resetOdometry(poseSeedChooser.getSelected());
            SmartDashboard.putBoolean("seed pose", false);
        }

        BaseStatusSignal.refreshAll(swerve.getPigeon2().getAccelerationX(), swerve.getPigeon2().getAccelerationY(), swerve.getPigeon2().getAccelerationZ());
        var acceleration = swerve.getPigeon2().getAccelerationX().getValue() + swerve.getPigeon2().getAccelerationY().getValue() + swerve.getPigeon2().getAccelerationZ().getValue();
        eyes.update(getPose(), acceleration, new Translation2d(getChassisSpeeds().vxMetersPerSecond, getChassisSpeeds().vyMetersPerSecond).getNorm());
        if(eyes.hasUpdated()){
            swerve.addVisionMeasurement(
                eyes.getPose(),
                eyes.getTimestamp(),
                eyes.getStDev()
            );
        }

        //update floor note tracking:
        var width = floorNoteWidth.getDouble(-1);
        var height = floorNoteHeight.getDouble(-1);

        if(width > 0 && height > 0) {
            var distance = (NOTE_WIDTH * Constants.Cameras.LIMELIGHT_FOCAL_LENGTH) / width;
            var angle = LimelightHelpers.getTX(Constants.Cameras.rearLimelight) + 180;
            noteFromRobot = Optional.of(new Translation2d(distance, Rotation2d.fromDegrees(angle)));
            noteFromField = Optional.of(getPose().getTranslation().plus(noteFromRobot.get()));
            timeSinceFloorNoteUpdate.restart();
        }
        if(timeSinceFloorNoteUpdate.get() > 0.2) {
            noteFromRobot = Optional.empty();
            noteFromField = Optional.empty();
        }

        //TODO: update limelight telemetry
        // LimelightTelemetry.update(Constants.Cameras.frontLimelight, swerve.getPose3d());
    }

    public Optional<Translation2d> getNoteFromRobot() {
        return noteFromRobot;
    }

    public Optional<Translation2d> getNoteFromField() {
        return noteFromField;
    }

    @Override
    public void simulationPeriodic() {
        swerve.updateSimState(Constants.period, 12);
    }

    public void register(Joystick j){
        Inspiration.fullPeacce(j);
    }

    private static final Swerve instance = new Swerve();
    public static Swerve getInstance(){
        return instance;
    }
}

