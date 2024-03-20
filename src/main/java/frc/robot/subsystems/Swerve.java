package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.RobotCentric;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Joystick;
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
import frc.lib.vision.PeaccyVision;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.RobotContainer;
import frc.robot.OI;
import frc.robot.RobotContainer;
import frc.robot.planners.AimPlanner;

import static frc.robot.Constants.Swerve.*;

public class Swerve extends SubsystemBase {
    protected final PeaccefulSwerve swerve;

    private final SwerveRequest.ApplyChassisSpeeds autonomousRequest = new SwerveRequest.ApplyChassisSpeeds()
                                                                                        .withDriveRequestType(DriveRequestType.Velocity);
    private final SendableChooser<Pose2d> poseSeedChooser = new SendableChooser<>();

    private Transform2d visionDiscrepancy = new Transform2d();
    // private LimelightHelper limelight;

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

    public void characterizeSteer(){
        swerve.setControl(new SwerveRequest.SysIdSwerveSteerGains().withVolts(null));
    }

    public void characterizeTranslation(){
        swerve.setControl(new SwerveRequest.SysIdSwerveTranslation());
    }

    public void characterizeRotation(){
        swerve.setControl(new SwerveRequest.SysIdSwerveRotation());
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

    private static PeaccyVision eyes = new PeaccyVision(
        new ApriltagCamera.ApriltagPhotonvision(Constants.Cameras.primaryPhotonvision, Constants.Cameras.robotToPrimaryPhotonvision, FieldConstants.aprilTags, 1),
        new ApriltagCamera.ApriltagPhotonvision(Constants.Cameras.secondaryPhotonvision, Constants.Cameras.robotToSecondaryPhotonvision, FieldConstants.aprilTags, 0.5),
        new ApriltagCamera.ApriltagLimelight(Constants.Cameras.frontLimelight, 0.1)
    );

    @Override
    public void periodic() {
        if(SmartDashboard.getBoolean("seed pose", false)) {
            resetOdometry(poseSeedChooser.getSelected());
            SmartDashboard.putBoolean("seed pose", false);
        }

        BaseStatusSignal.refreshAll(swerve.getPigeon2().getAccelerationX(), swerve.getPigeon2().getAccelerationY(), swerve.getPigeon2().getAccelerationZ());
        var acceleration = swerve.getPigeon2().getAccelerationX().getValue() + swerve.getPigeon2().getAccelerationY().getValue() + swerve.getPigeon2().getAccelerationZ().getValue();
        eyes.update(getPose(), acceleration, new Translation2d(getChassisSpeeds().vxMetersPerSecond, getChassisSpeeds().vyMetersPerSecond).getNorm());

        swerve.addVisionMeasurement(
            eyes.getPose(),
            eyes.getTimestamp(),
            eyes.getStDev()
        );

        //TODO: update limelight telemetry
        // LimelightTelemetry.update(Constants.Cameras.frontLimelight, swerve.getPose3d());
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

