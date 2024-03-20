package frc.robot.commands;

import static frc.robot.Constants.Swerve.lockHeadingKP;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.swerve.PeaccyRequest;
import frc.lib.swerve.SwerveDescription.PidGains;
import frc.lib.telemetry.SwerveTelemetry;
import frc.lib.util.JoystickCurves;
import frc.lib.util.JoystickCurves.CurveType;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

/**
 * The organization / lack thereof of this file is a sin 
 * and i am sorry for writing it. To atone here is a fun poem about $h!tty code:
 * Messy code like poops on a shingle,
 * I'm sorry for writing it, it makes me tingle.
 * This file cannot be understood,
 * I'm sorry for writing it, it's not good.
 * - Peaccy
 */
public class PeaccyTuner extends Command {
    /* CONSTANTS BUT MUTABLE */
    private double teleopLinearMultiplier  = Constants.Swerve.teleopLinearMultiplier;
    private double teleopAngularMultiplier = Constants.Swerve.teleopAngularMultiplier;

    private double teleopLinearSpeedLimit = Constants.Swerve.teleopLinearSpeedLimit;
    private double teleopLowBatteryLinearSpeedLimit = Constants.Swerve.teleopLowBatteryLinearSpeedLimit;
    private double teleopLinearAngleLimit = Constants.Swerve.teleopLinearAngleLimit;
    private double teleopAngularRateLimit = Constants.Swerve.teleopAngularRateLimit;

    private double teleopLinearSpeedDeadband = Constants.Swerve.teleopLinearSpeedDeadband;
    private double teleopAngularVelocityDeadband = Constants.Swerve.teleopAngularVelocityDeadband;
    private double teleopDeadbandDebounceTime = Constants.Swerve.teleopDeadbandDebounceTime;

    private CurveType teleopLinearSpeedCurve = CurveType.HERRA4_5_F_CURVE;
    private CurveType teleopAngularVelocityCurve = CurveType.POWER;

    private double teleopLinearSpeedCurveSensitivity = 6;
    private double teleopAngularVelocityCurveSensitivity = 2;

    private int teleopPositionCorrectionIters = Constants.Swerve.teleopPositionCorrectionIters;

    private double autoHeadingKP = Constants.Swerve.autoHeadingKP;
    private double autoHeadingKV = Constants.Swerve.autoHeadingKV;
    private double autoHeadingKA = Constants.Swerve.autoHeadingKA;
    private double autoHeadingMaxVelocity = Constants.Swerve.autoHeadingMaxVelocity;
    private double autoHeadingMaxAcceleration = Constants.Swerve.autoHeadingMaxAcceleration;
    private double softHeadingCurrentLimit = Constants.Swerve.softHeadingCurrentLimit;
    private boolean useSoftHoldHeading = Constants.Swerve.useSoftHoldHeading;

    private PidGains driveGains = Constants.Swerve.driveGains;
    private PidGains angleGains = Constants.Swerve.angleGains;
    
    
    
    
    /* NORMAL VARIABLES */
    //(i copy/pasted this :D) - peaccy
    /* Initialize the suppliers to default values */
    private DoubleSupplier xVelocitySup = () -> 0,
                        yVelocitySup = () -> 0, 
                        angularVelocitySup = () -> 0, 
                        autoHeadingAngleSup = () -> 0;
    private BooleanSupplier isAutoAngleSup = () -> false, 
                            isFieldRelativeSup = () -> false, 
                            isOpenLoopSup = () -> false, 
                            isLockInSup = () -> false,
                            isZeroOdometrySup = () -> false;

    private Swerve driveTrain;

    /* "Swerve Requests" are what the drivetrain subsystem accepts. They figure out how to orient and drive the wheels. */
    private PeaccyRequest request; //custom fancy request than handles everything
    private final SwerveRequest.SwerveDriveBrake lockInRequest = new SwerveRequest.SwerveDriveBrake().withDriveRequestType(DriveRequestType.Velocity); //for X-locking the wheels

    /* Acceleration limiters for a consistent feel and to reduce power draw. */
    private SlewRateLimiter linearSpeedLimiter;
    private SlewRateLimiter lowBatteryLinearSpeedLimiter;
    private SlewRateLimiter linearAngleLimiter;
    private SlewRateLimiter angularVelocityLimiter;

    /* Debounce the deadband to prevent jitter when the joystick is near the edge of the deadband */
    private Debouncer linearDeadbandDebouncer;
    private Debouncer angularDeadbandDebouncer;

    private FallbackMode fallback = FallbackMode.Normal; //disable risky features

    /* DATA */
    private double maxLinearVelocity = 0;
    private double maxLinearAcceleration = 0;
    private double maxAngularVelocity = 0;
    private double maxAngularAcceleration = 0;

    private SendableChooser<CurveType> linearCurveChooser = new SendableChooser<>();
    private SendableChooser<CurveType> angularCurveChooser = new SendableChooser<>();

    /* NETWORKTABLES */
    //i decided to split the networktables into different tables for different catagoies of constants
    NetworkTable tunerTable = NetworkTableInstance.getDefault().getTable("Drivetrain Tuner");
    NetworkTable gains = tunerTable.getSubTable("Gains"); //pid gains & trajectory whatevers
    NetworkTable limits = tunerTable.getSubTable("Limits"); //max speeds, rate limiters, multipliers, etc.
    NetworkTable deadbands = tunerTable.getSubTable("Deadbands"); //deadbands & debounce time for said deadbands
    NetworkTable curves = tunerTable.getSubTable("Curves"); //joystick curves (no $h!t sherlock)
    NetworkTable data = tunerTable.getSubTable("Data"); //data to be displayed on the dashboard (why am i writing this comment)
    NetworkTable control = tunerTable.getSubTable("Control"); //just whatever to control the robot or whatever (zeroing odometry, updating gains, etc.)

    //gains
    DoubleEntry drivekPEntry = gains.getDoubleTopic("Drive kP").getEntry(driveGains.kP);
    DoubleEntry drivekIEntry = gains.getDoubleTopic("Drive kI").getEntry(driveGains.kI);
    DoubleEntry drivekDEntry = gains.getDoubleTopic("Drive kD").getEntry(driveGains.kD);
    DoubleEntry drivekVEntry = gains.getDoubleTopic("Drive kV").getEntry(driveGains.kV);
    DoubleEntry drivekAEntry = gains.getDoubleTopic("Drive kA").getEntry(driveGains.kA);

    DoubleEntry anglekPEntry = gains.getDoubleTopic("Angle kP").getEntry(angleGains.kP);
    DoubleEntry anglekIEntry = gains.getDoubleTopic("Angle kI").getEntry(angleGains.kI);
    DoubleEntry anglekDEntry = gains.getDoubleTopic("Angle kD").getEntry(angleGains.kD);
    DoubleEntry anglekVEntry = gains.getDoubleTopic("Angle kV").getEntry(angleGains.kV);
    DoubleEntry anglekAEntry = gains.getDoubleTopic("Angle kA").getEntry(angleGains.kA);

    DoubleEntry autoHeadingkPEntry = gains.getDoubleTopic("Auto Heading kP").getEntry(autoHeadingKP);
    DoubleEntry autoHeadingkVEntry = gains.getDoubleTopic("Auto Heading kV").getEntry(autoHeadingKV);
    DoubleEntry autoHeadingkAEntry = gains.getDoubleTopic("Auto Heading kA").getEntry(autoHeadingKA);
    DoubleEntry autoHeadingMaxVelocityEntry = gains.getDoubleTopic("Auto Heading Max Velocity").getEntry(autoHeadingMaxVelocity);
    DoubleEntry autoHeadingMaxAccelerationEntry = gains.getDoubleTopic("Auto Heading Max Acceleration").getEntry(autoHeadingMaxAcceleration);
    DoubleEntry softHeadingCurrentLimitEntry = gains.getDoubleTopic("Soft Heading Current Limit").getEntry(softHeadingCurrentLimit);

    IntegerEntry teleopPositionCorrectionItersEntry = gains.getIntegerTopic("Teleop Position Correction Iters").getEntry(teleopPositionCorrectionIters);
    
    //limits
    DoubleEntry teleopLinearMultiplierEntry = limits.getDoubleTopic("Teleop Linear Multiplier").getEntry(teleopLinearMultiplier);
    DoubleEntry teleopAngularMultiplierEntry = limits.getDoubleTopic("Teleop Angular Multiplier").getEntry(teleopAngularMultiplier);

    DoubleEntry teleopLinearSpeedLimitEntry = limits.getDoubleTopic("Teleop Linear Speed Limit").getEntry(teleopLinearSpeedLimit);
    DoubleEntry teleopLowBatteryLinearSpeedLimitEntry = limits.getDoubleTopic("Teleop Low Battery Linear Speed Limit").getEntry(teleopLowBatteryLinearSpeedLimit);
    DoubleEntry teleopLinearAngleLimitEntry = limits.getDoubleTopic("Teleop Linear Angle Limit").getEntry(teleopLinearAngleLimit);
    DoubleEntry teleopAngularRateLimitEntry = limits.getDoubleTopic("Teleop Angular Rate Limit").getEntry(teleopAngularRateLimit);

    //deadbands
    DoubleEntry teleopLinearSpeedDeadbandEntry = deadbands.getDoubleTopic("Teleop Linear Speed Deadband").getEntry(teleopLinearSpeedDeadband);
    DoubleEntry teleopAngularVelocityDeadbandEntry = deadbands.getDoubleTopic("Teleop Angular Velocity Deadband").getEntry(teleopAngularVelocityDeadband);
    DoubleEntry teleopDeadbandDebounceTimeEntry = deadbands.getDoubleTopic("Teleop Deadband Debounce Time").getEntry(teleopDeadbandDebounceTime);

    //curves
    DoubleEntry teleopLinearSpeedCurveSensitivityEntry = curves.getDoubleTopic("Teleop Linear Speed Curve Sensitivity").getEntry(teleopLinearSpeedCurveSensitivity);
    DoubleEntry teleopAngularVelocityCurveSensitivityEntry = curves.getDoubleTopic("Teleop Angular Velocity Curve Sensitivity").getEntry(teleopAngularVelocityCurveSensitivity);

    //data
    DoublePublisher maxLinearVelocityPublisher = data.getDoubleTopic("Max Linear Velocity").publish();
    DoublePublisher maxLinearAccelerationPublisher = data.getDoubleTopic("Max Linear Acceleration").publish();
    DoublePublisher maxAngularVelocityPublisher = data.getDoubleTopic("Max Angular Velocity").publish();
    DoublePublisher maxAngularAccelerationPublisher = data.getDoubleTopic("Max Angular Acceleration").publish();
    DoublePublisher wheelRadiusPublisher = data.getDoubleTopic("Wheel Radius").publish();
    DoubleArrayPublisher driveMotorErrors = data.getDoubleArrayTopic("Drive Motor Errors").publish();
    DoubleArrayPublisher angleMotorErrors = data.getDoubleArrayTopic("Angle Motor Errors").publish();

    //control
    BooleanEntry updateDriveGainsEntry = control.getBooleanTopic("Update Drive Gains").getEntry(false);
    BooleanEntry updateAngleGainsEntry = control.getBooleanTopic("Update Angle Gains").getEntry(false);
    BooleanEntry updatePeaccyRequestEntry = control.getBooleanTopic("Update Peaccy Request").getEntry(false);
    BooleanEntry updateLimitsDeadbandsCurvesEntry = control.getBooleanTopic("Update Limits, Deadbands, Curves").getEntry(false);
    BooleanEntry zeroOdometry = control.getBooleanTopic("Zero Odometry").getEntry(false);
    DoubleEntry wheelRadiusCalculatorDistance = control.getDoubleTopic("Wheel Radius Calculator Distance (m)").getEntry(2);
    
    BooleanEntry resetMaxVelocitiesEntry = control.getBooleanTopic("Reset Max Velocities").getEntry(false);
    BooleanEntry useSoftHoldHeadingEntry = control.getBooleanTopic("Use Soft Hold Heading").getEntry(useSoftHoldHeading);

    //update the limits, deadbands, and curves - all the now mutable constants - from the dashboard
    private void updateConstants(){
        //gains
        driveGains.kP = drivekPEntry.get(driveGains.kP);
        driveGains.kI = drivekIEntry.get(driveGains.kI);
        driveGains.kD = drivekDEntry.get(driveGains.kD);
        driveGains.kV = drivekVEntry.get(driveGains.kV);
        driveGains.kA = drivekAEntry.get(driveGains.kA);

        angleGains.kP = anglekPEntry.get(angleGains.kP);
        angleGains.kI = anglekIEntry.get(angleGains.kI);
        angleGains.kD = anglekDEntry.get(angleGains.kD);
        angleGains.kV = anglekVEntry.get(angleGains.kV);
        angleGains.kA = anglekAEntry.get(angleGains.kA);

        autoHeadingKP = autoHeadingkPEntry.get(autoHeadingKP);
        autoHeadingKV = autoHeadingkVEntry.get(autoHeadingKV);
        autoHeadingKA = autoHeadingkAEntry.get(autoHeadingKA);
        autoHeadingMaxVelocity = autoHeadingMaxVelocityEntry.get(autoHeadingMaxVelocity);
        autoHeadingMaxAcceleration = autoHeadingMaxAccelerationEntry.get(autoHeadingMaxAcceleration);
        softHeadingCurrentLimit = softHeadingCurrentLimitEntry.get(softHeadingCurrentLimit);

        teleopPositionCorrectionIters = (int) teleopPositionCorrectionItersEntry.get(teleopPositionCorrectionIters);

        //limits
        teleopLinearMultiplier = teleopLinearMultiplierEntry.get(teleopLinearMultiplier);
        teleopAngularMultiplier = teleopAngularMultiplierEntry.get(teleopAngularMultiplier);

        teleopLinearSpeedLimit = teleopLinearSpeedLimitEntry.get(teleopLinearSpeedLimit);
        teleopLowBatteryLinearSpeedLimit = teleopLowBatteryLinearSpeedLimitEntry.get(teleopLowBatteryLinearSpeedLimit);
        teleopLinearAngleLimit = teleopLinearAngleLimitEntry.get(teleopLinearAngleLimit);
        teleopAngularRateLimit = teleopAngularRateLimitEntry.get(teleopAngularRateLimit);


        //deadbands
        teleopLinearSpeedDeadband = teleopLinearSpeedDeadbandEntry.get(teleopLinearSpeedDeadband);
        teleopAngularVelocityDeadband = teleopAngularVelocityDeadbandEntry.get(teleopAngularVelocityDeadband);
        teleopDeadbandDebounceTime = teleopDeadbandDebounceTimeEntry.get(teleopDeadbandDebounceTime);

        //curves
        teleopLinearSpeedCurveSensitivity = teleopLinearSpeedCurveSensitivityEntry.get(teleopLinearSpeedCurveSensitivity);
        teleopAngularVelocityCurveSensitivity = teleopAngularVelocityCurveSensitivityEntry.get(teleopAngularVelocityCurveSensitivity);

        teleopLinearSpeedCurve = linearCurveChooser.getSelected();
        teleopAngularVelocityCurve = angularCurveChooser.getSelected();

        //control
        useSoftHoldHeading = useSoftHoldHeadingEntry.get(useSoftHoldHeading);
    }

    private void updateLimiters(){
        linearSpeedLimiter = new SlewRateLimiter(teleopLinearSpeedLimit);
        lowBatteryLinearSpeedLimiter = new SlewRateLimiter(teleopLowBatteryLinearSpeedLimit);
        linearAngleLimiter = new SlewRateLimiter(teleopLinearAngleLimit);
        angularVelocityLimiter = new SlewRateLimiter(teleopAngularRateLimit);

        linearDeadbandDebouncer = new Debouncer(teleopDeadbandDebounceTime, DebounceType.kBoth);
        angularDeadbandDebouncer = new Debouncer(teleopDeadbandDebounceTime, DebounceType.kBoth);
    }

    private void updatePeaccyRequest(){
        request  = new PeaccyRequest(
            autoHeadingMaxVelocity, 
            autoHeadingMaxAcceleration,
            Constants.Swerve.lockHeadingMaxVelocity,
            Constants.Swerve.lockHeadingMaxAcceleration,
            Constants.Swerve.teleopLinearMultiplier,
            autoHeadingKP, 
            autoHeadingKV, 
            autoHeadingKA, 
            lockHeadingKP,
            driveTrain::getChassisSpeeds, 
            driveTrain::getTotalDriveCurrent, 
            softHeadingCurrentLimit
        ).withRotationalDeadband(teleopAngularVelocityDeadband)
        .withSoftHoldHeading(useSoftHoldHeading)
        .withPositionCorrectionIterations(teleopPositionCorrectionIters);
    }

    private void updateDriveGains(){
        driveTrain.updateDriveGains(driveGains);
    }

    private void updateAngleGains(){
        driveTrain.updateAngleGains(angleGains);
    }
    
    
    /**
     * Updated helper to tune the drivetrain constants & drive mode.
     * Functionally similar to PeaccyDrive but with:
     * - mutable limiters and constants
     * - ability to write new pid gains to modules
     * - tons and tons of dashboard controls
     * @param driveTrain the swerve subsystem
     */
    public PeaccyTuner(Swerve driveTrain) { 
        this.driveTrain = driveTrain;
        updateLimiters();
        updatePeaccyRequest();

        addRequirements(driveTrain);
    }

    /**
     * give the supplier to control robot translatoin
     * @param xVelocitySup supplier that gives desired x velocity as a percentage of max velocity
     * @return this (so u can chain em)
     */
    public PeaccyTuner withTranslation(DoubleSupplier xVelocitySup){
        this.xVelocitySup = xVelocitySup;
        return this;
    }

    /**
     * give the supplier to control robot strafe
     * @param yVelocitySup supplier that gives desired y velocity as a percentage of max velocity
     * @return this (so u can chain em)
     */
    public PeaccyTuner withStrafe(DoubleSupplier yVelocitySup){
        this.yVelocitySup = yVelocitySup;
        return this;
    }

    /**
     * give the supplier to control robot rotation
     * @param angularVelocitySup supplier that gives desired angular velocity as a percentage of max velocity
     * @return this (so u can chain em)
     */
    public PeaccyTuner withRotation(DoubleSupplier angularVelocitySup){
        this.angularVelocitySup = angularVelocitySup;
        return this;
    }

    /**
     * give the supplier to control robot heading - will be overrided by rotation, if the rotation value is nonzero (below the deadband)
     * @param headingSup supplier that gives desired heading in field-centric degrees
     * @return this (so u can chain em)
     */
    public PeaccyTuner withHeading(DoubleSupplier headingSup){
        this.autoHeadingAngleSup = headingSup;
        return this;
    }

    /**
     * give the supplier for whether to X-lock the wheels when stopped
     * @param isLockInSup supplier that gives whether to X-lock the wheels when stopped
     * @return this (so u can chain em)
     */
    public PeaccyTuner isLockIn(BooleanSupplier isLockInSup){
        this.isLockInSup = isLockInSup;
        return this;
    }

    /**
     * give the supplier for whether to use field centric controls
     * @param isFieldRelativeSup supplier that gives whether to use field centric controls
     * @return this (so u can chain em)
     */
    public PeaccyTuner isFieldRelative(BooleanSupplier isFieldRelativeSup){
        this.isFieldRelativeSup = isFieldRelativeSup;
        return this;
    }

    /**
     * give the supplier for whether to use open loop controls (no velocity controllers on the drive motors)
     * @param isOpenLoopSup supplier that gives whether to use open loop controls
     * @return this (so u can chain em)
     */
    public PeaccyTuner isOpenLoop(BooleanSupplier isOpenLoopSup){
        this.isOpenLoopSup = isOpenLoopSup;
        return this;
    }

    /**
     * give the supplier for whether to zero the odometry (use with caution ofc since this resets field position)
     * @param isZeroOdometrySup supplier that gives whether to zero the odometry
     * @return this (so u can chain em)
     */
    public PeaccyTuner isZeroOdometry(BooleanSupplier isZeroOdometrySup){
        this.isZeroOdometrySup = isZeroOdometrySup;
        return this;
    }

    /**
     * whether to update the robot's target heading using the specified heading.
     * (otherwise, withHeading will be ignored)
     * @param useHeadingSup supplier that gives whether to use the heading
     * @return this (so u can chain em)
     */
    public PeaccyTuner useHeading(BooleanSupplier useHeadingSup){
        this.isAutoAngleSup = useHeadingSup;
        return this;
    }

    @Override
    public void initialize(){
        driveTrain.resetOdometry();
        request.withHeading(driveTrain.getPose().getRotation().getRadians());
        SmartDashboard.putString("Using DriveTrainTuner", "Yes, in case you're looking at the log and wondering what's going on, we are using DriveTrainTuner :)");

        /* WRITE INITIAL DASHBOARD VALUES */
        //gains
        drivekPEntry.set(driveGains.kP);
        drivekIEntry.set(driveGains.kI);
        drivekDEntry.set(driveGains.kD);
        drivekVEntry.set(driveGains.kV);
        drivekAEntry.set(driveGains.kA);

        anglekPEntry.set(angleGains.kP);
        anglekIEntry.set(angleGains.kI);
        anglekDEntry.set(angleGains.kD);
        anglekVEntry.set(angleGains.kV);
        anglekAEntry.set(angleGains.kA);

        autoHeadingkPEntry.set(autoHeadingKP);
        autoHeadingkVEntry.set(autoHeadingKV);
        autoHeadingkAEntry.set(autoHeadingKA);
        autoHeadingMaxVelocityEntry.set(autoHeadingMaxVelocity);
        autoHeadingMaxAccelerationEntry.set(autoHeadingMaxAcceleration);
        softHeadingCurrentLimitEntry.set(softHeadingCurrentLimit);

        teleopPositionCorrectionItersEntry.set(teleopPositionCorrectionIters);

        //limits
        teleopLinearMultiplierEntry.set(teleopLinearMultiplier);
        teleopAngularMultiplierEntry.set(teleopAngularMultiplier);

        teleopLinearSpeedLimitEntry.set(teleopLinearSpeedLimit);
        teleopLowBatteryLinearSpeedLimitEntry.set(teleopLowBatteryLinearSpeedLimit);
        teleopLinearAngleLimitEntry.set(teleopLinearAngleLimit);
        teleopAngularRateLimitEntry.set(teleopAngularRateLimit);


        //deadbands
        teleopLinearSpeedDeadbandEntry.set(teleopLinearSpeedDeadband);
        teleopAngularVelocityDeadbandEntry.set(teleopAngularVelocityDeadband);
        teleopDeadbandDebounceTimeEntry.set(teleopDeadbandDebounceTime);

        //curves
        teleopLinearSpeedCurveSensitivityEntry.set(teleopLinearSpeedCurveSensitivity);
        teleopAngularVelocityCurveSensitivityEntry.set(teleopAngularVelocityCurveSensitivity);

        //data
        maxLinearVelocityPublisher.set(maxLinearVelocity);
        maxLinearAccelerationPublisher.set(maxLinearAcceleration);
        maxAngularVelocityPublisher.set(maxAngularVelocity);
        maxAngularAccelerationPublisher.set(maxAngularAcceleration);

        //control
        updateDriveGainsEntry.set(false);
        updateAngleGainsEntry.set(false);
        updatePeaccyRequestEntry.set(false);
        updateLimitsDeadbandsCurvesEntry.set(false);
        zeroOdometry.set(false);

        wheelRadiusCalculatorDistance.set(2);

        resetMaxVelocitiesEntry.set(false);
        useSoftHoldHeadingEntry.set(useSoftHoldHeading);

        

        //curve choosers
        linearCurveChooser.addOption("Linear", CurveType.LINEAR);
        linearCurveChooser.addOption("Power Curve", CurveType.POWER);
        linearCurveChooser.addOption("Exponential", CurveType.EXPONENTIAL);
        linearCurveChooser.setDefaultOption("Herra 4.5 F Curve", CurveType.HERRA4_5_F_CURVE);
        linearCurveChooser.addOption("Herra 9 F Curve", CurveType.HERRA9_F_CURVE);
        linearCurveChooser.addOption("Herra Mixed", CurveType.HERRA_MIXED);
        linearCurveChooser.addOption("Cheesy Curve", CurveType.CHEESY_CURVE);

        angularCurveChooser.addOption("Linear", CurveType.LINEAR);
        angularCurveChooser.setDefaultOption("Power Curve", CurveType.POWER);
        angularCurveChooser.addOption("Exponential", CurveType.EXPONENTIAL);
        angularCurveChooser.addOption("Herra 4.5 F Curve", CurveType.HERRA4_5_F_CURVE);
        angularCurveChooser.addOption("Herra 9 F Curve", CurveType.HERRA9_F_CURVE);
        angularCurveChooser.addOption("Herra Mixed", CurveType.HERRA_MIXED);
        angularCurveChooser.addOption("Cheesy Curve", CurveType.CHEESY_CURVE);

        SmartDashboard.putData("Linear Curve", linearCurveChooser);
        SmartDashboard.putData("Angular Curve", angularCurveChooser);

    }

    @Override
    public void execute() {
        updateConstants(); 

        boolean updateDriveGains = updateDriveGainsEntry.get(false);
        boolean updateAngleGains = updateAngleGainsEntry.get(false);
        boolean updateAutoHeadingGains = updatePeaccyRequestEntry.get(false);
        boolean updateLimitsDeadbandsCurves = updateLimitsDeadbandsCurvesEntry.get(false);

        if(updateDriveGains) {
            updateDriveGains();
            updateDriveGainsEntry.set(false);
        }

        if(updateAngleGains) {
            updateAngleGains();
            updateAngleGainsEntry.set(false);
        }

        if(updateAutoHeadingGains) {
            updatePeaccyRequest();
            updatePeaccyRequestEntry.set(false);
        }

        if(updateLimitsDeadbandsCurves) {
            updateLimiters();
            request.withRotationalDeadband(teleopAngularVelocityDeadband);
            updateLimitsDeadbandsCurvesEntry.set(false);
        }

        //calculate the wheel radius from encoder distance and wheel rotations
        //start by going from odometry x distance to wheel rotations with the current radius:
        double wheelRotations = driveTrain.getPose().getTranslation().getX() / (2 * Math.PI * Constants.Swerve.gearing.wheelRadius);
        if(wheelRotations != 0){
            //then calculate the wheel radius to get to the specified distance with the wheel rotations:
            double wheelRadius = wheelRadiusCalculatorDistance.get(2) / (2 * Math.PI * wheelRotations);
            wheelRadiusPublisher.set(wheelRadius);
        } else {
            wheelRadiusPublisher.set(0);
        }


        double xVelocity = xVelocitySup.getAsDouble();
        double yVelocity = yVelocitySup.getAsDouble();
        double angularVelocity = angularVelocitySup.getAsDouble();
        double autoHeadingAngle = autoHeadingAngleSup.getAsDouble();

        boolean isAutoHeading = isAutoAngleSup.getAsBoolean();
        boolean isFieldRelative = isFieldRelativeSup.getAsBoolean();
        boolean isOpenLoop = isOpenLoopSup.getAsBoolean();
        boolean isLockIn = isLockInSup.getAsBoolean();
        boolean isZeroOdometry = isZeroOdometrySup.getAsBoolean() || zeroOdometry.get(false);

        if(isZeroOdometry) {
            driveTrain.resetOdometry();
            request.withHeading(driveTrain.getPose().getRotation().getRadians());
            zeroOdometry.set(false);
        }

        // handle smoothing and deadbanding
        Translation2d rawLinearVelocity = new Translation2d(xVelocity, yVelocity);
        Translation2d linearVelocity = smoothAndDeadband(rawLinearVelocity).times(Constants.Swerve.teleopLinearMultiplier);
        angularVelocity = smoothAndDeadband(angularVelocity) * Constants.Swerve.teleopAngularMultiplier;

        rawLinearVelocity = rawLinearVelocity.times(Constants.Swerve.teleopLinearMultiplier); //apples to apples comparison for logging

        // log data
        SwerveTelemetry.updateSwerveCommand(
            linearVelocity.getX(), 
            linearVelocity.getY(), 
            rawLinearVelocity.getX(),
            rawLinearVelocity.getY(),
            angularVelocity, 
            autoHeadingAngle, 
            isAutoHeading, 
            isFieldRelative, 
            isOpenLoop, 
            isLockIn, 
            isZeroOdometry
        );



        //handle lock in
        if (isLockIn) {
            //only lock in if we're stopped
            if (linearVelocity.equals(new Translation2d(0,0)) && angularVelocity == 0) {
                //use CTRE lock in, then return so it's not overridden by the normal request :)
                driveTrain.drive(lockInRequest.withDriveRequestType(isOpenLoop ? DriveRequestType.OpenLoopVoltage : DriveRequestType.Velocity));
                return;
            }
        }

       request.withVelocityX(linearVelocity.getX())
            .withVelocityY(linearVelocity.getY())
            .withRotationalRate(angularVelocity)
            .withIsOpenLoop(isOpenLoop)
            .withIsFieldCentric(isFieldRelative)
            .withHoldHeading(true) //always hold our current heading.
            .withPositionCorrectionIterations(4);

        //update the robot's target heading if we're using auto heading
        if(isAutoHeading) {
            request.withHeading(Rotation2d.fromDegrees(autoHeadingAngle).getRadians());
        }

        //disable risky features if we're in fallback mode
        if(fallback != FallbackMode.Normal){
            request.withPositionCorrectionIterations(0); //position correction depends on odometry
        }
        if(fallback == FallbackMode.PigeonFailure){
            //these depend on the robot's angle
            request.withIsFieldCentric(false);
            request.withHoldHeading(false);
            request.withSoftHoldHeading(false);
        }

        //yay peaccyrequest will handle the rest :D
        driveTrain.drive(request);
    }

    private Translation2d smoothAndDeadband (Translation2d linearVelocity) {
        //handle deadband and reset the rate limiter if we're in the deadband
        double rawLinearSpeed = handleDeadbandFixSlope(teleopLinearSpeedDeadband,0.1,linearVelocity.getNorm(), linearDeadbandDebouncer);
        if(Math.abs(rawLinearSpeed) < teleopLinearSpeedDeadband) linearSpeedLimiter.reset(0);
        rawLinearSpeed = JoystickCurves.curve(teleopLinearSpeedCurve, teleopLinearSpeedCurveSensitivity, rawLinearSpeed);

        boolean useLowBetteryLimiter = RobotController.getBatteryVoltage() < 10.5;

        double lowBatSpeed = lowBatteryLinearSpeedLimiter.calculate(rawLinearSpeed);
        //limit the linear acceleration
        double linearSpeed = linearSpeedLimiter.calculate(rawLinearSpeed);
        if(useLowBetteryLimiter) linearSpeed = lowBatSpeed;


        //limit the change in direction
        double rawLinearAngle = linearVelocity.getAngle().getRadians();
        double linearAngle = linearAngleLimiter.calculate(rawLinearAngle);

        // override the smoothing of the direction if it lags too far behind the raw value
        // (mainly after stopping and changing direction)
        if (Math.abs(linearAngle - rawLinearAngle) > Math.PI/4) {
            linearAngleLimiter.reset(rawLinearAngle);
            linearAngle = rawLinearAngle;
        }

        return new Translation2d(linearSpeed, new Rotation2d(linearAngle));
    }

    private double smoothAndDeadband (double angularVelocity) {
        //apply deadband to angular velocity
        angularVelocity = handleDeadbandFixSlope(teleopAngularVelocityDeadband, 0.05, angularVelocity, angularDeadbandDebouncer);

        angularVelocity = JoystickCurves.curve(teleopAngularVelocityCurve, teleopAngularVelocityCurveSensitivity, angularVelocity);

        //limit the angular acceleration
        angularVelocity = angularVelocityLimiter.calculate(angularVelocity);

        return angularVelocity;
    }

    /**
     * This function handles deadband by increases the slope of the output after
     * the deadband, so the output is 0 at the end of the deadband but still 100% for 100% input. 
     * This preserves fine control while still eliminating unnecessary current draw and preventing joystick drift.
     * @param deadband the deadband to use
     * @param value the value to apply the deadband to
     * @return the value with the deadband applied (like magic)
     */
    private double handleDeadbandFixSlope (double modband, double deadband, double value, Debouncer debounce) {
        if (debounce.calculate(Math.abs(value) < deadband)) return 0;
        var mod = (value - (deadband * Math.signum(value)))/(1 - deadband);
        return Math.abs(mod) > deadband ? mod : 0;
    }

    public void fallback(){
        if(fallback == FallbackMode.Normal) fallback = FallbackMode.OdmetryFailure;
        if(fallback == FallbackMode.OdmetryFailure) fallback = FallbackMode.PigeonFailure;
    }

    public void resetFallback(){
        fallback = FallbackMode.Normal;
    }
    
    public enum FallbackMode{
        Normal,
        OdmetryFailure,
        PigeonFailure
    }
}
