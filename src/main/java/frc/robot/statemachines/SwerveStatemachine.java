package frc.robot.statemachines;

import static frc.robot.Constants.Swerve.aimTolerance;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.state.StateMachine;
import frc.lib.swerve.PeaccyRequest;
import frc.lib.telemetry.SwerveTelemetry;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.planners.AimPlanner;
import frc.robot.subsystems.Swerve;

/**
 * Adopted from Command Based architecture so it's not the neatest :|
 */
public class SwerveStatemachine extends StateMachine<SwerveStatemachine.SwerveState> {
    /* Initialize the suppliers to default values */
    private DoubleSupplier xVelocitySup = OI.Swerve.translation,
                           yVelocitySup = OI.Swerve.strafe, 
                           angularVelocitySup = OI.Swerve.rotation, 
                           autoHeadingAngleSup = OI.Swerve.heading;
    private BooleanSupplier isZeroOdometrySup = OI.Swerve.isZeroOdometry,
                            useHeadingSup = OI.Swerve.useHeading;

    private Swerve driveTrain;

    /* "Swerve Requests" are what the drivetrain subsystem accepts. They figure out how to orient and drive the wheels. */
    private final PeaccyRequest request; //custom fancy request than handles everything
    private final SwerveRequest.SwerveDriveBrake lockInRequest = new SwerveRequest.SwerveDriveBrake().withDriveRequestType(DriveRequestType.Velocity); //for X-locking the wheels

    /* Acceleration limiters for a consistent feel and to reduce power draw. */
    private final SlewRateLimiter linearSpeedLimiter = new SlewRateLimiter(Constants.Swerve.teleopLinearSpeedLimit); //limit the change in speed
    private final SlewRateLimiter agressiveLimiter = new SlewRateLimiter(Constants.Swerve.teleopLowBatteryLinearSpeedLimit); //more aggresive to prevent brownouts
    private final SlewRateLimiter linearAngleLimiter = new SlewRateLimiter(Constants.Swerve.teleopLinearAngleLimit); //limit the change in direction
    private final SlewRateLimiter angularVelocityLimiter = new SlewRateLimiter(Constants.Swerve.teleopAngularRateLimit); //limit the change in angular velocity

    private SwerveState state = SwerveState.OPEN_LOOP_TELEOP;

    /* the path we're following if the state has a path in it */
    private Command pathCommand = null;
    private Command driveToNoteCommand = null;
    private boolean pathInitialized = false;
    private boolean pathFinished = false;
    private Timer pathTimer = new Timer();

    private final AimPlanner aimPlanner;

    private final LinearFilter noteTrackingFliter = LinearFilter.movingAverage(4);

    /**
     * PeaccyDrive is a swerve drive command designed to handle all the different
     * modes of driving that we want to use.
     *
     * It has advanced input smoothing and deadbanding,
     * field centric and robot centric modes,
     * and auto angle (automatic heading adjustment) modes.
     * @param driveTrain the swerve subsystem
     */
    public SwerveStatemachine(Swerve driveTrain, AimPlanner aimPlanner) {
        this.driveTrain = driveTrain;
        this.aimPlanner = aimPlanner;

        request  = new PeaccyRequest(
            Constants.Swerve.autoHeadingMaxVelocity, 
            Constants.Swerve.autoHeadingMaxAcceleration,
            Constants.Swerve.lockHeadingMaxVelocity,
            Constants.Swerve.lockHeadingMaxAcceleration,
            Constants.Swerve.teleopLinearMultiplier,
            Constants.Swerve.autoHeadingKP, 
            Constants.Swerve.autoHeadingKV, 
            Constants.Swerve.autoHeadingKA, 
            Constants.Swerve.lockHeadingKP,
            driveTrain::getChassisSpeeds, 
            driveTrain::getTotalDriveCurrent, 
            Constants.Swerve.softHeadingCurrentLimit
        ).withRotationalDeadband(Constants.Swerve.teleopAngularVelocityDeadband)
        .withSoftHoldHeading(Constants.Swerve.useSoftHoldHeading)
        .withPositionCorrectionIterations(Constants.Swerve.teleopPositionCorrectionIters);

        System.out.println("PeacyDrive initialized");
    }


    @Override
    public void requestState(SwerveState state){
        if(state == this.state) return;

        if(state == SwerveState.LOCK_IN){
            if(Math.abs(xVelocitySup.getAsDouble()) > Constants.Swerve.teleopLinearSpeedDeadband || 
                Math.abs(yVelocitySup.getAsDouble()) > Constants.Swerve.teleopLinearSpeedDeadband || 
                Math.abs(angularVelocitySup.getAsDouble()) > Constants.Swerve.teleopAngularVelocityDeadband){
                return;
            }
        }
            
        //handle path following
        //handle command end function
        if(this.state == SwerveState.FOLLOW_PATH && state != SwerveState.FOLLOW_PATH && pathCommand != null){
            pathCommand.end(!pathFinished);
            pathTimer.stop();
        }
        if(state == SwerveState.FOLLOW_PATH && this.state != SwerveState.FOLLOW_PATH){
            pathInitialized = false;
            pathFinished = false;
            pathTimer.reset();
        }

        //handle drive to note
        //handle command end function
        if(this.state == SwerveState.DRIVE_TO_NOTE && state != SwerveState.DRIVE_TO_NOTE && driveToNoteCommand != null){
            driveToNoteCommand.end(!pathFinished);
            driveToNoteCommand = null;
            pathTimer.stop();
        }
        if(state == SwerveState.FOLLOW_PATH && this.state != SwerveState.FOLLOW_PATH){
            pathInitialized = false;
            pathFinished = false;
            driveToNoteCommand = null;
            pathTimer.reset();
        }
        this.state = state;
    }

    public void setPathCommand(Command command){
        if(command == pathCommand) return;
        pathCommand = command;
        pathInitialized = false;
        pathFinished = false;
    }

    public void resetPathTimer() {
        pathTimer.reset();
    }

    public double getPathTime() {
        return pathTimer.get();
    }

    private void updateState() {
        //unlock the wheels if we wanna move
        if(state == SwerveState.LOCK_IN){
            if(Math.abs(xVelocitySup.getAsDouble()) > Constants.Swerve.teleopLinearSpeedDeadband || 
                Math.abs(yVelocitySup.getAsDouble()) > Constants.Swerve.teleopLinearSpeedDeadband || 
                Math.abs(angularVelocitySup.getAsDouble()) > Constants.Swerve.teleopAngularVelocityDeadband){
                requestState(SwerveState.OPEN_LOOP_TELEOP);
            }
        }
    }
    @Override
    public SwerveState getState(){
        return state;
    }

    @Override
    public void update() {
        driveTrain.periodic();
        updateState();

        SmartDashboard.putString("Swerve State", state.name());

        if(Robot.isSimulation()) driveTrain.simulationPeriodic();
        /* PATH FOLLOWING */
        if(state == SwerveState.FOLLOW_PATH && pathCommand != null && RobotState.isEnabled()){
            if(!pathInitialized){
                pathCommand.initialize();
                pathTimer.reset();
                pathTimer.start();
                pathInitialized = true;
            }
            pathCommand.execute();
            pathFinished = pathCommand.isFinished();
            return;
        }

        /* GO TO NOTEZ */
        if(state == SwerveState.DRIVE_TO_NOTE && driveToNoteCommand != null && RobotState.isEnabled()){
            if(!pathInitialized){
                driveToNoteCommand.initialize();
                pathTimer.reset();
                pathTimer.start();
                pathInitialized = true;
            }
            driveToNoteCommand.execute();
            pathFinished = driveToNoteCommand.isFinished();
            return;
        }

        /* TELEOP */
        //get the values from the suppliers
        double xVelocity = xVelocitySup.getAsDouble();
        double yVelocity = yVelocitySup.getAsDouble();
        double angularVelocity = angularVelocitySup.getAsDouble();
        double autoHeadingAngle = autoHeadingAngleSup.getAsDouble();

        boolean useHeading = useHeadingSup.getAsBoolean();
        boolean holdHeading = state.isHoldHeading();
        boolean isFieldRelative = !state.isRobotCentric();
        boolean isOpenLoop = state.isOpenLoop();
        boolean isLockIn = state.isLockIn();
        boolean isZeroOdometry = isZeroOdometrySup.getAsBoolean();

        if(isZeroOdometry) {
            driveTrain.resetOdometry();
            request.withHeading(driveTrain.getPose().getRotation().getRadians());
        }

        if(OI.Swerve.isAttemptProperZero.getAsBoolean()) driveTrain.attemptProperFieldCentricZeroing();

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
            useHeading, 
            isFieldRelative, 
            isOpenLoop, 
            isLockIn, 
            isZeroOdometry
        );



        //X-lock the wheels if we're stopped and the driver wants to
        if (isLockIn) {
            driveTrain.drive(lockInRequest.withDriveRequestType(isOpenLoop ? DriveRequestType.OpenLoopVoltage : DriveRequestType.Velocity));
            return;
        }

        //update my very nice swerve request with the values we've calculated
        request.withVelocityX(linearVelocity.getX())
               .withVelocityY(linearVelocity.getY())
               .withRotationalRate(angularVelocity)
               .withIsOpenLoop(isOpenLoop)
               .withIsFieldCentric(isFieldRelative)
               //true will use pid control to maintain heading, or set to "isAutoHeading" if you want to only turn to specified headings 
               //rather than holding whatever direction you're facing :)
               .withHoldHeading(holdHeading)
               .withLockHeading(false)
               .withPositionCorrectionIterations(Constants.Swerve.teleopPositionCorrectionIters);

        //update the robot's target heading if we're using auto heading
        if(useHeading) {
            request.withHeading(Rotation2d.fromDegrees(autoHeadingAngle).getRadians());
        }


        if(state == SwerveState.ROBOT_CENTRIC){
            //these depend on the robot's angle
            request.withIsFieldCentric(false);
            request.withHoldHeading(false);
            request.withSoftHoldHeading(false);
        }

        if(state == SwerveState.AIM){
            //set the target to heading to the heading from the aim planner
            //use smooth auto heading for the first part of the motion before
            //locking on aggressively, to avoid excessive current draw
            var wantedAngle = aimPlanner.getTargetDrivetrainAngle().getRadians();

            request.withHeading(wantedAngle);
            request.withLockHeading(true);
            request.withLockHeadingVelocity(Units.degreesToRadians(aimPlanner.getDrivetrainAngularVelocity()));
        }

        if(state == SwerveState.ALIGN_INTAKING){
            //align with a note automatically for intaking, using the limelight
            // var results = LimelightHelpers.getLatestResults(Constants.Cameras.rearLimelight).targetingResults.targets_Detector;
            // if(results.length > 0){
            //     var heading = Swerve.getInstance().getPose().getRotation().getRadians() - Units.degreesToRadians(results[0].tx * 0.5);
            //     heading = noteTrackingFliter.calculate(AllianceFlipUtil.shouldFlip() ? heading - Math.PI - 0.06 : heading - 0.06);
            //     request.withHeading(heading);

            //     request.withLockHeading(true);
            //     request.withLockHeadingVelocity(0);
            // }
        } else {
            noteTrackingFliter.reset();
        }

        if(OI.Overrides.disableAutoHeading.getAsBoolean()) {
            request.withHoldHeading(false)
                    .withLockHeading(false)
                    .withSoftHoldHeading(false);
        }

        //yay peaccyrequest,
        //(my beautiful swerve request),
        //will handle the rest :D,
        //and make the robot go vroom vroom,
        //and do the thing,
        //and be very nice,
        //and make me happy,
        //for the rest of my life.
        // -peaccy
        driveTrain.drive(request);

        //create drive to note command:
        //if we're in the right state and the note is detected, create a command to drive to the note
        if(state == SwerveState.DRIVE_TO_NOTE && driveToNoteCommand == null){
            var noteTranslation = driveTrain.getNoteFromField();
            if(noteTranslation.isEmpty()) return;
            driveToNoteCommand = AutoBuilder.pathfindToPose(
                new Pose2d(noteTranslation.get(), driveTrain.getPose().getRotation()), 
                Constants.Swerve.autoMaxSpeed
            );
        }
    }

    public void zeroAutoHeading() {
        request.withHeading(driveTrain.getPose().getRotation().getRadians());
    }

    @Override
    public boolean transitioning(){
        if(state == SwerveState.FOLLOW_PATH){
            return !pathFinished;
        }
        if(state == SwerveState.AIM) {
            return Math.abs(driveTrain.getPose().getRotation().getDegrees() - aimPlanner.getTargetDrivetrainAngle().getDegrees()) > aimTolerance;
        }
        return false;
    }

    /**
     * Handle deadbanding and smoothing of the linear velocity.
     * Does a nice deadband to get rid of joystick drift while minimizing loss of precise control.
     * Also does a nice rate limiter to make the robot feel nice to drive.
     * Rate limits are on the speed and angle of the velocity rather than X and Y components to prevent
     * the angle of the velocity feeling inconsistent, 
     * and help you (me) pretend to be a good driver who can make the robot go straight.
     * Oh yeah it also does a nice curve to make the robot feel nice to drive,
     * and hopefully stop me from driving thru any more large metal workbenches
     * @param linearVelocity the raw linear velocity as a Translation2d
     * @return the much more gooder linear velocity
     */
    private Translation2d smoothAndDeadband (Translation2d linearVelocity) {
        //handle deadband and reset the rate limiter if we're in the deadband
        double rawLinearSpeed = handleDeadbandFixSlope(0.001,0.1,linearVelocity.getNorm());
        if(Math.abs(rawLinearSpeed) < Constants.Swerve.teleopLinearSpeedDeadband) linearSpeedLimiter.reset(0);
        rawLinearSpeed = Constants.Swerve.teleopLinearSpeedCurve.apply(rawLinearSpeed);

        boolean useAggresiveLimiter = RobotController.getBatteryVoltage() < 10.5 
                                    || OI.Inputs.enableShootWhileMoving.getAsBoolean();

        double lowSpeed = agressiveLimiter.calculate(rawLinearSpeed);
        //limit the linear acceleration
        double linearSpeed = linearSpeedLimiter.calculate(rawLinearSpeed);
        if(useAggresiveLimiter) linearSpeed = lowSpeed;

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

    /**
     * super simple lil deadbander and smoother for the spinny part.
     * just a deadband, curve and rate limiter.
     * @param angularVelocity the raw angular velocity
     * @return the vastly superior angular velocity
     */
    private double smoothAndDeadband (double angularVelocity) {
        //apply deadband to angular velocity
        angularVelocity = handleDeadbandFixSlope(0.001, 0.1, angularVelocity);

        angularVelocity = Constants.Swerve.teleopAngularVelocityCurve.apply(angularVelocity);

        //limit the angular acceleration
        angularVelocity = angularVelocityLimiter.calculate(angularVelocity);

        return angularVelocity;
    }

    /**
     * This function handles deadband by increases the slope of the output after
     * the deadband, so the output is 0 at the end of the deadband but still 100% for 100% input. 
     * This preserves fine control while still eliminating unnecessary current draw and preventing joystick drift.
     * (all the benefits of a deadband lol)
     * @param modband I gave up on naming things. This is the deadband with a slope increase afterward
     * @param deadband normal deadband that is applied after the """"modband"""" [rolleyes emoji]. Use if you wanna overcome the static friction of the motors right after the deadband.
     * @param value the value to apply the deadband to
     * @return the value with the deadband applied (like magic)
     */
    private double handleDeadbandFixSlope (double modband, double deadband, double value) {
        if(Math.abs(value) < deadband) return 0;
        var mod = (value - (deadband * Math.signum(value)))/(1 - deadband);
        return Math.abs(mod) > deadband ? mod : 0;
    }

    @Override
    public boolean isDynamic() {
        return true;
    }
    
    public enum SwerveState{
        OPEN_LOOP_TELEOP,
        CLOSED_LOOP_TELEOP  (false, false, false, true),
        ROBOT_CENTRIC       (true, true, false, false),
        LOCK_IN             (true, false, true, false),
        AIM,
        ALIGN_INTAKING, //align to the note with vision
        FOLLOW_PATH,
        DRIVE_TO_NOTE; // follow the path thats been set with setPathCommand

        boolean openLoop = false;
        boolean robotCentric = false;
        boolean lockIn = false;
        boolean holdHeading;

        private SwerveState(boolean openLoop, boolean robotCentric, boolean lockIn, boolean holdHeading){
            this.openLoop = openLoop;
            this.robotCentric = robotCentric;
            this.lockIn = lockIn;
            this.holdHeading = holdHeading;
        }

        private SwerveState() {
            openLoop = true;
            robotCentric = false;
            lockIn = false;
            holdHeading = true;
        }

        public boolean isOpenLoop(){
            return openLoop;
        }

        public boolean isRobotCentric(){
            return robotCentric;
        }

        public boolean isLockIn(){
            return lockIn;
        }

        public boolean isHoldHeading(){
            return holdHeading;
        }
    }
}
