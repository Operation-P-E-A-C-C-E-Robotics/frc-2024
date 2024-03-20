package frc.lib.swerve;

import java.util.LinkedList;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.SteerRequestType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.motion.Trajectory;
import frc.lib.telemetry.SwerveTelemetry;
import frc.lib.util.AllianceFlipUtil;
import frc.lib.util.Util;
import frc.robot.subsystems.Swerve;

/**
 * The most epic swerve request ever. Does all the things. Made by the one and only Peaccy.
 * Please help, I keep waking up with crappy code under my behind so I put it here.
 * I don't know what I'm doing, it just falls out of my anus. It seems to work though. Occasionally
 * I'll get a good one, but I don't know how to make it happen. I just keep putting coffee in my mouth and it keeps coming out.
 * Anyway. This is a swerve request that does all the things. It's got a heading controller, a position correction controller,
 * and a bunch of other stuff. It's pretty cool. I think. I don't know. I'm just a monkey with a keyboard.
 * 
 * UPDATE: it was a good one. :D
 * 
 * here's a poem about actually thinking you wrote good code (and probably being delusional):
 * Yeah bro my code legit so works,
 * It always works, it never quirks,
 * I'm sure it's fine,
 * cause I tested it a million goddamn times
 * So I can believe my code legit so works.
 * Please don't tell me it doesn't work.
 * I legit am getting pissed off with computer daily
 * These poems are gettings worse and worse
 * I'm sure it's fine,
 * I'm sure it's fine,
 * I'm sure it's fine,
 * I'm sure it's fine,
 * I'm sure it's fine,
 * I'm sure it's fine,
 * I'm sure it's fine,
 * I'm sure it's fine,
 * I'm sure it's fine,
 * I hate programming.
 * -Peaccy
 */
public class PeaccyRequest implements SwerveRequest {

    /* INPUT PARAMETERS */
    public double VelocityX = 0;
    public double VelocityY = 0;
    public double RotationalRate = 0;
    public double Heading = 0; //will hold both the target heading when explicitly set, and the current heading when holding the heading
    public double RotationalDeadband = 0;
    public double LockHeadingVelocity = 0;

    public boolean HoldHeading = false; //keeps the robot facing Heading unless RotationalRate is over the RotationalDeadband
    public boolean LockHeading = false; //HoldHeading but way faster and more aggressive
    public boolean SoftHoldHeading = false; //scales HoldHeading by an allowed total drive current draw to prevent tread wear and brownouts
    public boolean IsOpenLoop = false; //if true, the robot will not use the drive velocity controller
    public boolean IsFieldCentric = false; //if false, position correction will not be applied

    //integrates the requested velocities on top of the past robot pose to figure out where we actually wanted the robot to be,
    //and modify the requested velocities to compensate for the error
    public double PositionCorrectionIterations = 0; //how many past inputs to integrate
    public double PositionCorrectionWeight = 1;

    /* HEADING CONTROLLER */
    private Trajectory headingTrajectory = new Trajectory(new TrapezoidProfile.State(0, 0)); //make it smooth
    private SimpleMotorFeedforward headingFeedforward = new SimpleMotorFeedforward(0, 0, 0); //make it good
    private double holdHeadingkP = 0; //make it work
    private double lockHeadingkP = 0; //make it work faster

    //heading trajectory constraints
    private double holdHeadingVelocity = 0;
    private double holdHeadingAcceleration = 0;
    private double lockHeadingVelocity = 0;
    private double lockHeadingAcceleration = 0;

    private Timer holdHeadingTrajectoryTimer = new Timer();
    private final Timer robotMovingTimer = new Timer();
    private final Timer robotNotMovingTimer = new Timer();

    //soft heading current limit parameters
    private DoubleSupplier totalDriveCurrent;
    private double totalDriveCurrentLimit;

    
    /* POSITION CORRECTION PAST TRANSLATIONS */
    private LinkedList<Translation2d> positionCorrectionRealPositions = new LinkedList<>();
    private LinkedList<Translation2d> positionCorrectionRequestedVelocities = new LinkedList<>();

    private Supplier<ChassisSpeeds> getChassisSpeeds;


    private final double CURRENT_LIMIT_THRESHOLD = 0.01; //percent of the current limit to start throttling at.
    private final SlewRateLimiter currentLimitSmoother = new SlewRateLimiter(10); //limit the amps per second for the current to change, to help find equilibrium.
    private double maxLinearVelocity;


    /**
     * The most epic swerve request ever. Does all the things.
     * Made by the one and only Peaccy.
     * @param maxAngularVelocity the maximum angular velocity to use when turning to the set trajectory
     * @param maxAngularAcceleration the maximum angular acceleration to use when turning to the set trajectory
     * @param maxLinearVelocity the maximum linear velocity (for scaling between open loop and closed loop)
     * @param holdHeadingkP the proportional gain to use when turning to the set trajectory
     * @param holdHeadingkV the velocity feedforward to use when turning to the set trajectory
     * @param holdHeadingkA the acceleration feedforward to use when turning to the set trajectory
     * @param chassisSpeedsSupplier a supplier for the current chassis speeds (I literally dont remember what this is for)
     * @param totalDriveCurrentSupplier a supplier for the sum of current draw of all the drive motors (used for soft heading current limiting)
     * @param softHeadingCurrentLimit the maximum current draw allowed for the heading correction in soft heading mode
     */
    public PeaccyRequest(double holdHeadingVelocity, 
                    double holdHeadingAcceleration, 
                    double lockHeadingVelocity,
                    double lockHeadingAcceleration,
                    double maxLinearVelocity,
                    double holdHeadingkP, 
                    double holdHeadingkV, 
                    double holdHeadingkA, 
                    double lockHeadingkP,
                    Supplier<ChassisSpeeds> chassisSpeedsSupplier,
                    DoubleSupplier totalDriveCurrentSupplier,
                    double softHeadingCurrentLimit) {
        this.holdHeadingVelocity = holdHeadingVelocity;
        this.holdHeadingAcceleration = holdHeadingAcceleration;
        this.lockHeadingVelocity = lockHeadingVelocity;
        this.lockHeadingAcceleration = lockHeadingAcceleration;

        headingFeedforward = new SimpleMotorFeedforward(0, holdHeadingkV, holdHeadingkA);
        this.holdHeadingkP = holdHeadingkP;
        this.lockHeadingkP = lockHeadingkP;

        this.maxLinearVelocity = maxLinearVelocity;
        
        this.getChassisSpeeds = chassisSpeedsSupplier;
        this.totalDriveCurrent = totalDriveCurrentSupplier;
        this.totalDriveCurrentLimit = softHeadingCurrentLimit;
        robotMovingTimer.start();
        robotNotMovingTimer.start();

        System.out.println("PeaccyRequest Initialized");
    }

    @Override
    public StatusCode apply(SwerveControlRequestParameters parameters, SwerveModule... modulesToApply) {
        Translation2d toApplyTranslation = new Translation2d(VelocityX, VelocityY);
        double toApplyRotation = RotationalRate;

        if(IsOpenLoop) {
            toApplyTranslation = toApplyTranslation.times(12/maxLinearVelocity);
        }

        //position correction only works for field centric :|
        if(IsFieldCentric) toApplyTranslation = applyPositionCorrection(toApplyTranslation, parameters.currentPose, parameters.updatePeriod);

        if(toApplyTranslation.getNorm() < 0.1){
            robotMovingTimer.reset();
            robotMovingTimer.start();
        } else {
            robotNotMovingTimer.reset();
            robotNotMovingTimer.start();
        }

        //we only do auto heading if there is no manually requested rotational rate
        if(Math.abs(toApplyRotation) <= RotationalDeadband) {
            toApplyRotation = 0;
            if ((HoldHeading || SoftHoldHeading)) {
                toApplyRotation = applyAutoHeading(parameters);
            } 
        } else {
            //Update the set heading to the current heading. This means that when there is no rotational rate requested,
            //the robot will hold its current heading if HoldHeading or SoftHoldHeading is true,
            //unless Heading is explicitly set to something else.
            Heading = Swerve.getInstance().getPose().getRotation().getRadians();
            if(AllianceFlipUtil.shouldFlip()) Heading += Math.PI;
        }

        SmartDashboard.putNumber("Requested X Velocity", toApplyTranslation.getX());
        SmartDashboard.putNumber("Requested Y Velocity", toApplyTranslation.getY());
        

        //very standard ChassisSpeeds blah blah blah.
        ChassisSpeeds speeds = ChassisSpeeds.discretize(
            IsFieldCentric ? ChassisSpeeds.fromFieldRelativeSpeeds(
                toApplyTranslation.getX(), 
                toApplyTranslation.getY(), 
                toApplyRotation,
                parameters.currentPose.getRotation()
            ) : new ChassisSpeeds(
                toApplyTranslation.getX(), 
                toApplyTranslation.getY(), 
                toApplyRotation
            ),
            parameters.updatePeriod
        );

        //wowie make it go.
        var states = parameters.kinematics.toSwerveModuleStates(speeds, new Translation2d());

        SwerveTelemetry.updateRequestedState(states);

        for (int i = 0; i < modulesToApply.length; ++i) {
            modulesToApply[i].apply(states[i], IsOpenLoop ? DriveRequestType.OpenLoopVoltage : DriveRequestType.Velocity, SteerRequestType.MotionMagic); //TODO change to motion magic expo
        }

        return StatusCode.OK;
    }

    /**
     * Set the x velocity. duh.
     * @param velocityX the x velocity. duh.
     * @return this (so you can chain em nicely :D)
     */
    public PeaccyRequest withVelocityX(double velocityX) {
        this.VelocityX = velocityX;
        return this;
    }

    /**
     * Set the y velocity. duh.
     * @param velocityY the y velocity. duh.
     * @return this (so you can chain em nicely :D)
     */
    public PeaccyRequest withVelocityY(double velocityY) {
        this.VelocityY = velocityY;
        return this;
    }

    /**
     * Set the rotational rate. duh.
     * Also, this will override any set heading if it's over the rotational deadband.
     * @param rotationalRate the rotational rate. duh.
     * @return this (so you can chain em nicely :D)
     */
    public PeaccyRequest withRotationalRate(double rotationalRate) {
        this.RotationalRate = rotationalRate;
        return this;
    }

    /**
     * Set the robot's target heading when holding the heading.
     * Won't do nothin of anything if HoldHeading and SoftHoldHeading are false.
     * @param heading the target heading
     * @return this (so you can chain em nicely :D)
     */
    public PeaccyRequest withHeading(double heading) {
        this.Heading = heading;
        return this;
    }

    /**
     * Set the deadband for rotation. THIS IS ACTUALLY IMPORTANT HERE!!
     * Because heading correction is only applied if your requested rotational rate is under the rotational deadband,
     * @param rotationalDeadband the rotational deadband
     * @return this (so you can chain em nicely :D)
     */
    public PeaccyRequest withRotationalDeadband(double rotationalDeadband) {
        this.RotationalDeadband = rotationalDeadband;
        return this;
    }

    /**
     * Set whether or not to hold the heading.
     * @param holdHeading whether or not to hold the heading
     * @return this (so you can chain em nicely :D)
     */
    public PeaccyRequest withHoldHeading(boolean holdHeading) {
        this.HoldHeading = holdHeading;
        return this;
    }

    /**
     * Set whether or not to lock the heading.
     * @param lockHeading whether or not to lock the heading
     * @return this (so you can chain em nicely :D)
     */
    public PeaccyRequest withLockHeading(boolean lockHeading) {
        this.LockHeading = lockHeading;
        return this;
    }

    /**
     * just an extra velocity feedforward for lockheading, to use for aiming
     * @param lockHeadingVelocity velocity target for feedforward
     * @return this (so you can chain em nicely :I)
     */
    public PeaccyRequest withLockHeadingVelocity(double lockHeadingVelocity) {
        this.LockHeadingVelocity = lockHeadingVelocity;
        return this;
    }

    /**
     * Set whether or not to scale the heading correction by the allowed total drive current.
     * Also, this will enable heading correction even if HoldHeading is false,
     * but it overrides HoldHeading if both are true.
     * @param softHoldHeading whether or not to scale the heading correction by the allowed total drive current
     * @return this (so you can chain em nicely :D)
     */
    public PeaccyRequest withSoftHoldHeading(boolean softHoldHeading) {
        this.SoftHoldHeading = softHoldHeading;
        return this;
    }

    /**
     * Set whether or not to disable the drive velocity controllers.
     * No promises on how good positional correction will be in open loop.
     * @param isOpenLoop whether or not to disable the drive velocity controllers
     * @return this (so you can chain em nicely :D)
     */
    public PeaccyRequest withIsOpenLoop(boolean isOpenLoop) {
        this.IsOpenLoop = isOpenLoop;
        return this;
    }

    /**
     * Set whether or not to use field centric mode.
     * Position correction only works in field centric mode.
     * Why would you not use field centric mode?
     * Oh yea, auto :(
     * @param isFieldCentric whether or not to use field centric mode
     * @return this (so you can chain em nicely :D)
     */
    public PeaccyRequest withIsFieldCentric(boolean isFieldCentric) {
        this.IsFieldCentric = isFieldCentric;
        return this;
    }

    /**
     * If you're wondering what the heck position correction is, I've got no clue.
     * It just seemed like a good idea at the time. It just looks at the past robot poses and
     * inputs, tries to figure out where we actually wanted the robot to be, and then
     * modifies the requested velocities to compensate for the error.
     * @param positionCorrectionIterations how many past robot poses are considered in position correction
     * @return this (so you can chain em nicely :D)
     */
    public PeaccyRequest withPositionCorrectionIterations(double positionCorrectionIterations) {
        this.PositionCorrectionIterations = positionCorrectionIterations;
        return this;
    }

    /**
     * If position correction ends up sucking, you can just turn it down.
     * @param positionCorrectionWeight multiple of the position correction to apply to the requested velocities. BETWEEN 0 AND 1 PLEASE.
     * @return this (so you can chain em nicely :D)
     */
    public PeaccyRequest withPositionCorrectionWeight(double positionCorrectionWeight) {
        this.PositionCorrectionWeight = Util.limit(positionCorrectionWeight,0,1);
        return this;
    }
    

    double prevHeading = 0;
    /**
     * Get the rotation rate to apply to the robot to go to the target heading
     */
    private double applyAutoHeading(SwerveControlRequestParameters parameters) {
        var currentHeading = Swerve.getInstance().getPose().getRotation().getRadians();
        if(AllianceFlipUtil.shouldFlip()) currentHeading += Math.PI;

        //make sure our odometry heading is within +/- 180 degrees of the target heading to prevent it from wrapping LIKE CTRE DOES >:(
        while (Math.abs(currentHeading - Heading) > Math.PI) {
            if (currentHeading > Heading) {
                currentHeading -= 2 * Math.PI;
            } else {
                currentHeading += 2 * Math.PI;
            }
        }

        //regenerate the trajectory if the target heading has changed
        if(Heading != headingTrajectory.getTarget().position || Math.abs(currentHeading - prevHeading) > (Math.PI/4)) {
            headingTrajectory = Trajectory.trapezoidTrajectory(
                new State(currentHeading, 0), 
                new State(Heading, 0), 
                LockHeading ? lockHeadingVelocity : holdHeadingVelocity,
                LockHeading ? lockHeadingAcceleration : holdHeadingAcceleration 
            );
            
            holdHeadingTrajectoryTimer.reset();
            holdHeadingTrajectoryTimer.start();
        }

        prevHeading = currentHeading;

        
        //calculate the correction
        var target = headingTrajectory.calculate(holdHeadingTrajectoryTimer.get() + (parameters.updatePeriod*1.5));
        if (LockHeading) {
            target.position = Heading;
            target.velocity = LockHeadingVelocity;
        }
        var kP = LockHeading ? lockHeadingkP : holdHeadingkP;
        var error = target.position - currentHeading;

        var acceleration = (target.velocity - getChassisSpeeds.get().omegaRadiansPerSecond);

        if(robotMovingTimer.get() < 0.3 && Math.abs(error) < 0.01 && !LockHeading){
            return 0;
        }

        var feedforward = headingFeedforward.calculate(target.velocity, acceleration);
        var pGain = error * kP * parameters.updatePeriod;
        if(SoftHoldHeading) {
            var currentDraw = currentLimitSmoother.calculate(totalDriveCurrent.getAsDouble());
            pGain = pGain * compress(currentDraw, totalDriveCurrentLimit, CURRENT_LIMIT_THRESHOLD);
        }
        var delta = pGain + feedforward;

        SwerveTelemetry.updateAutoHeading(
            Heading,
            error,
            pGain,
            feedforward,
            target.velocity,
            acceleration,
            target.position,
            SoftHoldHeading
        );

        return delta;
    }
    
    /**
     * Integrate the requested velocities to get the requested position, and compare it to the actual position to get the error.
     * @param requestedTranslation the requested velocity
     * @param currentPose the current pose of the robot
     * @param updatePeriod the time between the last update and this one
     * @return the more gooder velocity
     */
    private Translation2d applyPositionCorrection(Translation2d requestedTranslation, Pose2d currentPose, double updatePeriod){
        if(PositionCorrectionIterations == 0) return requestedTranslation;

        positionCorrectionRealPositions.add(currentPose.getTranslation());
        positionCorrectionRequestedVelocities.add(requestedTranslation);

        if (positionCorrectionRealPositions.size() > PositionCorrectionIterations) {
            positionCorrectionRealPositions.removeFirst();
        }

        if (positionCorrectionRequestedVelocities.size() > PositionCorrectionIterations) {
            positionCorrectionRequestedVelocities.removeFirst();
        }

        if(robotMovingTimer.get() < 0.05) return requestedTranslation;

        //integrate the requested velocities to get the requested position
        Translation2d requestedPositionDelta = new Translation2d(0, 0);
        for (Translation2d requestedVelocity : positionCorrectionRequestedVelocities) {
            requestedPositionDelta = requestedPositionDelta.plus(requestedVelocity.times(updatePeriod));
        }

        Translation2d realPositionDelta = currentPose.getTranslation().minus(positionCorrectionRealPositions.getFirst());
        Translation2d positionError = requestedPositionDelta.minus(realPositionDelta);

        SwerveTelemetry.updatePositionCorrection(
            requestedPositionDelta,
            realPositionDelta
        );

        Translation2d newRequestedVelocity = requestedTranslation.plus(positionError.times(PositionCorrectionWeight));
        return newRequestedVelocity;
    }

    /**
     * similar to an audio compressor. does nothing until the threshold, then scales the value linearly to the limit.
     * This function just returns the scalar for the value, so you can multiply it by the value to compress it.
     * This lets you use it to "sidechain" the compressor to another value, like *ahem* using the drive current to compress 
     * the heading correction.
     * @param value the value to compress
     * @param limit the value where the output should go to 0
     * @param threshold the threshold to start compressing at (as a percentage of the limit)
     * @return the multiplier for compression
     */
    private static double compress(double value, double limit, double threshold) {
        var l = (1-threshold)*limit;
        var linear = (limit/l)-((1/l)*value);
        return Util.limit(linear, 0, 1);
    }
}
