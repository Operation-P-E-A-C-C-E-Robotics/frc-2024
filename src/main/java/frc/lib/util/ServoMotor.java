package frc.lib.util;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.motion.Trajectory;

import java.util.ArrayList;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

public class ServoMotor extends SubsystemBase {
    //state space controller:
    private final LinearSystem<N2, N1, N2> plant;
    private final LinearSystemLoop<N2, N1, N2> loop;
    private final ArrayList<Feedforward> feedforwards = new ArrayList<>();

    //constants:
    private final SystemConstants constants;

    //variables to keep track of the trajectory:
    private final Timer profileTimer = new Timer();
    private boolean followingProfile = false, looping = false;

    //trajectory:
    private Trajectory trajectory;
    private State trajectoryStart = new State(0,0), trajectoryEnd = new State(0,0);
    private boolean recalculateTrajectory = false; //whether we need to recalculate the trajectory

    //setters and getters:
    private DoubleConsumer voltDriveFunction;
    private DoubleSupplier positionSupplier, velocitySupplier;

    private Runnable superPeriodic; // a function to run every periodic.

    /**
     * A messy ass helper class to run trajectories on a DC Motor state space controller.
     * #authentic2023experience
     * @param constants The constants for the system
     */
    public ServoMotor(SystemConstants constants, DoubleConsumer voltDriveFunction, DoubleSupplier positionSupplier, DoubleSupplier velocitySupplier) {
        this.constants = constants;
        this.voltDriveFunction = voltDriveFunction;
        this.positionSupplier = positionSupplier;
        this.velocitySupplier = velocitySupplier;

        //set up teh statey spacey contraollr
        plant = LinearSystemId.createDCMotorSystem( //create the mathy calculator thingy
                constants.motor,
                constants.inertia,
                constants.gearing
        );
        //this thingy figures out where the motor is.
        //it does this by using the plant to predict where the motor is, and then using the sensor to correct it.
        KalmanFilter<N2, N1, N2> observer = new KalmanFilter<>(
                Nat.N2(),
                Nat.N2(),
                plant,
                VecBuilder.fill(constants.kalmanModelAccuracyPosition, constants.kalmanModelAccuracyVelocity),
                VecBuilder.fill(constants.kalmanSensorAccuracyPosition, constants.kalmanSensorAccuracyVelocity),
                constants.dt
        );
        // this figures out how to make motor go spinny to get to where it needs to go
        // I haveno fukin clue how this works. sorry, no explanations here.
        LinearQuadraticRegulator<N2, N1, N2> linearQuadraticRegulator = new LinearQuadraticRegulator<>(
                plant,
                VecBuilder.fill(constants.lqrPositionTolerance, constants.lqrVelocityTolerance),
                VecBuilder.fill(constants.lqrControlEffortTolerance),
                constants.dt
        );

        //compensate for latency if needed. This can make things be weird though.
        if(constants.lqrSensorDelay > 0) linearQuadraticRegulator.latencyCompensate(plant, constants.dt, constants.lqrSensorDelay);
        //the loop knows how to make everything work together, because I have no idea how to do that.
        loop = new LinearSystemLoop<>(
                plant,
                linearQuadraticRegulator,
                observer,
                constants.maxVoltage,
                constants.dt
        );
        //set up a default trajectory (to avoid errors)
        trajectory = Trajectory.trapezoidTrajectory(trajectoryStart, trajectoryEnd, constants.maxVelocity, constants.maxAcceleration);
    }

    /**
     * add a feedforward to the controller
     * @param feedforward the feedforward to add
     */
    public void addFeedforward(Feedforward feedforward){
        feedforwards.add(feedforward);
    }

    /**
     * get the linear system that is used to calculate the state space controller
     * @return the system
     */
    public LinearSystem<N2, N1, N2> getSystem(){
        return plant;
    }

    /**
     * enable the feedback loop with a consumer to set the voltage, and a supplier to get the position and velocity
     */
    public void enableLoop() {
        if(looping) return;
        loop.reset(VecBuilder.fill(positionSupplier.getAsDouble(),velocitySupplier.getAsDouble()));
        looping = true;
    }

    /**
     * disable the feedback loop (and give control back to the subsystem)
     */
    public void disableLoop() {
        looping = false;
        stopTrajectory();
    }

    /**
     * @return whether the feedback loop is enabled
     */
    public boolean isLooping() {
        return looping;
    }

    /**
     * stop following the current trajectory, but continue holding
     * the current reference
     */
    public void stopTrajectory() {
        followingProfile = false;
    }

    /**
     * generate a trajectory from the current position to the given position,
     * and start following it
     * @param position position setpoin
     */
    public void goToState(double position) {
        if(!looping){
            throw new IllegalStateException("Cannot set state without enabling loop");
        }
        // 3 different options:
        // 1. Create a brand new trajectory from the current state to the target state, and reset the timer
        // 2. Create a trajectory from the current state of the last trajectory to the target state, without resetting the timer
        // 3. Create a trajectory from the initial state of the last trajectory to the target state, without resetting the timer
        // we want to do option 1 if we are at the end of the current profile, or the new distance is far away from the current profile
        // we want to do option 2 if the new distance is close to the current profile, and we are close to the end of the profile
        // we want to do option 3 if the new distance is close to the current profile, and we are not close to the end of the profile

        // if(!followingProfile || Math.abs(position - trajectoryEnd.position) > 0.03){
        //     // option 1
        //     trajectoryStart = new State(positionSupplier.getAsDouble(), velocitySupplier.getAsDouble());
        //     trajectoryEnd = new State(position, 0);
        //     trajectory = Trajectory.trapezoidTrajectory(trajectoryStart, trajectoryEnd, constants.maxVelocity, constants.maxAcceleration);
        //     profileTimer.reset();
        //     profileTimer.start();
        //     followingProfile = true;
        //     recalculateTrajectory = false;
        // } else if(Math.abs(position - trajectoryEnd.position) < 0.03 && profileTimer.get() > trajectory.getTotalTime() - 0.5){
        //     // option 2
        //     trajectoryStart = new State(positionSupplier.getAsDouble(), velocitySupplier.getAsDouble());
        //     trajectoryEnd = new State(position, 0);
        //     trajectory = Trajectory.trapezoidTrajectory(trajectoryStart, trajectoryEnd, constants.maxVelocity, constants.maxAcceleration);
        //     recalculateTrajectory = false;
        // } else {
        //     // option 3
        //     trajectoryEnd = new State(position, 0);
        //     recalculateTrajectory = true;
        // }
        if(Math.abs(position - trajectoryEnd.position) < 0.01 && followingProfile){
            // // option 2
            // trajectoryEnd = new State(position, 0);
            // recalculateTrajectory = true;
            return;
        }
        trajectoryStart = new State(positionSupplier.getAsDouble(), velocitySupplier.getAsDouble());
        trajectoryEnd = new State(position, 0);
        trajectory = Trajectory.trapezoidTrajectory(trajectoryStart, trajectoryEnd, constants.maxVelocity, constants.maxAcceleration);
        profileTimer.reset();
        profileTimer.start();
        followingProfile = true;
        recalculateTrajectory = false;
    }

    /**
     * set the next reference for the feedback loop -
     * you probably don't want to use this, use goToState instead.
     * This set's the reference for the next iteration of the feedback loop directly,
     * and does not generate a trajectory.
     * @param position position reference
     * @param velocity velocity reference
     */
    public void setNextR(double position, double velocity) {
        loop.setNextR(VecBuilder.fill(position, velocity));
    }

    /**
     * allow the subsystem to run code in the periodic method
     * @param superPeriodic a function to run every periodic loop
     */
    public void setPeriodicFunction(Runnable superPeriodic) {
        this.superPeriodic = superPeriodic;
    }

    @Override
    public final void periodic() {
        // run the subsystem's periodic code
        if(superPeriodic != null) superPeriodic.run();
        if(!looping) return; // don't run the feedback loop if it's not enabled

        var time = profileTimer.get(); // get the time since the profile started
        double position = positionSupplier.getAsDouble(), velocity = velocitySupplier.getAsDouble();

        var feedforward = 0.0;

        if(recalculateTrajectory){
            trajectory = Trajectory.trapezoidTrajectory(trajectoryStart, trajectoryEnd, constants.maxVelocity, constants.maxAcceleration);
            recalculateTrajectory = false;
        }

        // if we're following a profile, calculate the next reference
        if(followingProfile){
            var output = trajectory.calculate(time + (constants.dt/2));
            setNextR(output.position, output.velocity);
            for (var i : feedforwards) {
                feedforward += i.calculate(output.position, output.velocity);
            }
        } else {
            // otherwise, just calculate the feedforward to hold the current reference
            for (var i : feedforwards) {
                feedforward += i.calculate(loop.getNextR(0), loop.getNextR(1));
            }
        }

        // run the feedback
        loop.correct(VecBuilder.fill(position, velocity));
        loop.predict(constants.dt);

        voltDriveFunction.accept(loop.getU(0) + feedforward);
    }

    public static final class SystemConstants {
        public double inertia,
                gearing,
                cpr,
                maxVelocity,
                maxAcceleration,
                kalmanModelAccuracyPosition,
                kalmanModelAccuracyVelocity,
                kalmanSensorAccuracyPosition,
                kalmanSensorAccuracyVelocity,
                lqrPositionTolerance,
                lqrVelocityTolerance,
                lqrControlEffortTolerance,
                lqrSensorDelay,
                maxVoltage,
                dt;
        public final DCMotor motor;

        /**
         * @param motor The motor to use
         * @param inertia The inertia of the system
         * @param gearing The gearing of the system
         * @param cpr The counts per revolution of the encoder
         * @param maxVelocity The maximum angular velocity the system can attain
         * @param maxAcceleration The maximum angular acceleration the system can attain
         * @param kalmanModelAccuracyPosition The accuracy of the model for position
         * @param kalmanModelAccuracyVelocity The accuracy of the model for velocity
         * @param kalmanSensorAccuracyPosition The accuracy of the sensor for position
         * @param kalmanSensorAccuracyVelocity The accuracy of the sensor for velocity
         * @param lqrPositionTolerance How aggressively to correct for position error
         * @param lqrVelocityTolerance How aggressively to correct for velocity error
         * @param lqrControlEffortTolerance How aggressively to minimize control effort
         * @param maxVoltage The maximum voltage to try to attain
         * @param dt The time between each loop
         */
        public SystemConstants(DCMotor motor,
                               double inertia,
                               double gearing,
                               double cpr,
                               double maxVelocity,
                               double maxAcceleration,
                               double kalmanModelAccuracyPosition,
                               double kalmanModelAccuracyVelocity,
                               double kalmanSensorAccuracyPosition,
                               double kalmanSensorAccuracyVelocity,
                               double lqrPositionTolerance,
                               double lqrVelocityTolerance,
                               double lqrControlEffortTolerance,
                               double lqrSensorDelay,
                               double maxVoltage,
                               double dt) {
            this.motor = motor;
            this.inertia = inertia;
            this.gearing = gearing;
            this.cpr = cpr;
            this.maxVelocity = maxVelocity;
            this.maxAcceleration = maxAcceleration;
            this.kalmanModelAccuracyPosition = kalmanModelAccuracyPosition;
            this.kalmanModelAccuracyVelocity = kalmanModelAccuracyVelocity;
            this.kalmanSensorAccuracyPosition = kalmanSensorAccuracyPosition;
            this.kalmanSensorAccuracyVelocity = kalmanSensorAccuracyVelocity;
            this.lqrPositionTolerance = lqrPositionTolerance;
            this.lqrVelocityTolerance = lqrVelocityTolerance;
            this.lqrControlEffortTolerance = lqrControlEffortTolerance;
            this.lqrSensorDelay = lqrSensorDelay;
            this.maxVoltage = maxVoltage;
            this.dt = dt;
        }
    }

    public static interface Feedforward{
        double calculate(double position, double velocity);
    }
}
