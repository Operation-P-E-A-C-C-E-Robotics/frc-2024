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
import frc.lib.util.ServoMotor.SystemConstants;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

public class ServoArm extends SubsystemBase {
    private final LinearSystem<N2, N1, N1> plant;
    private final LinearSystemLoop<N2, N1, N1> loop;
    private final SystemConstants constants;
    private Trajectory trajectory;
    private State trajectoryStart, trajectoryEnd;
    private boolean recalculateTrajectory = false; //whether we need to recalculate the trajectory


    private boolean followingProfile = false, looping = false;
    private final Timer profileTimer = new Timer();
    private DoubleConsumer voltDriveFunction;
    private DoubleSupplier getPosition, getVelocity;
    private Runnable superPeriodic;
    /**
     * A messy ass helper class to run trajectories on an arm
     * #authentic2023experience
     * @param constants The constants for the system
     * @param armLength The length of the arm (m)
     * @param armMass The mass of the arm (kg)
     */
    public ServoArm(SystemConstants constants, DoubleSupplier armLength, DoubleConsumer voltDriveFunction, DoubleSupplier getPosition, DoubleSupplier getVelocity, double armMass) {
        this.constants = constants;
        this.voltDriveFunction = voltDriveFunction;
        this.getPosition = getPosition;
        this.getVelocity = getVelocity;
        plant = LinearSystemId.createSingleJointedArmSystem(constants.motor, constants.inertia, constants.gearing);
        KalmanFilter<N2, N1, N1> observer = new KalmanFilter<N2, N1, N1>(
                Nat.N2(),
                Nat.N1(),
                plant,
                VecBuilder.fill(constants.kalmanModelAccuracyPosition, constants.kalmanModelAccuracyVelocity),
                VecBuilder.fill(constants.kalmanSensorAccuracyPosition),
                constants.dt
        );
        LinearQuadraticRegulator<N2, N1, N1> linearQuadraticRegulator = new LinearQuadraticRegulator<N2, N1, N1>(
                plant,
                VecBuilder.fill(constants.lqrPositionTolerance, constants.lqrVelocityTolerance),
                VecBuilder.fill(constants.lqrControlEffortTolerance),
                constants.dt
        );
        loop = new LinearSystemLoop<N2, N1, N1>(
                plant,
                linearQuadraticRegulator,
                observer,
                constants.maxVoltage,
                constants.dt
        );
    }

    /**
     * get the LinearSystem being used by the loop
     * @return the LinearSystem
     */
    public LinearSystem<N2, N1, N1> getSystem(){
        return plant;
    }

    /**
     * enable the feedback loop with a consumer to set the voltage, and a supplier to get the position and velocity
     */
    public void enableLoop() {
        if(trajectory == null){
            trajectoryStart = trajectoryEnd = new State(getPosition.getAsDouble(), getVelocity.getAsDouble());
            trajectory = Trajectory.trapezoidTrajectory(trajectoryStart, trajectoryEnd, constants.maxVelocity, constants.maxAcceleration);
        }
        if(looping) return;
        loop.reset(VecBuilder.fill(getPosition.getAsDouble(), getVelocity.getAsDouble()));
        looping = true;
    }

    /**
     * disable the feedback loop (and give control back to the subsystem)
     */
    public void disableLoop() {
        looping = false;
    }

    /**
     * tell whether the feedback loop is enabled (and thus whether the subsystem has control)
     * @return whether the loop is enabled
     */
    public boolean isLooping() {
        return looping;
    }

    /**
     * calculate additional feedforward to account for gravity
     */
    public double calculateGravityFeedforward(double position, double velocity){
        // var armLength = this.armLength.getAsDouble();
        // var force = armMass     * armLength
        //                         * 9.8
        //                         * 3.0
        //                         * constants.inertia
        //                         / (armMass * armLength * armLength)
        //                         * Math.cos(position - Math.PI*1.5);
        // //account for gearing:
        // force /= constants.gearing;
        //calculate voltage needed to counteract force:
        return 0;// constants.motor.getVoltage(force, velocity);
    }

    /**
     * stop following the current trajectory, but continue holding
     * the current reference
     */
    public void stopTrajectory() {
        followingProfile = false;
    }

    /**
     * generate and follow a new trajectory from the current state to the given state
     * @param position The position to go to
     * @param velocity The velocity to go to
     */
    public void goToState(double position, double velocity) {
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

        if(!followingProfile || Math.abs(position - trajectoryEnd.position) > 0.2){
            // option 1
            trajectoryStart = new State(getPosition.getAsDouble(), getVelocity.getAsDouble());
            trajectoryEnd = new State(position, 0);
            trajectory = Trajectory.trapezoidTrajectory(trajectoryStart, trajectoryEnd, constants.maxVelocity, constants.maxAcceleration);
            profileTimer.reset();
            profileTimer.start();
            followingProfile = true;
            recalculateTrajectory = false;
        } else if(Math.abs(position - trajectoryEnd.position) < 0.2 && profileTimer.get() > trajectory.getTotalTime() - 0.5){
            // option 2
            trajectoryStart = new State(getPosition.getAsDouble(), getVelocity.getAsDouble());
            trajectoryEnd = new State(position, 0);
            trajectory = Trajectory.trapezoidTrajectory(trajectoryStart, trajectoryEnd, constants.maxVelocity, constants.maxAcceleration);
            recalculateTrajectory = false;
        } else {
            // option 3
            trajectoryEnd = new State(position, 0);
            recalculateTrajectory = true;
        }
    }

    /**
     * set the next reference for the feedback loop
     * @param position The position reference
     * @param velocity The velocity reference
     */
    public void setNextR(double position, double velocity) {
        loop.setNextR(VecBuilder.fill(position, velocity));
    }

    /**
     * allow the subsystem to run a periodic function
     * @param superPeriodic The function to run - will run before the feedback loop every periodic call
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
         double position = getPosition.getAsDouble();

         var feedforward = 0.0;

         if(recalculateTrajectory){
             trajectory = Trajectory.trapezoidTrajectory(trajectoryStart, trajectoryEnd, constants.maxVelocity, constants.maxAcceleration);
             recalculateTrajectory = false;
         }

         // if we're following a profile, calculate the next reference
         if(followingProfile){
 //            var output = profile.calculate(time);
             var output = trajectory.calculate(time + 0.1);
             setNextR(output.position, output.velocity);
            //  for (var i : feedforwards) {
            //      feedforward += i.calculate(output.position, output.velocity);
            //  }
            feedforward = calculateGravityFeedforward(output.position, output.velocity);
         } else {
            feedforward = calculateGravityFeedforward(loop.getXHat(0), loop.getXHat(1));
         }

         // run the feedback
         loop.correct(VecBuilder.fill(position));
         loop.predict(constants.dt);

         voltDriveFunction.accept(loop.getU(0) + feedforward);
    }

    public static void main(String args[]){
        var armMass = 1;
        var armLength = 1;
        var gearing = 1;
        var motor = DCMotor.getFalcon500(2);

        //test the feedforward
        for(var i = 0; i < 360; i++){
            var angle = Math.toRadians(i) + Math.PI*1.5;
            var force = (armMass * 9.80665) * armLength * Math.cos(angle + Math.PI);
            force *= 1.0/gearing;
            var voltage = motor.getVoltage(force, 0);
            System.out.println(i + "," + voltage);
        }
    }
}
