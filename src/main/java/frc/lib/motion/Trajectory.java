package frc.lib.motion;

import edu.wpi.first.math.trajectory.TrapezoidProfile.State;

import java.util.ArrayList;

public class Trajectory {
    private Motion[] waypoints;
    private final State initialState;
    private State finalState;

    /**
     * You can tell how panicked I was when I wrote this because I didn't even bother to document it.
     * Late-2023 season shenanigans. I will not be held responsible for any damage caused by this code.
     * And I am not going to document it, so you can get the authentic experience of 2023 season.
     * Also so that the next programmer won't be able to reuse all my stuff and pretend they are the programming god I am.
     * (I'm just kidding bro, message me on slack if you need help with anything, or cheifdelphi @Phlogiston)
     * #authentic2023experience
     * @param waypoints the waypoints to follow
     */
    public Trajectory(State... waypoints){
        this.initialState = waypoints[0];
        this.waypoints = new Motion[waypoints.length - 1];
        for(int i = 1; i < waypoints.length; i++){
            this.waypoints[i - 1] = Motion.fromState(waypoints[i - 1], waypoints[i]);
        }
        this.finalState = waypoints[waypoints.length-1];
    }

    public State calculate(double time){
        if(time >= getTotalTime()) return finalState;
        var waypoint = waypoints[0];
        var timeInWaypoint = 0.0;
        var position = initialState.position;
        for (Motion motion : waypoints) {
            if (timeInWaypoint + motion.time > time) {
                waypoint = motion;
                break;
            }
            position += motion.deltaPosition;
            timeInWaypoint += motion.time;
        }
        var interpolated = waypoint.interpolateTime(time - timeInWaypoint);
        return new State(position + interpolated.deltaPosition, interpolated.finalVelocity);
    }

    public State getTarget(){
        return finalState;
    }

    public double getTotalTime(){
        var time = 0.0;
        for (Motion motion : waypoints) {
            time += motion.time;
        }
        return time;
    }

    public void append(Trajectory other){
        var newWaypoints = new Motion[waypoints.length + other.waypoints.length];
        System.arraycopy(waypoints, 0, newWaypoints, 0, waypoints.length);
        System.arraycopy(other.waypoints, 0, newWaypoints, waypoints.length, other.waypoints.length);
        waypoints = newWaypoints;
    }

    public static Trajectory trapezoidTrajectory(State current, State target, double maxVelocity, double maxAcceleration){
        //calculate the distance to accelerate and coast
        var deltaPosition = Math.abs(target.position - current.position);
        var distanceSign = Math.signum(target.position - current.position);
        ArrayList<State> states = new ArrayList<>();

        states.add(current);

        // if the current velocity is in the opposite direction of the target, decelerate to 0 and then accelerate
        // if we don't do this, the math gets pissy.
        if(distanceSign != Math.signum(current.velocity) && distanceSign != 0 && Math.signum(current.velocity) != 0){
            //calculate the distance to decelerate from the current velocity to 0:
            var decelerationDistance = Math.pow(current.velocity, 2) / (2 * maxAcceleration);

            //calculate the two trajectories
            var intermediate = new State(current.position + decelerationDistance * Math.signum(current.velocity), 0);

            states.add(intermediate);
            current = intermediate;
            deltaPosition += decelerationDistance;
        }

        var accelerationDistance = Math.pow(maxVelocity, 2) / (2 * maxAcceleration);
        var coastDistance = deltaPosition - 2 * accelerationDistance;

        // if the distance it will take to accelerate is higher than the distance to the target, accelerate
        // to a lower velocity and don't coast (triangular profile)
        if(accelerationDistance > deltaPosition/2){
            maxVelocity = Math.sqrt(2 * maxAcceleration * deltaPosition);
            accelerationDistance = deltaPosition / 2;
            coastDistance = 0;
        }

        // reapply the sign to the outputs
        accelerationDistance *= distanceSign;
        coastDistance *= distanceSign;
        maxVelocity *= distanceSign;

        states.add(new State(current.position + accelerationDistance, maxVelocity));
        states.add(new State(current.position + accelerationDistance + coastDistance, maxVelocity));
        states.add(target);

        return new Trajectory(
                states.toArray(new State[0])
        );
    }

    public static void main(String[] args){
        var test = Trajectory.trapezoidTrajectory(
                new State(9.9, 5),
                new State(10, 0),
                5,
                5
        );
        System.out.println(test.getTotalTime());
        System.out.println(test.calculate(100).position);
        for(int i = 0; i < 100; i++){
            var state = test.calculate((i / 100.0) * test.getTotalTime());
            System.out.println(state.position);
        }
    }
}
