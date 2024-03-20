package frc.lib.motion;

import frc.lib.util.Util;

public class LinearAcceleration {
    public final double acceleration, velocity, position, time;

    /**
     * create an acceleration/deceleration curve based on a right triangle:
     * <br/> - the area of the triangle represents the total position change
     * <br/> - one leg represents time
     * <br/> - the other leg represents initial velocity (for deceleration) or final velocity (for acceleration)
     * <br/> - the slope of the hypotenuse (velocity/time) represents acceleration
     * #authentic2023experience
     */
    private LinearAcceleration(double position, double velocity, double acceleration, double time){
        this.acceleration = acceleration;
        this.velocity = velocity;
        this.position = position;
        this.time = time;
    }

    /**
     * find a linear acceleration describing the change along this triangle after a given time
     * @param time the time to interpolate to
     * @return the linear acceleration describing the change along this triangle after a given time
     */
    public LinearAcceleration interpolateAccel(double time){
        return fromAcceleration(acceleration, time);
    }

    /**
     * find a linear acceleration describing the change along this triangle after a given time,
     * but moving from the end of the triangle to the beginning
     * @param time the time to interpolate to
     * @return the linear acceleration describing the change along this triangle after a given time
     */
    public LinearAcceleration interpolateDecel(double time){
        var delta = interpolateAccel(this.time - time);
        return fromPositionVelocity(position - delta.position, velocity - delta.velocity);
    }

    /**
     * Limit the acceleration and velocity of this acceleration to the given maximums,
     * which will change position and time. This is useful for constraining the acceleration to
     * the capabilities of the robot.
     * @param maxVelocity the maximum velocity
     * @param maxAcceleration the maximum acceleration
     * @return the linear acceleration with the given maximums
     */
    public LinearAcceleration limit(double maxVelocity, double maxAcceleration){
        return fromVelocityAcceleration(Util.limit(velocity, maxVelocity), Util.limit(acceleration, maxAcceleration));
    }

    //note from the future: don't worry about what these do, documentation is overrated.
    public LinearAcceleration limitAcceleration(double maxAcceleration){
        return fromAcceleration(Util.limit(acceleration, maxAcceleration), time);
    }

    public LinearAcceleration limitPosition(double maxPosition){
        return fromPositionVelocity(Util.limit(position, maxPosition), velocity);
    }

    public LinearAcceleration limitVelocity(double maxVelocity){
        return fromVelocityAcceleration(Util.limit(velocity, maxVelocity), acceleration);
    }

    public static LinearAcceleration fromPosition(double position, double time){
        return fromVelocity((2*position) / time, time);
    }

    /**
     * find the position, and acceleration needed for the given change in velocity over the given time
     * @param velocity the change in velocity
     * @param time the time to change velocity over
     * @return the linear acceleration describing the change in velocity over the given time
     */
    public static LinearAcceleration fromVelocity(double velocity, double time){
        return fromAcceleration(velocity / time, time);
    }

    /**
     * find the position, and velocity accomplished by accelerating by the given acceleration over the given time
     * @param acceleration the acceleration
     * @param time the time to accelerate over
     * @return the linear acceleration describing the acceleration over the given time
     */
    public static LinearAcceleration fromAcceleration(double acceleration, double time){
        return new LinearAcceleration(0.5 * acceleration * time * time, time * acceleration, acceleration, time);
    }

    /**
     * find the time, and acceleration needed to attain the given change in position and velocity
     * @param position the change in position
     * @param velocity the change in velocity
     * @return the linear acceleration describing the change in position and velocity
     */
    public static LinearAcceleration fromPositionVelocity(double position, double velocity){
        return fromVelocity(velocity, (2 * position) / velocity);
    }

    /**
     * find the time, and position needed to attain the given velocity and acceleration
     * @param velocity the change in velocity
     * @param acceleration the acceleration
     * @return the linear acceleration describing the change in velocity and acceleration
     */
    public static LinearAcceleration fromVelocityAcceleration(double velocity, double acceleration){
        return fromAcceleration(acceleration, velocity / acceleration);
    }

    public String toString(){
        return "position: " + position + ", velocity: " + velocity + ", acceleration: " + acceleration + ", time: " + time;
    }
}