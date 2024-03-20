package frc.lib.util;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;

//#authentic2023experience
public class PoseFilter {
    public static final double TRANSLATION_RATE_LIMIT = 0.001;
    public static final double ROTATION_RATE_LIMIT = 0.1;
    public static double RESET_THRESHOLD = 1;

    SlewRateLimiter xFilter = new SlewRateLimiter(TRANSLATION_RATE_LIMIT);
    SlewRateLimiter yFilter = new SlewRateLimiter(TRANSLATION_RATE_LIMIT);
    SlewRateLimiter zFilter = new SlewRateLimiter(TRANSLATION_RATE_LIMIT);
    SlewRateLimiter pitchFilter = new SlewRateLimiter(ROTATION_RATE_LIMIT);
    SlewRateLimiter yawFilter = new SlewRateLimiter(ROTATION_RATE_LIMIT);
    SlewRateLimiter rollFilter = new SlewRateLimiter(ROTATION_RATE_LIMIT);
    Pose3d prev;
    

    private double calculateX(double input) {
        if (Double.isNaN(input)) {
            return input;
        }
        return xFilter.calculate(input);
    }
    
    private double calculateY(double input) {
        if (Double.isNaN(input)) {
            return input;
        }

        return yFilter.calculate(input);
    }
    
    private double calculateZ(double input) {
        if (Double.isNaN(input)) {
            return input;
        }

        return zFilter.calculate(input);
    }

    private double calculatePitch(double input) {
        if (Double.isNaN(input)) {
            return input;
        }

        return pitchFilter.calculate(input);
    }

    private double calculateYaw(double input) {
        if (Double.isNaN(input)) {
            return input;
        }
        return yawFilter.calculate(input);
    }

    private double calculateRoll(double input) {
        if (Double.isNaN(input)) {
           return input;
        }

        return rollFilter.calculate(input);
    }


    public Pose3d calculate(Pose3d unfilteredPose3d) {
        if(prev == null){
            reset(unfilteredPose3d);
            return unfilteredPose3d;
        }
        var delta = prev.minus(unfilteredPose3d);
        if(Math.max(Math.abs(delta.getX()), Math.abs(delta.getY())) > RESET_THRESHOLD){
            reset(unfilteredPose3d);
            return unfilteredPose3d;
        }

        prev = unfilteredPose3d;
        
        return new Pose3d(
            calculateX(unfilteredPose3d.getTranslation().getX()),
            calculateY(unfilteredPose3d.getTranslation().getY()),
            calculateZ(unfilteredPose3d.getTranslation().getZ()),
            new Rotation3d(
                calculateRoll(unfilteredPose3d.getRotation().getX()),
                calculatePitch(unfilteredPose3d.getRotation().getY()),
                calculateYaw(unfilteredPose3d.getRotation().getZ())
            )
        );
    }

    public void reset(Pose3d position) {
        xFilter.reset(position.getX());
        yFilter.reset(position.getY());
        zFilter.reset(position.getZ());
        rollFilter.reset(position.getRotation().getX());
        pitchFilter.reset(position.getRotation().getY());
        yawFilter.reset(position.getRotation().getZ());
    }

}
