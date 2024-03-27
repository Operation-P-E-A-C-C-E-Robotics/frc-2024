package frc.lib.util;

import edu.wpi.first.math.geometry.*;

import java.util.ArrayList;
import java.util.List;

/**
 * Contains basic functions that are used often.
 */
public class Util {

    public static final double EPSILON = 1e-12;

    /**
     * Prevent this class from being instantiated.
     */
    private Util() {
    }

    /**
     * Limits the given input to the given magnitude.
     */
    public static double limit(double v, double maxMagnitude) {
        return limit(v, -maxMagnitude, maxMagnitude);
    }

    /**
     * Limits the given input to the given range.
     * @param v the value to limit
     * @param min the minimum value
     * @param max the maximum value
     */
    public static double limit(double v, double min, double max) {
        return Math.min(max, Math.max(min, v));
    }

    /**
     * interpolate between two values
     * @param a start value
     * @param b end value
     * @param x interpolation value (0.0 - 1.0)
     * @return interpolated value
     */
    public static double interpolate(double a, double b, double x) {
        x = limit(x, 0.0, 1.0);
        return a + (b - a) * x;
    }

    /**
     * join a list of strings with a delimiter
     * @param delim delimiter
     * @param strings list of strings
     * @return joined string
     */
    public static String joinStrings(final String delim, final List<?> strings) {
        StringBuilder sb = new StringBuilder();
        for (int i = 0; i < strings.size(); ++i) {
            sb.append(strings.get(i).toString());
            if (i < strings.size() - 1) {
                sb.append(delim);
            }
        }
        return sb.toString();
    }

    /**
     * determine if two doubles are equal within a given epsilon
     * @param a first double
     * @param b second double
     * @param epsilon epsilon - how close to be considered equal
     * @return true if equal
     */
    public static boolean epsilonEquals(double a, double b, double epsilon) {
        return (a - epsilon <= b) && (a + epsilon >= b);
    }

    /**
     * determine if two doubles are equal within the default epsilon
     * of 1e-12
     * @param a first double
     * @param b second double
     * @return true if equal
     */
    public static boolean epsilonEquals(double a, double b) {
        return epsilonEquals(a, b, EPSILON);
    }

    /**
     * determine if two ints are equal within a given epsilon
     * @param a first int
     * @param b second int
     * @param epsilon epsilon - how close to be considered equal
     * @return true if equal
     */
    public static boolean epsilonEquals(int a, int b, int epsilon) {
        return (a - epsilon <= b) && (a + epsilon >= b);
    }

    /**
     * determine if all values in a list are within a given epsilon
     * to a given value
     * @param list list of doubles
     * @param value value to compare to
     * @param epsilon epsilon - how close to be considered equal
     * @return true if all values are within epsilon of value
     */
    public static boolean allCloseTo(final List<Double> list, double value, double epsilon) {
        boolean result = true;
        for (Double value_in : list) {
            result &= epsilonEquals(value_in, value, epsilon);
        }
        return result;
    }

    /**
     * get a sublist of a list
     * @param list list to get sublist from
     * @param start start index
     * @param end end index
     * @return sublist
     */
    public static List<Double> subList(List<Double> list, int start, int end){
        ArrayList<Double> newList = new ArrayList<>();
        for(int i = start; i <= end; i++){
            newList.add(list.get(i));
        }
        return newList;
    }

    /**
     * determine if a value is within a given range
     * @param in value to check
     * @param range range to check
     * @return true if in is within range
     */
    public static boolean inRange(double in, double range){
        return (Math.abs(in) < Math.abs(range));
    }

    /**
     * index a double list from the end
     * @param ar double list
     * @param i index from back of list
     * @return list element
     */
    public static Double last(List<Double> ar, int i){
        return ar.get(ar.size() - 1 - i);
    }

    /**
     * index an array from the end
     * @param ar double array
     * @param i index from back of array
     * @return array element
     */
    public static double last(double[] ar, int i){
        return ar[ar.length - 1 - i];
    }

    /**
     * shift an element into the left side of a double array, moving all elements one step
     */
    public static double[] shiftLeft(double[] ar, double newv){
        int i = 0;
        for (;i < ar.length - 1; i++){
            ar[i] = ar[i + 1];
        }
        ar[i] = newv;
        return ar;
    }

    /**
     * handle deadband on a joystick
     * by setting the value to zero if it is within the deadband
     * @param joystickPosition joystick position
     * @param deadband deadband
     * @return joystick position or zero if within deadband
     */
    public static double handleDeadband(double joystickPosition, double deadband) {
        if(inRange(joystickPosition, deadband)) return 0;
        return joystickPosition;
    }

    public static double handleDeadbandWithSlopeIncrease(double joystickPosition, double deadband){
        if(inRange(joystickPosition, deadband)) return 0;
        return joystickPosition - deadband * Math.signum(joystickPosition);
    }

    /**
     * convert a Translation2d to a Pose3d
     * by setting the z, roll, pitch, and yaw to zero
     * @param translation translation to convert
     * @return pose3d
     */
    public static Pose3d toPose3d(Translation2d translation) {
        return toPose3d(new Pose2d(translation, new Rotation2d()));
    }
    public static Pose3d toPose3d(Translation2d translation, double z) {
        return toPose3d(new Pose2d(translation, new Rotation2d()), z, 0, 0);
    }

    /**
     * convert a Pose2d to a Pose3d
     * by setting the z, roll, and pitch to zero
     * @param pose pose to convert
     * @return pose3d
     */
    public static Pose3d toPose3d(Pose2d pose){
        return toPose3d(pose, 0,0,0);
    }

    /**
     * convert a Pose2d to a Pose3d, with the option to set the z, roll, and pitch
     * @param pose pose to convert
     * @param z z
     * @param roll roll
     * @param pitch pitch
     * @return pose3d
     */
    public static Pose3d toPose3d(Pose2d pose, double z, double roll, double pitch){
        return new Pose3d(
                pose.getX(),
                pose.getY(),
                0,
                new Rotation3d(roll, pitch, pose.getRotation().getRadians())
        );
    }

    /**
     * convert a Pose3d to a Pose2d
     * by ignoring the z, roll, and pitch
     * @param pose pose to convert
     * @return pose2d
     */
    public static Pose2d toPose2d(Pose3d pose){
        return new Pose2d(pose.getX(), pose.getY(), new Rotation2d(pose.getRotation().getZ()));
    }

    /**
     * convert a pose from one coordinate system to another, where the origin of
     * the local coordinate system is known
     * @param localOrigin the pose of the base of the local coordinate system relative to the global origin
     * @param localPoint the local pose to convert
     * @return the pose relative to the global origin
     */
    public static Pose3d localToGlobalPose(Pose3d localOrigin, Pose3d localPoint) {
        Translation3d rotatedTranslation = rotateBy(localPoint.getTranslation(), localOrigin.getRotation());
        Rotation3d rotatedRotation = localPoint.getRotation().plus(localOrigin.getRotation());

        return new Pose3d(
                localOrigin.getTranslation().plus(rotatedTranslation),
                rotatedRotation
        );
    }

    /**
     * convert a pose from a global coordinate system to a local coordinate system, where the origin of
     * the local coordinate system is known
     * @param localOrigin the pose of the base of the local coordinate system relative to the global origin
     * @param globalPoint the global pose to convert
     * @return the pose relative to the local origin
     */
    public static Pose3d globalToLocalPose(Pose3d localOrigin, Pose3d globalPoint) {
        Translation3d localTranslation = globalPoint.getTranslation().minus(localOrigin.getTranslation());
        Translation3d rotatedTranslation = rotateBy(localTranslation, new Rotation3d().minus(localOrigin.getRotation()));
        Rotation3d rotatedRotation = globalPoint.getRotation().minus(localOrigin.getRotation());
        return new Pose3d(
                rotatedTranslation,
                rotatedRotation
        );
    }

    //https://stackoverflow.com/questions/34050929/3d-point-rotation-algorithm
    /**
     * rotate a point by a rotation
     * @param point point to rotate
     * @param rotation rotation to rotate by
     * @return rotated point
     */
    public static Translation3d rotateBy(Translation3d point, Rotation3d rotation){
        double cosa = Math.cos(rotation.getZ());
        double sina = Math.sin(rotation.getZ());

        double cosb = Math.cos(rotation.getY());
        double sinb = Math.sin(rotation.getY());

        double cosc = Math.cos(rotation.getX());
        double sinc = Math.sin(rotation.getX());

        var Axx = cosa*cosb;
        var Axy = cosa*sinb*sinc - sina*cosc;
        var Axz = cosa*sinb*cosc + sina*sinc;

        var Ayx = sina*cosb;
        var Ayy = sina*sinb*sinc + cosa*cosc;
        var Ayz = sina*sinb*cosc - cosa*sinc;

        var Azx = -sinb;
        var Azy = cosb*sinc;
        var Azz = cosb*cosc;

        return new Translation3d(
                Axx*point.getX() + Axy*point.getY() + Axz*point.getZ(),
                Ayx*point.getX() + Ayy*point.getY() + Ayz*point.getZ(),
                Azx*point.getX() + Azy*point.getY() + Azz*point.getZ()
        );
    }

    //functions to convert from encoder counts to rotations and back
    /**
     * convert encoder counts to rotations
     * @param counts encoder counts
     * @param cpr counts per rotation
     * @param gearRatio gear ratio
     * @return rotations
     */
    public static double countsToRotations(double counts, double cpr, double gearRatio){
        return (counts / cpr) / gearRatio;
    }

    /**
     * convert encoder counts to rotations
     * assuming 2048 counts per rotation and a gear ratio of 1
     * @param counts encoder counts
     * @return rotations
     */
    public static double countsToRotations(double counts){
        return countsToRotations(counts, 2048, 1);
    }

    /**
     * convert encoder counts to rotations
     * @param counts encoder counts
     * @param cpr counts per rotation
     * @return rotations
     */
    public static double countsToRotations(double counts, double cpr){
        return countsToRotations(counts, cpr, 1);
    }

    /**
     * convert encoder counts to rotations
     * @param counts encoder counts
     * @param cpr counts per rotation
     * @param gearRatio gear ratio
     * @param wheelDiameter diameter of the wheel
     * @return distance traveled
     */
    public static double countsToRotations(double counts, double cpr, double gearRatio, double wheelDiameter){
        return countsToRotations(counts, cpr, gearRatio) * wheelDiameter * Math.PI;
    }
    /**
     * convert rotations to encoder counts
     * @param rotations rotations
     * @param cpr counts per rotation
     * @param gearRatio gear ratio
     * @return encoder counts
     */
    public static double rotationsToCounts(double rotations, double cpr, double gearRatio){
        return rotations * cpr * gearRatio;
    }

    /**
     * convert rotations to encoder counts
     * assuming 2048 counts per rotation and a gear ratio of 1
     * @param rotations rotations
     * @return encoder counts
     */
    public static double rotationsToCounts(double rotations){
        return rotationsToCounts(rotations, 2048, 1);
    }

    /**
     * convert rotations to encoder counts
     * assuming a gear ratio of 1
     * @param rotations rotations
     * @param cpr counts per rotation
     * @return encoder counts
     */
    public static double rotationsToCounts(double rotations, double cpr){
        return rotationsToCounts(rotations, cpr, 1);
    }

    /**
     * convert rotations to encoder counts
     * @param rotations rotations
     * @param cpr counts per rotation
     * @param gearRatio gear ratio
     * @param wheelDiameter diameter of the wheel
     * @return encoder counts
     */
    public static double rotationsToCounts(double rotations, double cpr, double gearRatio, double wheelDiameter){
        return rotationsToCounts(rotations, cpr, gearRatio) / (wheelDiameter * Math.PI);
    }

    static{commentsAreForCommoners("so i can be sarcastic about the lack of comments in certain software i write");}
    public static void commentsAreForCommoners(String notAComment){}

    public static void main(String args[]){
        for(double i = -1; i < 1; i += 0.02){
            System.out.println(i + " " + Util.handleDeadbandWithSlopeIncrease(i, 0.4));
        }
    }
}