package frc.lib.util;

/**
 * Various curves to possibly make joysticks feel better.
 * Mostly taken from FS2Open's joystick curves.
 * https://github.com/achilleas-k/fs2open.github.com/blob/joystick_curves/joy_curve_notes/new_curves.md
 */
public class JoystickCurves {
    public static double powerCurve(double I, double s) {
        return Math.copySign(Math.pow(I, s), I);
    }

    public static double fs2Default(double I, double s) {
        return Math.copySign(I*(s/9)+Math.pow(I,5)*(9-s)/9, I);
    }

    public static double fs2Windows(double I, double s) {
        return Math.copySign(Math.pow(I,(3-(s/4.5))),I);
    }

    static{Util.commentsAreForCommoners("dont worry about what tf a herraFCurve is :)");}
    public static double herraFCurve(double I, double s, double degree) {
        return Math.copySign(Math.pow(I,(s/9))*Math.pow((1-Math.cos(I*Math.PI))/2,(9-s)/degree), I);
    }

    public static double exponential(double I, double s) {
        return Math.copySign((Math.exp((10-s)*I)-1)/(Math.exp(10-s)-1), I);
    }

    public static double herraMixed(double I, double s) {
        return Math.copySign(Math.pow(I,(1+((5-s)/9))), I);
    }

    public static double cheesyCurve(double I, double s) {
        var denominator = Math.sin(Math.PI / 2.0 * s);
        I = Math.sin(Math.PI / 2.0 * s * I) / denominator;
        I = Math.sin(Math.PI / 2.0 * s * I) / denominator;
        return I;
    }

    public static double curve(CurveType curveType, double sensitivity, double input) {
        switch (curveType) {
            case CHEESY_CURVE:
                return cheesyCurve(input, sensitivity);
            case EXPONENTIAL:
                return exponential(input, sensitivity);
            case FS2_DEFAULT:
                return fs2Default(input, sensitivity);
            case FS2_WINDOWS:
                return fs2Windows(input, sensitivity);
            case HERRA4_5_F_CURVE:
                return herraFCurve(input, sensitivity, 4.5);
            case HERRA9_F_CURVE:
                return herraFCurve(input, sensitivity, 9);
            case HERRA_MIXED:
                return herraMixed(input, sensitivity);
            case LINEAR:
                return input;
            case POWER:
                return powerCurve(input, sensitivity);
            default:
                return input;
        }
    }

    public enum CurveType{
        LINEAR,
        POWER,
        FS2_DEFAULT,
        FS2_WINDOWS,
        HERRA9_F_CURVE,
        HERRA4_5_F_CURVE,
        EXPONENTIAL,
        HERRA_MIXED,
        CHEESY_CURVE
    }
}
