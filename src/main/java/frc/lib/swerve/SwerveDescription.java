package frc.lib.swerve;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.ClosedLoopOutputType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants.SteerFeedbackType;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * CTRE's way of writing out swerve constants is messy, hard to read, and documented like a wet fart, so I made this.
 * 
 * Here's a fun poem about improving other people's trash code:
 * It is indeed quite a feat,
 * writing code so impossibly neat,
 * that's why you didn't do it, bro,
 * so now you have to read my code,
 * and wish you had written it so.
 * Seriously, why couldn't you have written it so?
 * Then I could have slept or fished or whatever, but no.
 * -Peaccy
 */
public class SwerveDescription {

    /**
     * Generates a swerve drivetrain from a bunch of constants.
     * @param dimensions the dimensions of the chassis
     * @param frontLeftIDs the CAN bus ID's for the front left module
     * @param frontRightIDs the CAN bus ID's for the front right module
     * @param rearLeftIDs the CAN bus ID's for the rear left module
     * @param rearRightIDs the CAN bus ID's for the rear right module
     * @param gearing the gearing of the modules
     * @param offsets the encoder offsets of the modules
     * @param inversion the inversion of the modules
     * @param physics the physical parameters of the drivetrain
     * @param driveGains the PID gains for the drive motors
     * @param angleGains the PID gains for the steer motors
     * @param pigeonCANId the CAN bus ID of the pigeon
     * @param invertSteerMotors whether or not to invert the steer motors
     * @return a CTRE swerve drivetrain
     */
    public static PeaccefulSwerve generateDrivetrain (
                                Dimensions dimensions, 
                                CANIDs frontLeftIDs,
                                CANIDs frontRightIDs,
                                CANIDs rearLeftIDs,
                                CANIDs rearRightIDs,
                                Gearing gearing,
                                EncoderOffsets offsets,
                                Inversion inversion,
                                Physics physics,
                                PidGains driveGains,
                                PidGains angleGains,
                                int pigeonCANId,
                                boolean invertSteerMotors){
        SwerveDrivetrainConstants swerveConstants = new SwerveDrivetrainConstants()
                                                    .withPigeon2Id(pigeonCANId)
                                                    .withCANbusName("rio");
        SwerveModuleConstantsFactory globalModuleConstants = new SwerveModuleConstantsFactory()
                                    .withDriveMotorGearRatio(gearing.driveRatio)
                                    .withDriveMotorClosedLoopOutput(ClosedLoopOutputType.TorqueCurrentFOC)
                                    .withSteerMotorGearRatio(gearing.steerRatio)
                                    .withSteerMotorClosedLoopOutput(ClosedLoopOutputType.Voltage)
                                    .withWheelRadius(gearing.wheelRadius)
                                    .withSlipCurrent(physics.wheelSlipCurrent)
                                    .withSteerMotorGains(angleGains)
                                    .withDriveMotorGains(driveGains)
                                    .withSpeedAt12VoltsMps(physics.freeSpeed)
                                    .withSteerInertia(physics.angularInertia)
                                    .withDriveInertia(physics.linearInertia)
                                    .withFeedbackSource(SteerFeedbackType.FusedCANcoder)
                                    .withCouplingGearRatio(gearing.steerCouplingRatio)
                                    .withSteerMotorInverted(invertSteerMotors);

    
        SwerveModuleConstants frontLeft = globalModuleConstants.createModuleConstants(
            frontLeftIDs.steerMotor, 
            frontLeftIDs.driveMotor, 
            frontLeftIDs.encoder, 
            offsets.frontLeft,
            dimensions.frontLeft.getX(), 
            dimensions.frontLeft.getY(), 
            inversion.frontLeft
        );

        SwerveModuleConstants frontRight = globalModuleConstants.createModuleConstants(
            frontRightIDs.steerMotor, 
            frontRightIDs.driveMotor, 
            frontRightIDs.encoder, 
            offsets.frontRight,
            dimensions.frontRight.getX(), 
            dimensions.frontRight.getY(), 
            inversion.frontRight
        );

        SwerveModuleConstants rearLeft = globalModuleConstants.createModuleConstants(
            rearLeftIDs.steerMotor, 
            rearLeftIDs.driveMotor, 
            rearLeftIDs.encoder, 
            offsets.rearLeft,
            dimensions.rearLeft.getX(), 
            dimensions.rearLeft.getY(), 
            inversion.rearLeft
        );

        SwerveModuleConstants rearRight = globalModuleConstants.createModuleConstants(
            rearRightIDs.steerMotor, 
            rearRightIDs.driveMotor, 
            rearRightIDs.encoder, 
            offsets.rearRight,
            dimensions.rearRight.getX(), 
            dimensions.rearRight.getY(), 
            inversion.rearRight
        );

        return new PeaccefulSwerve(
            swerveConstants,
            frontLeft,
            frontRight,
            rearLeft,
            rearRight
        );
    }

    public static class Dimensions {
        public final double width, height;
        public final Translation2d frontLeft, frontRight, rearLeft, rearRight;

        /**
         * the wheel-to-wheel dimensions of the swerve chassis in meters
         * @param width the width of the chassis
         * @param height the height of the chassis
         */
        public Dimensions (double width, double height) {
            this.width = width;
            this.height = height;

            frontLeft = new Translation2d(height/2, width/2);
            frontRight = new Translation2d(height/2, -width/2);
            rearLeft = new Translation2d(-height/2, width/2);
            rearRight = new Translation2d(-height/2, -width/2);

        }
    }

    public static class Physics {
        public final double linearInertia, angularInertia, wheelSlipCurrent, freeSpeed;
 
        /**
         * the physical parameters of the drivetrain. Inertia is only used for simulation (I'm pretty sure).
         * @param linearInertia the linear inertia of the drivetrain in unknown units (maybe kg*m^2)
         * @param angularInertia the angular inertia of the drivetrain in unknown units
         * @param wheelSlipCurrent the current at which the wheels begin to slip in amps I THINK (not sure) WRONG!!!!!!! ITS THE GODDAMN CURRENT LIMIT.
         * @param freeSpeed the free speed of the drivetrain in meters per second
         */
        public Physics (double linearInertia, double angularInertia, double wheelSlipCurrent, double freeSpeed) {
            this.linearInertia = linearInertia;
            this.angularInertia = angularInertia;
            this.wheelSlipCurrent = wheelSlipCurrent;
            this.freeSpeed = freeSpeed;
        }
    }
    
    public static class EncoderOffsets {
        public final double frontLeft, frontRight, rearLeft, rearRight;

        /**
         * the encoder offsets for the rotation motors to be straight in NEGATED ROTATIONS (wtf ctre)
         * @param frontLeft the offset for the front left module
         * @param frontRight the offset for the front right module
         * @param rearLeft the offset for the rear left module
         * @param rearRight the offset for the rear right module
         */
        public EncoderOffsets (double frontLeft, double frontRight, double rearLeft, double rearRight) {
            this.frontLeft = frontLeft;
            this.frontRight = frontRight;
            this.rearLeft = rearLeft;
            this.rearRight = rearRight;
        }
    }

    public static class Inversion {
        public final boolean frontLeft, frontRight, rearLeft, rearRight;

        /**
         * describes the inversion of the drive motors. Just trial / error I guess 
         * since which ones are inverted don't really make sense on my robot.
         * My bot with mk4i's is front left and rear right inverted
         * @param frontLeft the inversion of the front left module
         * @param frontRight the inversion of the front right module
         * @param rearLeft the inversion of the rear left module
         * @param rearRight the inversion of the rear right module
         */
        public Inversion (boolean frontLeft, boolean frontRight, boolean rearLeft, boolean rearRight) {
            this.frontLeft = frontLeft;
            this.frontRight = frontRight;
            this.rearLeft = rearLeft;
            this.rearRight = rearRight;
        }

        /**
         * describes the inversion of the drive motors. Just trial / error I guess 
         * since which ones are inverted don't really make sense on my robot.
         * SHORTCUT IF ONLY INVERTING LEFT AND RIGHT. Doesn't work with my bot with mk4i's
         * @param left the inversion of the left modules
         * @param right the inversion of the right modules
         */
        public Inversion (boolean left, boolean right) {
            this.frontLeft = left;
            this.frontRight = right;
            this.rearLeft = left;
            this.rearRight = right;
        }
    }

    public static class Gearing {
        public final double driveRatio, steerRatio, wheelRadius, steerCouplingRatio;

        /**
         * describes all the gearing for both drive and steer.
         * The only weird thing is the steer coupling ratio, which is the ratio between turns of the steer motor and turns of the wheel.
         * This is an actual thing because of how the drive motor is coupled through the steering, the steering does have an effect on the drive wheel.
         * No clue what CTRE actually uses this for. Could be to reduce scrub or improve odometry.
         * You can leave it 0 and you'll be just fine.
         * @param driveRatio ratio of drive motor turns to wheel turns
         * @param steerRatio ratio of steer motor turns to steer turns
         * @param wheelRadius radius of the wheel in INCHES WTF CTRE
         * @param steerCouplingRatio ratio of steer motor turns to wheel turns.
         */
        public Gearing (double driveRatio, double steerRatio, double wheelRadius, double steerCouplingRatio) {
            this.driveRatio = driveRatio;
            this.steerRatio = steerRatio;
            this.wheelRadius = wheelRadius;
            this.steerCouplingRatio = steerCouplingRatio;
        }
    }

    public static class CANIDs {
        public final int driveMotor,steerMotor,encoder;

        /**
         * CAN Bus ID's for one module.
         * @param driveMotor drive motor ID
         * @param steerMotor steer motor ID
         * @param encoder CANCoder ID
         */
        public CANIDs (int driveMotor, int steerMotor, int encoder) {
            this.driveMotor = driveMotor;
            this.steerMotor = steerMotor;
            this.encoder = encoder;
        }
    }

    public static class PidGains extends Slot0Configs {

        /**
         * PID gains for either steer or drive motors. (same constants for both).
         * You can get away with only using a P gain, if you don't care about your robot working well.
         */
        public PidGains(double kP, double kI, double kD, double kV, double kS) {
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
            this.kV = kV;
            this.kS = kS;
        }
  }
}
