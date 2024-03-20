package frc.lib.swerve;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;

/**
 * OH MY LORD DONT MAKE ME EXTEND YOUR STUPID CLASS TO ADD BASIC FUNCTIONALITY.
 * ADD ONE FUNCTION SO I CAN GET THE GODDAMN KNEMATICS DONT MAKE THEM """"PROTECTED"""" SO I HAVE TO DO ALL THIS NONSENSE JUST TO GET THE
 * CHASSIS SPEEDS. 
 * Okay maybe I wanted to do a couple other things too...
 * 
 * Heres a terrible poem about having to re-write other ppls code because they didn't include the function you wanted,
 * and that kind of irritated you, and so you made the stupid class and then ended up adding a bunch of other crap too,
 * so it was probably for the better in the first place, and so it is what it is:
 * I litterally don't know what i'm doing with my life,
 * writing poems about my superficial strife,
 * stupiditiy creating stupidity,
 * like a stupid casarole or whatever.
 * -Peaccy
 */
public class PeaccefulSwerve extends SwerveDrivetrain {
    private Rotation3d pigeonOdometryOffset = new Rotation3d();
    private double visionZ = 0;


    public PeaccefulSwerve(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency,
            SwerveModuleConstants[] modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
    }

    public PeaccefulSwerve(SwerveDrivetrainConstants driveTrainConstants,
            SwerveModuleConstants[] modules, int pigeonCANId) {
        super(driveTrainConstants, modules);
    }
 
    public PeaccefulSwerve(SwerveDrivetrainConstants swerveConstants, SwerveModuleConstants frontLeft,
            SwerveModuleConstants frontRight, SwerveModuleConstants rearLeft, SwerveModuleConstants rearRight) {
        super(swerveConstants, frontLeft, frontRight, rearLeft, rearRight);
    }

    public void optimizeBusUtilization() {
        for(SwerveModule i : Modules) {
            ParentDevice.optimizeBusUtilizationForAll(i.getCANcoder(), i.getDriveMotor(), i.getSteerMotor());
        }
    }


    /**
     * get the """PROWTECTED""" chassis speeds
     * @return the chassis speeds
     */
    public ChassisSpeeds getChassisSpeeds() {
        var moduleStates = getState().ModuleStates;
        if (moduleStates == null) return new ChassisSpeeds();
        return m_kinematics.toChassisSpeeds(moduleStates);
    }

    /**
     * get angle motor closed loop errors for all modules 
     * @return a double array of the closed loop errors
     */
    public double[] getAngleClosedLoopErrors() {
        double[] errors = new double[4];
        int i = 0;
        for (SwerveModule module : Modules) {
            errors[i] = module.getSteerMotor().getClosedLoopError().getValueAsDouble();
        }
        return errors;
    }

    /**
     * get drive motor closed loop errors for all modules
     * @return a double array of the closed loop errors
     */
    public double[] getDriveClosedLoopErrors() {
        double[] errors = new double[4];
        int i = 0;
        for (SwerveModule module : Modules) {
            errors[i] = module.getDriveMotor().getClosedLoopError().getValueAsDouble();
        }
        return errors;
    }

    /**
     * apply different pid configs to all the drive motors.
     * @param configs the new pidva gains
     */
    public void applyDriveConfigs (Slot0Configs configs) {
        for (SwerveModule module : Modules) {
            module.getDriveMotor().getConfigurator().apply(configs);
        }
    }

    /**
     * apply different pid configs to all the angle motors.
     * @param configs the new pidva gains
     */
    public void applySteerConfigs (Slot0Configs configs) {
        for (SwerveModule module : Modules) {
            module.getSteerMotor().getConfigurator().apply(configs);
        }
    }

    /**
     * apply a current limit to the angle motors.
     * @param amps the current limit in amps
     */
    public void setSteerCurrentLimit (double amps) {
        var limit = new CurrentLimitsConfigs();
        limit.StatorCurrentLimit = amps;
        limit.StatorCurrentLimitEnable = true;
        for (SwerveModule module : Modules) {
            module.getSteerMotor().getConfigurator().apply(limit);
        }
    }

    public Pose3d getPose3d(){
        var odometry = getState().Pose;
        var imu = new Rotation3d(new Quaternion(
            m_pigeon2.getQuatW().getValueAsDouble(), 
            m_pigeon2.getQuatX().getValueAsDouble(),
            m_pigeon2.getQuatY().getValueAsDouble(),
            m_pigeon2.getQuatZ().getValueAsDouble()
        ));//.minus(pigeonOdometryOffset);
        return new Pose3d(new Translation3d(odometry.getX(), odometry.getY(), visionZ), imu);
    }

    @Override
    public void tareEverything(){
        //TODO zero pigeon for 3d pose
        super.tareEverything();
    }

    public void updateVision(Pose3d pose, double latency, Matrix<N3, N1> stdevs){
        //TODO update pigeon offset
        visionZ = pose.getTranslation().getZ();

        var imu = new Rotation3d(new Quaternion(
            m_pigeon2.getQuatW().getValueAsDouble(), 
            m_pigeon2.getQuatX().getValueAsDouble(),
            m_pigeon2.getQuatY().getValueAsDouble(),
            m_pigeon2.getQuatZ().getValueAsDouble()
        )).minus(pigeonOdometryOffset);

        //update pigeon offset with vision, by moving it towards the vision pose by a small multiple (for now)
        //TODO better way of doing this
        var visionOffset = pose.getRotation().minus(imu).times(0.02);
        pigeonOdometryOffset = pigeonOdometryOffset.plus(visionOffset);

        //write the vision pose to the odometry
        addVisionMeasurement(pose.toPose2d(), Timer.getFPGATimestamp() - latency, stdevs);
    }

    public double getModuleSteerCurrent(int i) {
        return Modules[i].getSteerMotor().getStatorCurrent().getValueAsDouble();
    }

    public double getModuleDriveCurrent(int i) {
        return Modules[i].getDriveMotor().getStatorCurrent().getValueAsDouble();
    }

    public double getModuleAngle(int i) {
        return Modules[i].getCANcoder().getAbsolutePosition().getValueAsDouble();
    }

    public double getModuleAngleError(int i) {
        return Modules[i].getSteerMotor().getClosedLoopError().getValueAsDouble();
    }

    public double getModuleRotationalRate(int i) {
        return Modules[i].getCANcoder().getVelocity().getValueAsDouble();
    }

    public double getModuleDriveError(int i) {
        return Modules[i].getDriveMotor().getClosedLoopError().getValueAsDouble();
    }

    public double getTotalDriveCurrent(){
        double total = 0;
        for (SwerveModule module : Modules) {
            total += module.getDriveMotor().getStatorCurrent().getValueAsDouble();
        }
        return total;
    }

    /**
     * ONLY USE IN TEST MODE. WILL DO NOTHING IF NOT IN TEST MODE.
     * @param speed the speed to spin the angle motors at.
     */
    public void spinAngleMotors(double speed) {
        if(!RobotState.isTest()){
            System.err.println("WARNING (PeaccefulSwerve.spinAngleMotors): This method can only be used in test mode.");
            return;
        }
        setControl(new SwerveRequest.Idle());
        for (SwerveModule module : Modules) {
            module.getSteerMotor().set(speed);
        }
    }

}
