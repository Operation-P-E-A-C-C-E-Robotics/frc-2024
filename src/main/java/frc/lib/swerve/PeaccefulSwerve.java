package frc.lib.swerve;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * OH MY LORD DONT MAKE ME EXTEND YOUR STUPID CLASS TO ADD BASIC FUNCTIONALITY.
 * ADD ONE FUNCTION SO I CAN GET THE GODDAMN KNEMATICS DONT MAKE THEM """"PROTECTED"""" SO I HAVE TO DO ALL THIS NONSENSE JUST TO GET THE
 * CHASSIS SPEEDS. 
 */
public class PeaccefulSwerve extends SwerveDrivetrain {
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

    public ChassisSpeeds getChassisSpeeds() {
        return m_kinematics.toChassisSpeeds(getState().ModuleStates);
    }

    public double[] getAngleClosedLoopErrors() {
        double[] errors = new double[4];
        int i = 0;
        for (SwerveModule module : Modules) {
            errors[i] = module.getSteerMotor().getClosedLoopError().getValueAsDouble();
        }
        return errors;
    }

    public double[] getDriveClosedLoopErrors() {
        double[] errors = new double[4];
        int i = 0;
        for (SwerveModule module : Modules) {
            errors[i] = module.getDriveMotor().getClosedLoopError().getValueAsDouble();
        }
        return errors;
    }

    public void applyDriveConfigs (Slot0Configs configs) {
        for (SwerveModule module : Modules) {
            module.getDriveMotor().getConfigurator().apply(configs);
        }
    }

    public void applySteerConfigs (Slot0Configs configs) {
        for (SwerveModule module : Modules) {
            module.getSteerMotor().getConfigurator().apply(configs);
        }
    }

    public void setSteerCurrentLimit (double amps) {
        var limit = new CurrentLimitsConfigs();
        limit.StatorCurrentLimit = amps;
        limit.StatorCurrentLimitEnable = true;
        for (SwerveModule module : Modules) {
            module.getSteerMotor().getConfigurator().apply(limit);
        }
    }

}