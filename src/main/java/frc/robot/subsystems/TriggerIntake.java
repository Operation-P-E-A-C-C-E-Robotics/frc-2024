package frc.robot.subsystems;

import static frc.robot.Constants.TriggerIntake.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import frc.lib.util.Reporter;
import frc.lib.util.Util;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.SoftLimitDirection;

public class TriggerIntake {
    /* HARDWARE */
    private CANSparkMax deployMotor = new CANSparkMax(triggerIntakeDeployMotorId, MotorType.kBrushless);
    private WPI_TalonSRX rollerMotor = new WPI_TalonSRX(triggerIntakeRollerMotorId);

    private RelativeEncoder deployEncoder = deployMotor.getEncoder();

    /* CONTROLLERS */
    private SparkPIDController deployController = deployMotor.getPIDController();

    /* STATE */
    private double targetRotation = 0.0;

    /* TELEMETRY */
    private final NetworkTable table = NetworkTableInstance.getDefault().getTable("Intake");
    private final DoublePublisher deployAnglePublisher = table.getDoubleTopic("Rear Deploy Angle").publish();

    DataLog log = DataLogManager.getLog();
    DoubleLogEntry targetDeployAngleLog = new DoubleLogEntry(log, "Intake/Target Rear Deploy Angle");
    BooleanLogEntry deployedToSetpointLog = new BooleanLogEntry(log, "Intake/Rear Deployed to Setpoint");
    DoubleLogEntry rollerSpeedLog = new DoubleLogEntry(log, "Intake/Rear Roller Speed");

    private TriggerIntake () {
        // configure motors n whatnot
        deployMotor.restoreFactoryDefaults();

        rollerMotor.setInverted(triggerIntakeRollerMotorInverted);
        deployMotor.setInverted(triggerIntakeDeployMotorInverted);
        
        Reporter.report(
            deployMotor.setSoftLimit(SoftLimitDirection.kForward, triggerIntakeDeployMaxAngle), 
            "Couldn't set rear intake forward soft limit");
        Reporter.report(
            deployMotor.setSoftLimit(SoftLimitDirection.kReverse, triggerIntakeDeployMinAngle), 
            "Couldn't set rear intake reverse soft limit"
        );
        Reporter.report(
            deployMotor.enableSoftLimit(SoftLimitDirection.kForward, false), 
            "Couldn't enable rear intake forward soft limit"
        );
        Reporter.report(
            deployMotor.enableSoftLimit(SoftLimitDirection.kReverse, false), 
            "Couldn't enable rear intake reverse soft limit"
        );

        Reporter.report(
            deployController.setP(triggerIntakeDeployKp), 
            "Couldn't configure rear intake kP"
        );
        Reporter.report(
            deployController.setI(triggerIntakeDeployKi), 
            "Couldn't configure rear intake kI"
        );
        Reporter.report(
            deployController.setD(triggerIntakeDeployKd), 
            "Couldn't configure rear intake kD"
        );

        Reporter.report(
            deployEncoder.setPositionConversionFactor(triggerIntakeDeployGearing),
            "Couldn't configure rear intake encoder gearing"
        );


        Reporter.report(
            deployMotor.setSmartCurrentLimit(triggerIntakeDeployStallCurrentLimit, triggerIntakeDeployFreeCurrentLimit),
            "Couldn't configure rear intake current limits"
        );

        Reporter.report(
            deployMotor.burnFlash(),
            "Couldn't burn flash on rear intake deploy motor"
        );

        rollerMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, triggerIntakeCurrentLimit, triggerIntakeCurrentLimit, 0.01));
    }

    /**
     * Set the deployment angle of the trigger intake.
     * an ange of 0 is the stowed position, and higher angles are more deployed.
     * @param angle the angle to set the trigger intake to
     */
    public void setDeploymentAngle (Rotation2d angle) {
        targetRotation = angle.getRotations(); 
        targetDeployAngleLog.append(angle.getRotations());
        Reporter.report(
            deployController.setReference(angle.getRotations(), CANSparkMax.ControlType.kPosition),
            "couldn't set rear intake deploy angle"
        );
    }

    public void setDeploymentSpeed(double percent) {
        deployController.setReference(percent, ControlType.kDutyCycle);
    }

    /**
     * Set the speed of the roller on the trigger intake.
     * @param speed the speed of the roller (-1 to 1, positive intakes)
     */
    public void setRollerSpeed (double speed) {
        rollerSpeedLog.append(speed);
        rollerMotor.set(speed);
    }

    /**
     * Get the current deployment angle of the trigger intake.
     * @return the current deployment angle
     */
    public Rotation2d getDeploymentAngle () {
        var rotations = deployEncoder.getPosition();
        deployAnglePublisher.accept(rotations);
        return Rotation2d.fromRotations(rotations);
    }

    /**
     * @return whether the trigger intake has attained is goal deployment angle.
     */
    public boolean deployedToSetpoint () {
        var deployed = Util.inRange(getDeploymentAngle().getRotations() - targetRotation, triggerIntakeDeployTolerance);
        deployedToSetpointLog.append(deployed);
        return deployed;
    }

    private static final TriggerIntake instance = new TriggerIntake();
    public static TriggerIntake getInstance(){
        return instance;
    }
}
