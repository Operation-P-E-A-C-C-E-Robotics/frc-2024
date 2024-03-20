package frc.robot.subsystems;

import static frc.robot.Constants.FlywheelIntake.*;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
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

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.SoftLimitDirection;

//software for the intake that is on the front of the robot
public class FlywheelIntake {
    /* HARDWARE */
    private CANSparkMax deployMotor = new CANSparkMax(flywheelIntakeDeployMotorId, MotorType.kBrushless);
    private WPI_TalonSRX rollerMotor = new WPI_TalonSRX(flywheelIntakeRollerMotorId);

    private RelativeEncoder deployEncoder = deployMotor.getEncoder();

    /* CONTROLLERS */
    private SparkPIDController deployPIDController = deployMotor.getPIDController();

    /* STATE */
    private double targetRotations = 0;

    /* TELEMETRY */
    private final NetworkTable table = NetworkTableInstance.getDefault().getTable("Intake");
    private final DoublePublisher deployAnglePublisher = table.getDoubleTopic("Front Deploy Angle").publish();

    DataLog log = DataLogManager.getLog();
    DoubleLogEntry targetDeployAngleLog = new DoubleLogEntry(log, "Intake/Target Front Deploy Angle (rotations)");
    BooleanLogEntry deployedToSetpointLog = new BooleanLogEntry(log, "Intake/Front Deployed to Setpoint (rotations)");
    DoubleLogEntry rollerSpeedLog = new DoubleLogEntry(log, "Intake/Front Roller Speed %");

    private FlywheelIntake () {
        rollerMotor.configFactoryDefault();

        rollerMotor.setInverted(flywheelIntakeRollerMotorInverted);
        deployMotor.setInverted(flywheelIntakeDeployMotorInverted);

        //«««Warning»»» : dont let stretch interact with this code by any means. It has so much hatred 
        //for this code that it will break its own neck without warning
        Reporter.report(
            deployMotor.setSoftLimit(SoftLimitDirection.kForward, flywheelIntakeDeployMaxAngle), 
            "Couldn't set front intake forward soft limit"
        );
        Reporter.report(
            deployMotor.setSoftLimit(SoftLimitDirection.kReverse, flywheelIntakeDeployMinAngle), 
            "Couldn't set front intake reverse soft limit"
        );
        Reporter.report(
            deployMotor.enableSoftLimit(SoftLimitDirection.kForward, false), 
            "Couldn't enable front intake forward soft limit"
        );
        Reporter.report(
            deployMotor.enableSoftLimit(SoftLimitDirection.kReverse, false), 
            "Couldn't enable front intake reverse soft limit"
        );

        Reporter.report(
            deployPIDController.setP(flywheelIntakeDeployKp), 
            "Couldn't configure front intake kP"
        );   //Hehe, thats funny, it spells PID.... WAIT
        Reporter.report(
            deployPIDController.setI(flywheelIntakeDeployKi), 
            "Couldn't configure front intake kI"
        );
        Reporter.report(
            deployPIDController.setD(flywheelIntakeDeployKd), 
            "Couldn't configure front intake kD"
        );

        Reporter.report(
            deployEncoder.setPositionConversionFactor(flywheelIntakeDeployGearing), 
            "Couldn't configure front intake encoder"
        );

        Reporter.report(
            deployMotor.setSmartCurrentLimit(flywheelIntakeDeployStallCurrentLimit, flywheelIntakeDeployFreeCurrentLimit), 
            "Couldn't configure front intake current limits"
        );

        Reporter.report(
            deployMotor.burnFlash(),
            "Couldn't burn flash on front intake deploy motor"
        );
    }

    
    /**
     * set the target angle for the intake to deploy to, in rotations
     */
    public void setDeploymentAngle (Rotation2d angle) {
        targetRotations = angle.getRotations(); 
        targetDeployAngleLog.append(angle.getRotations());
        Reporter.log(
            deployPIDController.setReference(angle.getRotations(), CANSparkMax.ControlType.kPosition),
            "couldn't set front intake deploy angle"
        );
    }

    /**
     * set the speed of the roller, from -1 to 1
     */
    public void setRollerSpeed (double speed) {
        rollerSpeedLog.append(speed);
        rollerMotor.set(speed);
    }

    /**
     * get the current angle of the intake, in rotations
     */
    public Rotation2d getDeploymentAngle(){
        var rotations = deployEncoder.getPosition();
        deployAnglePublisher.accept(rotations);
        return Rotation2d.fromRotations(rotations);
    }
    /**
     * get whether the deploy motor is at its setpoint
     */

    public boolean deployedToSetpoint () {
        var deployed = Util.inRange(getDeploymentAngle().getRotations() - targetRotations, flywheelIntakeDeployTolerance);
        deployedToSetpointLog.append(deployed);
        return deployed;
    }

    private static FlywheelIntake instance = new FlywheelIntake();
    public static FlywheelIntake getInstance(){
        return instance;
    }
}