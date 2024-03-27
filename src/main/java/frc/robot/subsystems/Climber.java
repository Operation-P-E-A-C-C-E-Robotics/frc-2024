package frc.robot.subsystems;

import static frc.robot.Constants.Climber.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.lib.util.Reporter;

public class Climber {
    /* HARDWARE */
    private TalonFX leftMotor = new TalonFX(climberLeftMotorId);
    private TalonFX rightMotor = new TalonFX(climberRightMotorId);

    /* STATUS SIGNALS */
    private StatusSignal <Double> leftPosition = leftMotor.getPosition(), 
                                    leftDutyCycle = leftMotor.getDutyCycle();

    /* CONTROLLERS */
    private StrictFollower followControl = new StrictFollower(leftMotor.getDeviceID());

    private MotionMagicExpoTorqueCurrentFOC climbPositionControl = new MotionMagicExpoTorqueCurrentFOC(0);
    private DutyCycleOut climbDutyCycleControl = new DutyCycleOut(0).withEnableFOC(true);

    /* TELEMETRY */
    private final NetworkTable networkTable = NetworkTableInstance.getDefault().getTable("Climber");
    private final DoublePublisher setpointPublisher = networkTable.getDoubleTopic("Setpoint").publish();
    private final DoublePublisher dutyCyclePublisher = networkTable.getDoubleTopic("Duty Cycle").publish();
    private final DoublePublisher positionPublisher = networkTable.getDoubleTopic("Position").publish();

    private Climber () {
        //configure motors
        var leftInversionConfig = new MotorOutputConfigs();
        leftInversionConfig.Inverted = climberLeftMotorIsInverted;
        leftInversionConfig.NeutralMode = NeutralModeValue.Brake;

        var rightInversionConfig = new MotorOutputConfigs();
        rightInversionConfig.Inverted = climberRightMotorIsInverted;
        rightInversionConfig.NeutralMode = NeutralModeValue.Brake;

        Reporter.report(
            leftMotor.getConfigurator().apply(climberConfigs.withMotorOutput(leftInversionConfig)),
            "climber left motor config failed"
        );
        Reporter.report(
            rightMotor.getConfigurator().apply(climberConfigs.withMotorOutput(rightInversionConfig)), 
            "climber right motor config failed"
        );

        leftMotor.optimizeBusUtilization();
        rightMotor.optimizeBusUtilization();

        BaseStatusSignal.setUpdateFrequencyForAll(100, leftPosition, leftDutyCycle);

        rightMotor.setControl(followControl);
    }

    /**
     * sets the position of both sides of the climber, in meters
     * @param position
     */
    public void setClimberPosition (double position) {
        setpointPublisher.accept(position);
        leftMotor.setControl (climbPositionControl.withPosition(position));
    }

    public void setClimberPercent (double percent) {
        dutyCyclePublisher.accept(percent);
        leftMotor.setControl (climbDutyCycleControl.withOutput(percent));
    }

    public double getClimberPosition(){
        leftPosition.refresh();
        var position = leftPosition.getValue();
        positionPublisher.accept(position);
        return position;
    }

    /**
     * get whether the climber is at its setpoint
     */
    public boolean atSetpoint () {
         return true; //TODO
    }

    private static Climber instance = new Climber();
    public static Climber getInstance(){
        return instance;
    }
}
