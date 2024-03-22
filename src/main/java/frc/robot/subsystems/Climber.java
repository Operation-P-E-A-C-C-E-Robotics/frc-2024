package frc.robot.subsystems;

import static frc.robot.Constants.Climber.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.lib.util.Reporter;
import frc.lib.util.Util;

public class Climber {
    private TalonFX leftMotor = new TalonFX(climberLeftMotorId);
    private TalonFX rightMotor = new TalonFX(climberRightMotorId);
    //declare two TalonFXs, one for the left climber, one for the right climber

    private PositionVoltage leftControl = new PositionVoltage(0);
    private PositionVoltage rightControl = new PositionVoltage(0);

    private StatusSignal <Double> leftPosition = leftMotor.getPosition(), 
                                    rightPosition = rightMotor.getPosition(),
                                    leftDutyCycle = leftMotor.getDutyCycle();

    private StrictFollower followControl = new StrictFollower(leftMotor.getDeviceID());

    private MotionMagicExpoTorqueCurrentFOC climbPositionControl = new MotionMagicExpoTorqueCurrentFOC(0);
    private DutyCycleOut climbDutyCycleControl = new DutyCycleOut(0).withEnableFOC(true);

    private double setpoint = 0.0;

    private Climber () {
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

        BaseStatusSignal.setUpdateFrequencyForAll(100, leftPosition, rightPosition, leftDutyCycle);

        rightMotor.setControl(followControl);
    }

    /**
     * sets the position of both sides of the climber, in meters
     * @param position
     */
    public void setClimberPosition (double position) {
        leftMotor.setControl (climbPositionControl.withPosition(position));
    }

    public void setClimberPercent (double percent) {
        leftMotor.setControl (climbDutyCycleControl.withOutput(percent));
    }

    public double getClimberPosition(){
        leftPosition.refresh();
        return leftPosition.getValue();
    }

    /**
     * get whether the climber is at its setpoint
     */
    public boolean atSetpoint () {
         return Util.inRange(getClimberPosition() - setpoint, climberTolerance);
    }

    private static Climber instance = new Climber();
    public static Climber getInstance(){
        return instance;
    }
}
