package frc.robot.subsystems;

import static frc.robot.Constants.Climber.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.PositionVoltage;
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

    private StatusSignal <Double> leftPosition, rightPosition;

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

        leftPosition = leftMotor.getPosition();
        rightPosition = rightMotor.getPosition();

        leftMotor.optimizeBusUtilization();
        rightMotor.optimizeBusUtilization();

        BaseStatusSignal.setUpdateFrequencyForAll(100, leftPosition, rightPosition);

    }

    /**
     * sets the position of both sides of the climber, in meters
     * @param position
     */
    public void setClimberPosition (double position) {
        setClimberPosition(position, position);
    }

    /**
     * sets the position of the left and right sides of the climber, in meters
     * @param left the position of the left side of the climber
     * @param right the position of the right side of the climber
     */
    public void setClimberPosition (double left, double right) {
        setpoint = (left + right) / 2;

        Reporter.log(leftMotor.setControl(leftControl.withPosition(left)), "setting left climber position");                                 //ask sean  if this is ok :| sean: it's not ok, it's aok
        Reporter.log(rightMotor.setControl(rightControl.withPosition(right)), "setting right climber position");
    }

    public void setClimberPercent (double percent) {
        setClimberPercent(percent, percent);
    }

    public void setClimberPercent(double left, double right){
        leftMotor.set(left);
        rightMotor.set(right);
    }

    //i changed this from a double to void so i could grab both values and store them in different variables.
    public double getClimberPosition(){
        return (getLeftPosition() + getRightPosition()) / 2;
    }

    public double getLeftPosition(){
        leftPosition.refresh();
        return leftPosition.getValue();
    }

    public double getRightPosition(){
        rightPosition.refresh();
        return rightPosition.getValue();
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
