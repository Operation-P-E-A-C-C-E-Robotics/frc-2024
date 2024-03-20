package frc.robot.subsystems;

import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.lib.util.Reporter;

import static frc.robot.Constants.Diverter.*;

public class Diverter {
    private final TalonFX deployMotor = new TalonFX(diverterDeployMotorId);
    private final TalonFX rollerMotor = new TalonFX(diverterRollerMotorId);

    private final MotionMagicExpoVoltage deployControl = new MotionMagicExpoVoltage(0);

    private final PositionVoltage rollerPositionControl = new PositionVoltage(0);

    private Diverter () {
        Reporter.report(deployMotor.getConfigurator().apply(diverterDeployConfigs), "Couldn't configure diverter deploy motor");
        Reporter.report(rollerMotor.getConfigurator().apply(diverterRollerConfigs), "Couldn't configure diverter roller motor");
    }

    public void setDiverterExtension (double position) {
        Reporter.log(deployMotor.setControl(deployControl.withPosition(position)), "set diverter extension");
    }

    public void setDiverterExtensionPercent (double percent) {
        deployMotor.set(percent);
    }

    public void setDiverterRoller (double speed) {
        rollerMotor.set(speed);
    }

    public void lockRollerPosition () {
        rollerPositionControl.withPosition(rollerMotor.getPosition().getValue());
        Reporter.log(rollerMotor.setControl(rollerPositionControl), "lock diverter roller position");
    }

    public double getDiverterExtension () {
        return deployMotor.getPosition().getValue();
    }
    
    public boolean atSetpoint () {
        return deployMotor.getClosedLoopError().getValue() < diverterDeployTolerance;
    }

    private static final Diverter instance = new Diverter();
    public static Diverter getInstance(){
        return instance;
    }
}
