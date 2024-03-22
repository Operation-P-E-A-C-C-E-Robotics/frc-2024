package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.lib.util.Reporter;

import static frc.robot.Constants.Thing.*;

public class Thing {
    private final TalonFX deployMotor = new TalonFX(thingDeployMotorId);

    private final MotionMagicExpoTorqueCurrentFOC deployControl = new MotionMagicExpoTorqueCurrentFOC(0);

    private final StatusSignal<Double> getDeployExtension = deployMotor.getPosition();

    private Thing () {
        Reporter.report(deployMotor.getConfigurator().apply(diverterDeployConfigs), "Couldn't configure diverter deploy motor");

        deployMotor.optimizeBusUtilization();
        BaseStatusSignal.setUpdateFrequencyForAll(100, getDeployExtension);
    }

    public void setThingExtension (double position) {
        Reporter.log(deployMotor.setControl(deployControl.withPosition(position)), "set diverter extension");
    }

    public void setThingExtensionPercent (double percent) {
        deployMotor.set(percent);
    }

    public double getThingExtension () {
        return deployMotor.getPosition().getValue();
    }
    
    public boolean atSetpoint () {
        return deployMotor.getClosedLoopError().getValue() < diverterDeployTolerance;
    }

    private static final Thing instance = new Thing();
    public static Thing getInstance(){
        return instance;
    }
}
