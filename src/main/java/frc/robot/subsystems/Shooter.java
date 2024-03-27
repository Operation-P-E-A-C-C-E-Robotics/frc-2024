package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.util.Reporter;
import frc.lib.util.Util;
import frc.robot.Constants;

import static frc.robot.Constants.Shooter.*;

public class Shooter {
    /* HARDWARE */
    private final TalonFX topFlywheelMotor = new TalonFX(upperFlywheelMotorId);
    private final TalonFX bottomFlywheelMotor = new TalonFX(lowerFlywheelMotorId);
    
    private final CANSparkMax triggerMotor = new CANSparkMax(triggerMotorId, MotorType.kBrushless);

    private final DigitalInput flywheelSwitch = new DigitalInput(flywheelSwitchId);
    private final DigitalInput triggerSwitch = new DigitalInput(triggerSwitchId);

    /* CONTROLLERS */
    private final LinearSystem<N1, N1, N1> topFlywheelSystem = LinearSystemId.identifyVelocitySystem(flywheelKv, flywheelKa);

    private final KalmanFilter<N1, N1, N1> topFlywheelObserver = new KalmanFilter<>(
        Nat.N1(), Nat.N1(),
        topFlywheelSystem,
        VecBuilder.fill(flywheelModelStDev),
        VecBuilder.fill(flywheelEncoderStDev),
        Constants.period
    );

    private final LinearQuadraticRegulator<N1, N1, N1> topFlywheelController = new LinearQuadraticRegulator<>(
        topFlywheelSystem,
        VecBuilder.fill(flywheelErrorTolerance),
        VecBuilder.fill(flywheelControlEffort),
        Constants.period
    );

    private final LinearSystemLoop<N1, N1, N1> topFlywheelLoop = new LinearSystemLoop<>(topFlywheelSystem, topFlywheelController, topFlywheelObserver, 12.0, Constants.period);

    private final LinearSystem<N1, N1, N1> bottomFlywheelSystem = LinearSystemId.identifyVelocitySystem(flywheelKv, flywheelKa);

    private final KalmanFilter<N1, N1, N1> bottomFlywheelObserver = new KalmanFilter<>(
        Nat.N1(), Nat.N1(),
        bottomFlywheelSystem,
        VecBuilder.fill(flywheelModelStDev),
        VecBuilder.fill(flywheelEncoderStDev),
        Constants.period
    );

    private final LinearQuadraticRegulator<N1, N1, N1> bottomFlywheelController = new LinearQuadraticRegulator<>(
        bottomFlywheelSystem,
        VecBuilder.fill(flywheelErrorTolerance),
        VecBuilder.fill(flywheelControlEffort),
        Constants.period
    );

    private final LinearSystemLoop<N1, N1, N1> bottomFlywheelLoop = new LinearSystemLoop<>(bottomFlywheelSystem, bottomFlywheelController, bottomFlywheelObserver, 12.0, Constants.period);
   
    /* CONTROL REQUESTS */
    private final VoltageOut topControl = new VoltageOut(0).withEnableFOC(true);
    private final VoltageOut bottomControl = new VoltageOut(0).withEnableFOC(true);

    /* STATUS SIGNALS */
    private final StatusSignal <Double> topFlywheelVelocity;
    private final StatusSignal <Double> bottomFlywheelVelocity;
    private final StatusSignal <Double> topFlywheelAcceleration;
    private final StatusSignal <Double> bottomFlywheelAcceleration;

    private final Timer shotTimer = new Timer();
    private final Timer timeSinceTriggerRun = new Timer();

    /* TELEMETRY */
    private final NetworkTable shooterTable = NetworkTableInstance.getDefault().getTable("Shooter");
    private final DoublePublisher topVelocityPub = shooterTable.getDoubleTopic("Top Flywheel Velocity").publish();
    private final DoublePublisher bottomVelocityPub = shooterTable.getDoubleTopic("Bottom Flywheel Velocity").publish();
    private final DoublePublisher targetTopVelocityPub = shooterTable.getDoubleTopic("Top Flywheel Target Velocity").publish();
    private final DoublePublisher targetBottomVelocityPub = shooterTable.getDoubleTopic("Bottom Flywheel Target Velocity").publish();
    private final DoublePublisher triggerPub = shooterTable.getDoubleTopic("Trigger Percent").publish();
    private final BooleanPublisher shotDetectedPub = shooterTable.getBooleanTopic("Shot Detected").publish();


    private double topSetpoint = 0;
    private double bottomSetpoint = 0;

    private Shooter () {
        Reporter.report(
            topFlywheelMotor.getConfigurator().apply(flywheelConfigs), 
            "Couldn't configure top flywheel motor"
        );
        Reporter.report(
            bottomFlywheelMotor.getConfigurator().apply(flywheelConfigs), 
            "Couldn't configure bottom flywheel motor"
        );

        topFlywheelMotor.setInverted(topFlywheelMotorInverted);
        bottomFlywheelMotor.setInverted(bottomFlywheelMotorInverted);
        triggerMotor.setInverted(triggerMotorInverted);

        //disable all CAN signals we don't use to minimize bus usage
        topFlywheelMotor.optimizeBusUtilization();
        bottomFlywheelMotor.optimizeBusUtilization();

        topFlywheelVelocity = topFlywheelMotor.getVelocity();
        bottomFlywheelVelocity = bottomFlywheelMotor.getVelocity();
        topFlywheelAcceleration = topFlywheelMotor.getAcceleration();
        bottomFlywheelAcceleration = bottomFlywheelMotor.getAcceleration();

        triggerMotor.setSmartCurrentLimit(20);

        BaseStatusSignal.setUpdateFrequencyForAll(100, 
            topFlywheelVelocity,
            bottomFlywheelVelocity,
            topFlywheelAcceleration,
            bottomFlywheelAcceleration
        );

        topControl.withUpdateFreqHz(100);
        bottomControl.withUpdateFreqHz(100);
    }

    /**
     * Set the velocity of both flyweels to the same value
     * @param velocity the velocity to set the flywheels to in RPS
     */
    public void setFlywheelVelocity (double velocity) {
        setFlywheelVelocity(velocity, velocity);
    }

    /**
     * Set the velocity of the flywheels independently
     * @param top the velocity to set the top flywheel to in RPS
     * @param bottom the velocity to set the bottom flywheel to in RPS
     */
    public void setFlywheelVelocity (double top, double bottom) {
        var topVelocityCompensated = getTopFlywheelVelocity();
        var bottomVelocityCompensated = getBottomFlywheelVelocity();

        topSetpoint = top;
        bottomSetpoint = bottom;

        topVelocityPub.accept(topVelocityCompensated);
        bottomVelocityPub.accept(bottomVelocityCompensated);

        targetTopVelocityPub.accept(top);
        targetBottomVelocityPub.accept(bottom);

        topFlywheelLoop.setNextR(VecBuilder.fill(top));
        topFlywheelLoop.correct(VecBuilder.fill(topVelocityCompensated));
        topFlywheelLoop.predict(Constants.period);

        bottomFlywheelLoop.setNextR(VecBuilder.fill(bottom));
        bottomFlywheelLoop.correct(VecBuilder.fill(bottomVelocityCompensated));
        bottomFlywheelLoop.predict(Constants.period);

        topControl.withOutput(topFlywheelLoop.getU(0));
        bottomControl.withOutput(bottomFlywheelLoop.getU(0));

        topFlywheelMotor.setControl(topControl.withOverrideBrakeDurNeutral(true));
        bottomFlywheelMotor.setControl(bottomControl.withOverrideBrakeDurNeutral(true));
    }

    /**
     * Set the flywheels to a percent output
     * @param percent the percent output to set the flywheels to
     */
    public void setFlywheelPercent (double percent) {
        topFlywheelMotor.set(percent);
        bottomFlywheelMotor.set(percent);
        topFlywheelLoop.reset(VecBuilder.fill(topFlywheelVelocity.getValue()));
        bottomFlywheelLoop.reset(VecBuilder.fill(bottomFlywheelVelocity.getValue()));
    }

    /**
     * Set the flywheels to coast (for handoff)
     */
    public void coastFlywheel () {
        topFlywheelMotor.setControl(topControl.withOutput(0).withOverrideBrakeDurNeutral(false));
        bottomFlywheelMotor.setControl(topControl.withOutput(0).withOverrideBrakeDurNeutral(false));
    }

    /**
     * Set the flywheels to brake for ramping down without wasting power
     */
    public void brakeFlywheel () {
        topFlywheelMotor.setControl(topControl.withOutput(0).withOverrideBrakeDurNeutral(true));
        bottomFlywheelMotor.setControl(topControl.withOutput(0).withOverrideBrakeDurNeutral(true));
    }

    /**
     * Set the trigger to a percent output
     * @param percent the percent output to set the trigger to
     */
    public void setTriggerPercent (double percent) {
        triggerPub.accept(percent);
        triggerMotor.set(percent);
        if(percent > 0.1){
            timeSinceTriggerRun.start();
            timeSinceTriggerRun.reset();
        }
    }

    /**
     * @return latency-compensated flywheel velocity in rotations/second
     */
    public double getTopFlywheelVelocity () {
        topFlywheelVelocity.refresh();
        topFlywheelAcceleration.refresh();
        return BaseStatusSignal.getLatencyCompensatedValue(topFlywheelVelocity, topFlywheelAcceleration);
    }

    /**
     * @return latency-compensated flywheel velocity in rotations/second
     */
    public double getBottomFlywheelVelocity () {
        bottomFlywheelVelocity.refresh();
        topFlywheelAcceleration.refresh();
        return BaseStatusSignal.getLatencyCompensatedValue(bottomFlywheelVelocity, bottomFlywheelAcceleration);
    }

    /**
     * Get the average velocity of the flywheels
     * @return the average velocity of the flywheels in RPS
     */
    public double getFlywheelVelocity () {
        return (getTopFlywheelVelocity() + getBottomFlywheelVelocity()) / 2;
    }

    /**
     * Get the average acceleration of the flywheels
     * @return the average acceleration of the flywheels in rotations/s^2
     */
    public double getFlywheelAcceleration () {
        topFlywheelAcceleration.refresh();
        bottomFlywheelAcceleration.refresh();
        return (topFlywheelAcceleration.getValue() + bottomFlywheelAcceleration.getValue()) / 2;
    }

    /**
     * Check if the flywheels are at their target velocity
     * @return true if the flywheels are at their target velocity
     */
    public boolean flywheelAtTargetVelocity () {
        
        return Util.inRange(Math.abs(topSetpoint - getTopFlywheelVelocity()), flywheelTolerance) && Util.inRange(Math.abs(bottomSetpoint - getBottomFlywheelVelocity()), flywheelTolerance);
    }

    /**
     * Check if the front limit switch is tripped by a note
     * @return true if the limit switch is tripped
     */
    public boolean flywheelSwitchTripped () {
        return !flywheelSwitch.get();
    }

    /**
     * Check if the rear limit switch is tripped by a note
     * @return true if the limit switch is tripped
     */
    public boolean triggerSwitchTripped () {
        return !triggerSwitch.get();
    }

    /**
     * Check if a shot has been detected based on the acceleration of the flywheels
     * @return true if a shot has been detected in the past 0.1 seconds
     */
    public boolean shotDetected() {
        if (getFlywheelVelocity() > shotDetectionMinVelocity && getFlywheelAcceleration() < shotDetectionAccelerationThreshold && timeSinceTriggerRun.get() < 0.5) {
            shotTimer.start();
        } 
        if (shotTimer.hasElapsed(shotDetectionResetTime)) {
            shotTimer.stop();
            shotTimer.reset();
        }
        if (shotTimer.hasElapsed(shotDetectionTimeThreshold)) {
            shotDetectedPub.accept(true);
            return true;
        }
        shotDetectedPub.accept(false);
        return false;
    }

    private static final Shooter instance = new Shooter();
    public static Shooter getInstance(){
        return instance;
    }
}
