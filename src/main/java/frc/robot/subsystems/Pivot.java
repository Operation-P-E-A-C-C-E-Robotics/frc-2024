package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.util.Reporter;
import frc.lib.util.Util;
import frc.robot.Robot;

import static frc.robot.Constants.Pivot.*;

public class Pivot {
    /* HARDWARE */
    private final TalonFX pivotMaster = new TalonFX(pivotMasterID);
    private final TalonFX pivotFollower = new TalonFX(pivotFollowerID);
    private final CANcoder pivotEncoder = new CANcoder(pivotConfigs.Feedback.FeedbackRemoteSensorID);

    /* CONTROLLERS / CONTROL REQUESTS */
    private final MotionMagicExpoVoltage pivotControl = new MotionMagicExpoVoltage(restingAngle).withEnableFOC(true);
    private final DutyCycleOut pivotSpringy = new DutyCycleOut(0.001);
    private final ArmFeedforward gravityFeedforward = new ArmFeedforward(0, gravityFeedforwardkG, 0);

    /* TELEMETRY */
    private final Mechanism2d pivotMech = new Mechanism2d(100, 100);
    private final MechanismLigament2d pivotLigament = pivotMech.getRoot("pivot", 50, 50).append(new MechanismLigament2d("pivot", 30, 0));
    private final NetworkTable pivotTable = NetworkTableInstance.getDefault().getTable("Pivot");
    private final DoublePublisher pivotAnglePublisher = pivotTable.getDoubleTopic("pivot angle").publish();
    private final DoublePublisher targetPivotAnglePublisher = pivotTable.getDoubleTopic("target pivot angle").publish();

    /* STATUS SIGNALS */
    private final StatusSignal <Double> positionSignal;
    private final StatusSignal <Double> velocitySignal;
    private final StatusSignal <Double> errorSignal;

    private Pivot () {
        Reporter.report(
            pivotMaster.getConfigurator().apply(pivotConfigs),
            "Couldn't configure pivot master"
        );

        Reporter.report(
            pivotFollower.getConfigurator().apply(pivotConfigs),
            "Couldn't configure pivot follower"
        );

        pivotMaster.setInverted(true);
        pivotFollower.setInverted(false);

        
        Reporter.report(
            pivotEncoder.getConfigurator().apply(cancoderConfiguration),
            "Couldn't configure pivot encoder"
        );
            
        //disable all unused status signals to minimize CAN usage
        ParentDevice.optimizeBusUtilizationForAll(pivotMaster, pivotFollower, pivotEncoder);
        positionSignal = pivotEncoder.getPosition();
        velocitySignal = pivotEncoder.getVelocity();
        errorSignal = pivotMaster.getClosedLoopError();
        BaseStatusSignal.setUpdateFrequencyForAll(
            100, 
            positionSignal, 
            velocitySignal, 
            errorSignal, 
            pivotMaster.getDutyCycle(),
            pivotMaster.getPosition(),
            pivotMaster.getVelocity()
        );
            
        Reporter.report(
            pivotFollower.setControl(new StrictFollower(pivotMaster.getDeviceID())),
            "Couldn't configure pivot slave"
        );
        SmartDashboard.putData("pivot mech", pivotMech);
    }

    /**
     * set the position of the pivot, with 0 being horizontal
     * @param position the position duh
     */
    public void setPivotPosition (Rotation2d position) {
        // return;
        var gravity = gravityFeedforward.calculate(getPivotPosition().getRadians(), 0);

        pivotControl.withFeedForward(gravity).withPosition(position.getRotations() + 0.005); //fudge factor because of goddamnclimber

        Reporter.log(pivotMaster.setControl(pivotControl), "couldn't set pivot position");

        targetPivotAnglePublisher.accept(position.getDegrees());

        if(Robot.isSimulation()) {
            pivotEncoder.getSimState().setRawPosition(position.getRotations());
            pivotLigament.setAngle(getPivotPosition());
        }
    }

    public void setPivotPercent (double percent) {
        pivotMaster.setControl(new DutyCycleOut(percent));
    }

    public void climbMode(){
        pivotMaster.setControl(pivotSpringy);
    }
    /**
     * Get the latency-compensated pivot position
     * @return the pivot position with 0 being fully horizontal
     */
    public Rotation2d getPivotPosition () {
        positionSignal.refresh();
        Reporter.report(positionSignal.getStatus(), "Couldn't read pivot position");
        var compensatedRotations = positionSignal.getValue();

        var angle = Rotation2d.fromRotations(compensatedRotations);
        pivotAnglePublisher.accept(angle.getDegrees());
        return angle;
    }

    public boolean atSetpoint () {
        errorSignal.refresh();
        return Util.inRange(errorSignal.getValue(), pivotTolerance);
    }

    private static final Pivot instance = new Pivot();
    public static Pivot getInstance(){
        return instance;
    }
}
