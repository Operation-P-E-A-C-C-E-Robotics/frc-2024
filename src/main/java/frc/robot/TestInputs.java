package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.util.EnumSendableChooser;
import frc.robot.RobotStatemachine.SuperstructureState;
import frc.robot.statemachines.ClimberStatemachine;
import frc.robot.statemachines.ThingStatemachine;
import frc.robot.statemachines.FlywheelIntakeStatemachine;
import frc.robot.statemachines.FlywheelIntakeStatemachine.FlywheelIntakeState;
import frc.robot.statemachines.PivotStatemachine;
import frc.robot.statemachines.ShooterStatemachine;
import frc.robot.statemachines.SwerveStatemachine;
import frc.robot.statemachines.SwerveStatemachine.SwerveState;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;
import frc.robot.statemachines.TriggerIntakeStatemachine;

/**
 * This class is used to control the robot during testing. It allows the user to
 * override the state of each subsystem from Shuffleboard.
 */
public class TestInputs {
    private TestInputs() {}

    EnumSendableChooser<RobotStatemachine.SuperstructureState> robotStateChooser = new EnumSendableChooser<>(SuperstructureState.values());
    EnumSendableChooser<SwerveState> swerveStateChooser = new EnumSendableChooser<>(SwerveState.values());
    EnumSendableChooser<FlywheelIntakeState> flywheelIntakeStateChooser = new EnumSendableChooser<>(FlywheelIntakeState.values());
    EnumSendableChooser<TriggerIntakeStatemachine.TriggerIntakeState> triggerIntakeStateChooser = new EnumSendableChooser<>(TriggerIntakeStatemachine.TriggerIntakeState.values());
    EnumSendableChooser<PivotStatemachine.PivotState> pivotStateChooser = new EnumSendableChooser<>(PivotStatemachine.PivotState.values());
    EnumSendableChooser<ShooterStatemachine.ShooterState> shooterStateChooser = new EnumSendableChooser<>(ShooterStatemachine.ShooterState.values());
    EnumSendableChooser<ThingStatemachine.FlipperState> diverterStateChooser = new EnumSendableChooser<>(ThingStatemachine.FlipperState.values());
    EnumSendableChooser<ClimberStatemachine.ClimberState> climberStateChooser = new EnumSendableChooser<>(ClimberStatemachine.ClimberState.values());
    
    public void putChoosers() {
        SmartDashboard.putBoolean("Override Robot State", false);
        SmartDashboard.putBoolean("Override Swerve State", false);
        SmartDashboard.putBoolean("Override Flywheel Intake State", false);
        SmartDashboard.putBoolean("Override Trigger Intake State", false);
        SmartDashboard.putBoolean("Override Pivot State", false);
        SmartDashboard.putBoolean("Override Shooter State", false);
        SmartDashboard.putBoolean("Override Diverter State", false);
        SmartDashboard.putBoolean("Override Climber State", false);

        SmartDashboard.putData("Robot State Chooser", robotStateChooser);
        SmartDashboard.putData("Swerve State Chooser", swerveStateChooser);
        SmartDashboard.putData("Flywheel Intake State Chooser", flywheelIntakeStateChooser);
        SmartDashboard.putData("Trigger Intake State Chooser", triggerIntakeStateChooser);
        SmartDashboard.putData("Pivot State Chooser", pivotStateChooser);
        SmartDashboard.putData("Shooter State Chooser", shooterStateChooser);
        SmartDashboard.putData("Diverter State Chooser", diverterStateChooser);
        SmartDashboard.putData("Climber State Chooser", climberStateChooser);

        SmartDashboard.putBoolean("Override Shooter Velocity", false);
        SmartDashboard.putNumber("Shooter Velocity", 0);

        SmartDashboard.putBoolean("Override Pivot Angle", false);
        SmartDashboard.putNumber("Pivot Angle Deg", 0);
    }

    public void update (RobotStatemachine teleopStatemachine, 
                        SwerveStatemachine swerveStatemachine,
                        FlywheelIntakeStatemachine flywheelIntakeStatemachine,
                        TriggerIntakeStatemachine triggerIntakeStatemachine,
                        ShooterStatemachine shooterStatemachine, 
                        PivotStatemachine pivotStatemachine,
                        ThingStatemachine diverterStatemachine,
                        ClimberStatemachine climberStatemachine) {

        if(SmartDashboard.getBoolean("Override Robot State", false)) {
            teleopStatemachine.requestState(robotStateChooser.getSelected());
        } 

        if(SmartDashboard.getBoolean("Override Swerve State", false)) {
            teleopStatemachine.requestSwerveState(swerveStateChooser.getSelected());
        }

        teleopStatemachine.update();

        if(SmartDashboard.getBoolean("Override Flywheel Intake State", false)) {
            flywheelIntakeStatemachine.updateWithState(flywheelIntakeStateChooser.getSelected());
        }

        if(SmartDashboard.getBoolean("Override Trigger Intake State", false)) {
            triggerIntakeStatemachine.updateWithState(triggerIntakeStateChooser.getSelected());
        }

        if(SmartDashboard.getBoolean("Override Pivot State", false)) {
            pivotStatemachine.updateWithState(pivotStateChooser.getSelected());
        }

        if(SmartDashboard.getBoolean("Override Shooter State", false)) {
            shooterStatemachine.updateWithState(shooterStateChooser.getSelected());
        }

        if(SmartDashboard.getBoolean("Override Diverter State", false)) {
            diverterStatemachine.updateWithState(diverterStateChooser.getSelected());
        }

        if(SmartDashboard.getBoolean("Override Climber State", false)) {
            climberStatemachine.updateWithState(climberStateChooser.getSelected());
        }

        if(SmartDashboard.getBoolean("Override Shooter Velocity", false)) {
            Shooter.getInstance().setFlywheelVelocity(SmartDashboard.getNumber("Shooter Velocity", 0));
        }

        if(SmartDashboard.getBoolean("Override Pivot Angle", false)) {
            Pivot.getInstance().setPivotPosition(Rotation2d.fromDegrees(SmartDashboard.getNumber("Pivot Angle Deg", 0)));
        }
    }

    private static TestInputs instance = new TestInputs();
    public static TestInputs getInstance() {
        return instance;
    }
}
