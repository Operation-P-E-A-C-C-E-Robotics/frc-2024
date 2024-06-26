package frc.robot.statemachines;

import java.sql.Driver;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.state.StateMachine;
import frc.robot.OI;
import frc.robot.planners.AimPlanner;
import frc.robot.planners.MotionPlanner;
import frc.robot.subsystems.Pivot;

public class PivotStatemachine extends StateMachine<PivotStatemachine.PivotState> {
    private PivotState state = PivotState.REST;

    private final Pivot pivot;
    private final AimPlanner aimPlanner;
    private final MotionPlanner motionPlanner;

    public PivotStatemachine(Pivot pivot, AimPlanner aimPlanner, MotionPlanner motionPlanner){
        this.pivot = pivot;
        this.aimPlanner = aimPlanner;
        this.motionPlanner = motionPlanner;

        SmartDashboard.putNumber("pivot sim debug", 0);
    }

    /**
     * Request a state for the mechanism to attain
     * Won't allow for a state that would result in a collision or other dangerous situation
     * e.g. flattening to intake before the intake is extended
     * @param state
     */
    @Override
    public void requestState(PivotState state){
        this.state = state;
    }

    /**
     * Make the mechanism attain the desired state
     */
    @Override
    public void update(){
        // if (state == PivotState.INTAKE && !intakeMotionPlanner.canFlattenPivot()) state = PivotState.REST;
        SmartDashboard.putString("Pivot State", state.name());

        if(state == PivotState.AUTO_AIM) {
            var angle = aimPlanner.getTargetPivotAngle();
            if(DriverStation.isAutonomousEnabled()) angle = angle.plus(Rotation2d.fromDegrees(2));
            pivot.setPivotPosition(angle);
            return;
        }

        if(state == PivotState.AMP && OI.Inputs.wantsPlace.getAsBoolean()) state = PivotState.AMP_PUSH;

        var angle = state.getAngle();
        if(angle.getDegrees() > 80 && !motionPlanner.canFlipPivot()) angle = Rotation2d.fromDegrees(80);

        pivot.setPivotPosition(angle);
    }

    @Override
    public PivotState getState(){
        return state;
    }
    
    @Override
    public boolean transitioning() {
       return !pivot.atSetpoint();
    }

    @Override
    public boolean isDynamic() {
        return state == PivotState.INTAKE || state == PivotState.AUTO_AIM;
    }

    public enum PivotState {
        REST(Rotation2d.fromDegrees(33)),
        INTAKE(Rotation2d.fromDegrees(17)),
        STOW(Rotation2d.fromDegrees(20)),
        AMP(Rotation2d.fromDegrees(82)),
        AMP_PUSH(Rotation2d.fromDegrees(50)),
        INTAKE_SOURCE(Rotation2d.fromDegrees(56)),
        PRE_CLIMB(Rotation2d.fromDegrees(70)),
        CLIMB(Rotation2d.fromDegrees(70)),
        AIM_LAYUP(Rotation2d.fromDegrees(53)),
        AIM_PROTECTED(Rotation2d.fromDegrees(37)),
        AIM_UNDER_STAGE(Rotation2d.fromDegrees(30)),
        AIM_WINGLINE(Rotation2d.fromDegrees(27)),
        AIM_CENTERLINE(Rotation2d.fromDegrees(25)),
        AIM_SHUTTLE(Rotation2d.fromDegrees(45)),
        AUTO_AIM(Rotation2d.fromDegrees(30));

        private Rotation2d angle;

        public Rotation2d getAngle(){
            return angle;
        }

        private PivotState (Rotation2d angle){
            this.angle = angle;
        }
    }
}
