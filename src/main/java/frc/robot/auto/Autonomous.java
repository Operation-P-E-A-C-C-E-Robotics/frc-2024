package frc.robot.auto;

import frc.robot.RobotStatemachine;
import frc.robot.RobotStatemachine.SuperstructureState;
import frc.robot.statemachines.SwerveStatemachine.SwerveState;
import frc.robot.subsystems.Shooter;

import java.util.function.BooleanSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class Autonomous {

    public static final TimedAuto twoNoteCenter = new TimedAuto(
        shoot(),
        intakeAndFollowPath(Path.START2_WING2),
        shoot(),
        end()
    );

    public static final TimedAuto twoNoteAmpSide = new TimedAuto(
        shoot(),
        intakeAndFollowPath(Path.START3_WING3),
        shoot(),
        end()
    );

    public static final TimedAuto twoNoteStageSide = new TimedAuto(
        shoot(),
        intakeAndFollowPath(Path.START1_WING1),
        shoot(),
        end()
    );

    public static final TimedAuto fourNote = new TimedAuto(
        shoot(),
        intakeAndFollowPath(Path.START3_WING3),
        shoot(),
        intakeAndFollowPath(Path.WING3_WING2),
        shoot(),
        intakeAndFollowPath(Path.WING2_WING1),
        shoot(),
        end()
    );

    public static final TimedAuto start3ThreeNoteCenter5 = new TimedAuto(
        shoot(),
        intakeAndFollowPath(Path.START3_WING3),
        shoot(),
        intakeAndFollowPath(Path.WING3_CENTER5),
        wait(0.5),
        followPath(Path.CENTER5_SHOOT),
        shoot(),
        end()
    );

    public static final TimedAuto start1ThreeNoteCenter2 = new TimedAuto(
        shoot(),
        intakeAndFollowPath(Path.START1_WING1),
        shoot(),
        intakeAndFollowPath(Path.WING1_CENTER2),
        followPath(Path.CENTER2_SHOOT),
        shoot(),
        end()
    );

    public static final TimedAuto start1ThreeNoteCenter3 = new TimedAuto(
        shoot(),
        intakeAndFollowPath(Path.START1_WING1),
        shoot(),
        intakeAndFollowPath(Path.WING1_CENTER3),
        wait(0.5),
        followPath(Path.CENTER3_SHOOT),
        shoot(),
        end()
    );

    public static final TimedAuto start2ThreeNoteCenter4 = new TimedAuto(
        shoot(),
        intakeAndFollowPath(Path.START2_WING2),
        shoot(),
        intakeAndFollowPath(Path.WING2_CENTER4),
        followPath(Path.CENTER4_SHOOT_FROM_WING2),
        shoot(),
        end()
    );

    public static final TimedAuto start2FourNote = new TimedAuto(
        shoot(),
        intakeAndFollowPath(Path.START2_WING2),
        shoot(),
        intakeAndFollowPath(Path.WING2_CENTER4),
        followPath(Path.CENTER4_SHOOT_FROM_WING2),
        shoot(),
        intakeAndFollowPath(Path.CENTER4_SHOOT_CENTER3),
        followPath(Path.CENTER3_SHOOT),
        shoot(),
        end()
    );

    public static final TimedAuto start3ThreeNoteCenter4 = new TimedAuto(
        shoot(),
        intakeAndFollowPath(Path.START3_WING3),
        shoot(),
        intakeAndFollowPath(Path.WING3_CENTER4),
        wait(0.5),
        followPath(Path.CENTER4_SHOOT_FROM_WING3),
        shoot(),
        end()
    );

    public static final TimedAuto defence1 = new TimedAuto(
        shoot(),
        followPath(Path.DEFENCE_1),
        end()
    );
    public static final TimedAuto defence2 = new TimedAuto(
        shoot(),
        followPath(Path.DEFENCE_2),
        end()
    );
    public static final TimedAuto defence3 = new TimedAuto(
        shoot(),
        followPath(Path.DEFENCE_3),
        end()
    );
    public static final TimedAuto defence4R = new TimedAuto(
        shoot(),
        followPath(Path.DEFENCE_4_R),
        end()
    );
    public static final TimedAuto defence4L = new TimedAuto(
        shoot(),
        followPath(Path.DEFENCE_4_L),
        end()
    );
    public static final TimedAuto defence5R = new TimedAuto(
        shoot(),
        followPath(Path.DEFENCE_5_R),
        end()
    );

    public static final TimedAuto layupOnly = new TimedAuto(
        layupShot(),
        end()
    );

    public static final TimedAuto shootOnly = new TimedAuto(
        shoot(),
        end()
    );

    public static final TimedAuto doNothing = new TimedAuto(
        end()
    );

    private static Action[] layupShot(){
        return new Action[]{
            new Action(SuperstructureState.AIM_LAYUP, 1),
            new Action(SuperstructureState.SHOOT, 0.2, () -> Shooter.getInstance().shotDetected())
        };
    }

    private static Action[] intakeAndFollowPath(Path path){
        return new Action[]{
            // new Action(SuperstructureState.INTAKE_BACK, 0.5),
            new Action(SuperstructureState.INTAKE_BACK, path.command, path.duration)
        };
    }

    // private static Action[] followPathAndShoot(Path path) {
    //     return new Action[] {
    //         new Action(SuperstructureState.AUTO_AIM, path.command, path.duration + 1, () -> Shooter.getInstance().flywheelSwitchTripped()),
    //         new Action(SuperstructureState.SHOOT, 0.25)
    //     };
    // }

    private static Action[] followPath(Path path) {
        return new Action[] {
            new Action(path.command, path.duration),
        };
    }

    // private static Action[] intakeNShoot(Path path) {
    //     return new Action[] {
    //         new Action(SuperstructureState.INTAKE_N_AIM, path.command, path.duration),
    //         new Action(SuperstructureState.INTAKE_N_AIM, 1),
    //         new Action(SuperstructureState.INTAKE_N_PIVOT_AIM, 1),
    //         new Action(SuperstructureState.INTAKE_N_SHOOT, 0.15)
    //     };
    // }

    private static Action[] shoot() {
        return new Action[]{
            // new Action(SuperstructureState.REST, 0.5), //allow the shooter to index
            new Action(SuperstructureState.AUTO_AIM, 1),
            new Action(SuperstructureState.SHOOT, 0.15)
        };
    }

    private static Action[] end(){
        return new Action[]{new Action(15)};
    }

    private static Action[] wait(double timeout) {
        return new Action[]{new Action(timeout)};
    }

    public static class TimedAuto {
        private final Action[] actions;
        private final Timer timer = new Timer();
        private int currentAction = 0;

        public TimedAuto(Action... actions) {
            this.actions = actions;
        }

        public TimedAuto(Action[]... actions){
            //concatenate the arrays
            int totalLength = 0;
            for (Action[] action : actions) {
                totalLength += action.length;
            }
            this.actions = new Action[totalLength];
            int index = 0;
            for (Action[] action : actions) {
                for (Action action2 : action) {
                    this.actions[index] = action2;
                    index++;
                }
            }
        }

        public void reset() {
            timer.reset();
            currentAction = 0;
        }

        public void run(RobotStatemachine superstructure) {
            timer.start();
            // for (int i = currentAction; i < actions.length; i++) {
            //     if (actions[i].wantsRun(timer.get())) {
            //         currentAction = i;
            //         superstructure.requestSwervePath(actions[i].getPath());
            //         superstructure.requestSwerveState(actions[i].getSwerveState());
            //         superstructure.requestState(actions[i].getState());
            //         return;
            //     }
            // }
            if(actions[currentAction].isDone(timer.get()) && !(currentAction + 1 >= actions.length)) {
                currentAction++;
                timer.reset();
            }

            var action = actions[currentAction];
            superstructure.requestSwervePath(action.getPath());
            superstructure.requestSwerveState(action.getSwerveState());
            superstructure.requestState(action.getState());
        }
    }

    public static class Action {
        private final SuperstructureState state;
        private final SwerveState swerveState;
        private final Command path;
        private final double timeout;
        private final BooleanSupplier endCondition;

        public Action(SuperstructureState state, SwerveState swerveState, Command path, double timeout, BooleanSupplier endCondition) {
            this.state = state;
            this.swerveState = swerveState;
            this.path = path;
            this.timeout = timeout;
            this.endCondition = endCondition;
        }

        public Action(SuperstructureState state, Command path, double timeout) {
            this(state, SwerveState.FOLLOW_PATH, path, timeout, () -> false);
        }

        public Action(RobotStatemachine.SuperstructureState state, Command path, double timeout, BooleanSupplier endCondition) {
            this(state, SwerveState.FOLLOW_PATH, path, timeout, endCondition);
        }

        public Action(SuperstructureState state, SwerveState swerveState, Command path, double timeout) {
            this(state, swerveState, path, timeout, () -> false);
        }

        public Action(SwerveState swerveState, double timeout, BooleanSupplier endCondition) {
            this(SuperstructureState.REST, swerveState, new InstantCommand(), timeout, endCondition);
        }

        public Action(SwerveState swerveState, double timeout) {
            this(swerveState, timeout, () -> false);
        }

        public Action(Command path, double timeout, BooleanSupplier endCondition) {
            this(SuperstructureState.REST, SwerveState.FOLLOW_PATH, path, timeout, endCondition);
        }

        public Action(Command path, double timeout) {
            this(path, timeout, () -> false);
        }

        public Action(SuperstructureState state, double timeout, BooleanSupplier endCondition) {
            this(
                state, 
                state == SuperstructureState.AUTO_AIM || state == SuperstructureState.SHOOT || state == SuperstructureState.INTAKE_N_AIM || state == SuperstructureState.INTAKE_N_SHOOT ? SwerveState.AIM : SwerveState.LOCK_IN, 
                new InstantCommand(), timeout, endCondition
            );
        }

        public Action(SuperstructureState state, double timeout) {
            this(state, timeout, () -> false);
        }

        public Action(double timeout, BooleanSupplier endCondition) {
            this(SuperstructureState.REST, timeout, endCondition);
        }

        public Action(double timeout) {
            this(timeout, () -> false);
        }

        public boolean wantsRun(double time) {
            return time <= timeout && !endCondition.getAsBoolean();
        }

        public SuperstructureState getState() {
            return state;
        }

        public SwerveState getSwerveState() {
            return swerveState;
        }

        public Command getPath() {
            return path;
        }

        public boolean isDone(double time) {
            return time >= timeout || endCondition.getAsBoolean();
        }

        public double gettimeout() {
            return timeout;
        }

        public BooleanSupplier getEndCondition() {
            return endCondition;
        }
    }

    public static enum Path {
        TEST_PATH("test path", 3),
        START2_WING2("start 2 wing 2", 1.9),
        START3_WING3("start 3 wing 3", 2.2),
        START1_WING1("start 1 wing 1", 1.7),
        WING3_WING2("wing 3 wing 2", 2.4),
        WING2_WING1("wing 2 wing 1", 2.1),
        WING3_CENTER5("wing 3 center 5", 3.5),
        CENTER5_SHOOT("center 5 shoot", 3.3),
        WING3_CENTER4("wing 3 center 4", 3.5),
        CENTER4_SHOOT_FROM_WING3("center 4 shoot from wing 3", 3.1),
        WING1_CENTER2("wing 1 center 2", 4.5),
        CENTER2_SHOOT("center 2 shoot", 3.7),
        WING1_CENTER3("wing 1 center 3", 5.1),
        CENTER3_SHOOT("center 3 shoot", 3.3),
        WING2_CENTER4("wing 2 center 4", 4.2),
        CENTER4_SHOOT_FROM_WING2("center 4 shoot from wing 2", 3.4),
        CENTER4_SHOOT_CENTER3("center 4 shoot center 3", 3.3),
        DEFENCE_1("defence 1", 4.1),
        DEFENCE_2("defence 2", 4.3),
        DEFENCE_3("defence 3", 4.4),
        DEFENCE_4_R("defence 4 R", 4.4),
        DEFENCE_4_L("defence 4 L", 4.5),
        DEFENCE_5_R("defence 5 R", 4.2);
    
        public final String pathName;
        public final Command command;
        public final double duration;
    
        private Path(String pathName, double duration) {
            this.pathName = pathName;
            this.duration = duration;
            this.command = AutoBuilder.followPath(PathPlannerPath.fromChoreoTrajectory(pathName));
        }
    }
}
