package frc.robot.auto;

import java.util.function.BooleanSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.TeleopStatemachine;
import frc.robot.TeleopStatemachine.TeleopState;
import frc.robot.planners.NoteTracker;
import frc.robot.planners.NoteTracker.NoteLocation;
import frc.robot.statemachines.SwerveStatemachine;
import frc.robot.statemachines.SwerveStatemachine.SwerveState;
import frc.robot.subsystems.Shooter;

public class Autonomous {

    public static final AutoMode testAuto = new AutoMode(
        new AutoSegment(
            null,
            new TimedRobotState(TeleopState.AUTO_AIM, 0, 0.75),
            new TimedRobotState(TeleopState.SHOOT, 0.75, 1, () -> Shooter.getInstance().shotDetected()),
            new TimedRobotState(TeleopState.INTAKE_BACK, 1, 3)
        ),
        new AutoSegment(
            Path.DRIVE_OFF_LINE,
            new TimedRobotState(TeleopState.INTAKE_BACK, 0, 5, () -> NoteTracker.getLocation() == NoteLocation.INDEXING),
            new TimedRobotState(TeleopState.REST, 5, 5.5),
            new TimedRobotState(TeleopState.AUTO_AIM, 5.5, 10)
        ),
        new AutoSegment(
            null
        )
    );

    public static class TimedRobotState {
        public final TeleopState teleopState;
        public final double startPercent;
        public final double endPercent;
        public final BooleanSupplier endCondition;

        public TimedRobotState(TeleopState teleopState, double startPercent, double endPercent, BooleanSupplier endCondition) {
            this.teleopState = teleopState;
            this.startPercent = startPercent;
            this.endPercent = endPercent;
            this.endCondition = endCondition;
        }

        public TimedRobotState(TeleopState teleopState, double startPercent, double endPercent) {
            this(teleopState, startPercent, endPercent, () -> false);
        }
        
        public boolean wantsRun(double time) {
            SmartDashboard.putNumber("autonomous state time", time);
            return time >= startPercent && time <= endPercent && !endCondition.getAsBoolean();
        }
    }

    public static class AutoSegment {
        private final Path path;
        private final TimedRobotState[] timedStates;

        public AutoSegment(Path path, TimedRobotState... timedStates) {
            this.path = path;
            this.timedStates = timedStates;
        }

        public TeleopState getTeleopState(double time) {
            for (TimedRobotState state : timedStates) {
                if (state.wantsRun(time)) {
                    return state.teleopState;
                }
            }
            return TeleopState.REST;
        }

        public Path getPath() {
            return path;
        }

        public boolean isDone(double time) {
            for (TimedRobotState state : timedStates) {
                if (state.endPercent > time && !state.endCondition.getAsBoolean()) {
                    return false;
                }
            }
            return true;
        }
    }

    public static class AutoMode {
        private final AutoSegment[] segments;
        private int currentSegment = 0;
        private boolean newSegment = true;
        private final Timer segmentTimer = new Timer();


        public AutoMode(AutoSegment... segments) {
            this.segments = segments;
        }

        public void run(SwerveStatemachine swerve, TeleopStatemachine robot) {
            SmartDashboard.putNumber("auto segment", currentSegment);
            if(RobotState.isDisabled()) {
                currentSegment = 0;
                swerve.setPathCommand(null);
                swerve.resetPathTimer();
                newSegment = true;
                segmentTimer.reset();
                return;
            }

            if(currentSegment > segments.length - 1) currentSegment = segments.length - 1;
            AutoSegment current = segments[currentSegment];

            if(newSegment) {
                if(current.getPath() != null) {
                    robot.requestSwerveState(SwerveState.FOLLOW_PATH);
                    swerve.setPathCommand(current.getPath().command);
                }
                segmentTimer.reset();
                segmentTimer.start();
                newSegment = false;
            }

            double time = segmentTimer.get();
            // System.out.println(time);
            robot.requestState(current.getTeleopState(time));

            if(current.isDone(time)) {
                currentSegment++;
                newSegment = true;
            }

            if(current.getPath() == null) {
                if(current.getTeleopState(time) == TeleopState.AUTO_AIM) robot.requestSwerveState(SwerveState.AIM);
                robot.requestSwerveState(SwerveState.LOCK_IN);
                return;
            }

            if(time > current.getPath().duration) {
                if(current.getTeleopState(time) == TeleopState.AUTO_AIM) robot.requestSwerveState(SwerveState.AIM);
                else robot.requestSwerveState(SwerveState.LOCK_IN);
            }
        }
    }

    public static enum Path {
        TEST_PATH("test path", 3),
        DRIVE_OFF_LINE("drive off line", 1.5);
    
        public final String pathName;
        public final double duration;
        public final Command command;
    
        private Path(String pathName, double duration) {
            this.pathName = pathName;
            this.duration = duration;
            this.command = AutoBuilder.followPath(PathPlannerPath.fromChoreoTrajectory(pathName));
        }
    }
}
