package frc.lib.telemetry;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.TeleopInputs;
import frc.robot.TeleopInputs.TeleopMode;
import frc.robot.planners.NoteTracker;
import frc.robot.planners.NoteTracker.NoteLocation;

import java.util.ArrayList;

/**
 * Log timing data for strategic analysis of the robot's performance
 */
public class StrategyTelemetry {
    private static Timer cycleTimer = new Timer();
    private static ArrayList<Cycle> cycles = new ArrayList<>();
    private static NoteLocation lastNoteLocation = NoteLocation.NONE;

    private static boolean hasMatchStarted = false;

    public static void update() {
        if(!hasMatchStarted) {
            if(DriverStation.isEnabled()) {
                hasMatchStarted = true;
                cycleTimer.start();
                cycleTimer.reset();
            } else {
                return;
            }
        } else {
            if(DriverStation.isDisabled()) {
                cycleTimer.stop();
                hasMatchStarted = false;
                var averageCycleTime = cycles.stream().mapToDouble(c -> c.time).average().orElse(0);
                var averageAmpCycleTime = cycles.stream().filter(c -> c.type == CycleType.AMP).mapToDouble(c -> c.time).average().orElse(0);
                var averageSpeakerCycleTime = cycles.stream().filter(c -> c.type == CycleType.SPEAKER).mapToDouble(c -> c.time).average().orElse(0);

                System.out.println("Match Complete!");
                System.out.println("Average Cycle Time: " + averageCycleTime);
                System.out.println("Average Amp Cycle Time: " + averageAmpCycleTime);
                System.out.println("Average Speaker Cycle Time: " + averageSpeakerCycleTime);
                System.out.println("Total Cycles: " + cycles.size());
                System.out.println("Total Amp Cycles: " + cycles.stream().filter(c -> c.type == CycleType.AMP).count());
                System.out.println("Total Speaker Cycles: " + cycles.stream().filter(c -> c.type == CycleType.SPEAKER).count());
                System.out.println("Cycles: ");
                cycles.forEach((c) -> System.out.println("   > " + c));
            }
        }
        if(NoteTracker.getLocation() == NoteLocation.NONE && lastNoteLocation != NoteLocation.NONE) {
            if(TeleopInputs.getInstance().getMode() == TeleopMode.CLIMB) return;
            var cycle = new Cycle(cycleTimer.get(), TeleopInputs.getInstance().getMode() == TeleopMode.AMP ? CycleType.AMP : CycleType.SPEAKER);
            cycles.add(cycle);
            System.out.println(cycle);
        }
    }

    private static class Cycle {
        public double time;
        public CycleType type;

        public Cycle(double time, CycleType type) {
            this.time = time;
            this.type = type;
        }

        public String toString() {
            return "Ran a " + type.name() + " cycle in " + time + " seconds";
        }
    }

    private static enum CycleType {
        AMP, SPEAKER
    }
}
