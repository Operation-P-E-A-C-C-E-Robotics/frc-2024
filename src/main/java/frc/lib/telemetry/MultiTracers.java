package frc.lib.telemetry;

import java.util.HashMap;
import edu.wpi.first.wpilibj.Tracer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * A class to manage multiple WPILib Tracers,
 * which are used to time different parts of the code, and
 * locate performance bottlenecks.
 * 
 * It lets you reference different tracers globally by a string,
 * and print the epochs of a specific tracer to the console.
 * 
 * It also can be fully enabled or disabled, so you can
 * leave the tracing code in the codebase without it
 * affecting performance when it's not needed.
 */
public class MultiTracers {
    private static HashMap<String, Tracer> tracers = new HashMap<String, Tracer>();

    private static boolean enabled = false;

    public static void enable() {
        enabled = true;
    }

    public static void disable() {
        enabled = false;
    }

    public static void trace(String tracer, String epochName) {
        if (!enabled) return;
        if (!tracers.containsKey(tracer)) {
            tracers.put(tracer, new Tracer());
        }
        tracers.get(tracer).addEpoch(epochName);
    }

    public static void print(String tracer) {
        if (!enabled) return;
        if (tracers.containsKey(tracer)) {
            tracers.get(tracer).printEpochs();
        }
    }

    public static void initDashboard() {
        if(!enabled) return;
        for (String tracer : tracers.keySet()) {
            SmartDashboard.putBoolean("Print " + tracer + " epochs", false);
        }
    }

    public static void updateDashboard() {
        if(!enabled) return;
        for (String tracer : tracers.keySet()) {
            if (SmartDashboard.getBoolean("Print " + tracer + " epochs", false)) {
                print(tracer);
            }
        }
    }
}
