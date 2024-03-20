package frc.lib.util;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

/**
 * very simple extension of SendableChooser that allows you to pass in an enum and have it automatically populate the options
 */
public class EnumSendableChooser <T extends Enum<T>> extends SendableChooser <T> {
    /**
     * Construct a sendable chooser with the values from the enum
     * @param values the enum values to populate the chooser with (get this by calling T.values())
     */
    public EnumSendableChooser(T[] values) {
        super();
        setDefaultOption(values[0].name(), values[0]);
        for (T value : values) {
            addOption(value.name(), value);
        }
    }
}
