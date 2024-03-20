package frc.lib.util;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * A class to map joysticks to commands with a less redundant syntax.
 * Describe the buttons first, then bind them all to a joystick.
 * No need to type JoystickButton 200 times.
 * #authentic2023experiencebutactuallygood
 */
public class ButtonMap {
    private final Joystick joystick;

    /**
     * Maps buttons to a joystick through an OIEntry
     * @param joystick The joystick to map to
     */
    public ButtonMap(Joystick joystick) {
        this.joystick = joystick;
    }

    /**
     * Maps buttons to a joystick through an OIEntry
     * You can split the mapping up into as many OIEntries as you want, just make sure they don't conflict.
     * @param entries The entries to map
     */
    public void map(OIEntry... entries){
        for(OIEntry entry : entries){
            var button = entry.getTrigger(joystick);
            entry.bindTo(button);
        }
    }

    /**
     * Interface for a button mapping that can later be bound to a joystick.
     */
    public static interface OIEntry{
        /**
         * Handles the binding of the entry to a trigger
         * @param trigger
         */
        public void bindTo(Trigger trigger);

        /**
         * Allow the entry to specify its own trigger type.
         * @param joystick
         * @return
         */
        public Trigger getTrigger(Joystick joystick);
    }

    public static class Button implements OIEntry{
        public final Command command;
        public final int buttonNumber;
        public final TriggerType triggerType;

        /**
         * Binds a button to a command
         * @param command The command to bind to
         * @param buttonNumber The button number to bind to
         * @param triggerType What trigger type to use (on press, on release, hold, toggle)
         */
        public Button(Command command, int buttonNumber, TriggerType triggerType){
            this.command = command;
            this.buttonNumber = buttonNumber;
            this.triggerType = triggerType;
        }

        public void bindTo(Trigger trigger){
            switch (triggerType) {
                case ON_PRESS -> trigger.onTrue(command);
                case ON_RELEASE -> trigger.onFalse(command);
                case WHILE_HOLD -> trigger.whileTrue(command);
                case TOGGLE -> trigger.toggleOnTrue(command);
            }
        }

        public Trigger getTrigger(Joystick joystick){
            return new JoystickButton(joystick, buttonNumber);
        }

        public static Button onHold(Command command, int buttonNumber){
            return new Button(command, buttonNumber, TriggerType.WHILE_HOLD);
        }

        public static Button onPress(Command command, int buttonNumber){
            return new Button(command, buttonNumber, TriggerType.ON_PRESS);
        }

        public static Button onRelease(Command command, int buttonNumber){
            return new Button(command, buttonNumber, TriggerType.ON_RELEASE);
        }

        public static Button toggle(Command command, int buttonNumber){
            return new Button(command, buttonNumber, TriggerType.TOGGLE);
        }


    }

    public static class MultiButton implements OIEntry {
        public final Command command;
        public final int[] buttonNumbers;
        public final TriggerType triggerType;

        /**
         * Binds multiple buttons to a command. ALL BUTTONS MUST BE PRESSED FOR THE COMMAND TO RUN
         * @param command The command to bind to
         * @param triggerType What trigger type to use (on press, on release, hold, toggle)
         * @param buttonNumbers The button numbers to bind to
         */
        public MultiButton(Command command, TriggerType triggerType, int... buttonNumbers){
            this.command = command;
            this.buttonNumbers = buttonNumbers;
            this.triggerType = triggerType;
        }

        public void bindTo(Trigger trigger){
            switch (triggerType) {
                case ON_PRESS -> trigger.onTrue(command);
                case ON_RELEASE -> trigger.onFalse(command);
                case WHILE_HOLD -> trigger.whileTrue(command);
                case TOGGLE -> trigger.toggleOnTrue(command);
            }
        }

        public Trigger getTrigger(Joystick joystick){
            Trigger[] triggers = new Trigger[buttonNumbers.length];
            for(int i = 0; i < buttonNumbers.length; i++){
                triggers[i] = new JoystickButton(joystick, buttonNumbers[i]);
            }
            return new Trigger(() -> {
                    for(Trigger trigger : triggers){
                        if(!trigger.getAsBoolean()){
                            return false;
                        }
                    }
                    return true;
            });
        }

        public static MultiButton onHold(Command command, int... buttonNumbers){
            return new MultiButton(command, TriggerType.WHILE_HOLD, buttonNumbers);
        }

        public static MultiButton onPress(Command command, int... buttonNumbers){
            return new MultiButton(command, TriggerType.ON_PRESS, buttonNumbers);
        }

        public static MultiButton onRelease(Command command, int... buttonNumbers){
            return new MultiButton(command, TriggerType.ON_RELEASE, buttonNumbers);
        }

        public static MultiButton toggle(Command command, int... buttonNumbers){
            return new MultiButton(command, TriggerType.TOGGLE, buttonNumbers);
        }
    }

    
    public static class InterferenceButton implements OIEntry{
        int runButton;
        int[] dontRunButtons;
        Command command;
        TriggerType triggerType;

        /**
        * A button that runs a command when pressed, but only if none of the other specified buttons are pressed.
        * @param command The command to run
        * @param triggerType What trigger type to use (on press, on release, hold, toggle)
        * @param runButton The button number to run the command on
        * @param dontRunButtons The button numbers that will prevent the command from running
        */
        public InterferenceButton(Command command, TriggerType triggerType, int runButton, int... dontRunButtons){
            this.command = command;
            this.triggerType = triggerType;
            this.runButton = runButton;
            this.dontRunButtons = dontRunButtons;
        }

        public void bindTo(Trigger trigger){
            switch (triggerType) {
                case ON_PRESS -> trigger.onTrue(command);
                case ON_RELEASE -> trigger.onFalse(command);
                case WHILE_HOLD -> trigger.whileTrue(command);
                case TOGGLE -> trigger.toggleOnTrue(command);
            }
        }

        public Trigger getTrigger(Joystick joystick){
            return new JoystickButton(joystick, runButton).and(new Trigger(() -> {
                for(int button : dontRunButtons){
                    if(new JoystickButton(joystick, button).getAsBoolean()){
                        return false;
                    }
                }
                return true;
            }));
        }

        public static InterferenceButton onHold(Command command, int runButton, int... dontRunButtons){
            return new InterferenceButton(command, TriggerType.WHILE_HOLD, runButton, dontRunButtons);
        }

        public static InterferenceButton onPress(Command command, int runButton, int... dontRunButtons){
            return new InterferenceButton(command, TriggerType.ON_PRESS, runButton, dontRunButtons);
        }

        public static InterferenceButton onRelease(Command command, int runButton, int... dontRunButtons){
            return new InterferenceButton(command, TriggerType.ON_RELEASE, runButton, dontRunButtons);
        }

        public static InterferenceButton toggle(Command command, int runButton, int... dontRunButtons){
            return new InterferenceButton(command, TriggerType.TOGGLE, runButton, dontRunButtons);
        }
    }

    public static class OIMode implements OIEntry{
        private final int selectButtonNumber;
        private final OIEntry[] entries;
        private final Joystick joystick;
        private boolean selected;

        /**
         * A way to switch between different sets of controls on a single joystick.
         * Toggles the layer on and off when the select button is pressed.
         * Use with caution, as it doesn't disable other bindings when selected, if they conflict they must also be in 
         * their own OIMode and be disabled by the same button.
         * UNTESTED.
         * @param selectButtonNumber The button number to toggle the layer on and off
         * @param defaultSelected Whether the layer should be on by default
         * @param joystick The joystick to bind to
         * @param entries The entries to bind
         */
        public OIMode(int selectButtonNumber, boolean defaultSelected, Joystick joystick, OIEntry... entries){
            this.selectButtonNumber = selectButtonNumber;
            this.entries = entries;
            this.joystick = joystick;
            selected = defaultSelected;
        }

        @Override
        public void bindTo(Trigger trigger) {
            trigger.onTrue(new InstantCommand(() -> selected = !selected));
            for(OIEntry entry : entries){
                entry.bindTo(entry.getTrigger(joystick).and(new Trigger(() -> selected)));
            }
        }

        @Override
        public Trigger getTrigger(Joystick joystick) {
            return new JoystickButton(joystick, selectButtonNumber);
        }
    }

    public static class SimplePOV implements OIEntry{
        public final Command command;
        public final int povNumber;
        public final TriggerType triggerType;

        /**
         * Triggers a command with a specific angle on the POV hat.
         * @param command The command to run
         * @param povNumber The pov direction to run the command on
         * @param triggerType What trigger type to use (on press, on release, hold, toggle)
         */
        public SimplePOV(Command command, int povNumber, TriggerType triggerType){
            this.command = command;
            this.povNumber = povNumber;
            this.triggerType = triggerType;
        }

        @Override
        public void bindTo(Trigger trigger) {
            switch (triggerType) {
                case ON_PRESS -> trigger.onTrue(command);
                case ON_RELEASE -> trigger.onFalse(command);
                case WHILE_HOLD -> trigger.whileTrue(command);
                case TOGGLE -> trigger.toggleOnTrue(command);
            }
        }

        @Override
        public Trigger getTrigger(Joystick joystick) {
            return new Trigger(() -> joystick.getPOV() == povNumber);
        }

        public static SimplePOV onHold(Command command, int povNumber){
            return new SimplePOV(command, povNumber, TriggerType.WHILE_HOLD);
        }

        public static SimplePOV onPress(Command command, int povNumber){
            return new SimplePOV(command, povNumber, TriggerType.ON_PRESS);
        }

        public static SimplePOV onRelease(Command command, int povNumber){
            return new SimplePOV(command, povNumber, TriggerType.ON_RELEASE);
        }

        public static SimplePOV toggle(Command command, int povNumber){
            return new SimplePOV(command, povNumber, TriggerType.TOGGLE);
        }
    }

    public static class AnyPOV implements OIEntry{
        Command command;
        TriggerType triggerType;

        /**
         * Triggers a command when the POV hat is pressed in any direction.
         * @param command The command to run
         * @param triggerType What trigger type to use (on press, on release, hold, toggle)
         */
        public AnyPOV(Command command, TriggerType triggerType){
            this.command = command;
            this.triggerType = triggerType;
        }

        @Override
        public void bindTo(Trigger trigger) {
            switch (triggerType) {
                case ON_PRESS -> trigger.onTrue(command);
                case ON_RELEASE -> trigger.onFalse(command);
                case WHILE_HOLD -> trigger.whileTrue(command);
                case TOGGLE -> trigger.toggleOnTrue(command);
            }
        }

        @Override
        public Trigger getTrigger(Joystick joystick) {
            return new Trigger(() -> joystick.getPOV() != -1);
        }
    }


    public static class POVAndButton implements OIEntry{
        public final Command command;
        public final int buttonNumber;
        public final TriggerType triggerType;

        /**
         * Triggers a command when the POV hat is pressed in any direction and a button is pressed.
         * @param command The command to run
         * @param buttonNumber The button number to run the command on
         * @param triggerType What trigger type to use (on press, on release, hold, toggle)
         */
        public POVAndButton(Command command, int buttonNumber, TriggerType triggerType){
            this.command = command;
            this.buttonNumber = buttonNumber;
            this.triggerType = triggerType;
        }

        @Override
        public void bindTo(Trigger trigger) {
            switch (triggerType) {
                case ON_PRESS -> trigger.onTrue(command);
                case ON_RELEASE -> trigger.onFalse(command);
                case WHILE_HOLD -> trigger.whileTrue(command);
                case TOGGLE -> trigger.toggleOnTrue(command);
            }
        }

        @Override
        public Trigger getTrigger(Joystick joystick) {
            Trigger povTrigger = new Trigger(() -> joystick.getPOV() != -1);
            return new JoystickButton(joystick, buttonNumber).and(povTrigger);
        }
    }

    public static class JoystickTrigger implements OIEntry{
        public final Joystick joystick;
        public final Command command;
        public final int axis;
        public final double threshold;
        public final TriggerType triggerType;

        public JoystickTrigger(Joystick joystick, Command command, int axis, double threshold, TriggerType triggerType){
            this.joystick = joystick;
            this.axis = axis;
            this.threshold = threshold;
            this.triggerType = triggerType;
            this.command = command;
        }

        @Override
        public void bindTo(Trigger trigger) {
            switch (triggerType) {
                case ON_PRESS -> trigger.onTrue(command);
                case ON_RELEASE -> trigger.onFalse(command);
                case WHILE_HOLD -> trigger.whileTrue(command);
                case TOGGLE -> trigger.toggleOnTrue(command);
            }
        }

        @Override
        public Trigger getTrigger(Joystick joystick) {
            return new Trigger(() -> Math.abs(this.joystick.getRawAxis(axis)) > threshold);
        }

        public static JoystickTrigger whileMoved(Joystick joystick, Command command, int axis, double threshold){
            return new JoystickTrigger(joystick, command, axis, threshold, TriggerType.WHILE_HOLD);
        }

        public static JoystickTrigger onMove(Joystick joystick, Command command, int axis, double threshold){
            return new JoystickTrigger(joystick, command, axis, threshold, TriggerType.ON_PRESS);
        }

        public static JoystickTrigger onZero(Joystick joystick, Command command, int axis, double threshold){
            return new JoystickTrigger(joystick, command, axis, threshold, TriggerType.ON_RELEASE);
        }

        public static JoystickTrigger toggle(Joystick joystick, Command command, int axis, double threshold){
            return new JoystickTrigger(joystick, command, axis, threshold, TriggerType.TOGGLE);
        }
    }

    public static class Password implements OIEntry {
        private final int[] password;
        private final Command command;
        private final Timer timer = new Timer();
        private int i = 0;

        public Password(Command command, int... password) {
            this.command = command;
            this.password = password;
        }

        @Override
        public void bindTo(Trigger trigger) {
            trigger.onTrue(command);
        }

        @Override
        public Trigger getTrigger(Joystick joystick) {
            return new Trigger(() -> {
                timer.start();
                if(joystick.getRawButton(password[i])) {
                    i++;
                    timer.reset();
                } else if(timer.get() > 0.7) {
                    i = 0;
                }
                if (i == password.length){
                    i = 0;
                    return true;
                };
                return false;
            });
        }

    }

    public static enum TriggerType {
        ON_PRESS, ON_RELEASE, WHILE_HOLD, TOGGLE
    }
}
