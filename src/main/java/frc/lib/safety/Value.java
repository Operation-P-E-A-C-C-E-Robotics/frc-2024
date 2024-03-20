package frc.lib.safety;

public class Value <T> {
    private final T value;
    private final ValueState state;

    /**
     * Hold a safe value that has a value and a state,
     * and can only be read with a default value.
     * Note from the future: This was a really stupid idea.
     * Here is a fun poem about stupid ideas:
     * Stupid ideas are stupid,
     * Stupid ideas are dumb,
     * Stupid ideas are stupid,
     * And so are you <3
     * - Peaccy, (almost) 2024
     * 
     * Here is a fun poem about writing mildly offensive poems but not actually wanting them to be offensive:
     * I'm sorry if I called you stupid,
     * I'm sorry if I called you dumb,
     * You probably are, but I'm sorry,
     * I like you, you're my chum.
     * -Peaccy
     * @param value the value
     */
    public Value(T value){
        this.value = value;
        this.state = ValueState.NORMAL;
    }

    /**
     * Hold a safe value that has a value and a state,
     * and can only be read with a default value.
     *
     * This constructor is used to create a null value with a state.
     * @param state the state
     */
    public Value(ValueState state){
        this.value = null;
        this.state = state;
    }

    /**
     * Hold a safe value that has a value and a state,
     * and can only be read with a default value.
     * @param value the value
     * @param state the state
     */
    public Value(T value, ValueState state){
        this.value = value;
        this.state = state;
    }

    /**
     * Create an invalid value with the state NOT_AVAILABLE
     * @return the value
     */
    public static <T> Value<T> notAvailable(){
        return new Value<>(ValueState.NOT_AVAILABLE);
    }

    /**
     * Create an invalid value with the state OUTDATED
     * @return the value
     */
    public static <T> Value<T> outdated(){
        return new Value<>(ValueState.OUTDATED);
    }

    /**
     * Create an invalid value with the state NONE
     * @return the value
     */
    public static <T> Value<T> none(){
        return new Value<>(ValueState.NONE);
    }

    /**
     * Check if the value is normal
     * @return true if the value is normal
     */
    public boolean isNormal(){
        return state == ValueState.NORMAL;
    }

    /**
     * Check if the value is not available
     * @return true if the value is not available
     */
    public boolean isNotAvailable(){
        return state == ValueState.NOT_AVAILABLE;
    }

    /**
     * Check if the value is outdated
     * @return true if the value is outdated
     */
    public boolean isOutdated(){
        return state == ValueState.OUTDATED;
    }

    /**
     * Check if the value is none
     * @return true if the value is none
     */
    public boolean isNone(){
        return state == ValueState.NONE;
    }

    /**
     * Check if all the values are normal
     * @param values the values
     * @return true if all the values are normal
     */
    public static boolean valid(Value<?>... values){
        for(Value<?> value : values){
            if(!value.isNormal()){
                return false;
            }
        }
        return true;
    }

    /**
     * Get the state of the value
     * @return the state
     */
    public ValueState getState(){
        return state;
    }

    /**
     * get the actual value of the value
     * @param defaultValue the default value
     * @return the value, or the default value if the value is not normal
     */
    public T get(T defaultValue){
        if(state == ValueState.NORMAL){
            return value;
        }else{
            return defaultValue;
        }
    }

    /**
     * Get the value of the value, even if it is outdated
     * @param defaultValue the default value
     * @return the value, or the default value if the value is not normal or outdated
     */
    public T getOutdatedValue(T defaultValue){
        if(state == ValueState.NORMAL || state == ValueState.OUTDATED){
            return value;
        }else{
            return defaultValue;
        }
    }

    public static Value<Double> of(double value){
        return new Value<>(value);
    }

    public static Value<Boolean> of(boolean value){
        return new Value<>(value);
    }

    public static Value<String> of(String value){
        return new Value<>(value);
    }

    public static Value<Integer> of(int value){
        return new Value<>(value);
    }

    public static Value<Long> of(long value){
        return new Value<>(value);
    }

    public static enum ValueState{
        NORMAL,
        NOT_AVAILABLE,
        OUTDATED,
        NONE
    }
}
