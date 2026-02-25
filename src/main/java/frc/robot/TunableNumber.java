package frc.robot;

import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;

import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

@SuppressWarnings("unused")
/*
 *
 *
 * */
public class TunableNumber implements DoubleSupplier{

    private final String key;
    private double defaultValue;
    private final DoubleEntry entry;
    private Map<Integer, Double> lastHasChangedValues = new HashMap<>();

    /**
     * Creates a double that can be updated through NetworkTables <br><br>
     * You are responsible for implementing an update function in the periodic be calling get() and hasChanged()
     *
     *  @param key String corresponding to the path for the {@link NetworkTableEntry} <br>
     *  key should be formatted as /datatable/topic <br>
     *  and you can add more folders by formatting like /datatable/subtable/topic
     *  @param defaultValue the default value of the network table number as a double, should be in constants
     */
    public TunableNumber(String key, double defaultValue){
        this.key = key;
        this.entry = NetworkTableInstance.getDefault().getDoubleTopic(key).getEntry(0.0);
        initDefault(defaultValue);
    }

    public void initDefault(double defaultValue){
        this.defaultValue = defaultValue; 
        entry.set(entry.get(defaultValue));
    }

    public double get() {
        return entry.get(defaultValue);
    }
     
    public boolean hasChanged(int id) {
        double currentValue = get();
        Double lastValue = lastHasChangedValues.get(id);
        
        if (lastValue == null || currentValue != lastValue) {
            lastHasChangedValues.put(id, currentValue);
            return true;
        }

        return false;
    }
    
    @Override
    public double getAsDouble(){
        return get();
    }

}
