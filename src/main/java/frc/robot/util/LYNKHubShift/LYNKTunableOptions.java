package frc.robot.util.LYNKHubShift;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Class for a tunable option.
 * Abstracts a boolean that can be managed via the dashboard.
 */
public class LYNKTunableOptions implements BooleanSupplier {
    // TODO Consider using raw NetworkTables and subscribe to updates, instead of
    // using SmartDashboard
    private static final String tableKey = "TunableOptions";

    private String key;
    private boolean defaultValue;

    public LYNKTunableOptions(String name, boolean defaultValue) {
        if (name.indexOf("/") >= 0) {
            key = name;
        } else {
            key = tableKey + "/" + name;
        }
        this.defaultValue = defaultValue;
        SmartDashboard.putBoolean(key, defaultValue);
    }

    public boolean get() {
        return SmartDashboard.getBoolean(key, defaultValue);
    }

    @Override
    public boolean getAsBoolean() {
        return get();
    }

    public boolean toggle() {
        return SmartDashboard.putBoolean(key, !get());
    }
    
    public void disable() {
        SmartDashboard.putBoolean(key, false);
    }

    public void enable() {
        SmartDashboard.putBoolean(key, true);
    }
}