/*
 * Copyright (c) 2015 Titan Robotics Club (http://www.titanrobotics.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package frclib.driverio;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Locale;
import java.util.Set;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import trclib.driverio.TrcDashboard;
import trclib.robotcore.TrcDbgTrace;
import trclib.robotcore.TrcRobot;
import trclib.robotcore.TrcTaskMgr;
import trclib.subsystem.TrcSubsystem;
import trclib.timer.TrcTimer;

/**
 * This class extends the SmartDashboard class and provides a way to send named
 * data to the Driver Station to be displayed, it also simulates an LCD display
 * similar to the NXT Mindstorms. The Mindstorms has only 8 lines but this
 * dashboard can support as many lines as the Driver Station can support. By
 * default, we set the number of lines to 16. By changing a constant here, you
 * can have as many lines as you want. This dashboard display is very useful for
 * displaying debug information.
 */
public class FrcDashboard extends TrcDashboard
{
    private static final String moduleName = FrcDashboard.class.getSimpleName();
    private static final long DASHBOARD_TASK_INTERVAL_MS = 10;      // in msec (100 Hz)
    public static final double DASHBOARD_UPDATE_INTERVAL = 0.2;     // in sec (5 Hz)

    private final HashMap<String, StatusUpdate> statusUpdateMap = new HashMap<>();
    private final ArrayList<StatusUpdate> statusUpdateList = new ArrayList<>();
    private TrcTaskMgr.TaskObject dashboardTaskObj = null;
    private Double nextDashboardUpdateTime =  null;
    private int dashboardStartLineNum = 1;
    private boolean showSubsystemStatus = false;
    private boolean dashboardUpdateEnabled = false;

    private static String[] display;

    /**
     * This method returns the instance of this object if one already exist, creates one if none existed.
     *
     * @param numLines specifies the number of display lines.
     * @return instance of the object, null if none existed.
     */
    public static FrcDashboard getInstance(int numLines)
    {
        if (instance == null)
        {
            instance = new FrcDashboard(numLines);
        }

        FrcDashboard dashboard = (FrcDashboard) instance;
        if (dashboard.dashboardTaskObj == null)
        {
            dashboard.dashboardTaskObj = TrcTaskMgr.createTask(moduleName + ".task", dashboard::dashboardTask);
            dashboard.dashboardTaskObj.registerTask(TrcTaskMgr.TaskType.STANDALONE_TASK, DASHBOARD_TASK_INTERVAL_MS);
            dashboard.disableDashboardUpdate();
        }

        return dashboard;
    }   //getInstance

    /**
     * This method returns the instance of this object if one already exist, creates one if none existed.
     *
     * @return instance of the object, null if none existed.
     */
    public static FrcDashboard getInstance()
    {
        return getInstance(MAX_NUM_TEXTLINES);
    }   //getInstance

    /**
     * Constructor: Creates an instance of the object.
     */
    private FrcDashboard(int numLines)
    {
        super(numLines);

        instance = this;
        display = new String[numLines];
        clearDisplay();
    }   //FrcDashboard

    /**
     * This method terminates the Dashboard Task.
     */
    public void terminateDashboardTask()
    {
        if (dashboardTaskObj != null)
        {
            dashboardTaskObj.unregisterTask();
            statusUpdateMap.clear();
            statusUpdateList.clear();
            dashboardTaskObj = null;
        }
    }   //terminateDashboardTask

    /**
     * This method enables Dashboard Update.
     *
     * @param startLineNum specifies line number of the dashboard to start the dashboard update.
     * @param showSubsystems specifies true to enable subsystem status, false to disable.
     */
    public void enableDashboardUpdate(int startLineNum, boolean showSubsystems)
    {
        TrcDbgTrace.globalTraceInfo(
            moduleName, "enableDashboardUpdate(start=%d, showSubsystem=%s)", startLineNum, showSubsystems);
        this.dashboardStartLineNum = startLineNum;
        this.showSubsystemStatus = showSubsystems;
        this.dashboardUpdateEnabled = true;
    }   //enableDashboardUpdate

    /**
     * This method disables Dashboard Update.
     */
    public void disableDashboardUpdate()
    {
        TrcDbgTrace.globalTraceInfo(moduleName, "disableDashboardUpdate()");
        this.dashboardUpdateEnabled = false;
        this.dashboardStartLineNum = 1;
        this.showSubsystemStatus = false;
        clearDisplay();
    }   //disableDashboardUpdate

    /**
     * This method checks if Dashboard Update is enabled.
     *
     * @return true if update is enabled, false if disabled.
     */
    public boolean isDashboardUpdateEnabled()
    {
        return dashboardUpdateEnabled;
    }   //isDashboardUpdateEnabled

    /**
     * This method adds a component for dashboard status update.
     *
     * @param name specifies the component name.
     * @param statusUpdate specifies the method to call for status update.
     * @return true if status update is added success, false if component is already in the list.
     */
    public boolean addStatusUpdate(String name, StatusUpdate statusUpdate)
    {
        boolean success = false;

        if (!statusUpdateMap.containsKey(name))
        {
            statusUpdateMap.put(name, statusUpdate);
            statusUpdateList.add(statusUpdate);
            success = true;
        }

        return success;
    }   //addStatusUpdate

    /**
     * This methods is called periodically to run the task.
     *
     * @param taskType specifies the type of task being run.
     * @param runMode specifies the competition mode (e.g. Autonomous, TeleOp, Test).
     * @param slowPeriodicLoop specifies true if it is running the slow periodic loop on the main robot thread,
     *        false otherwise.
     */
    private void dashboardTask(TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode, boolean slowPeriodicLoop)
    {
        double currTime = TrcTimer.getCurrentTime();
        boolean slowLoop = nextDashboardUpdateTime == null || currTime >= nextDashboardUpdateTime;

        if (slowLoop)
        {
            nextDashboardUpdateTime = currTime + DASHBOARD_UPDATE_INTERVAL;
        }

//        double startTime = TrcTimer.getCurrentTime();
        if (dashboardUpdateEnabled)
        {
            int lineNum = dashboardStartLineNum;
            if (showSubsystemStatus)
            {
                lineNum = TrcSubsystem.updateStatusAll(lineNum, slowLoop);
            }

            for (StatusUpdate update: statusUpdateList)
            {
                lineNum = update.statusUpdate(lineNum, slowLoop);
            }
        }
//        displayPrintf(10, "UpdateDashboardElapsedTime=%.6f", TrcTimer.getCurrentTime() - startTime);
    }   //dashboardTask

    /**
     * Returns the boolean array the key maps to. If the key does not exist or
     * is of different type, it will return the default value.
     *
     * @param key          the key to look up
     * @param defaultValue the value to be returned if no value is found
     * @return the value associated with the given key or the given default
     * value if there is no value associated with the key
     */
    public boolean[] getBooleanArray(String key, boolean[] defaultValue)
    {
        return getEntry(key).getBooleanArray(defaultValue);
    }   //getBooleanArray

    /**
     * Returns the boolean array the key maps to. If the key does not exist or
     * is of different type, it will return the default value.
     *
     * @param key          the key to look up
     * @param defaultValue the value to be returned if no value is found
     * @return the value associated with the given key or the given default
     * value if there is no value associated with the key
     */
    public Boolean[] getBooleanArray(String key, Boolean[] defaultValue)
    {
        return getEntry(key).getBooleanArray(defaultValue);
    }   //getBooleanArray

    /**
     * Put a boolean array in the table.
     *
     * @param key   the key to be assigned to
     * @param value the value that will be assigned
     * @throws RuntimeException if key already exists with a different type.
     */
    public void putBooleanArray(String key, boolean[] value)
    {
        if (!getEntry(key).setBooleanArray(value))
        {
            throw new RuntimeException("Key already exists with a different type.");
        }
    }   //putBooleanArray

    /**
     * Put a boolean array in the table.
     *
     * @param key   the key to be assigned to
     * @param value the value that will be assigned
     * @throws RuntimeException if key already exists with a different type.
     */
    public void putBooleanArray(String key, Boolean[] value)
    {
        if (!getEntry(key).setBooleanArray(value))
        {
            throw new RuntimeException("Key already exists with a different type.");
        }
    }   //putBooleanArray

    /**
     * Gets the current value in the table, setting it if it does not exist.
     *
     * @param key          the key
     * @param defaultValue the default value to set if key does not exist.
     * @throws RuntimeException if key already exists with a different type.
     */
    public void setDefaultBoolean(String key, boolean defaultValue)
    {
        if (!getEntry(key).setDefaultBoolean(defaultValue))
        {
            throw new RuntimeException("Key already exists with a different type.");
        }
    }   //setDefaultBoolean

    /**
     * Gets the current value in the table, setting it if it does not exist.
     *
     * @param key          the key
     * @param defaultValue the default value to set if key does not exist.
     * @throws RuntimeException if key already exists with a different type.
     */
    public void setDefaultBooleanArray(String key, boolean[] defaultValue)
    {
        if (!getEntry(key).setDefaultBooleanArray(defaultValue))
        {
            throw new RuntimeException("Key already exists with a different type.");
        }
    }   //setDefaultBooleanArray

    /**
     * Gets the current value in the table, setting it if it does not exist.
     *
     * @param key          the key
     * @param defaultValue the default value to set if key does not exist.
     * @throws RuntimeException if key already exists with a different type.
     */
    public void setDefaultBooleanArray(String key, Boolean[] defaultValue)
    {
        if (!getEntry(key).setDefaultBooleanArray(defaultValue))
        {
            throw new RuntimeException("Key already exists with a different type.");
        }
    }   //setDefaultBooleanArray

    /**
     * Returns the number array the key maps to. If the key does not exist or is
     * of different type, it will return the default value.
     *
     * @param key          the key to look up
     * @param defaultValue the value to be returned if no value is found
     * @return the value associated with the given key or the given default
     * value if there is no value associated with the key
     */
    public double[] getNumberArray(String key, double[] defaultValue)
    {
        return getEntry(key).getDoubleArray(defaultValue);
    }   //getNumberArray

    /**
     * Returns the number array the key maps to. If the key does not exist or is
     * of different type, it will return the default value.
     *
     * @param key          the key to look up
     * @param defaultValue the value to be returned if no value is found
     * @return the value associated with the given key or the given default
     * value if there is no value associated with the key
     */
    public Double[] getNumberArray(String key, Double[] defaultValue)
    {
        return getEntry(key).getDoubleArray(defaultValue);
    }   //getNumberArray

    /**
     * Put a number array in the table.
     *
     * @param key   the key to be assigned to
     * @param value the value that will be assigned
     * @throws RuntimeException if key already exists with a different type.
     */
    public void putNumberArray(String key, double[] value)
    {
        if (!getEntry(key).setDoubleArray(value))
        {
            throw new RuntimeException("Key already exists with a different type.");
        }
    }   //putNumberArray

    /**
     * Put a number array in the table.
     *
     * @param key   the key to be assigned to
     * @param value the value that will be assigned
     * @throws RuntimeException if key already exists with a different type.
     */
    public void putNumberArray(String key, Double[] value)
    {
        if (!getEntry(key).setNumberArray(value))
        {
            throw new RuntimeException("Key already exists with a different type.");
        }
    }   //putNumberArray

    /**
     * Gets the current value in the table, setting it if it does not exist.
     *
     * @param key          the key
     * @param defaultValue the default value to set if key does not exist.
     * @throws RuntimeException if key already exists with a different type.
     */
    public void setDefaultNumber(String key, double defaultValue)
    {
        if (!getEntry(key).setDefaultDouble(defaultValue))
        {
            throw new RuntimeException("Key already exists with a different type.");
        }
    }   //setDefaultNumber

    /**
     * Gets the current value in the table, setting it if it does not exist.
     *
     * @param key          the key
     * @param defaultValue the default value to set if key does not exist.
     * @throws RuntimeException if key already exists with a different type.
     */
    public void setDefaultNumberArray(String key, double[] defaultValue)
    {
        if (!getEntry(key).setDefaultDoubleArray(defaultValue))
        {
            throw new RuntimeException("Key already exists with a different type.");
        }
    }   //setDefaultNumberArray

    /**
     * Gets the current value in the table, setting it if it does not exist.
     *
     * @param key          the key
     * @param defaultValue the default value to set if key does not exist.
     * @throws RuntimeException if key already exists with a different type.
     */
    public void setDefaultNumberArray(String key, Double[] defaultValue)
    {
        if (!getEntry(key).setDefaultNumberArray(defaultValue))
        {
            throw new RuntimeException("Key already exists with a different type.");
        }
    }   //setDefaultNumberArray

    /**
     * Returns the string array the key maps to. If the key does not exist or is
     * of different type, it will return the default value.
     *
     * @param key          the key to look up
     * @param defaultValue the value to be returned if no value is found
     * @return the value associated with the given key or the given default
     * value if there is no value associated with the key
     */
    public String[] getStringArray(String key, String[] defaultValue)
    {
        return getEntry(key).getStringArray(defaultValue);
    }   //getStringArray

    /**
     * Put a string array in the table.
     *
     * @param key   the key to be assigned to
     * @param value the value that will be assigned
     * @throws RuntimeException if key already exists with a different type.
     */
    public void putStringArray(String key, String[] value)
    {
        if (!getEntry(key).setStringArray(value))
        {
            throw new RuntimeException("Key already exists with a different type.");
        }
    }   //putStringArray

    /**
     * Gets the current value in the table, setting it if it does not exist.
     *
     * @param key          the key
     * @param defaultValue the default value to set if key does not exist.
     * @throws RuntimeException if key already exists with a different type.
     */
    public void setDefaultString(String key, String defaultValue)
    {
        if (!getEntry(key).setDefaultString(defaultValue))
        {
            throw new RuntimeException("Key already exists with a different type.");
        }
    }   //setDefaultString

    /**
     * Gets the current value in the table, setting it if it does not exist.
     *
     * @param key          the key
     * @param defaultValue the default value to set if key does not exist.
     * @throws RuntimeException if key already exists with a different type.
     */
    public void setDefaultStringArray(String key, String[] defaultValue)
    {
        if (!getEntry(key).setDefaultStringArray(defaultValue))
        {
            throw new RuntimeException("Key already exists with a different type.");
        }
    }   //setDefaultStringArray

    /**
     * Returns the value at the specified key.
     *
     * @param key the key
     * @return the value
     * @throws IllegalArgumentException if the key is null
     */
    public Sendable getData(String key)
    {
        return SmartDashboard.getData(key);
    }   //getData

    /**
     * Maps the specified key to the specified value in this table. The key can
     * not be null. The value can be retrieved by calling the get method with a
     * key that is equal to the original key.
     *
     * @param key  the key
     * @param data the value
     * @throws IllegalArgumentException If key is null
     */
    public void putData(String key, Sendable data)
    {
        SmartDashboard.putData(key, data);
    }   //putData

    /**
     * Maps the specified key (where the key is the name of the
     * {@link Sendable} to the specified value in this table. The value can
     * be retrieved by calling the get method with a key that is equal to the
     * original key.
     *
     * @param value the value
     * @throws IllegalArgumentException If key is null
     */
    public void putData(Sendable value)
    {
        SmartDashboard.putData(value);
    }   //putData

    /**
     * Gets the entry for the specified key.
     *
     * @param key the key name
     * @return Network table entry.
     */
    public NetworkTableEntry getEntry(String key)
    {
        return SmartDashboard.getEntry(key);
    }   //getEntry

    /**
     * Get the keys stored in the SmartDashboard table of NetworkTables.
     *
     * @param types bitmask of types; 0 is treated as a "don't care".
     * @return keys currently in the table
     */
    public Set<String> getKeys(int types)
    {
        return SmartDashboard.getKeys(types);
    }   //getKeys

    /**
     * Get the keys stored in the SmartDashboard table of NetworkTables.
     *
     * @return keys currently in the table.
     */
    public Set<String> getKeys()
    {
        return SmartDashboard.getKeys();
    }   //getKeys

    /**
     * Checks the table and tells if it contains the specified key.
     *
     * @param key the key to search for
     * @return true if the table as a value assigned to the given key
     */
    public boolean containsKey(String key)
    {
        return SmartDashboard.containsKey(key);
    }   //containKey

    /**
     * Refreshes the entry value. If the entry doesn't exist, create it with the corresponding value.
     *
     * @param key          The name of the entry.
     * @param defaultValue The value to create the entry with if it doesn't exist.
     */
    public void refreshKey(String key, Object defaultValue)
    {
        SmartDashboard.getEntry(key).setDefaultValue(defaultValue);
    }   //refreshKey

    /**
     * Returns the raw value (byte array) the key maps to. If the key does not
     * exist or is of different type, it will return the default value.
     *
     * @param key          the key to look up
     * @param defaultValue the value to be returned if no value is found
     * @return the value associated with the given key or the given default
     * value if there is no value associated with the key
     */
    public byte[] getRaw(String key, byte[] defaultValue)
    {
        return getEntry(key).getRaw(defaultValue);
    }   //getRaw

    /**
     * Put a raw value (byte array) in the table.
     *
     * @param key   the key to be assigned to
     * @param value the value that will be assigned
     * @throws RuntimeException if key already exists with a different type.
     */
    public void putRaw(String key, byte[] value)
    {
        if (!getEntry(key).setRaw(value))
        {
            throw new RuntimeException("Key already exists with a different type.");
        }
    }   //putRaw

    /**
     * Gets the current value in the table, setting it if it does not exist.
     *
     * @param key          the key
     * @param defaultValue the default value to set if key does not exist.
     * @throws RuntimeException if key already exists with a different type.
     */
    public void setDefaultRaw(String key, byte[] defaultValue)
    {
        if (!getEntry(key).setDefaultRaw(defaultValue))
        {
            throw new RuntimeException("Key already exists with a different type.");
        }
    }   //setDefaultRaw

    /**
     * Makes a key's value persistent through program restarts. The key cannot
     * be null.
     *
     * @param key the key name
     */
    public void setPersistent(String key)
    {
        getEntry(key).setPersistent();
    }   //setPersistent

    /**
     * Stop making a key's value persistent through program restarts. The key
     * cannot be null.
     *
     * @param key the key name
     */
    public void clearPersistent(String key)
    {
        getEntry(key).clearPersistent();
    }   //clearPersistent

    /**
     * Returns whether the value is persistent through program restarts. The key
     * cannot be null.
     *
     * @param key the key name
     * @return True if the value is persistent.
     */
    public boolean isPersistent(String key)
    {
        return getEntry(key).isPersistent();
    }   //isPersistent

    /**
     * Puts all sendable data to the dashboard.
     */
    public void updateValues()
    {
        SmartDashboard.updateValues();
    }   //updateValues

    //
    // Implements TrcDashboard abstract methods.
    //

    /**
     * This method clears all the display lines.
     */
    @Override
    public void clearDisplay()
    {
        for (int i = 0; i < display.length; i++)
        {
            display[i] = "";
        }
        refreshDisplay();
    }   //clearDisplay

    /**
     * This method refresh the display lines to the Driver Station.
     */
    @Override
    public void refreshDisplay()
    {
        for (int i = 0; i < display.length; i++)
        {
            SmartDashboard.putString(String.format(displayKeyFormat, i), display[i]);
        }
    }   //refreshDisplay

    /**
     * This method displays a formatted message to the display on the Driver
     * Station.
     *
     * @param lineNum specifies the line number on the display.
     * @param msg specifies the message string.
     */
    @Override
    public void displayPrintf(int lineNum, String msg)
    {
        if (lineNum >= 0 && lineNum < display.length)
        {
            display[lineNum] = msg;
            SmartDashboard.putString(String.format(displayKeyFormat, lineNum), display[lineNum]);
        }
    }   //displayPrintf

    /**
     * This method displays a formatted message to the display on the Driver
     * Station.
     *
     * @param lineNum specifies the line number on the display.
     * @param format  specifies the format string.
     * @param args    specifies variable number of substitution arguments.
     */
    @Override
    public void displayPrintf(int lineNum, String format, Object... args)
    {
        displayPrintf(lineNum, String.format(Locale.US, format, args));
    }   //displayPrintf

    /**
     * Returns the boolean the key maps to. If the key does not exist or is of
     * different type, it will return the default value.
     *
     * @param key          the key to look up
     * @param defaultValue the value to be returned if no value is found
     * @return the value associated with the given key or the given default
     * value if there is no value associated with the key
     */
    @Override
    public boolean getBoolean(String key, boolean defaultValue)
    {
        return getEntry(key).getBoolean(defaultValue);
    }   //getBoolean

    /**
     * Put a boolean in the table.
     *
     * @param key   the key to be assigned to
     * @param value the value that will be assigned
     * @throws RuntimeException if key already exists with a different type.
     */
    @Override
    public void putBoolean(String key, boolean value)
    {
        if (!getEntry(key).setBoolean(value))
        {
            throw new RuntimeException("Key already exists with a different type.");
        }
    }   //putBoolean

    /**
     * This method returns the value associated with the given key. If the key
     * does not already exist, it will create the key and put the default value
     * in it and also return the default value.
     *
     * @param key          specifies the key.
     * @param defaultValue specifies the default value if the key does not
     *                     already exist.
     * @return value associated with the key or the default value if key does
     * not exist.
     */
    @Override
    public double getNumber(String key, double defaultValue)
    {
        double value = defaultValue;

        if (SmartDashboard.containsKey(key))
        {
            value = SmartDashboard.getNumber(key, defaultValue);
        }
        else
        {
            SmartDashboard.putNumber(key, defaultValue);
        }

        return value;
    }   //getNumber

    /**
     * Put a number in the table.
     *
     * @param key   the key to be assigned to
     * @param value the value that will be assigned
     * @throws RuntimeException if key already exists with a different type.
     */
    @Override
    public void putNumber(String key, double value)
    {
        if (!getEntry(key).setDouble(value))
        {
            throw new RuntimeException("Key already exists with a different type.");
        }
    }   //putNumber

    /**
     * This method returns the value associated with the given key. If the key
     * does not already exist, it will create the key and put the default value
     * in it and also return the default value.
     *
     * @param key          specifies the key.
     * @param defaultValue specifies the default value if the key does not
     *                     already exist.
     * @return value associated with the key or the default value if key does
     * not exist.
     */
    @Override
    public String getString(String key, String defaultValue)
    {
        String value = defaultValue;

        if (SmartDashboard.containsKey(key))
        {
            value = SmartDashboard.getString(key, defaultValue);
        }
        else
        {
            SmartDashboard.putString(key, defaultValue);
        }

        return value;
    }   //getString

    /**
     * Put a string in the table.
     *
     * @param key   the key to be assigned to
     * @param value the value that will be assigned
     * @throws RuntimeException if key already exists with a different type.
     */
    @Override
    public void putString(String key, String value)
    {
        if (!getEntry(key).setString(value))
        {
            throw new RuntimeException("Key already exists with a different type.");
        }
    }   //putString

    /**
     * This method returns the value associated with the given key. If the key
     * does not already exist, it will create the key and put the default value
     * in it and also return the default value.
     *
     * @param key          specifies the key.
     * @param defaultValue specifies the default value if the key does not
     *                     already exist.
     * @return value associated with the key or the default value if key does
     * not exist.
     */
    @Override
    public Object getObject(String key, Object defaultValue)
    {
        Object value = defaultValue;

        if (SmartDashboard.containsKey(key))
        {
            value = SmartDashboard.getEntry(key).getValue();
        }
        else
        {
            putObject(key, defaultValue);
        }

        return value;
    }   //getObject

    /**
     * Put an object in the table.
     *
     * @param key   the key to be assigned to
     * @param value the value that will be assigned
     * @throws RuntimeException if key already exists with a different type.
     */
    @Override
    public void putObject(String key, Object value)
    {
        if (!getEntry(key).setValue(value))
        {
            throw new RuntimeException("Key already exists with a different type.");
        }
    }   //putObject

}   // class FrcDashboard
