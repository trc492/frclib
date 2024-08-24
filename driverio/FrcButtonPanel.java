/*
 * Copyright (c) 2024 Titan Robotics Club (http://www.titanrobotics.com)
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

import java.util.HashMap;

import edu.wpi.first.wpilibj.DriverStation;
import trclib.driverio.TrcGameController;

/**
 * This class implements the platform dependent button panel. It provides monitoring of the panel buttons. If the
 * caller of this class provides a button notification handler, it will call it when there are button events.
 */
public class FrcButtonPanel extends TrcGameController
{
    //
    // Generic USB Button Panel:
    // UsagePage=0x01, Usage=0x04
    //
    private static final int RED1                       = (1);
    private static final int GREEN1                     = (1 << 1);
    private static final int BLUE1                      = (1 << 2);
    private static final int YELLOW1                    = (1 << 3);
    private static final int WHITE1                     = (1 << 4);
    private static final int RED2                       = (1 << 5);
    private static final int GREEN2                     = (1 << 6);
    private static final int BLUE2                      = (1 << 7);
    private static final int YELLOW2                    = (1 << 8);
    private static final int WHITE2                     = (1 << 9);

    public enum ButtonType
    {
        Red1,
        Green1,
        Blue1,
        Yellow1,
        White1,
        Red2,
        Green2,
        Blue2,
        Yellow2,
        White2
    }   //enum ButtonType

    /**
     * This interface, if provided, will allow this class to do a notification callback when there are button
     * activities.
     */
    public interface ButtonEventHandler
    {
        /**
         * This method is called when button event is detected.
         *
         * @param buttonType specifies the button type that generates the event.
         * @param pressed specifies true if the button is pressed, false otherwise.
         */
        void buttonEvent(ButtonType buttonType, boolean pressed);

    }   //interface ButtonEventHandler

    private static HashMap<Integer, ButtonType> buttonTypeMap = null;
    private final int port;
    private ButtonEventHandler buttonEventHandler = null;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param port specifies the USB port the joystick is plugged into.
     */
    public FrcButtonPanel(String instanceName, int port)
    {
        super(instanceName, 0.0);
        this.port = port;
        if (buttonTypeMap == null)
        {
            buttonTypeMap = new HashMap<>();
            buttonTypeMap.put(RED1, ButtonType.Red1);
            buttonTypeMap.put(GREEN1, ButtonType.Green1);
            buttonTypeMap.put(BLUE1, ButtonType.Blue1);
            buttonTypeMap.put(YELLOW1, ButtonType.Yellow1);
            buttonTypeMap.put(WHITE1, ButtonType.White1);
            buttonTypeMap.put(RED2, ButtonType.Red2);
            buttonTypeMap.put(GREEN2, ButtonType.Green2);
            buttonTypeMap.put(BLUE2, ButtonType.Blue2);
            buttonTypeMap.put(YELLOW2, ButtonType.Yellow2);
            buttonTypeMap.put(WHITE2, ButtonType.White2);
            super.init();
        }
    }   //FrcButtonPanel

    /**
     * This method sets the button event handler.
     *
     * @param buttonEventHandler specifies button event notification handler, null to disable event notification.
     */
    public void setButtonEventHandler(ButtonEventHandler buttonEventHandler)
    {
        this.buttonEventHandler = buttonEventHandler;
        setButtonEventEnabled(buttonEventHandler != null);
    }   //setButtonEventHandler

    //
    // Implements TrcGameController abstract methods.
    //

    /**
     * This method returns the button states in an integer by combining all the button states.
     *
     * @return button states.
     */
    @Override
    public int getButtons()
    {
        int buttons = DriverStation.getStickButtons(port);
        tracer.traceDebug(instanceName, "buttons=0x" + Integer.toHexString(buttons));
        return buttons;
    }   //getButtons

    /**
     * This method is called when a button event is detected. It translates the button value into button type and
     * calls the button event handler.
     *
     * @param buttonValue specifies the button value that generated the event.
     * @param pressed specifies true if the button is pressed, false if it is released.
     */
    @Override
    protected void notifyButtonEvent(int buttonValue, boolean pressed)
    {
        if (buttonEventHandler != null)
        {
            ButtonType buttonType = buttonTypeMap.get(buttonValue);
            buttonEventHandler.buttonEvent(buttonType, pressed);
        }
    }   //notifyButtonEvent

}   //class FrcPanelButtons

