/*
 * Copyright (c) 2017 Titan Robotics Club (http://www.titanrobotics.com)
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

package frclib.input;

import java.util.HashMap;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frclib.output.FrcDashboard;

/**
 * This class implements a choice menu where a number of choices are presented to the user on the dashboard. The user
 * can make the selection.
 */
public class FrcChoiceMenu<T>
{
    private final FrcDashboard dashboard = FrcDashboard.getInstance();
    private final String menuTitle;
    private SendableChooser<T> chooser;
    private HashMap<T, String> hashMap;

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param menuTitle specifies the title of the menu.
     */
    public FrcChoiceMenu(String menuTitle)
    {
        if (menuTitle == null)
        {
            throw new NullPointerException("menuTitle cannot be null.");
        }

        this.menuTitle = menuTitle;
        chooser = new SendableChooser<>();
        hashMap = new HashMap<>();
    }   //FrcChoiceMenu

    /**
     * This method returns the title text of this menu.
     *
     * @return title text.
     */
    public String getTitle()
    {
        return menuTitle;
    }   //getTitle

    /**
     * This method adds a choice to the menu. The choices will be displayed in the order of them being added.
     *
     * @param choiceText specifies the choice text that will be displayed on the dashboard.
     * @param choiceObject specifies the object to be returned if the choice is selected.
     * @param defChoice specifies true to set it the default choice, false otherwise.
     * @param lastChoice specifies true if this is the last choice added to the choice menu, false otherwise.
     */
    public void addChoice(String choiceText, T choiceObject, boolean defChoice, boolean lastChoice)
    {
        hashMap.put(choiceObject, choiceText);
        if (defChoice)
        {
            chooser.setDefaultOption(choiceText, choiceObject);
        }
        else
        {
            chooser.addOption(choiceText, choiceObject);
        }

        if (lastChoice)
        {
            dashboard.putData(menuTitle, chooser);
        }
    }   //addChoice

    /**
     * This method adds a choice to the menu. The choices will be displayed in the order of them being added.
     *
     * @param choiceText specifies the choice text that will be displayed on the dashboard.
     * @param choiceObject specifies the object to be returned if the choice is selected.
     */
    public void addChoice(String choiceText, T choiceObject)
    {
        addChoice(choiceText, choiceObject, false, false);
    }   //addChoice

    /**
     * This method returns the current selected choice item. Every menu has a current choice even if the user hasn't
     * picked a choice. In that case, the current choice is the default selection of the menu.
     *
     * @return current selected choice object.
     */
    public T getCurrentChoiceObject()
    {
        return chooser.getSelected();
    }   //getCurrentChoiceObject

    /**
     * This method returns the text of the current choice. Every menu has a current choice even if the menu hasn't
     * been displayed and the user hasn't picked a choice. In that case, the current choice is the default selection
     * of the menu which is the first choice in the menu. If the menu is empty, the current choice is null.
     *
     * @return current selected choice text, null if menu is empty.
     */
    public String getCurrentChoiceText()
    {
        return hashMap.get(chooser.getSelected());
    }   //getCurrentChoiceText

}   //class FrcChoiceMenu
