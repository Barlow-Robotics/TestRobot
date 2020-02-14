/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
 
package frc.robot.components;
 
 
import java.util.ArrayList;
import java.util.List;
 
public class ColourFilter {
    char currentColour;
    int filterLength;
    List<Character> listOfColours;
 
    public void addMeasurement(char c){
        listOfColours.add(c);
        if (listOfColours.size()>filterLength){
            listOfColours.remove(0);
        }
        for (char x: listOfColours){
            if (x!=c) {
                return;
            }  
        }  
        currentColour = c;
    }


 
public char getColour(){
    return currentColour;
}
 
public ColourFilter(int length, char startingColour){
    listOfColours = new ArrayList<Character>();
    currentColour = startingColour;
    filterLength = length;
}
 
}

