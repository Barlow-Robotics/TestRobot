package frc.robot.components;

import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;

import edu.wpi.first.wpilibj.util.Color;

//ESSENTIAL NOTE:
//THIS CLASS CANNOT BE IN Robot WHILE ARMSUBSYSTEM IS ALSO THERE
//THE ROBOT ONLY HAS 1 I2C PORT; TRYING TO GET MULTIPLE CLASSES TO CALL IT WILL RESULT IN FATAL ERRORS

public class ColorSensor{
    private ColorSensorV3 colorSensor;
    private ColorMatch colorMatcher = new ColorMatch();
    private final I2C.Port i2cPort =  I2C.Port.kOnboard;
    private char[] colors = {'G', 'B', 'Y', 'R'};

    private Color[] allColors = {colorMatcher.makeColor(0.197, 0.561, 0.240), //G
                                 colorMatcher.makeColor(0.143, 0.427, 0.429), //B
                                 colorMatcher.makeColor(0.361, 0.524, 0.113), //Y
                                 colorMatcher.makeColor(0.561, 0.232, 0.114)}; //R
    
    public ColorSensor(){
        colorSensor = new ColorSensorV3(i2cPort);
    }



    public void postData(){
        Color detectedColor = colorSensor.getColor();
        ColorMatchResult decidedColor = colorMatcher.matchClosestColor(detectedColor);
        SmartDashboard.putString("Color", decidedColor + " ");
        SmartDashboard.putNumber("Red", detectedColor.red);
        SmartDashboard.putNumber("Green", detectedColor.green);
        SmartDashboard.putNumber("Blue", detectedColor.blue);
    }



    public void postColor(){
        Color detectedColor = colorSensor.getColor();
        ColorMatchResult decidedColor = colorMatcher.matchClosestColor(detectedColor);
        SmartDashboard.putString("Color", decidedColor + " ");
    }



    public void postColor(int index){
        SmartDashboard.putString("Color: " + colors[index], allColors[index].toString());
    }



    public Color calibrateColor(int index){
        Color detectedColor = colorSensor.getColor();
        allColors[index] = colorMatcher.makeColor(detectedColor.red, detectedColor.green, detectedColor.blue);
        postData();
        postColor(index);
        return allColors[index];
    }


    public String getColorAtIndex(int index){
        return colors[index] + " ";
    }


    public Color getCurrentColor(){
        Color detectedColor = colorSensor.getColor();
        ColorMatchResult result = colorMatcher.matchClosestColor(detectedColor);
        return result.color;
    }



    public double getRed() {return colorSensor.getColor().red;}
    public double getBlue() {return colorSensor.getColor().blue;}
    public double getGreen() {return colorSensor.getColor().green;}


}