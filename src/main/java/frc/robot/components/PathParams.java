package frc.robot.components;

import frc.robot.Constants;

public class PathParams {
    private double leftRotations, rightRotations;
    private double radius, degreeChangeInRadians;


    public PathParams(double degreeChangeInRadians, double radius){
        this.degreeChangeInRadians = degreeChangeInRadians;
        this.radius = radius;
        //New formula for testing
        leftRotations = ((degreeChangeInRadians/(Math.PI * 2) + 1) * 2 * Math.PI //Accounts for ratio of degrees to full circumference of arclength circle
                          * (radius + Constants.halfDistanceBetweenWheels))      //+ or - depending on which side; allows for differential rotation
                          /(7.65 * Math.PI);                                     //Conversion for inches to rotations
        //Old nonfunctional formula
        rightRotations = (radius - Constants.halfDistanceBetweenWheels) * degreeChangeInRadians/(2 * Math.PI);
    }

    

    public double getLeftRotations(){
        return leftRotations * 8192.0;
    }



    public double getRightRotations(){
        return rightRotations * 8192.0;
    }
    
}