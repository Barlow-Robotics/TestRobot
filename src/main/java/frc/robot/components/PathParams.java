package frc.robot.components;

import frc.robot.Constants;

public class PathParams {
    private double leftRotations, rightRotations;
    private double radius, degreeChangeInRadians;


    public PathParams(double degreeChangeInRadians, double radius){
        this.degreeChangeInRadians = degreeChangeInRadians;
        this.radius = radius;
        leftRotations = (radius + Constants.halfDistanceBetweenWheels) * degreeChangeInRadians/(2 * Math.PI);
        rightRotations = (radius - Constants.halfDistanceBetweenWheels) * degreeChangeInRadians/(2 * Math.PI);
    }

    

    public double getLeftRotations(){
        return leftRotations * 8192.0;
    }



    public double getRightRotations(){
        return rightRotations * 8192.0;
    }
    
}