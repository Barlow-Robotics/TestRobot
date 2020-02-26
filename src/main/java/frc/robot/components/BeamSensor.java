package frc.robot.components;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogOutput;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class BeamSensor{
    // DigitalInput receiver;
    // DigitalOutput transmitter;

    DigitalInput receiver;
    DigitalOutput transmitter;
    Counter digitalCounter;

    public BeamSensor(int inputChannel, int outputChannel){
        receiver = new DigitalInput(inputChannel);
        transmitter = new DigitalOutput(outputChannel);
        digitalCounter = new Counter(receiver);
    }



    public boolean getSensorValue(){
        postData();
        return receiver.get();
    }

    public void setOutputValue(boolean outputValue){
        transmitter.set(outputValue);
    }

    public void postData(){
        SmartDashboard.putBoolean("Receiver State", receiver.get());
        SmartDashboard.putNumber("Time", System.currentTimeMillis());
    }
}