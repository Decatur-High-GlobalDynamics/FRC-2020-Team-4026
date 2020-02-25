package frc.robot;

public class Gains {
    public double kP, kI, kD, kF, kIZone, kPeakOutput;
    
    public Gains(double kP, double kI, double kD, double kF, double kIZone, double kPeakOutput){
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
        this.kIZone = kIZone;
        this.kPeakOutput = kPeakOutput;
    }
}