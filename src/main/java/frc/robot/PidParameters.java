package frc.robot;

public class PidParameters {
    public double kP, kI, kD, kF, kIZone, kPeakOutput;
    public int errorTolerance;
    
    public PidParameters(double kP, double kI, double kD, double kF, 
            double kIZone, double kPeakOutput,
            int errorTolerance){
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
        this.kIZone = kIZone;
        this.kPeakOutput = kPeakOutput;
        this.errorTolerance = errorTolerance;
    }


    @Override
    public PidParameters clone() {
        return new PidParameters(kP, kI, kD, kF, kIZone, kPeakOutput, errorTolerance);
    }

    @Override
    public boolean equals(Object obj) {
        if ( obj == null )
            return false;
        if (!(obj instanceof PidParameters))
            return false;
        
        PidParameters otherPP = (PidParameters) obj;

        return otherPP.kP==kP &&
            otherPP.kI==kI &&
            otherPP.kD==kD &&
            otherPP.kF==kF &&
            otherPP.kIZone==kIZone &&
            otherPP.kPeakOutput==kPeakOutput &&
            otherPP.errorTolerance==errorTolerance;
    }

}