package frc.robot.subsystems;

import frc.robot.TeamUtils;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Timer;

public class VisionSubsystem extends SubsystemBase {

    private double lastSeenTx = 4026;
    private boolean tv = false;
    private double lastSeenTy = 4026;
    private double lastTimeTargetSeen = -9999999;
    private double tvRecentTime = 0.25; //maximum time before tvRecent is set back to false after not seeing any targets

    public VisionSubsystem() {
        
    }
  
    @Override
    public void periodic() {
        try{
            boolean tv = (double) TeamUtils.getFromNetworkTable("limelight", "tv") >= 1;
            if(tv){
                this.lastTimeTargetSeen = Timer.getFPGATimestamp();
                double lastSeenTx = (double) TeamUtils.getFromNetworkTable("limelight", "tx");
                double lastSeenTy = (double) TeamUtils.getFromNetworkTable("limelight", "ty");
                this.lastSeenTx = lastSeenTx;
                this.lastSeenTy = lastSeenTy;
            
            }
        } catch (Exception e){

        }


    }

    public double getLastSeenTx(){
        return this.lastSeenTx;
    }
    public double getLastSeenTy(){
        return this.lastSeenTy;
    }
    public boolean getTv(){
        return this.tv;
    }

    public boolean isValid(){
        return (Timer.getFPGATimestamp() - this.lastTimeTargetSeen) > this.tvRecentTime;
    }
  }