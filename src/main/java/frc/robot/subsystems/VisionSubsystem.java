package frc.robot.subsystems;

import frc.robot.TeamUtils;

public class VisionSubsystem extends SubsystemBase {

    private double tx;
    private boolean tv;
    private double ty;

    public VisionSubsystem() {
        
    }
  
    @Override
    public void periodic() {
        try{
            double tx = (double) TeamUtils.getFromNetworkTable("limelight", "tx");
            boolean tv = (double) TeamUtils.getFromNetworkTable("limelight", "tv") == 1;
            double ty = (double) TeamUtils.getFromNetworkTable("limelight", "tv");

            if(tv){
                this.tx = tx;
                this.ty = ty;
            }
        } catch (Exception e){

        }


    }

    public double getTx(){
        return this.tx;
    }
    public double getTy(){
        return this.ty;
    }
    public boolean getTv(){
        return this.tv;
    }
  }