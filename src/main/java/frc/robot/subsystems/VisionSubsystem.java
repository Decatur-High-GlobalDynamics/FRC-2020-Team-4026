package frc.robot.subsystems;

import frc.robot.TeamUtils;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {

    private double tx = 4026;
    private boolean tv = false;
    private double ty = 4026;

    public VisionSubsystem() {
        
    }
  
    @Override
    public void periodic() {
        try{
            boolean tv = (double) TeamUtils.getFromNetworkTable("limelight", "tv") == 1;
            if(tv){
                double tx = (double) TeamUtils.getFromNetworkTable("limelight", "tx");
                double ty = (double) TeamUtils.getFromNetworkTable("limelight", "ty");

                if(tv){
                    this.tx = tx;
                    this.ty = ty;
                }
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