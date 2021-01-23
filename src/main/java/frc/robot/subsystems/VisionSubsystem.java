package frc.robot.subsystems;

import frc.robot.TeamUtils;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Timer;

public class VisionSubsystem extends SubsystemBase {

  private double lastSeenTx = 4026;
  private boolean tv = false;
  private double lastSeenTy = 4026;
  private double lastTimeTargetSeen = -9999999;
  private double tvRecentTime =
      0.25; // maximum time before tvRecent is set back to false after not seeing any targets
  private boolean ballSeen = false;
  private double ballX = 0;
  private double ballY = 0;

  public VisionSubsystem() {}

  @Override
  public void periodic() {
    try {
      boolean tv = (double) TeamUtils.getFromNetworkTable("limelight", "tv") >= 1;
      boolean ballSeen = (boolean) TeamUtils.getFromNetworkTable("ballVision", "hasTarget");
      if (tv) {
        this.lastTimeTargetSeen = Timer.getFPGATimestamp();
        double ballX = (double) TeamUtils.getFromNetworkTable("ballVision", "targetX");
        double ballY = (double) TeamUtils.getFromNetworkTable("ballVision", "targetY");
        double lastSeenTx = (double) TeamUtils.getFromNetworkTable("limelight", "tx");
        double lastSeenTy = (double) TeamUtils.getFromNetworkTable("limelight", "ty");
        this.lastSeenTx = lastSeenTx;
        this.lastSeenTy = lastSeenTy;
        this.ballX = ballX;
        this.ballY = ballY;
      }
    } catch (Exception e) {

    }

    try {
      boolean ballSeen = (boolean) TeamUtils.getFromNetworkTable("ballVision", "hasTarget");
      if (ballSeen) {
        double ballX = (double) TeamUtils.getFromNetworkTable("ballVision", "targetX");
        double ballY = (double) TeamUtils.getFromNetworkTable("ballVision", "targetY");
        this.ballX = ballX;
        this.ballY = ballY;
      }
    } catch (Exception e){

    }
  }

  public double getLastSeenTx() {
    return this.lastSeenTx;
  }

  public double getLastSeenTy() {
    return this.lastSeenTy;
  }

  public boolean getTv() {
    return this.tv;
  }

  public boolean isValid() {
    return (Timer.getFPGATimestamp() - this.lastTimeTargetSeen) > this.tvRecentTime;
  }

  public double getBallX(){
    return this.ballX;
  }
  public double getBallY(){
    return this.ballY;
  }
  public double getBallSeen(){
    return this.ballSeen;
  }
}
