package frc.robot;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

/**
 * A wrapper class for Motors that helps to consistently and easily perform the
 * following functions: -Keep current and max speeds -Get and Reset encoder
 * values -Lots and lots of SmartDashboard information
 */
public class TeamTalonFX extends WPI_TalonFX implements TeamTalon {
  
  private double lastTelemetryUpdate = 0;

  protected final String smartDashboardPrefix;

  protected int numEStops = 0;

  protected double maxSpeed = Double.MAX_VALUE;

  protected PidParameters pidProfiles[] = new PidParameters[4];

  public TeamTalonFX(String smartDashboardPrefix, int deviceNumber) {
    super(deviceNumber);
    this.smartDashboardPrefix = smartDashboardPrefix;
    // assuming quadencoder
    this.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
  }

  public long getCurrentEncoderValue() {
    // This should be configurable
    return (long) getSensorCollection().getIntegratedSensorPosition();
  }

  public void resetEncoder() {
    getSensorCollection().setIntegratedSensorPosition(0, 0);
  }

  public double getLastTelemetryUpdate() {
    return lastTelemetryUpdate;
  }

  public void setLastTelemetryUpdate(double val) {
    lastTelemetryUpdate = val;
  }

  public String getSmartDashboardPrefix() {
    return smartDashboardPrefix;
  }

  public int getNumEStops() {
    return numEStops;
  }

  public void setNumEStops (int val) {
    numEStops = val;
  }

  public double getMaxSpeed() {
    return maxSpeed;
  }

  public void setMaxSpeed(double val) {
    maxSpeed = val;
  }

  public PidParameters[] getPidProfiles() {
    return pidProfiles;
  }
}
