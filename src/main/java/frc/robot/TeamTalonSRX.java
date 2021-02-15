package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * A wrapper class for Motors that helps to consistently and easily perform the following functions:
 * -Keep current and max speeds -Get and Reset encoder values -Lots and lots of SmartDashboard
 * information
 */
public class TeamTalonSRX extends WPI_TalonSRX implements TeamTalon {
  public static double telemetryUpdateInterval_secs = 0.0;
  private double lastTelemetryUpdate = 0;

  protected final String smartDashboardPrefix;

  protected int numEStops = 0;

  protected double maxSpeed = Double.MAX_VALUE;

  protected PidParameters pidProfiles[] = new PidParameters[4];

  public TeamTalonSRX(String smartDashboardPrefix, int deviceNumber) {
    super(deviceNumber);
    this.smartDashboardPrefix = smartDashboardPrefix;
    // assuming quadencoder
    this.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
  }

  public long getCurrentEncoderValue() {
    // This should be configurable
    return getSensorCollection().getQuadraturePosition();
  }

  public void resetEncoder() {
    getSensorCollection().setQuadraturePosition(0, 0);
  }
}
