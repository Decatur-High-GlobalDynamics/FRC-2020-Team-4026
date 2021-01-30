package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANEncoder;

public class TeamSparkMAX extends CANSparkMax {
public static double telemetryUpdateInterval_secs = 0.0;
  private double lastTelemetryUpdate = 0;

  protected final String smartDashboardPrefix;

  protected int numEStops = 0;

  protected double maxSpeed = Double.MAX_VALUE;

  protected CANPIDController canPidController;

  protected CANEncoder canEncoder;

  protected PidParameters pidProfiles[] = new PidParameters[4];
  

  public TeamSparkMAX(String smartDashboardPrefix, int deviceID) { 
    super(deviceID, MotorType.kBrushless); //Neos are brushless
    this.smartDashboardPrefix = smartDashboardPrefix;
    canPidController = getPIDController();
    canEncoder = getEncoder();
  }

  private static boolean isPidControlMode(ControlMode mode) {
    switch (mode) {
      case Current:
        return false;
      case Disabled:
        return false;
      case Follower:
        return false;
      default:
        return true;
    }
  }

  public boolean isRunningPidControlMode() {
    return isPidControlMode(getControlMode());
  }

  public void noteEmergencyStop() {
    numEStops++;
  }

  public double getCurrentEncoderValue() {
    // This should be configurable
    return canEncoder.getPosition();
  }

  public void resetEncoder() {
    canEncoder.setPosition(0.0);
  }

  public void periodic() {
    double now = TeamUtils.getCurrentTime();

    if ((now - lastTelemetryUpdate) < telemetryUpdateInterval_secs) {
      return;
    }

    lastTelemetryUpdate = now;

    double currentEncoderValue = getCurrentEncoderValue();
    double currentSpeed = getSelectedSensorVelocity();

    if (maxSpeed == Double.MAX_VALUE || currentSpeed > maxSpeed) maxSpeed = currentSpeed;

    if (isRunningPidControlMode()) {
      SmartDashboard.putBoolean(smartDashboardPrefix + ".PID", true);
    } else {
      SmartDashboard.putBoolean(smartDashboardPrefix + ".PID", false);
    }

    if (getControlMode() == ControlMode.Velocity) {
      double currentError = getVelocityError();
      SmartDashboard.putNumber(smartDashboardPrefix + ".VelocityError", currentError);
    } else {
      SmartDashboard.putNumber(smartDashboardPrefix + ".VelocityError", 0);
    }

    SmartDashboard.putNumber(smartDashboardPrefix + ".PowerPercent", getMotorOutputPercent());

    SmartDashboard.putNumber(smartDashboardPrefix + ".Position-ticks", currentEncoderValue);

    SmartDashboard.putNumber(smartDashboardPrefix + ".speedPer100ms", currentSpeed);
    SmartDashboard.putNumber(smartDashboardPrefix + ".speedPerSec", currentSpeed * 10);

    SmartDashboard.putNumber(smartDashboardPrefix + ".maxSpeedPer100ms", maxSpeed);
    SmartDashboard.putNumber(smartDashboardPrefix + ".maxSpeedPerSec", maxSpeed * 10);

    SmartDashboard.putString(smartDashboardPrefix + "Mode", getControlMode().toString());
    SmartDashboard.putNumber(smartDashboardPrefix + "EmergencyStops", numEStops);

    switch (getControlMode()) {
      case Position:
      case Velocity:
        SmartDashboard.putNumber(smartDashboardPrefix + "Target", getClosedLoopTarget(0)); //0 is the primary closed-loop
        SmartDashboard.putNumber(smartDashboardPrefix + "Error", getClosedLoopError(0));
        break;
      default:
        // Fill in Zeros when we're not in a mode that is using it
        SmartDashboard.putNumber(smartDashboardPrefix + "Target", 0);
        SmartDashboard.putNumber(smartDashboardPrefix + "Error", 0);
    }
  }

  public double getVelocityError() {
    if (getControlMode() != ControlMode.Velocity) {
      return 0;
    }
    double currentSpeed = getSelectedSensorVelocity();
    return (double) (getClosedLoopTarget() - currentSpeed);
  }

  public void configureWithPidParameters(PidParameters pidParameters, int pidSlotIndex) {
    pidProfiles[pidSlotIndex] = pidParameters;

    canPidController.setFF(pidParameters.kF, pidSlotIndex); //Feed-forward
    canPidController.setP(pidParameters.kP, pidSlotIndex);
    canPidController.setI(pidParameters.kI, pidSlotIndex);
    canPidController.setD(pidParameters.kD, pidSlotIndex);

    canPidController.setOutputRange(-pidParameters.kPeakOutput, pidParameters.kPeakOutput);
    configAllowableClosedloopError(pidSlotIndex, pidParameters.errorTolerance, 30);
  }
    
}
