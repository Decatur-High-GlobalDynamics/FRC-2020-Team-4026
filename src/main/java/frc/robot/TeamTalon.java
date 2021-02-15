package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;

public interface TeamTalon {
    public static boolean isPidControlMode(ControlMode mode) {
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

    public default boolean isRunningPidControlMode() {
        return TeamTalon.isPidControlMode(getControlMode());
    }

    public default void noteEmergencyStop() {
        numEStops++;
    }

    public long getCurrentEncoderValue();

    public void resetEncoder();

    public default void periodic() {
        double now = TeamUtils.getCurrentTime();

        if ((now - lastTelemetryUpdate) < telemetryUpdateInterval_secs) {
        return;
        }

        lastTelemetryUpdate = now;

        long currentEncoderValue = getCurrentEncoderValue();
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
            SmartDashboard.putNumber(smartDashboardPrefix + "Target", getClosedLoopTarget(0));
            SmartDashboard.putNumber(smartDashboardPrefix + "Error", getClosedLoopError(0));
            break;
        default:
            // Fill in Zeros when we're not in a mode that is using it
            SmartDashboard.putNumber(smartDashboardPrefix + "Target", 0);
            SmartDashboard.putNumber(smartDashboardPrefix + "Error", 0);
        }
    };

    public double getVelocityError()
    {
        if (getControlMode() != ControlMode.Velocity) {
            return 0;
        }
        double currentSpeed = getSelectedSensorVelocity();
        return (getClosedLoopTarget() - currentSpeed);
    }

    public void configureWithPidParameters(PidParameters pidParameters, int pidSlotIndex) {
        pidProfiles[pidSlotIndex] = pidParameters;

        config_kF(pidSlotIndex, pidParameters.kF);
        config_kP(pidSlotIndex, pidParameters.kP);
        config_kI(pidSlotIndex, pidParameters.kI);
        config_kD(pidSlotIndex, pidParameters.kD);
        configPeakOutputForward(pidParameters.kPeakOutput);
        configPeakOutputReverse(-pidParameters.kPeakOutput);
        configAllowableClosedloopError(pidSlotIndex, pidParameters.errorTolerance, 30);
    }
}
