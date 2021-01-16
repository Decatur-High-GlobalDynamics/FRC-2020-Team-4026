package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * A wrapper class for Motors that helps to consistently and easily perform the following functions:
 *   -Keep current and max speeds
 *   -Get and Reset encoder values
 *   -Lots and lots of SmartDashboard information
 */
public class TeamTalonSRX extends WPI_TalonSRX {
    public static double telemetryUpdateInterval_secs = 0.0;
    private double lastTelemetryUpdate=0;


    protected final String smartDashboardPrefix;

    protected int numEStops=0;

    protected double maxSpeed = Double.MAX_VALUE;


    protected PidParameters pidProfiles[] = new PidParameters[4];


    public TeamTalonSRX(String smartDashboardPrefix, int deviceNumber) {
        super(deviceNumber);
        this.smartDashboardPrefix = smartDashboardPrefix;
        //assuming quadencoder
        this.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
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


    public long getCurrentEncoderValue() {
        // This should be configurable
        return getSensorCollection().getQuadraturePosition();
    }

    public void resetEncoder() {
        getSensorCollection().setQuadraturePosition(0, 0);
    }

    public void periodic() {
        double now = TeamUtils.getCurrentTime();

        if ( (now-lastTelemetryUpdate) < telemetryUpdateInterval_secs ) {
            return;
        }

        lastTelemetryUpdate = now;

        long currentEncoderValue = getCurrentEncoderValue();
        double currentSpeed = getSelectedSensorVelocity();

        if ( maxSpeed == Double.MAX_VALUE || currentSpeed>maxSpeed)
            maxSpeed = currentSpeed;


        if ( isRunningPidControlMode() ) {
            SmartDashboard.putBoolean(smartDashboardPrefix+".PID", true);
        } else {
            SmartDashboard.putBoolean(smartDashboardPrefix+".PID", false);
        }

        if ( getControlMode() == ControlMode.Velocity ) {
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

    }


    public double getVelocityError() {
        if (getControlMode() != ControlMode.Velocity){
            return 0;
        }
        double currentSpeed = getSelectedSensorVelocity();
        return (double) (getClosedLoopTarget() - currentSpeed);
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
