package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.PidParameters;

import java.util.ArrayList;
import java.util.List;

/**
 * A wrapper class for Motors that helps to consistently and easily perform the following functions:
 *   -Keep current and max speeds
 *   -Get and Reset encoder values
 *   -Lots and lots of SmartDashboard information
 */
public class TeamTalonSRX extends WPI_TalonSRX {
    protected final String smartDashboardPrefix;

    protected int numEStops=0;

    protected long previousMeasurementTime_ms;
    protected long previousEncoderValue = Long.MAX_VALUE;

    protected long currentEncoderChange_perLoop = 0;
    protected long maxEncoderChange_perLoop = Long.MAX_VALUE;

    protected long currentEncoderChange_perSec = 0;
    protected long maxEncoderChange_perSec = Long.MAX_VALUE;

    protected PidParameters pidProfiles[] = new PidParameters[4];


    public TeamTalonSRX(String smartDashboardPrefix, int deviceNumber) {
        super(deviceNumber);
        this.smartDashboardPrefix = smartDashboardPrefix;
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
        long currentMeasurementTime_ms = System.currentTimeMillis();
        long currentEncoderValue = getCurrentEncoderValue();

        // Skip the delta math the first time through the loop.
        if ( previousEncoderValue != Long.MAX_VALUE ) {
            double deltaTime = 1.0*(currentMeasurementTime_ms - previousMeasurementTime_ms)/1000;
            currentEncoderChange_perLoop = currentEncoderValue - previousEncoderValue;
            currentEncoderChange_perSec = Math.round(currentEncoderChange_perLoop/deltaTime);

            if ( maxEncoderChange_perSec ==Long.MAX_VALUE || currentEncoderChange_perSec > maxEncoderChange_perSec)
                maxEncoderChange_perSec = currentEncoderChange_perSec;
            if ( maxEncoderChange_perLoop ==Long.MAX_VALUE || currentEncoderChange_perLoop > maxEncoderChange_perLoop)
                maxEncoderChange_perLoop = currentEncoderChange_perLoop;
        }
        previousMeasurementTime_ms = currentMeasurementTime_ms;
        previousEncoderValue = currentEncoderValue;
        previousEncoderValue = currentEncoderValue;

        if ( isRunningPidControlMode() ) {
            SmartDashboard.putBoolean(smartDashboardPrefix+".PID", true);
        } else {
            SmartDashboard.putBoolean(smartDashboardPrefix+".PID", false);
        }

        SmartDashboard.putNumber(smartDashboardPrefix + ".PowerPercent", getMotorOutputPercent());

        SmartDashboard.putNumber(smartDashboardPrefix + ".Position-ticks", currentEncoderValue);
        SmartDashboard.putNumber(smartDashboardPrefix + ".Position-changePerLoop", currentEncoderChange_perLoop);
        SmartDashboard.putNumber(smartDashboardPrefix + ".Position-changePerSecond", currentEncoderChange_perSec);
        SmartDashboard.putNumber(smartDashboardPrefix + ".Position-changePer100ms", currentEncoderChange_perSec/10);
        SmartDashboard.putNumber(smartDashboardPrefix + ".Position-maxChangePerLoop", maxEncoderChange_perLoop);
        SmartDashboard.putNumber(smartDashboardPrefix + ".Position-maxChangePerSecond", maxEncoderChange_perSec);
        SmartDashboard.putNumber(smartDashboardPrefix + ".Position-maxChangePer100ms", maxEncoderChange_perSec/10);

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
