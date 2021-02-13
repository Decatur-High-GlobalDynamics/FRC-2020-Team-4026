/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.Objects;
import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Ports;
import frc.robot.TeamTalonSRX;

public class VerticalIndexerSubsystem extends SubsystemBase {
  private final TeamTalonSRX verticalIndexer;
  private final DigitalInput bottomSwitchA;
  private final DigitalInput bottomSwitchB;
  private final DigitalInput bottomSwitchC;
  private final DigitalInput middleSwitch;
  private final DigitalInput topSwitch;
  private final Timer idleTimerSeconds;

  public int ticksUntilTransfered = 6500;
  private String currCmd;
  private final double upSpeed = .35;
  private final double downSpeed = -.35;
  private double epsilonIdleTime = 1;

  public VerticalIndexerSubsystem() {
    throw new IllegalArgumentException(
        "not allowed! ctor must provide parameters for all dependencies");
  }

  public VerticalIndexerSubsystem(
      TeamTalonSRX verticalIndexer,
      DigitalInput bottomSwitchA,
      DigitalInput bottomSwitchB,
      DigitalInput bottomSwitchC,
      DigitalInput middleSwitch,
      DigitalInput topSwitch,
      Timer idleTimerSeconds) {
    this.verticalIndexer =
        Objects.requireNonNull(verticalIndexer, "verticalIndexer must not be null");
    this.bottomSwitchA = Objects.requireNonNull(bottomSwitchA, "bottomSwitchA must not be null");
    this.bottomSwitchB = Objects.requireNonNull(bottomSwitchB, "bottomSwitchB must not be null");
    this.bottomSwitchC = Objects.requireNonNull(bottomSwitchC, "bottomSwitchC must not be null");
    this.middleSwitch = Objects.requireNonNull(middleSwitch, "middleSwitch must not be null");
    this.topSwitch = Objects.requireNonNull(topSwitch, "topSwitch must not be null");
    this.idleTimerSeconds =
        Objects.requireNonNull(idleTimerSeconds, "idleTimerSeconds must not be null");

    this.idleTimerSeconds.reset();
    this.idleTimerSeconds.start();
  }

  public static VerticalIndexerSubsystem Create() {
    TeamTalonSRX verticalIndexer =
        new TeamTalonSRX("Subsystems.VerticalIndexer.VIndxMotor", Ports.IndexerVertCAN);
    DigitalInput bottomSwitchA = new DigitalInput(Ports.VerticalIndexer_BottomLimit_DIO_A);
    DigitalInput bottomSwitchB = new DigitalInput(Ports.VerticalIndexer_BottomLimit_DIO_B);
    DigitalInput bottomSwitchC = new DigitalInput(Ports.VerticalIndexer_BottomLimit_DIO_C);
    DigitalInput middleSwitch = new DigitalInput(Ports.VerticalIndexer_MiddleLimit_DIO);
    DigitalInput topSwitch = new DigitalInput(Ports.VerticalIndexer_TopLimit_DIO);
    Timer idleTimerSeconds = new Timer();
    return new VerticalIndexerSubsystem(
        verticalIndexer,
        bottomSwitchA,
        bottomSwitchB,
        bottomSwitchC,
        middleSwitch,
        topSwitch,
        idleTimerSeconds);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    verticalIndexer.periodic();
    SmartDashboard.putBoolean("Subsystems.VerticalIndexer.TopSwitch", topSwitchIsPressed());
    SmartDashboard.putBoolean("Subsystems.VerticalIndexer.MiddleSwitch", middleSwitchIsPressed());
    SmartDashboard.putBoolean("Subsystems.VerticalIndexer.BottomSwitch", bottomSwitchIsPressed());
    ticksUntilTransfered =
        (int)
            SmartDashboard.getNumber(
                "Subsystems.VerticalIndexer.TicksUntilTransferred", ticksUntilTransfered);
    SmartDashboard.putNumber(
        "Subsystems.VerticalIndexer.TicksUntilTransferred", ticksUntilTransfered);
    SmartDashboard.putNumber(
        "Subsystems.VerticalIndexer.currentPosition", verticalIndexer.getSelectedSensorPosition());
    epsilonIdleTime =
        SmartDashboard.getNumber("Subsystems.DriveTrain.epsilonIdleTime", epsilonIdleTime);
    SmartDashboard.putNumber("Subsystems.DriveTrain.epsilonIdleTime", epsilonIdleTime);
    if (this.getCurrentCommand() == null) {
      currCmd = "null";
    } else {
      currCmd = this.getCurrentCommand().toString();
    }
    SmartDashboard.putString("Subsystems.VerticalIndexer.currCommand", currCmd);

    if (topSwitchIsPressed() || verticalIndexer.get() == 0) {
      idleTimerSeconds.reset();
    }
  }

  public boolean isIdleForSeconds(double maxIdleSeconds) {
    return idleTimerSeconds.get() > maxIdleSeconds;
  }

  public boolean isIdle() {
    return idleTimerSeconds.get() > epsilonIdleTime;
  }

  public boolean topSwitchIsPressed() {
    // Negative because of opposite switch polarity
    return !topSwitch.get();
  }

  public boolean middleSwitchIsPressed() {
    // Negative because of opposite switch polarity
    return !middleSwitch.get();
  }

  public boolean bottomSwitchIsPressed() {
    // Negative because of opposite switch polarity

    // The bottomSwitchB is wired backwards because the other half
    // of the limit switch is broken!!
    return !bottomSwitchA.get() || bottomSwitchB.get() || !bottomSwitchC.get();
  }

  public double getPosition() {
    return Math.abs(verticalIndexer.getSelectedSensorPosition());
  }

  public void up() {
    verticalIndexer.set(ControlMode.PercentOutput, upSpeed);
  }

  public void down() {
    verticalIndexer.set(ControlMode.PercentOutput, downSpeed);
  }

  public void stop() {
    verticalIndexer.set(0);
  }
}
