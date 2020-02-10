/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;


import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class HorizontalIndexerSubsystem extends SubsystemBase {
  /**
   * Creates a new VerticalIndexerSubsystem.
   */
  private final VictorSP horizontalIndexer;

  private final double intakeSpeed = .5;
  private final double outtakeSpeed = -.5;
  public HorizontalIndexerSubsystem() {
    horizontalIndexer = new VictorSP(Constants.IndexerVertPWM);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void intake(){
      horizontalIndexer.set(intakeSpeed);
  }
  public void outtake(){
      horizontalIndexer.set(outtakeSpeed);
  }
  public void stop(){
      horizontalIndexer.set(0);
  }

}
