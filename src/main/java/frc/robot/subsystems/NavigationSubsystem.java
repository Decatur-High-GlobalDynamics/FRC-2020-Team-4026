/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class NavigationSubsystem extends SubsystemBase {
    private double robotX, robotZ, robotHeading;
    //private int VisionPI; //Change this to whatever type is needed



  /**
   * Creates a new NavigationSubsystem.
   */
  public NavigationSubsystem() {
    //VisionPI = Constants.VisionPI;
    //Change this to whatever constructor is nescessary
    robotX = 0;
    robotZ = 0;
    robotHeading = 0;
    //Change these to actual values
  }

  public double getX(){
    return robotX;
  }
  public double getY(){
    return robotZ;
  }
  public double getHeading(){
    return robotHeading;
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    /*
    robotX = VisionPI.get(x);
    robotZ = VisionPI.get(z);
    robotHeading = VisionPI.get(heading);

    Change to whatever protocol is needed
    */
  }
}
