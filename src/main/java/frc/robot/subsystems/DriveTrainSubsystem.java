/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class DriveTrainSubsystem extends SubsystemBase {
  /**
   * Creates a new DriveTrainSubsystem.
   */
  final WPI_TalonFX Right_Drive_Falcon_Main;
  final WPI_TalonFX Left_Drive_Falcon_Main;
  final WPI_TalonFX Right_Drive_Falcon_Sub;
  final WPI_TalonFX Left_Drive_Falcon_Sub;

  Encoder rightEncoder;
  Encoder leftEncoder;

  public static final double Max_Power_Change = 0.1;


  public DriveTrainSubsystem() {
    setDefaultCommand(new DriveTrainCommands.TankDrive());
    rightEncoder = new Encoder(Constants.Right_Encoder, Constants.Right_Encoder2, false);
    leftEncoder = new Encoder (Constants.Left_Encoder, Constants.Left_Encoder2, false);

    Right_Drive_Falcon_Main = new WPI_TalonFX(Constants.Right_Drive_Main_Falcon);
    Right_Drive_Falcon_Sub = new WPI_TalonFX(Constants.Right_Drive_Sub_Falcon);
    Left_Drive_Falcon_Main = new WPI_TalonFX(Constants.Left_Drive_Main_Falcon);
    Left_Drive_Falcon_Sub = new WPI_TalonFX(Constants.Left_Drive_Sub_Falcon);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
