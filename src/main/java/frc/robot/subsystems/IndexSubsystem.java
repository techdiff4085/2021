/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IndexSubsystem extends SubsystemBase {

    WPI_VictorSPX indexMotors = new WPI_VictorSPX(Constants.INDEX);
    
  /**
   * Creates a new ExampleSubsystem.
   */
  public IndexSubsystem() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //System.out.println("Shooter Encoder Value [" + encoder.getRate() + "]");
      }

  public void startMotor(){
    indexMotors.set(1.0);
  }

  public void stopMotor() {
    indexMotors.stopMotor();
  }
}

