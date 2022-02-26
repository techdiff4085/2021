/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;;

public class ElevatorSubsystem extends SubsystemBase {
  /**
   * Creates a new ExampleSubsystem.
   */
  WPI_VictorSPX eleMotors = new WPI_VictorSPX(Constants.PICKUP);
  //WPI_TalonFX eleMotors = new WPI_TalonFX(Constants.PICKUP);


  public ElevatorSubsystem() {

  }

  public void elevate(){
    eleMotors.set(1.0);
  }
  public void reverse(){
    eleMotors.set(-1.0);
  }

  public void stopElevate(){
    eleMotors.set(0.1);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

public boolean isDown() {
	return false;
}

public boolean isUp() {
	return false;
}
}
