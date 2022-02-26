/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.Limelight;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * An example command that uses an example subsystem.
 */
public class AimCommand extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final DriveSubsystem m_driveSubsystem;
  private final IndexSubsystem m_indexSubsystem;
  private final ShooterSubsystem m_shooterSubsystem;

  private final PIDController m_controller;

  private boolean targetNotFound = false;
  private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");


  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AimCommand(DriveSubsystem dSubsystem, IndexSubsystem iSubsystem, ShooterSubsystem sSubsystem) {
    m_driveSubsystem = dSubsystem;
    m_indexSubsystem = iSubsystem;
    m_shooterSubsystem = sSubsystem;
  
    m_controller = new PIDController(Constants.PID.Aim.KP, Constants.PID.Aim.KI, Constants.PID.Aim.KD);

    m_controller.setTolerance(Constants.PID.Aim.POS_TOL);//, Constants.PID.Aim.VEL_TOL);
    m_controller.setSetpoint(0);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_driveSubsystem, m_indexSubsystem, m_shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Limelight.getInstance().setPipeline(0);
    Limelight.getInstance().setLightState(Limelight.LightMode.ON);
    m_controller.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Limelight.getInstance().setLightState(Limelight.LightMode.ON);
    double error = Limelight.getInstance().getXAngle();
    double speed = m_controller.calculate(error);
    speed += Math.copySign(Constants.PID.Aim.FEED_FORWARD, speed);
    if(Limelight.getInstance().hasValidTarget() && !m_controller.atSetpoint()) {
      m_driveSubsystem.turnInPlace(speed);
    }
    else {
      m_driveSubsystem.drive(0, 0);
    }

      // LimelightValues values = new LimelightValues(table);

      // // Target is seen 
      // if(values.getV() == 1) {
      //     if(values.getX() < 0) {
      //       m_driveSubsystem.drive(-.65, .65);
      //     } else {
      //       m_driveSubsystem.drive(.65, -.65);
      //     }
      // } else {
      //     targetNotFound = true;
      // }
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      //if(interrupted)
        //Limelight.getInstance().setLightState(Limelight.LightMode.OFF);
      m_driveSubsystem.drive(0.0, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //System.out.println("AtSetpoint: " + m_controller.atSetpoint() + " Angle: " + Limelight.getInstance().getXAngle());
    return m_controller.atSetpoint();
    // LimelightValues values = new LimelightValues(table);

    // System.out.println("X is " + values.getX());
    // System.out.println("A is " + values.getA());
    // System.out.println("V is " + values.getV());

    // if (values.getV() == 1) {          // found a target
    //     if  (values.getX() > -1.0 && values.getX() < 0) {//move right  
    //         //Limelight.getInstance().setLightState(Limelight.LightMode.OFF);
    //         return true;
    //     }
    //     else if  (values.getX() < 1.0 && values.getX() > 0.0) {//move left
    //        //Limelight.getInstance().setLightState(Limelight.LightMode.OFF);
    //         return true;
    //     } 
    // } else if(targetNotFound == true) {
    //     return true;
    // }
    
    // // No target found - keep looking
    // return false;
  }

  private class LimelightValues {
      private double x;
      private double a;
      private double v;

      public LimelightValues(NetworkTable table) {
        NetworkTableEntry tx = table.getEntry("tx");
        NetworkTableEntry ta = table.getEntry("ta");
        NetworkTableEntry tv = table.getEntry("tv");
    
        this.x = tx.getDouble(0.0);
        this.a = ta.getDouble(0.0);
        this.v = tv.getDouble(0.0);
      }

      public double getX() {
          return x;
      }

      public double getA() {
          return a;
      }

      public double getV() {
          return v;
      }
  }
}
