/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.Limelight;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * An example command that uses an example subsystem.
 */
public class IndexStopCommand extends CommandBase {
    private String m_extendType;
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private final IndexSubsystem m_subsystem;
    private final ShooterSubsystem m_shootSubsystem;

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public IndexStopCommand(IndexSubsystem subsystem, ShooterSubsystem shootSubsystem) {
    this.m_subsystem = subsystem;
    this.m_shootSubsystem = shootSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //m_shooterEncoder.setUpSource(0);
    //m_shooterEncoder.setSemiPeriodMode(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
     //if (m_shootSubsystem.isShootRpmReady()) {
        this.m_subsystem.stopMotor();
      //} else {
      //  this.m_subsystem.stopMotor();
      //}

  }
  @Override
  public void end(boolean interrupted) {
    this.m_subsystem.stopMotor();
  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
      return false;
  }
}
