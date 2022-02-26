/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.commands;

import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.Button;

/**
 * An example command that uses an example subsystem.
 */
public class ElevatorCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ElevatorSubsystem m_subsystem;
  private final String extendType;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ElevatorCommand(ElevatorSubsystem subsystem, String extendType) {
    m_subsystem = subsystem;
    this.extendType = extendType;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (extendType.equals("On")){
      this.m_subsystem.elevate();
    }

    if (extendType.equals("Off")) {
      this.m_subsystem.stopElevate();
    }
    if (extendType.equals("Reverse")){
      this.m_subsystem.reverse();
    }

  }
 
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
        return true;
  }
}