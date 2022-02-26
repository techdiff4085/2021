package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
//import frc.robot.subsystems.ColorArmSubsystem;

/**
 * An example command that uses an example subsystem.
 */
public class DriveCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveSubsystem m_subsystem;
  //private final ColorArmSubsystem colorarmsubsystem;
  private final Joystick leftJoystick;
  private final Joystick rightJoystick;
  private boolean toggleOn = false;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  //public DriveCommand(DriveSubsystem subsystem, ColorArmSubsystem subsystem2, Joystick lJoystick, Joystick rJoystick) {
    public DriveCommand(DriveSubsystem subsystem, Joystick lJoystick, Joystick rJoystick) {
    m_subsystem = subsystem;
    leftJoystick = lJoystick;
    rightJoystick = rJoystick;
    //this.colorarmsubsystem = subsystem2;

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
      double leftAxis = leftJoystick.getY();
      double rightAxis = rightJoystick.getY();
      double xboxVertical = leftJoystick.getRawAxis(1);
      double xboxHorizontal = leftJoystick.getRawAxis(4);
      m_subsystem.drive(rightAxis*Math.abs(rightAxis)/1.1, leftAxis*Math.abs(leftAxis)/1.1);
      //m_subsystem.arcadeDrive(xboxVertical, -xboxHorizontal);

      if (toggleOn) {
        m_subsystem.leftMotors.setInverted(false);
        m_subsystem.rightMotors.setInverted(false);
      }
      else {
        m_subsystem.leftMotors.setInverted(true);
        m_subsystem.rightMotors.setInverted(true);
      }
      if(leftJoystick.getRawButton(2)){
        toggleOn = !toggleOn;
      }
  }



  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
    
  }
}
