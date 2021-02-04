package frc.robot.commands;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

/**
 * An example command that uses an example subsystem.
 */
public class AutonomousDriveCommand extends PIDCommand {
  private final DriveSubsystem m_subsystem;
  private double m_distance;
 //
  //private final Encoder encoderR;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AutonomousDriveCommand(DriveSubsystem subsystem, AHRS navX, double distance) {
    super(
      new PIDController(Constants.KP, Constants.KI, Constants.KD),  //controller that controls the output 
      navX::getYaw, //measurement source
      0, // target angle degrees
      output -> subsystem.drive(output, -output), //how to use the output of the navX
      subsystem);

    // Set the controller to be continuous (because it is an angle controller)
    getController().enableContinuousInput(-180, 180);

    /*
    // Set the controller tolerance - the delta tolerance ensures the robot is stationary at the
    // setpoint before it is considered as having reached the reference
    getController()
        .setTolerance(DriveConstants.kTurnToleranceDeg, DriveConstants.kTurnRateToleranceDegPerS);
    */
    
    m_distance = distance;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);

    m_subsystem = subsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_subsystem.resetEncoder();
  }

 

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //System.out.println(m_subsystem.getDistance());
    if (Math.abs(m_subsystem.getDistance()) < m_distance){
      return false; 
    } else {
      return true;
    }
  }
}
