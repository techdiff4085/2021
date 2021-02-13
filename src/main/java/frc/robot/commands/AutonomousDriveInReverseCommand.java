package frc.robot.commands;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

/**
 * An example command that uses an example subsystem.
 */
public class AutonomousDriveInReverseCommand extends PIDCommand {
  private final DriveSubsystem m_subsystem;
  private double m_distance;
  private AHRS m_navX;
 //
  //private final Encoder encoderR;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AutonomousDriveInReverseCommand(DriveSubsystem subsystem, AHRS navX, double distance, double turnToAngle ) {
    super(
      new PIDController(Constants.KP, Constants.KI, Constants.KD),  //controller that controls the output 
      navX::getAngle, //measurement source
      turnToAngle, // target angle degrees
      output -> AutonomousDriveInReverseCommand.driveRobot(subsystem, navX, output, turnToAngle),
      subsystem);

    // Set the controller to be continuous (because it is an angle controller)
    getController().enableContinuousInput(-360, 360);
    m_navX = navX;

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

  private static void driveRobot(DriveSubsystem subsystem, AHRS navX, double output, double turnToAngle){
    double difference = (navX.getAngle() - turnToAngle)/180;    
    System.out.println("Turning " + turnToAngle + ": difference "  + difference + " : yaw " + navX.getYaw() + ":" + output);
    //System.out.println(navX.getYaw() + ":" + difference + ":" + output);

    subsystem.drive(-1*(-0.65 + difference),-1 * ( -0.65 - difference)); //how to use the output of the navX
  }

}
