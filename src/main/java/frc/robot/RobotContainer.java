/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;



import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.AutonomousDriveCommand;
/*import frc.robot.commands.ColorWheelLeftCommand;
import frc.robot.commands.ColorWheelRightCommand;
import frc.robot.commands.ColorWheelStopCommand;*/

import frc.robot.commands.DriveCommand;
import frc.robot.commands.ElevatorCommand;
//import frc.robot.commands.ElevatorStopCommand;
//import frc.robot.commands.ExampleCommand;
import frc.robot.commands.IndexCommand;
import frc.robot.commands.IndexStopCommand;
import frc.robot.commands.ShootByDistanceCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
//import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.commands.AimCommand;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  //public static ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  public static ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
  //public static ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);
  public static ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();
  public static IndexSubsystem m_indexSubsystem = new IndexSubsystem();

  DriveSubsystem driveSubsystem = new DriveSubsystem();
  Joystick leftJoystick = new Joystick(0);
  Joystick rightJoystick = new Joystick(1);
  Joystick shooterJoystick = new Joystick(2);

  DriveCommand driveCommand = new DriveCommand(driveSubsystem, leftJoystick, rightJoystick);

  JoystickButton shootButton = new JoystickButton(shooterJoystick, Constants.SHOOT_BUTTON);
  JoystickButton stopShootButton = new JoystickButton(shooterJoystick, Constants.STOP_SHOOT_BUTTON);
  JoystickButton elevatorUpButton = new JoystickButton(rightJoystick, Constants.ELEVATOR_ON_BUTTON);
  JoystickButton elevatorDownButton = new JoystickButton(rightJoystick, Constants.ELEVATOR_OFF_BUTTON);
  JoystickButton elevatorReverseButton = new JoystickButton(rightJoystick, Constants.ELEVATOR_REVERSE_BUTTON);
  JoystickButton indexOnButton = new JoystickButton(shooterJoystick, Constants.INDEX_ON_BUTTON);
  JoystickButton wheelUpButton = new JoystickButton(shooterJoystick, Constants.ARM_UP_BUTTON);
  JoystickButton wheelDownButton = new JoystickButton(shooterJoystick, Constants.ARM_DOWN_BUTTON);
  //JoystickButton wheelFindButton = new JoystickButton(shooterJoystick, Constants.WHEEL_FIND_BUTTON);
  JoystickButton wheelSpinButton = new JoystickButton(shooterJoystick, Constants.WHEEL_SPIN_BUTTON);
  JoystickButton wheelStopButton = new JoystickButton(shooterJoystick, Constants.WHEEL_STOP_BUTTON);
  JoystickButton openClimbButton = new JoystickButton(shooterJoystick, Constants.OPEN_CLIMB_BUTTON);
  JoystickButton aimButton = new JoystickButton(shooterJoystick, Constants.AIM_BUTTON);
  JoystickButton shootDistance = new JoystickButton(shooterJoystick, Constants.DISTANCE_SHOOT);



  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure Limelight to stream both cameras
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("stream").setNumber(1);

    // Configure the button bindings
    configureButtonBindings();
    driveSubsystem.setDefaultCommand(driveCommand);
    
  }
 

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    
    JoystickButton ledOn = new JoystickButton(shooterJoystick, 8);
    
    Command alterLight = new ConditionalCommand(
      new InstantCommand(()->Limelight.getInstance().setLightState(Limelight.LightMode.ON)), 
      new InstantCommand(()->Limelight.getInstance().setLightState(Limelight.LightMode.OFF)), 
      ()->Limelight.getInstance().getLightState() == Limelight.LightMode.OFF);

    ledOn.whenPressed(alterLight);

    shootButton.whenPressed(new ShootCommand(m_shooterSubsystem, -0.58));// originally -0.75
    stopShootButton.whenPressed(new ShootCommand(m_shooterSubsystem, 0));
    elevatorUpButton.whenPressed(new ElevatorCommand(m_elevatorSubsystem,"On"));
    elevatorDownButton.whenPressed(new ElevatorCommand(m_elevatorSubsystem,"Off"));
    elevatorReverseButton.whenPressed(new ElevatorCommand(m_elevatorSubsystem,"Reverse"));
    //elevatorButton.whenReleased(new ElevatorStopCommand(m_elevatorSubsystem));
    indexOnButton.whenPressed(new IndexCommand(m_indexSubsystem, m_shooterSubsystem));
    indexOnButton.whenReleased(new IndexStopCommand(m_indexSubsystem, m_shooterSubsystem));
   // wheelFindButton.whenPressed(new ColorWheelFindCommand(m_colorWheelSubsystem, m_colorSensor));
    aimButton.whileHeld(new AimCommand(driveSubsystem, m_indexSubsystem, m_shooterSubsystem));
    shootDistance.whileHeld(m_shooterSubsystem::shootByDistance, m_shooterSubsystem);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    // This code works when the front left corner of the robot is 8'6" from the wall.
    //Limelight.getInstance().setLightState(Limelight.LightMode.ON);
System.out.println("in autonomous");
    return new SequentialCommandGroup(

      //Move the robot
      new AutonomousDriveCommand(driveSubsystem, 650)
      
      //if anything goes wrong its Grants fault
    ).withTimeout(15);
    
  }
}
