package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class LimitSubsystem extends SubsystemBase {
  /**
   * Creates a new LimitSubsystem.
   */
  final SpeedController climbTelescopeMotor = new Victor(5);
  final SpeedController climbArmMotor = new Victor(4);
  final SpeedController wheelArmMotor = new Victor(3);
  final SpeedController wheelSpinMotor = new Victor(2);
  final DigitalInput limitSwitch = new DigitalInput(1);
  final Counter counter = new Counter(limitSwitch);

  public LimitSubsystem() {
    
    
  }

  public void wheelUp() {
    
    wheelSpinMotor.set(0);
}

  public void wheelDown() {
    
  wheelArmMotor.set(0);
}

public void climbReleased() {
  climbArmMotor.set(0);
}

public void climbUp() {
  climbTelescopeMotor.set(0);
}

public void climbDown() {
  climbTelescopeMotor.set(0);
}
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
