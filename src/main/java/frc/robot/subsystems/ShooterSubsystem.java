
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Limelight;


public class ShooterSubsystem extends PIDSubsystem {
  /**
   * Creates a new ExampleSubsystem.
   */
  WPI_TalonFX m_shootMotor = new WPI_TalonFX (Constants.SHOOTER);
  WPI_VictorSPX m_indexMotor = new WPI_VictorSPX(Constants.INDEX);
  private double shootSpeed;

  public ShooterSubsystem() {
    
    super(new PIDController(Constants.KP, Constants.KI, Constants.KD));
    m_shootMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 10);

    //wheelSpins per 100 ms = RawEncoderUnits per 100 ms / RawEncoderUnitsPerRotation
    //eleMotors.getSelectedSensorVelocity(0) / kUnitsPerRotation;
    m_shootMotor.configOpenloopRamp(0);
    //this.disable();
  }

  public void shoot(double shootSpeed) {
    this.shootSpeed = shootSpeed;
    m_shootMotor.set(this.shootSpeed);
  }
                                                                                                  
  public void shootByDistance() {
    shoot(-0.5);
    /*
    System.out.println("Inside shoot by distance");
    if (!Limelight.getInstance().hasValidTarget()) {
      shoot(-0.45);
      System.out.println("default shoot. target not found");
    } else {
        double distance = Limelight.getInstance().getDistance();
        //Logic to correlate distance with shoot speed
        if (distance >= 48 && distance < 60) {
          shoot(0);
          System.out.println("A");
        } else if (distance >= 60 && distance < 96) {
            shoot(-.57);
            System.out.println("B");
        } else if (distance >= 96 && distance < 168) {
            shoot(-.51);
            System.out.println("C");
            
        } else if (distance >= 168 && distance < 180) {
            shoot(-.565);
            System.out.println("D");
        } else if (distance >= 180 && distance < 230) {
          // add code to move robot to the left  
          shoot(-.57);
            System.out.println("E");
        } else {
            shoot(-0.6);
            System.out.println("Stop Callie, too far. Bad choice");
        }
      }
      */
  }

  public void stopShooter() {
    m_shootMotor.set(0);
  }

  @Override
  protected void useOutput(double output, double setpoint) {
    m_shootMotor.setVoltage(output);
  }

  public double getShootRPM() { 
    //wheelSpins per 100 ms = RawEncoderUnits per 100 ms / RawEncoderUnitsPerRotation
    return -600 * (m_shootMotor.getSelectedSensorVelocity(0)) / 2048;
  }

  public boolean isShootRpmReady() {
    if (getShootRPM() > ((-6380 * shootSpeed) - 100) && getShootRPM() < ((-6380 * shootSpeed) + 100)) {
      return true;
    } else {
      return false;
    }
  }

  @Override
  protected double getMeasurement() {
    // TODO Auto-generated method stub
    return 0;
  }


}
