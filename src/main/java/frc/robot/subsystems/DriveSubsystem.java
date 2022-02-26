package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {
  /**
   * Creates a new ExampleSubsystem.
   */
  WPI_VictorSPX FRmotor = new WPI_VictorSPX(Constants.RIGHT_FRONT_DRIVE);
  WPI_VictorSPX BRmotor = new WPI_VictorSPX(Constants.RIGHT_BACK_DRIVE);
  WPI_VictorSPX FLmotor = new WPI_VictorSPX(Constants.LEFT_FRONT_DRIVE);
  WPI_VictorSPX BLmotor = new WPI_VictorSPX(Constants.LEFT_BACK_DRIVE);
  public SpeedControllerGroup leftMotors = new SpeedControllerGroup(FLmotor, BLmotor);
  public SpeedControllerGroup rightMotors = new SpeedControllerGroup(FRmotor, BRmotor);
  DifferentialDrive m_drive = new DifferentialDrive(leftMotors, rightMotors);
  public Encoder encoderL = new Encoder(5, 6, false);
  DigitalInput StopDrive = new DigitalInput(Constants.Stop_Drive_LS);
  AHRS m_navX;

  // Encoder driveEncoder1LF = new Encoder(Constants.Encode_Port1,
  // Constants.Encode_Port2, false, Encoder.EncodingType.k2X);
  // Encoder driveEncoder3RF = new Encoder(Constants.Encode_Port3,
  // Constants.Encode_Port4, false, Encoder.EncodingType.k2X);

  public DriveSubsystem() {
    FRmotor.configOpenloopRamp(0);
    FLmotor.configOpenloopRamp(0);
    BRmotor.configOpenloopRamp(0);
    BLmotor.configOpenloopRamp(0);
    FRmotor.setNeutralMode(NeutralMode.Brake);
    BRmotor.setNeutralMode(NeutralMode.Brake);
    FLmotor.setNeutralMode(NeutralMode.Brake);
    BLmotor.setNeutralMode(NeutralMode.Brake);
    
    leftMotors.setInverted(true);
    rightMotors.setInverted(true);
    encoderL.setDistancePerPulse(1 / 20.0);
    encoderL.reset();
  }

  public void drive(double rSpeed, double lSpeed) {
    //System.out.println(m_navX.getYaw());
    m_drive.tankDrive(lSpeed, rSpeed);
  }
// used when only one joystick is used to control the robot
  public void arcadeDrive(double verticalSpeed, double rotationalSpeed) {
    m_drive.arcadeDrive(verticalSpeed, rotationalSpeed, true);
  }

  public void turnInPlace(double speed) {
    m_drive.arcadeDrive(0, speed);
  }

  public double getDistance() {
    return encoderL.getDistance();
  }

  public void resetEncoder() {
    encoderL.reset();
  }

  public void setNavX(AHRS navX){
    m_navX = navX;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public boolean driveStopLimitSwitch() {
    //System.out.println("isDriveStopped()");
    if (this.StopDrive.get()) {
      m_drive.tankDrive(0, 0);
      return true;
    } else {
      return false;
    }
  }
}
