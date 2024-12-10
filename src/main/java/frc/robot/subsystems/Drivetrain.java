// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive.WheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.xrp.XRPMotor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.g;

public class Drivetrain extends SubsystemBase {

  // The XRP has the left and right motors set to
  // channels 0 and 1 respectively
  private final XRPMotor m_leftMotor = new XRPMotor(0);
  private final XRPMotor m_rightMotor = new XRPMotor(1);

  // The XRP has onboard encoders that are hardcoded
  // to use DIO pins 4/5 and 6/7 for the left and right
  private final Encoder m_leftEncoder = new Encoder(4, 5);
  private final Encoder m_rightEncoder = new Encoder(6, 7);

  // Set up the BuiltInAccelerometer
  private final BuiltInAccelerometer m_accelerometer = new BuiltInAccelerometer();

  /** Creates a new Drivetrain. */
  public Drivetrain() {


    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightMotor.setInverted(true);

    // Use inches as unit for encoder distances
    m_leftEncoder.setDistancePerPulse((Math.PI * g.DRIVETRAIN.WHEEL_DIAMETER_inch) / g.DRIVETRAIN.WHEEL_COUNTS_PER_REV);
    m_rightEncoder.setDistancePerPulse((Math.PI * g.DRIVETRAIN.WHEEL_DIAMETER_inch) / g.DRIVETRAIN.WHEEL_COUNTS_PER_REV);
    resetEncoders();
  }

  public void arcadeDrive(double _xSpeed, double _zRotation) {
    _xSpeed = MathUtil.applyDeadband(_xSpeed, g.OI.driverControllerDeadband);
    _zRotation = MathUtil.applyDeadband(_zRotation, g.OI.driverControllerDeadband);
    //_xSpeed *= g.DRIVETRAIN.speedLimiter;
    WheelSpeeds speeds = arcadeDriveIK(_xSpeed, _zRotation, true);
 
    m_leftMotor.set(speeds.left);
    m_rightMotor.set(speeds.right);

    SmartDashboard.putNumber("Motor Output Left Arcade", speeds.left);
    SmartDashboard.putNumber("Motor Output Right Arcade", speeds.right);
  }
  public static WheelSpeeds arcadeDriveIK(double xSpeed, double zRotation, boolean squareInputs) {
    xSpeed = MathUtil.clamp(xSpeed, -1.0, 1.0);
    zRotation = MathUtil.clamp(zRotation, -1.0, 1.0);

    // Square the inputs (while preserving the sign) to increase fine control
    // while permitting full power.
    if (squareInputs) {
      xSpeed = Math.copySign(xSpeed * xSpeed, xSpeed);
      zRotation = Math.copySign(zRotation * zRotation, zRotation);
    }

    double leftSpeed = xSpeed - zRotation;
    double rightSpeed = xSpeed + zRotation;

    // Find the maximum possible value of (throttle + turn) along the vector
    // that the joystick is pointing, then desaturate the wheel speeds
    double greaterInput = Math.max(Math.abs(xSpeed), Math.abs(zRotation));
    double lesserInput = Math.min(Math.abs(xSpeed), Math.abs(zRotation));
    if (greaterInput == 0.0) {
      return new WheelSpeeds(0.0, 0.0);
    }
    double saturatedInput = (greaterInput + lesserInput) / greaterInput;
    leftSpeed /= saturatedInput;
    rightSpeed /= saturatedInput;

    g.DRIVETRAIN.wheelSpeeds.left = leftSpeed;
    g.DRIVETRAIN.wheelSpeeds.right = rightSpeed;
    return  g.DRIVETRAIN.wheelSpeeds;
  }
  public void resetEncoders() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }

  public int getLeftEncoderCount() {
    return m_leftEncoder.get();
  }

  public int getRightEncoderCount() {
    return m_rightEncoder.get();
  }

  public double getLeftDistanceInch() {
    return m_leftEncoder.getDistance();
  }

  public double getRightDistanceInch() {
    return m_rightEncoder.getDistance();
  }

  public double getAverageDistanceInch() {
    return (getLeftDistanceInch() + getRightDistanceInch()) / 2.0;
  }

  /**
   * The acceleration in the X-axis.
   *
   * @return The acceleration of the XRP along the X-axis in Gs
   */
  public double getAccelX() {
    return m_accelerometer.getX();
  }








  /** Reset the gyro. */
  public void resetGyro() {
    g.ROBOT.gyro.reset();
  }

  public double getAngle(){
    return g.ROBOT.gyro.getAngle()-180.0;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Gyro Angle", getAngle());
    SmartDashboard.putNumber("LeftEncoderDistance_inch", getLeftDistanceInch());
    SmartDashboard.putNumber("RightEncoderDistance_inch", getRightDistanceInch());
  }

  public void toggleSpeed() {
    if(g.DRIVETRAIN.speedLimiter > 0.6){
      g.DRIVETRAIN.speedLimiter = 0.5;
    }else {
      g.DRIVETRAIN.speedLimiter = 1.0;
    }
  }
}
