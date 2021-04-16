/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;

public class Drivetrain extends SubsystemBase {
  /**
   * Creates a new DrivetrainSubsystem.
   */
  private final SpeedControllerGroup m_leftMotors =
  new SpeedControllerGroup(new WPI_VictorSPX(DrivetrainConstants.kFrontLeftMotorPort),
  new WPI_VictorSPX(DrivetrainConstants.kRearLeftMotorPort));;
        ;
        private final SpeedControllerGroup m_rightMotors =
        new SpeedControllerGroup(new WPI_VictorSPX(DrivetrainConstants.kFrontRightMotorPort),
        new WPI_VictorSPX(DrivetrainConstants.kRearRightMotorPort));;
              ;
  
         
  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);
  
  private final ADXRS450_Gyro gyro = new ADXRS450_Gyro();

  private final DifferentialDriveOdometry m_odometry;

  private final Encoder leftWheelEncoder =  new Encoder(DrivetrainConstants.leftEncoderPort1,DrivetrainConstants.leftEncoderPort2);
  private final Encoder rightWheelEncoder =  new Encoder(DrivetrainConstants.rightEncoderPort1,DrivetrainConstants.rightEncoderPort2);

  public Drivetrain() {
    //m_frontRightMotor.follow(m_rearRightMotor);
    //m_frontLeftMotor.follow(m_rearLeftMotor);
    
    gyro.calibrate();
    leftWheelEncoder.setDistancePerPulse(15.24*Math.PI/2048);
    rightWheelEncoder.setDistancePerPulse(15.24*Math.PI/2048);

    m_odometry = new DifferentialDriveOdometry(gyro.getRotation2d());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public double getDistance(){
    return (leftWheelEncoder.getDistance()+rightWheelEncoder.getDistance())/2;
  }
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public void joystickDrive(DoubleSupplier linearSpeed, DoubleSupplier angularSpeed){
    m_drive.arcadeDrive(linearSpeed.getAsDouble(), angularSpeed.getAsDouble());
  }
  public void arcadeDrive(double fwd, double rot) {
    m_drive.arcadeDrive(fwd, rot, true);
  }
  public double getHeading() {
    return Math.IEEEremainder(gyro.getAngle(), 360);
  }
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_leftMotors.setVoltage(leftVolts);
    m_rightMotors.setVoltage(-rightVolts);
    m_drive.feed();
  }
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(pose, gyro.getRotation2d());
  }
}
