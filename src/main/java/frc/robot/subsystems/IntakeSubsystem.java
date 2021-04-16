/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  /**
   * Creates a new IntakeSubsystem.
   */

   private final WPI_VictorSPX intakeMotor = new WPI_VictorSPX(IntakeConstants.kIntakeMotorPort);
   
   private final DoubleSolenoid intakeSolenoid = new DoubleSolenoid(IntakeConstants.solenoidForwardPin,
      IntakeConstants.solenoidBackwardPin);
  public IntakeSubsystem() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void runIntake(double speed){
    intakeMotor.set(speed);
  }
  public void stopIntake(){
    intakeMotor.set(0);
  }
  public void openIntake() {
    intakeSolenoid.set(Value.kForward);
  }

  public void closeIntake() {
    intakeSolenoid.set(Value.kReverse);
  }

  public void stopCylinders() {
    intakeSolenoid.set(Value.kOff);
  }
}
