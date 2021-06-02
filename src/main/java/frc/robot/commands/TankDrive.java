// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;
import frc.robot.OI;
import frc.robot.input.AttackThree;
import frc.robot.input.AttackThree.AttackThreeAxis;
import edu.wpi.first.wpilibj.Timer;
import java.util.ArrayList;

public class TankDrive extends CommandBase {
  private final Drive m_drive; 
  private final AttackThreeAxis yAxis = AttackThreeAxis.kY;
  private static OI oi;

  int m_counter = 0;
  double m_startTime = 0;
  double m_priorAutoSpeed = 0;
  double[] m_numberArray = new double[10];
  ArrayList<Double> m_entries = new ArrayList<>();
 
  /** Creates a new TankDrive. */
  public TankDrive(Drive drive) {
    // Use addRequirements() here to declare subsystem dependencies.
      m_drive = drive;
      oi = OI.getInstance();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drive.resetRightEncoder();
    m_drive.resetLeftEncoder();
    m_startTime = Timer.getFPGATimestamp();
    m_counter = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    AttackThree leftJoystick = oi.getLeftStick();
    AttackThree rightJoystick = oi.getRightStick();
    double leftJoystickSpeed = leftJoystick.getAxis(yAxis);
    double rightJoystickSpeed = rightJoystick.getAxis(yAxis);
    
    leftJoystickSpeed = leftJoystick.getAxis(yAxis);
    rightJoystickSpeed = rightJoystick.getAxis(yAxis);
 
    m_drive.setRightMotorSpeed(rightJoystickSpeed);
    m_drive.setLeftMotorSpeed(leftJoystickSpeed);

    SmartDashboard.putNumber("Right JoyStck", rightJoystickSpeed);
    SmartDashboard.putNumber("Left JoyStick", leftJoystickSpeed);

    double leftCurrent = m_drive.getleftMotoCurrnt();
    SmartDashboard.putNumber("Left Current", leftCurrent);

    double rightCurrent = m_drive.getrightMotoCurrnt();
    SmartDashboard.putNumber("Right Current", rightCurrent);

    double leftTemp = m_drive.getleftMotorTemp();
    SmartDashboard.putNumber("Left Temp", leftTemp);
   
    double rightTemp = m_drive.getrightMotorTemp();
    SmartDashboard.putNumber("Right Temp", rightTemp);

    double rightRate = m_drive.getrightMotorRate();
    SmartDashboard.putNumber("Right Rate", rightRate);

    double leftRate = m_drive.getleftMotorRate();
    SmartDashboard.putNumber("Left Rate", leftRate);

    double rightPosition = m_drive.getrightMotorPosition();
    SmartDashboard.putNumber("Right Position", rightPosition);

    double leftPosition = m_drive.getleftMotorPosition();
    SmartDashboard.putNumber("Left Position", leftPosition);
 
    double gyroAngle = m_drive.getgyro();
    SmartDashboard.putNumber("Gryo", gyroAngle);
  }
 

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}
