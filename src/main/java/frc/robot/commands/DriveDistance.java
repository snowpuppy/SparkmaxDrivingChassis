// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.revrobotics.CANEncoder;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;

public class DriveDistance extends CommandBase {
  static Drive m_drive;
  boolean m_parameterized;
  double m_distance;
  double m_speed;
  

  /** Creates a new DriveDistance. */
  // distance is in inches
  public DriveDistance(double distance, double speed, boolean parameterized) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = Drive.getInstance();
    addRequirements(m_drive);
    m_parameterized = parameterized;
    m_distance = distance;
    m_speed = speed;
  }

  public static void registerWithTestingDashboard() {
    Drive drive = Drive.getInstance();
    DriveDistance cmd = new DriveDistance(12.0, Drive.INITIAL_SPEED, false);


  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    CANEncoder leftEncoder = m_drive.getLeftEncoder();
    CANEncoder rightEncoder = m_drive.getRightEncoder();
    m_drive.resetRightEncoder();
    m_drive.resetLeftEncoder();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    m_drive.setRightMotorSpeed(m_speed);
    m_drive.setLeftMotorSpeed(m_speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    CANEncoder leftEncoder = m_drive.getLeftEncoder();
    CANEncoder rightEncoder = m_drive.getRightEncoder();
    boolean finished = false;
    if (m_distance >= 0) {
      if (m_drive.getrightMotorPosition() >= m_distance && m_drive.getleftMotorPosition() >= m_distance) {
        finished = true;
      }
    } else if (m_distance < 0) {
      if (m_drive.getrightMotorPosition() <= m_distance && m_drive.getleftMotorPosition() <= m_distance) {
        finished = true;
      }
    }
    return finished;
  }
}
