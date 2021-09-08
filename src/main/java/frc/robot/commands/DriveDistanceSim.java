// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.TestingDashboard;
import frc.robot.subsystems.Drive;

public class DriveDistanceSim extends CommandBase {
  static Drive m_drive;
  boolean m_parameterized;
  double m_distance;
  double m_speed;
  double m_rightDistance;
  double m_leftDistance;
  

  /** Creates a new DriveDistance. */
  // distance is in inches
  public DriveDistanceSim(double distance, double speed, boolean parameterized) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = Drive.getInstance();
    addRequirements(m_drive);
    m_parameterized = parameterized;
    m_distance = distance;
    m_speed = speed;
    m_rightDistance = 0;
    m_leftDistance = 0;
    
  }

  public static void registerWithTestingDashboard() {
    Drive drive = Drive.getInstance();
    DriveDistanceSim cmd = new DriveDistanceSim(Drive.INITIAL_SPEED, Drive.INITIAL_SPEED, false);
    TestingDashboard.getInstance().registerCommand(drive, "DriveDistanceSim", cmd);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_rightDistance = 0;
    m_leftDistance = 0;
    }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!m_parameterized) {
      m_speed = TestingDashboard.getInstance().getNumber(m_drive, "drivingSpeed");
      m_distance = TestingDashboard.getInstance().getNumber(m_drive, "drivingDistance");
    }
    m_rightDistance += Drive.DISTANCE * m_speed;
    m_leftDistance += Drive.DISTANCE * m_speed;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean finished = false;
    double leftPosition = m_leftDistance;
    double rightPosition = m_rightDistance;

    if (m_distance >= 0 && m_speed >= 0) {
      if (leftPosition >= m_distance && rightPosition >= m_distance) {
        finished = true;
        System.out.println("leftPosition = " + leftPosition + " rightPosition = " + rightPosition + " m_distance = " + m_distance);
        TestingDashboard.getInstance().updateNumber(m_drive, "actualDistance", leftPosition);
        TestingDashboard.getInstance().updateNumber(m_drive, "stoppingDistance", leftPosition - TestingDashboard.getInstance().getNumber(m_drive, "drivingDistance"));
      }
    } 

    else if (m_distance > 0 && m_speed < 0) {
      if (rightPosition <= -m_distance && leftPosition <= -m_distance) {
      finished = true;
      System.out.println("leftPosition = " + leftPosition + " rightPosition = " + rightPosition + " m_distance = " + m_distance);
      TestingDashboard.getInstance().updateNumber(m_drive, "actualDistance", leftPosition);
      TestingDashboard.getInstance().updateNumber(m_drive, "stoppingDistance", leftPosition - TestingDashboard.getInstance().getNumber(m_drive, "drivingDistance"));
      }

    else if (m_distance < 0 && m_speed >= 0) {
      if (rightPosition <= m_distance && leftPosition <= m_distance) {
      finished = true;
      System.out.println("leftPosition = " + leftPosition + " rightPosition = " + rightPosition + " m_distance = " + m_distance);
      TestingDashboard.getInstance().updateNumber(m_drive, "actualDistance", leftPosition);
      TestingDashboard.getInstance().updateNumber(m_drive, "stoppingDistance", leftPosition - TestingDashboard.getInstance().getNumber(m_drive, "drivingDistance"));
    }

    else if (m_distance * m_speed < 0) {
        if (rightPosition <= -m_distance && leftPosition <= -m_distance) {
        finished = true;
        System.out.println("leftPosition = " + leftPosition + " rightPosition = " + rightPosition + " m_distance = " + m_distance);
        TestingDashboard.getInstance().updateNumber(m_drive, "actualDistance", leftPosition);
        TestingDashboard.getInstance().updateNumber(m_drive, "stoppingDistance", leftPosition - TestingDashboard.getInstance().getNumber(m_drive, "drivingDistance"));
        }
    }

  }
    
  }

    return finished;
  }
}
