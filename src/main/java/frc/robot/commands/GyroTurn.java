package frc.robot.commands;

import com.revrobotics.CANEncoder;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;
import frc.robot.TestingDashboard;

public class GyroTurn extends CommandBase{
    private double m_degrees = 0.0;
    private double m_speed = 0.0; 
    private double m_volts = 0.0;
    private boolean m_parameterized = false;

    static Drive m_drive;

    public GyroTurn(double degrees, double speed, boolean parameterized) {
        m_degrees = degrees;
        m_speed = speed;
        m_parameterized = parameterized;
        addRequirements(Drive.getInstance());
      }

    public static void registerWithTestingDashboard() {
        Drive drive = Drive.getInstance();
        //DriveDistance cmd = new DriveDistance(12.0, Drive.INITIAL_SPEED, false);
        GyroTurn cmdTurn = new GyroTurn(0, 0, false);
        TestingDashboard.getInstance().registerCommand(drive, "GyroTurn", cmdTurn);
        TestingDashboard.getInstance().registerNumber(drive, "GyroTurn", "turnSpeed", Drive.INITIAL_SPEED);
        TestingDashboard.getInstance().registerNumber(drive, "GyroTurn", "degrees", Drive.INITIAL_DEGREES);
      }

    @Override
    public void initialize() {
        Drive.getInstance().resetRightEncoder();
        Drive.getInstance().resetLeftEncoder();
        Drive.getInstance().resetGyro();
  }

  @Override
  public void execute() {
    m_volts = m_speed * 10;
    if (!m_parameterized){

    }
    Drive.getInstance().setRightMotorVoltage(m_volts);
    Drive.getInstance().setLeftMotorVoltage(-m_volts);
  
    double rightPosition = Drive.getInstance().getrightMotorPosition();
    double leftPosition = Drive.getInstance().getleftMotorPosition();
  }
    
  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    double turnValue = Drive.getInstance().getgyro();
    return (Math.abs(turnValue) > Math.abs(m_degrees));
  }
  
}
