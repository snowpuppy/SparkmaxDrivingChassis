// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.revrobotics.CANEncoder;

import frc.robot.TestingDashboard;
import frc.robot.subsystems.Drive;

import java.util.Hashtable;


/** Add your docs here. */
public class DriveCompensatedDistance extends DriveDistance{
    static Hashtable<Double,Double> averageErrorMap;
    
    public DriveCompensatedDistance(double distance, double speed, boolean parameterized) {
        // Use addRequirements() here to declare subsystem dependencies.
       super(distance, speed, parameterized);
       averageErrorMap = new Hashtable<Double, Double>();
       averageErrorMap.put(-1.0, 2.116624451);
       averageErrorMap.put(-0.9, 1.344509888);
       averageErrorMap.put(-0.8, 1.951171875);
       averageErrorMap.put(-0.7, 1.413448334);
       averageErrorMap.put(-0.6, 0.8136802673);
       averageErrorMap.put(-0.5, 0.9239845276);
       averageErrorMap.put(-0.4, 0.6482292175);
       averageErrorMap.put(-0.3, 0.3586853027);
       averageErrorMap.put(-0.2, 0.2966415405);
       averageErrorMap.put(-0.1, 0.1173988342);
       averageErrorMap.put(0.1, 0.2483825684);
       averageErrorMap.put(0.2, 0.269065094);
       averageErrorMap.put(0.3, 0.5792892456);
       averageErrorMap.put(0.4, 0.7171676636);
       averageErrorMap.put(0.5, 0.8826210022);
       averageErrorMap.put(0.6, 1.006710815);
       averageErrorMap.put(0.7, 1.406555176);
       averageErrorMap.put(0.8, 1.130799103);
       averageErrorMap.put(0.9, 1.454813385);
       averageErrorMap.put(1.0, 1.48928299);


      }

   
    public static void registerWithTestingDashboard() {
        Drive drive = Drive.getInstance();
        DriveCompensatedDistance cmd = new DriveCompensatedDistance(Drive.INITIAL_SPEED, Drive.INITIAL_SPEED, false);
        TestingDashboard.getInstance().registerCommand(drive, "DriveCompensatedDistance", cmd);
    
    
      }
    

    @Override
    public void execute(){
        if (!m_parameterized) {
            m_speed = TestingDashboard.getInstance().getNumber(m_drive, "drivingSpeed");
            m_distance = TestingDashboard.getInstance().getNumber(m_drive, "drivingDistance") - averageErrorMap.get(m_speed);
          }
          
          CANEncoder leftEncoder = m_drive.getLeftEncoder();
          CANEncoder rightEncoder = m_drive.getRightEncoder();
          m_drive.setRightMotorSpeed(m_speed);
          m_drive.setLeftMotorSpeed(m_speed);
    }
}
