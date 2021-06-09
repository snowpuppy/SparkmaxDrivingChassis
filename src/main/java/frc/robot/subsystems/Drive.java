// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANError;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.RobotMap;
import frc.robot.TestingDashboard;

import com.analog.adis16470.frc.ADIS16470_IMU;

public class Drive extends SubsystemBase {
  private CANSparkMax m_leftMotor1;
  private CANSparkMax m_rightMotor1;
  private CANEncoder m_rightEncoder1;
  private CANEncoder m_leftEncoder1;
  private ADIS16470_IMU m_imu;

  private static Drive drive;

  public static final double WHEEL_DIAMETER_IN_INCHES = 4; 
  public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER_IN_INCHES * Math.PI;
  public static final double GEAR_RATIO = 8.68; //number of times the motor rotates to rotate wheel once
  public static final double CONVERSION_FACTOR = WHEEL_CIRCUMFERENCE / GEAR_RATIO; //conversion factor * circumference = distance
  public final static double DISTANCE = CONVERSION_FACTOR * WHEEL_CIRCUMFERENCE;
  public final static double INITIAL_SPEED = 0.3;

  /** Creates a new Drive. */
  private Drive() {

    /**
     * SPARK MAX controllers are intialized over CAN by constructing a CANSparkMax
     * object
     * 
     * The CAN ID, which can be configured using the SPARK MAX Client, is passed as
     * the first parameter
     * 
     * The motor type is passed as the second parameter. Motor type can either be:
     * com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless
     * com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushed
     * 
     * The example below initializes four brushless motors with CAN IDs 1 and 2.
     * Change these parameters to match your setup
     */
    m_leftMotor1 = new CANSparkMax(RobotMap.leftDeviceID, MotorType.kBrushless);
    m_rightMotor1 = new CANSparkMax(RobotMap.rightDeviceID, MotorType.kBrushless);
    m_imu = new ADIS16470_IMU();

    m_leftEncoder1 = m_leftMotor1.getEncoder();
    m_rightEncoder1 = m_rightMotor1.getEncoder();

    /**
     * The RestoreFactoryDefaults method can be used to reset the configuration parameters
     * in the SPARK MAX to their factory default state. If no argument is passed, these
     * parameters will not persist between power cycles
     */
    m_leftMotor1.restoreFactoryDefaults();
    m_rightMotor1.restoreFactoryDefaults();


    m_leftMotor1.setInverted(true);
    m_rightMotor1.setInverted(false);


  
    if(m_leftMotor1.setIdleMode(IdleMode.kBrake) != CANError.kOk){
      SmartDashboard.putString("Left Motor1 Idle Mode ", "Error");
    }
  
    if(m_rightMotor1.setIdleMode(IdleMode.kBrake) != CANError.kOk){
      SmartDashboard.putString("Right Motor1 Idle Mode ", "Error");
    }
  
    if(m_leftEncoder1.setPositionConversionFactor(1.4476) != CANError.kOk){
      SmartDashboard.putString("Left Conversion Factor ", "Error");
    }
  
    if(m_rightEncoder1.setPositionConversionFactor(1.4476) != CANError.kOk){
      SmartDashboard.putString("Right Conversion Factor ", "Error");
    } 
  }

  public static Drive getInstance() {
    if (drive == null) {
      drive = new Drive();
      TestingDashboard.getInstance().registerSubsystem(drive, "Drive");
    }
    return drive;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  
  public void setRightMotorSpeed(double speed){
    m_rightMotor1.set(speed);
  }

  public void setLeftMotorSpeed(double speed){
    m_leftMotor1.set(speed);
  } 
  
  public void setRightMotorVoltage(double voltage){
    m_rightMotor1.setVoltage(voltage);
  }

  public void setLeftMotorVoltage(double voltage){
    m_leftMotor1.setVoltage(voltage);
  }

  public void resetRightEncoder() {
    m_rightEncoder1.setPosition(0);
   } 

   public void resetLeftEncoder() {
    m_leftEncoder1.setPosition(0);
   } 

   public double getleftMotoCurrnt() {
    return  m_leftMotor1.getOutputCurrent();
   }

   public double getrightMotoCurrnt() {
    return  m_rightMotor1.getOutputCurrent();
   } 

   public double getleftMotorTemp() {
    return  m_leftMotor1.getMotorTemperature();
   }

   public double getrightMotorTemp() {
    return  m_rightMotor1.getMotorTemperature();
   } 

   public double getrightMotorRate() {
    return  m_rightEncoder1.getVelocity();
   } 

   public double getleftMotorRate() {
    return  m_leftEncoder1.getVelocity();
   } 

   public double getrightMotorPosition() {
    return  m_rightEncoder1.getPosition();
   } 

   public double getleftMotorPosition() {
    return  m_leftEncoder1.getPosition();
   } 

   public CANEncoder getRightEncoder() {
    return m_rightEncoder1;
  }

   public CANEncoder getLeftEncoder() {
     return m_leftEncoder1;
   }

 


   public void setIdleMode(IdleMode mode) {
     SmartDashboard.putString("Brake Mode", mode == IdleMode.kBrake? "Brake":"Coast");
     m_leftMotor1.setIdleMode(mode);
     m_rightMotor1.setIdleMode(mode);
   }

  public void enableVoltageCompensation(double NominalVoltage) {
    m_leftMotor1.enableVoltageCompensation(NominalVoltage);
    m_rightMotor1.enableVoltageCompensation(NominalVoltage);
   }

   public double getgyro() {
    return  m_imu.getAngle();
   }

    public void resetGyro(){
      m_imu.reset();
}


}
