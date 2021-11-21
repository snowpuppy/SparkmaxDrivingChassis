// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANError;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.RobotMap;
import frc.robot.TestingDashboard;

import com.analog.adis16470.frc.ADIS16470_IMU;
import edu.wpi.first.wpilibj.Timer;

public class Drive extends SubsystemBase {
  private CANSparkMax m_leftMotor1;
  private CANSparkMax m_rightMotor1;
  private CANEncoder m_rightEncoder1;
  private CANEncoder m_leftEncoder1;
  private ADIS16470_IMU m_imu;
  private double[] accelValues;
  private double[] timeValues;
  private double[] velocityValues;
  private double[] distanceValues;
  private double xDirection;
  private double yDirection;
  private boolean m_measureVelocity;
  private boolean m_measureDistance;
  
  

  private static Drive m_drive;

  public static final double WHEEL_DIAMETER_IN_INCHES = 4; 
  public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER_IN_INCHES * Math.PI;
  public static final double GEAR_RATIO = 8.68; //number of times the motor rotates to rotate wheel once
  public static final double CONVERSION_FACTOR = WHEEL_CIRCUMFERENCE / GEAR_RATIO; //conversion factor * circumference = distance
  public final static double DISTANCE = CONVERSION_FACTOR * WHEEL_CIRCUMFERENCE;
  public final static double INITIAL_SPEED = 0.3;
  public final static double INITIAL_DISTANCE = 12;
  public final static double INITIAL_DEGREES = 90;

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

    accelValues = new double[2];
    timeValues = new double[2];
    velocityValues = new double[2];
    distanceValues = new double[2];

    m_measureVelocity = false;
    m_measureDistance = false;


    /**
     * The RestoreFactoryDefaults method can be used to reset the configuration parameters
     * in the SPARK MAX to their factory default state. If no argument is passed, these
     * parameters will not persist between power cycles
     */
    m_leftMotor1.restoreFactoryDefaults();
    m_rightMotor1.restoreFactoryDefaults();


    m_leftMotor1.setInverted(true);
    m_rightMotor1.setInverted(false);


  /** 
    if(m_leftMotor1.setIdleMode(IdleMode.kBrake) != CANError.kOk){
      System.out.println("Could not set idle mode on left motor 1 ");
      System.exit(1);
    }
  
    if(m_rightMotor1.setIdleMode(IdleMode.kBrake) != CANError.kOk){
      System.out.println("Could not set idle mode on right motor 1 ");
      System.exit(1);
    }
  
    if(m_leftEncoder1.setPositionConversionFactor(CONVERSION_FACTOR) != CANError.kOk){ 
      System.out.println("Could not set position conversion factor on left encoder 1");
      //System.exit(1);
    }
  
    if(m_rightEncoder1.setPositionConversionFactor(CONVERSION_FACTOR) != CANError.kOk){
      System.out.println("Could not set position conversion factor on right encoder 1");
      //System.exit(1);
    } 
  */
    m_leftMotor1.setIdleMode(IdleMode.kBrake);
    m_rightMotor1.setIdleMode(IdleMode.kBrake);
    m_leftEncoder1.setPositionConversionFactor(CONVERSION_FACTOR);
    m_rightEncoder1.setPositionConversionFactor(CONVERSION_FACTOR);

  }

  public static Drive getInstance() {
    if (m_drive == null) {
      m_drive = new Drive();
      TestingDashboard.getInstance().registerSubsystem(m_drive, "Drive");
      TestingDashboard.getInstance().registerNumber(m_drive, "Encoders", "RightMotorDistance", 0);
      TestingDashboard.getInstance().registerNumber(m_drive, "Encoders", "LeftMotorDistance", 0);
      TestingDashboard.getInstance().registerNumber(m_drive, "MotorSpeed", "RightMotorSpeed", 0);
      TestingDashboard.getInstance().registerNumber(m_drive, "MotorSpeed", "LeftMotorSpeed", 0);
      TestingDashboard.getInstance().registerNumber(m_drive, "Gyro", "CurrentAngle", 0);
      TestingDashboard.getInstance().registerNumber(m_drive, "Data", "leftVelocity", 0);
      TestingDashboard.getInstance().registerNumber(m_drive, "Data", "rightVelocity", 0);
      TestingDashboard.getInstance().registerNumber(m_drive, "Data", "actualDistance", 0);
      TestingDashboard.getInstance().registerNumber(m_drive, "Data", "stoppingDistance", 0);
      TestingDashboard.getInstance().registerNumber(m_drive, "DriveDistance", "drivingSpeed", Drive.INITIAL_SPEED);
      TestingDashboard.getInstance().registerNumber(m_drive, "DriveDistance", "drivingDistance", Drive.INITIAL_DISTANCE);
      TestingDashboard.getInstance().registerNumber(m_drive, "Accelerometer", "instantAccel", 0);
      TestingDashboard.getInstance().registerNumber(m_drive, "Accelerometer", "currentTime", 0);
      
    }
    return m_drive;
  }


  public double getAccelerometerMagnitude() {
    double x = m_imu.getAccelInstantX();
    double y = m_imu.getAccelInstantY();
    double magnitude = Math.sqrt(x*x + y*y);
    return magnitude;
  }

  public static double integrate(double tInitial, double tFinal, double vInitial, double vFinal) { // v for value
    double tInterval = tFinal - tInitial;
    double area = (tInterval * (vInitial + vFinal)) / 2;
    return area;
}

  public void startMeasuringVelocity() {
    m_measureVelocity = true;
  }

  
  public void stopMeasuringVelocity() {
    m_measureVelocity = false;
  }

  
  public void startMeasuringDistance() {
    m_measureDistance = true;
  }

  
  public void stopMeasuringDistance() {
    m_measureDistance = false;
  }

  public void setInitialVelocity(double velocity) {
    velocityValues[0] = velocity;
  }

  public void setInitialDistance(double distance) {
    distanceValues[0] = distance;
  }

  public void updateXYDirections() {
    double angle = m_imu.getAngle(); 
    angle = Math.toRadians(angle);
    xDirection = Math.cos(angle);
    yDirection = Math.sin(angle);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_drive.updateXYDirections();
    accelValues[0] = accelValues[1];
    accelValues[1] = -1 * m_drive.getAccelerometerMagnitude(); // assume always slowing down
    timeValues[0] = timeValues[1];
    timeValues[1] = Timer.getFPGATimestamp();

    if (m_measureVelocity) {
      double oldVelocity = velocityValues[0];
      velocityValues[0] = velocityValues[1];
      velocityValues[1] = oldVelocity + integrate(timeValues[0], timeValues[1], accelValues[0], accelValues[1]);
    }

    if (m_measureDistance) {
      double oldDistance = distanceValues[0];
      distanceValues[0] = distanceValues[1];
      distanceValues[1] = oldDistance + integrate(timeValues[0], timeValues[1], velocityValues[0], velocityValues[1]);
    }

    TestingDashboard.getInstance().updateNumber(m_drive, "RightMotorDistance", getrightMotorPosition());
    TestingDashboard.getInstance().updateNumber(m_drive, "LeftMotorDistance", getleftMotorPosition());
    TestingDashboard.getInstance().updateNumber(m_drive, "RightMotorSpeed", m_rightMotor1.get());
    TestingDashboard.getInstance().updateNumber(m_drive, "LeftMotorSpeed", m_leftMotor1.get());
    TestingDashboard.getInstance().updateNumber(m_drive, "CurrentAngle", m_imu.getAngle());
    TestingDashboard.getInstance().updateNumber(m_drive, "instantAccel", accelValues[1]);
    TestingDashboard.getInstance().updateNumber(m_drive, "currentTime", timeValues[1]);

    CANEncoder leftEncoder = m_drive.getLeftEncoder();
    CANEncoder rightEncoder = m_drive.getRightEncoder();
    TestingDashboard.getInstance().updateNumber(m_drive, "leftVelocity", leftEncoder.getVelocity());
    TestingDashboard.getInstance().updateNumber(m_drive, "rightVelocity", rightEncoder.getVelocity());
    //TestingDashboard.getInstance().updateNumber(m_drive, "actualDistance", m_drive.getrightMotorPosition());
    //TestingDashboard.getInstance().updateNumber(m_drive, "stoppingDistance", m_drive.getrightMotorPosition() - TestingDashboard.getInstance().getNumber(m_drive, "drivingDistance"));


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
