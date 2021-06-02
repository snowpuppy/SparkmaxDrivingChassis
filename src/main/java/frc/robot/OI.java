/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

//import frc.robot.commands.EncoderSpin;


import frc.robot.input.AttackThree;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {

  private static OI oi;

  public static AttackThree leftStick;
  public static AttackThree rightStick;
    
  /**
   * Used outside of the OI class to return an instance of the class.
   * @return Returns instance of OI class formed from constructor.
   */
  public static OI getInstance() {
    if (oi == null) {
      oi = new OI();
    }
    return oi;
  }
  
  public OI() {
    // User Input
    leftStick = new AttackThree(RobotMap.U_JOYSTICK_LEFT, 0.05);
    rightStick = new AttackThree(RobotMap.U_JOYSTICK_RIGHT, 0.05);
    
    //rightStick.getButton(1).whenPressed(new EncoderSpin(90, 0.3));
  
  }

  /**
   * Returns the left Joystick
   * @return the leftStick
   */
  public AttackThree getLeftStick() {
    return leftStick;
  }

  /**
   * Returns the right Joystick
   * @return the rightStick
   */
  public AttackThree getRightStick() {
    return rightStick;
  }
}

