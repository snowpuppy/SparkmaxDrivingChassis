package frc.robot;

import com.analog.adis16470.frc.ADIS16470_IMU;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Vector;

public class AccelerometerHelper {

    // Solution using vectors
    private double[] timeValues;
    private Vector robotDirection;
    private Vector[] accelVec;
    private Vector[] velocityVec;
    private Vector[] distanceVec;

    private ADIS16470_IMU m_imu;

    public AccelerometerHelper(ADIS16470_IMU imu) {
        m_imu = imu;
        
        // Solution using vectors
        robotDirection = new Vector();
        accelVec = new Vector[2];
        velocityVec = new Vector[2];
        distanceVec = new Vector[2];
    }

    public static double integrate(double tInitial, double tFinal, double vInitial, double vFinal) { // v for value
        double tInterval = tFinal - tInitial;
        double area = (tInterval * (vInitial + vFinal)) / 2;
        return area;
    }

    public void updateDirectionOfRobot() {
        double angle = m_imu.getAngle();
        angle = Math.toRadians(angle);
        robotDirection.x = Math.cos(angle);
        robotDirection.y = Math.sin(angle);
    }

    public void captureTimeData() {
        timeValues[0] = timeValues[1];
        timeValues[1] = Timer.getFPGATimestamp();
    }

    public void captureAccelerometerData() {
        // Eliminate the oldest value and move
        // the second oldest value back one
        accelVec[0].x = accelVec[1].x;
        accelVec[0].y = accelVec[1].y;
        // Capture the new value
        accelVec[1].x = m_imu.getAccelInstantX();
        accelVec[1].y = m_imu.getAccelInstantY();
    }

    public void resetVelocity() {
        initializeVelocity(0);
    }

    public void initializeVelocity(double val) {
        // Initialize the velocity vector to the direction
        // that we're going multiplied by the vector of the
        // direction we're facing
        velocityVec[0].x = val*robotDirection.x;
        velocityVec[0].y = val*robotDirection.y;
        velocityVec[1].x = 0;
        velocityVec[1].y = 0;
    }

    public void resetDistance() {
        initializeDistance(0);
    }

    public void initializeDistance(double val) {
        distanceVec[0].x = val*robotDirection.x;
        distanceVec[0].y = val*robotDirection.y;
        distanceVec[1].x = 0;
        distanceVec[1].y = 0;
    }

    public void calculateVelocity() {
        Vector oldVelocity = new Vector(velocityVec[0].x, velocityVec[0].y);
        velocityVec[0].x = velocityVec[1].x;
        velocityVec[0].y = velocityVec[1].y;
        velocityVec[1].x = oldVelocity.x + integrate(timeValues[0], timeValues[1], accelVec[0].x, accelVec[1].x);
        velocityVec[1].y = oldVelocity.y + integrate(timeValues[0], timeValues[1], accelVec[0].y, accelVec[1].y);
    }

    public void calculateDistance() {
        Vector oldDistance = new Vector(distanceVec[0].x, distanceVec[0].y);
        distanceVec[0].x = distanceVec[1].x;
        distanceVec[0].y = distanceVec[1].y;
        distanceVec[1].x = oldDistance.x + integrate(timeValues[0], timeValues[1], velocityVec[0].x, velocityVec[1].x);
        distanceVec[1].y = oldDistance.y + integrate(timeValues[0], timeValues[1], velocityVec[0].y, velocityVec[1].y);
    }

    public double getAccelerometerMagnitude() {
        double result = Vector.mag(accelVec[1]);
        return result;
    }

    public double getVelocityMagnitude() {
        double result = Vector.mag(velocityVec[1]);
        return result;
    }

    public double getDistanceMagnitued() {
        double result = Vector.mag(distanceVec[1]);
        return result;
    }

    public double getCurrentTime() {
        return timeValues[1];
    }
}
