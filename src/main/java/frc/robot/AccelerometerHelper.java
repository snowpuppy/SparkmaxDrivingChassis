package frc.robot;

import com.analog.adis16470.frc.ADIS16470_IMU;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Vector;
import frc.robot.subsystems.Drive;

public class AccelerometerHelper {

    // Solution using vectors
    private double[] timeValues;
    private Vector robotDirection;
    private Vector[] accelVec;
    private Vector[] velocityVec;
    private Vector[] distanceVec;

    private ADIS16470_IMU m_imu;

    public final static double ACCEL_STATIONARY_MAX = 18.5;
    public final static double ACCEL_STATIONARY_MIN = 14;
    public final static double ACCEL_STATIONARY_RANGE_DIV_TWO = 0.0390954238;
    public final static double GRAVITY_TO_INCHES_PER_SECOND_SQUARED = 386.088583;
    public final static double ACCEL_X_REST_AVERAGE = 0.026546; // unit: gravity
    public final static double ACCEL_X_REST_AVERAGED_MAX = 0.005; // unit: gravity
    public final static double ACCEL_X_REST_AVERAGED_MIN = -0.0045; // unit: gravity
    public final static double ACCEL_Y_REST_AVERAGE = -0.032664; // unit: gravity
    public final static double ACCEL_Y_REST_AVERAGED_MAX = 0.0045; // unit: gravity
    public final static double ACCEL_Y_REST_AVERAGED_MIN = -0.005; // unit: gravity

    public AccelerometerHelper(ADIS16470_IMU imu) {
        m_imu = imu;
        
        // Solution using vectors
        robotDirection = new Vector();
        accelVec = new Vector[] {new Vector(), new Vector()};
        velocityVec = new Vector[] {new Vector(), new Vector()};
        distanceVec = new Vector[] {new Vector(), new Vector()};
        timeValues = new double[2];
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
        double xRaw = 0;
        double yRaw = 0;
        // Eliminate the oldest value and move
        // the second oldest value back one
        accelVec[0].x = accelVec[1].x; // unit: gravity
        accelVec[0].y = accelVec[1].y;
        // Capture the new value
        xRaw = m_imu.getAccelInstantX(); // unit: gravity
        yRaw = m_imu.getAccelInstantY();

        // Subtract center of mass (average) of data to center noise around zero
        accelVec[1].x = xRaw - ACCEL_X_REST_AVERAGE;
        accelVec[1].y = yRaw - ACCEL_Y_REST_AVERAGE;

        // Create dead band based on max and min of averaged at-rest data for X
        if (accelVec[1].x < ACCEL_X_REST_AVERAGED_MAX && accelVec[1].x > ACCEL_X_REST_AVERAGED_MIN) {
            accelVec[1].x = 0;
        }

        // Create dead band based on max and min of averaged at-rest data for Y
        if (accelVec[1].y < ACCEL_Y_REST_AVERAGED_MAX && accelVec[1].y > ACCEL_Y_REST_AVERAGED_MIN) {
            accelVec[1].y = 0;
        }

        Drive drive = Drive.getInstance();
        TestingDashboard.getInstance().updateNumber(drive, "xInstantAccel", accelVec[1].x);
        TestingDashboard.getInstance().updateNumber(drive, "yInstantAccel", accelVec[1].y);
        TestingDashboard.getInstance().updateNumber(drive, "xInstantAccelRaw", xRaw);
        TestingDashboard.getInstance().updateNumber(drive, "yInstantAccelRaw", yRaw);
        
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

    public double getAccelerometerMagnitudeGravity() {
        double result = Vector.mag(accelVec[1]);
        return result;
    }

    public double getAccelerometerMagnitudeInchesPerSecondSquared() {
        double result = Vector.mag(accelVec[1]) * GRAVITY_TO_INCHES_PER_SECOND_SQUARED;
        return result;
    }

    public double getVelocityMagnitudeGravity() {
        double result = Vector.mag(velocityVec[1]);
        return result;
    }

    public double getVelocityMagnitudeInchesPerSecondSquared() {
        double result = Vector.mag(velocityVec[1]) * GRAVITY_TO_INCHES_PER_SECOND_SQUARED;
        return result;
    }

    public double getDistanceMagnitudeGravity() {
        double result = Vector.mag(distanceVec[1]);
        return result;
    }

    public double getDistanceMagnitudeInchesPerSecondSquared() {
        double result = Vector.mag(distanceVec[1]) * GRAVITY_TO_INCHES_PER_SECOND_SQUARED;
        return result;
    }

    public double getCurrentTime() {
        return timeValues[1];
    }
}
