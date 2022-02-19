package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Shooter {

    // two talons. 1 master, 1 slave
    // 1 spark max for hood for hood

    WPI_TalonFX masterShooterMotor = new WPI_TalonFX(Constants.masterShooterMotorIndex);
    WPI_TalonFX slaveShooterMotor = new WPI_TalonFX(Constants.slaveShooterMotorIndex);

    CANSparkMax hoodMotor = new CANSparkMax(Constants.hoodMotorIndex, MotorType.kBrushless);

    boolean shooting = false;
    boolean isTargetCenter = false; // x axis. y axis obviously can't be because of the angle

    final double maxHoodEncoders = 500; // could literally set position to angle // need to find through testing
    final double maxTurretEncoders = 500; // could literally set position to angle // need to find through testing

    

    public double shooterIndexerEncoderCount = 200; // encoder count to go from under shooter to shooter.

    

    public void shooterInit() {

        slaveShooterMotor.follow(masterShooterMotor);
        hoodMotor.getEncoder().setPosition(0.0); // want to start hood at the bottom. find encoder to angle ratio
        hoodMotor.setInverted(true);
        hoodMotor.getEncoder().setPositionConversionFactor(0.65222);
        hoodMotor.getEncoder().setPosition(10.5);
        // limelight pipeline = 1; // for vision tape
        //hoodMotor.getEncoder().setPositionConversionFactor() // find what it is to convert to degrees or radian
        
        // ideas:
        // set slave to update faster. frame period or something
        // set master shooter to have a conversion factor to convert into our in/s
    }

    final static double gravity = 386.103; // in/s^2
    final static double y = 104.016; // in // upper hub height by default. Should have button to switch between them
    final static double y0 = 22.5; // in
    final static double minimumVelocity = 267.7;
    final static double maximumVelocity = 405.5;
    final static double minimumAngle = 0.804;
    final static double maximumAngle = 1.387; // 1.352
    final static int velocityDecimals = 1;
    final static int angleDecimals = 3;
    final static double distanceFromTapeToCenterOfHub = 26.6875; // in

    private static double cameraHeight = 21.0; // may need to account for difference between camera height and shooter
                                               // height
    private static double cameraAngle = 0.0; // make sure you account for radians vs degrees

    // Error from the limelight when reading the angle. It appears to be consistent
    private static double angleError = 0.04;

    static double calculatedVelocity = 0.0;
    static double calculatedAngle = 0.000;

    public static void testVelocity(double x) {

        double velocity = 0.00;
        for (double i = minimumVelocity; i <= maximumVelocity; i += (1.0 / Math.pow(10, velocityDecimals))) {
            if ((testAngle(i, x) > minimumAngle) && (testAngle(i, x) < maximumAngle)) {
                velocity = i;
                break;
            }
        }
        calculatedVelocity = velocity;
        calculatedAngle = testAngle(velocity, x);
    }

    public static double testAngle(double velocity, double x) {

        double angle = minimumAngle;

        double[] possibleAngles = new double[(int) ((maximumAngle * Math.pow(10, angleDecimals))
                - (minimumAngle * Math.pow(10, angleDecimals)) + 1)];
        for (int i = 0; i < possibleAngles.length; i++) {
            possibleAngles[i] = angle;
            angle = angle + (1 / Math.pow(10, angleDecimals));
        }

        for (double possibleAngle : possibleAngles) {
            double slope = slopeOfLine(velocity, possibleAngle, x);
            if (!Double.isNaN(slope)) {
                if (slope > 0.00) {
                    angle = possibleAngle;
                    break;
                }
            }
        }

        while (slopeOfLine(velocity, angle, x) > 0.000) {
            angle += (1.0 / Math.pow(10, angleDecimals));
        }
        return angle;
    }

    public static double slopeOfLine(double velocity, double possibleAngle, double x) {

        return ((y - x) / (((-1) * ((-1) * velocity * Math.sin(possibleAngle) - Math.sqrt(Math.pow(velocity, 2)
                * Math.pow(Math.sin(possibleAngle), 2) + ((2 * gravity) * (y0 - y)))) / gravity)
                - (x / (velocity * Math.cos(possibleAngle)))));
    }

    public static double getXDistanceFromCenterOfHub(double verticalAngle) {
        return ((y - cameraHeight) / Math.tan(Math.toRadians(verticalAngle + cameraAngle - angleError))) + distanceFromTapeToCenterOfHub; // ADD DISTANCE FROM TAPE TO CENTER
    }

    public static double shooterWheelLinearVelocityToMotorVelocity(double shooterWheelLinearVelocity){
        return 2.625 * (shooterWheelLinearVelocity * (1.00/(4.00*Math.PI)) * (1.00/1.21) * 2048.00 * (1.00/10.00));
        // needs to be defined based on trial and error
        //return shooterWheelLinearVelocity; // calculate that initial v based on y = y0 + v*sin(theta)*t + 0.5*a*t^2
    }

    public void shooterTeleop() {

        /*
         * HAVE BUTTON TO SWITCH BETWEEN y's. AKA upper hub vs lower hub
         * notes:
         * robot needs to stop,
         * inactivate player controls,
         * turn itself and the turret until it finds the target
         * calculate the x distance to the target using limelight (angle + known height
         * of target)
         * calculate velocity and angle once
         * set motors and hood to go to those positions
         * verify they're at those positions
         * move the indexer wheel that number of encoder counts
         */

        // if(ColorSensor.getColor()){
        // Robot.intake.colorSensor2Activated = true;
        // }

        if ((Robot.intake.ballCount > 0) && Constants.xbox.getRawButtonPressed(2)) {
            shooting = true;
            Robot.drive.isDriverControlEnabled = false;
            Robot.drive.stopMotors();
            isTargetCenter = false;
        }

        if (Robot.intake.ballCount == 0) {
            shooting = false;
            masterShooterMotor.set(0.0);
            Robot.drive.isDriverControlEnabled = true;
        }

        if (shooting) {
            if (!NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getBoolean(false)) {
                Robot.drive.mecanumDrive.driveCartesian(0.0, 0.0, 0.3); // arbitrary 0.3, just says how fast to turn in
                                                                        // order to find a target
            } else {
                if (!isTargetCenter) {
                    if (NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx")
                            .getDouble(1000.0) > 5.0) { // within 5 degrees
                        Robot.drive.mecanumDrive.driveCartesian(0.0, 0.0, -0.3);
                        /*if (turretMotor.getEncoder().getPosition() < maxTurretEncoders) {
                            turretMotor.set(-0.1);
                        }*/
                    } else if (NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx")
                            .getDouble(1000.0) < -5.0) { // within 5 degrees
                        Robot.drive.mecanumDrive.driveCartesian(0.0, 0.0, 0.3);
                    } else {
                        Robot.drive.stopMotors();
                        testVelocity(getXDistanceFromCenterOfHub(NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(1000.0)));
                        isTargetCenter = true;
                    }
                } else {
                    if (Robot.intake.colorSensor2Activated) {
                        masterShooterMotor.set(ControlMode.Velocity, shooterWheelLinearVelocityToMotorVelocity(calculatedVelocity)); // should be the speed according to some formula that sets it to the // initial velocity we need
                        //hoodMotor.set(Constants.quadraticPositionAndSpeed(0.05, 0.15, calculatedAngle, hoodMotor.getEncoder().getPosition())); // what is the best way to do this
                        if(hoodMotor.getEncoder().getPosition() > calculatedAngle){
                            hoodMotor.set(-0.1);
                        } else {
                            hoodMotor.set(0.1);
                        }
                        // angleToShooterAngle(calculatedAngle)
                        Robot.intake.indexerMotor.getEncoder().setPosition(0.0);
        
                        // at this point, we need to check that the angle is right, the velocity is
                        // right, and the orientation of the robot is good
        
                        if ((masterShooterMotor.getSelectedSensorVelocity() >= 0.5) 
                        && (hoodMotor.getEncoder().getPosition() > calculatedAngle - 1) 
                        && (hoodMotor.getEncoder().getPosition() < calculatedAngle + 1)) {
                            hoodMotor.set(0.0);
                            // might have to be slightly less // find out conversion factor

                            //Robot.intake.indexerMotor.set(Constants.quadraticPositionAndSpeed(0.05, 0.4, 
                            //shooterIndexerEncoderCount, Robot.intake.indexerMotor.getEncoder().getPosition()));
                             // this doesn't have to be quadratic, probably just go at set speed until reach
                             // a certain encoder count, then set to 0
                            Robot.intake.indexerMotor.set(0.2);
                            
                            hoodMotor.set(0.0); // do I have to maintain hood position some code way? probably not
                            if (Robot.intake.indexerMotor.getEncoder().getPosition() > shooterIndexerEncoderCount) {
                                Robot.intake.indexerMotor.set(0.0);
                                Robot.intake.indexerMotor.getEncoder().setPosition(0.0);
                                Robot.intake.ballCount--;
                                Robot.intake.colorSensor2Activated = false;
                            }
                        } else {
                            Robot.intake.indexerMotor.set(0.0); // wait for shooter motor to re-speed up after shooting one
                        }
                    }
                }
            }
        }

        // here I have two options:
        // 1: use a timer and wait a certain amount of time (could be constant or based
        // on wanted motor speed)
        // 2: use an if statement and verify the speed is what we want. Contact with the
        // ball will slow down the motors. Does that matter?

        // if(timer is past certain point or motor is up to speed){
        // Robot.intake.indexerMotor.set(quadraticController(0.05,
        // Robot.intake.indexerMotorSpeed,
        // Robot.intake.indexerMotor.getEncoder().getPosition(), encoderCount));
        // if(Robot.intake.indexerMotor.getEncoder().getPosition() > encoderCount){
        // Robot.intake.indexerMotor.set(0.0);
        // Robot.intake.indexerMotor.getEncoder().setPosition(0.0);
        // Robot.intake.colorSensor2Activated = false;
        // }
        // }
    }
}