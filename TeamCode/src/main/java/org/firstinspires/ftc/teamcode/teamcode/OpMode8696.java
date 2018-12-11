package org.firstinspires.ftc.teamcode.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoControllerEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.teamcode.teamcode.buttonEvents.ButtonEvent;
import org.firstinspires.ftc.teamcode.teamcode.buttonEvents.ButtonEventManager;

// TODO: figure out how to use adb to graph some numbers.
// TODO: use it to optimize turning and driving in auto
// TODO: testing framework

/**
 * Superclass used by all of team 8696's opModes.
 * Contains all the methods and functionality that
 * any generic robot might have.
 */
public abstract class OpMode8696 extends LinearOpMode8696 {

    protected DcMotor leftFront;
    protected DcMotor rightFront;
    protected DcMotor lift = null;
    //private DcMotor extenderin = null;
    protected Servo flipper = null;
    protected CRServo latch = null;

    protected ElapsedTime runtime = new ElapsedTime();

    /**
     * Array containing the four motors in the drive train <br>
     * leftFront, rightFront, leftBack, rightBack
     *
     */
    protected DcMotor[] motors = new DcMotor[2];

    protected BNO055IMU imu;

    protected Acceleration gravity;
    protected Orientation angles;

    protected double driveSpeed = 1;

    public static final boolean RED = true;
    public static final boolean BLUE = false;

    /**
     * constant for how many encoder counts are
     * equivalent to rotating the robot one degree.
     */
    private static final double TURN_COEFFICIENT = 15 / 1.4;

    private long lastPeriodicCall = 0;
    protected VuforiaTrackable relicTemplate;

    protected ButtonEventManager buttonEvents;

    /**
     * Check if enough time has passed to call {@link #periodic()} again.
     * If so, call it. Should be called every iteration of the main loop.
     * @param ms number of milliseconds between every {@link #periodic()}
     * @see #periodic()
     */
    protected void periodic(long ms) {
        long t = System.currentTimeMillis();
        if (t >= lastPeriodicCall + ms) {
            lastPeriodicCall = t;
            periodic();
        }
    }

    /**
     * Method that will be called periodically.
     * @see #periodic(long ms)
     */
    protected void periodic() {

    }

    protected void initRobot() {
        initDriveTrain();
        lastPeriodicCall = System.currentTimeMillis();
        buttonEvents = new ButtonEventManager(gamepad1, gamepad2);
    }

    protected void initDriveTrain() {
        leftFront  = hardwareMap.get(DcMotor.class, "left_drive"); //0
        rightFront = hardwareMap.get(DcMotor.class, "right_drive");//1

        lift = hardwareMap.get(DcMotor.class, "lift");
        flipper = hardwareMap.get(Servo.class, "flipper");
        latch = hardwareMap.get(CRServo.class, "latch");

        motors[0] = leftFront;
        motors[1] = rightFront;

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        lift.setDirection(DcMotor.Direction.REVERSE);
    }


    /**
     * Add events for a specific button
     * @param gamepad which gamepad the button is on
     * @param event the event to be called
     */
    protected void addButtonEvent(int gamepad, ButtonEvent event) {
        buttonEvents.addButtonEvent(gamepad, event);
    }

    protected void initGyro() {
        telemetry.log().add("initializing imu...");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        boolean initialized = imu.initialize(parameters);

        telemetry.log().clear();
        telemetry.log().add(initialized ? "imu initialized successfully" : "imu failed to initialize");

    }

    protected double getGamepadAngle(double x, double y) {
        double angle  = Math.atan(-y / x);
        if (x < 0) angle += Math.PI;

        return angle;
    }

    protected double getMagnitude(double x, double y) {
        double magnitude = Math.sqrt(x*x + y*y);
        return Range.clip(magnitude, -1, 1);
    }

    /*protected void autoTurn(double angle, double power, double timeoutSeconds, boolean useEncoders) {
        autoTurn(angle, power, timeoutSeconds, useEncoders, new RunUntil() {
            public boolean stop() {
                return false;
            }
        });
    }

    protected void autoTurn(double angle, double power, double timeoutSeconds, boolean useEncoders, RunUntil runUntil) {
        runtime.reset();
        getGyroData();

        while (opModeIsActive() &&
                onHeading(angle, power, 1.0, useEncoders) &&
                runtime.seconds() < timeoutSeconds &&
                !runUntil.stop()) {
            idle();
            runMotors();
            for (DcMotor motor: motors) {
                motor.setPower(0);
            }
        }
        runMotors();
    }
    */

    protected boolean onHeading(double angle, double power, double maxError, boolean useEncoders) {
        getGyroData();

        double diff = adjustAngle(angle, angles.firstAngle);

        if (Math.abs(diff) > maxError) {
            if (useEncoders)
                encoderTurning(diff, power);
            else
                crappyTurning(diff, power);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Incrementally rotate the robot using the encoders, which makes use of the built in PID stuff.
     *
     * @param diff how much the robot needs to rotate to get to its target orientation.
     * @param power how much power to send to the motors.
     */
    private void encoderTurning(double diff, double power) {
        int prevCounts[] = new int[2];
        int count = 0;
        double prevPower = 0.0;

        for (DcMotor motor : motors) {
            prevCounts[count] = motor.getCurrentPosition();
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            count += 1;
        }
        leftFront .setTargetPosition(prevCounts[0] - ((int) (diff * TURN_COEFFICIENT)));
        rightFront.setTargetPosition(prevCounts[1] - (int)-(diff * TURN_COEFFICIENT));

        for (DcMotor motor : motors) {
            prevPower = motor.getPower();
            motor.setPower(prevPower + power);
        }
    }

    /**
     * Incrementally rotate the robot using just the powers. Adjust the power as it gets close to the target.
     *
     * @param diff how much the robot needs to rotate to get to its target orientation.
     * @param power how much power to send to the motors.
     */
    private void crappyTurning(double diff, double power) {
        for (DcMotor motor : motors) {
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            telemetry.addData(motor.getPortNumber() + "", motor.getCurrentPosition());
        }

        double temp = diff/30 * power;
        if (Math.abs(temp) < Math.abs(power))
            power = temp;
        else if (temp < 0) {
            power *= -1;
        }
        if (Math.abs(power) <= 0.15) {
            if (power < 0)
                power = -0.15;
            else
                power = 0.15;
        }

        telemetry.addData("power", power);
        telemetry.update();

        leftFront .setPower( power);
        rightFront.setPower(-power);
    }

    /**
     * Calculate the difference between two angles and set the output to be in a range.
     * Used for calculating how far the robot needs to turn to get to a target rotation.
     * @param target The target rotation
     * @param currentRotation The current robot orientation, (preferably) found by a gyro sensor.
     * @return Adjusted angle, -180 <= angle <= 180
     */
    protected double adjustAngle(double target, double currentRotation) {
        double diff = target - currentRotation;
        while (Math.abs(diff) > 180) {
            target += (diff >= 180) ? -360 : 360;
            diff = target - currentRotation;
        }
        return diff;
    }

    /*protected void autoDrive(double inches, double power, double timeoutSeconds) {
        autoDrive(inches, power, timeoutSeconds, new RunUntil() {
            @Override
            public boolean stop() {
                return false;
            }
        });
    }

    /*protected void autoDrive(double inches, double power, double timeoutSeconds, RunUntil runUntil) {
        for ( motor : motors) {
            motor.storePosition();
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            motor.setRelativeTarget((int) (-inches / (4 * Math.PI) * Motor8696.COUNTS_PER_REVOLUTION));

            motor.setPower(power);
        }

        runtime.reset();

        while (opModeIsActive() &&
                runtime.seconds() < timeoutSeconds &&
                Motor8696.motorsBusy(motors) &&
                !runUntil.stop()) {
            idle();
        }

        for (Motor8696 motor : motors) {
            motor.setPower(0);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    protected void runMotors() {
        for (Motor8696 motor : motors) {
            motor.setPower();
        }
    }

    protected void enableEncoders() {
        for (Motor8696 motor : motors) {
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /**
     * Stores the gyro data into instance fields.
     */
    protected void getGyroData() {
        gravity = imu.getGravity();
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    }
}
