package org.firstinspires.ftc.teamcode.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Superclass used by all of team 8696's opModes.
 * Contains all the methods and functionality that
 * any generic robot might have.
 */
public abstract class OpMode8696 extends LinearOpMode8696 {

    protected ElapsedTime runtime = new ElapsedTime();
    protected DcMotor lift = null;
    protected DcMotor leftPivot = null;
    protected DcMotor rightPivot = null;
    protected CRServo latch = null;
    protected DcMotor leftFront;
    protected DcMotor leftBack;
    protected DcMotor rightFront;
    protected DcMotor rightBack;
    protected DcMotor bigBertha;

    protected DcMotor[] motors = new DcMotor[4];

    /**
     * constant for how many encoder counts are
     * equivalent to rotating the robot one degree.
     */

    protected void initRobot() {
        initDriveTrain();
        initOtherStuff();
    }
    
    protected void initDogeCV(){

        telemetry.addData("DogeBoi TM", "Sampling dectector is good to go!");

        // Setup detector
        detector = new SamplingOrderDetector(); // Create the detector
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance()); // Initialize detector with app context and camera
        detector.useDefaults(); // Set detector to use default settings

        detector.downscale = 0.4; // How much to downscale the input frames

        // Optional tuning
        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.maxAreaScorer.weight = 0.001;

        detector.ratioScorer.weight = 15;
        detector.ratioScorer.perfectRatio = 1.0;

        detector.enable(); // Start detector
    }

    protected void initDriveTrain() {

        rightBack = hardwareMap.dcMotor.get("right_back");
        leftBack = hardwareMap.dcMotor.get("left_back");
        rightFront = hardwareMap.dcMotor.get("right_front");
        leftFront = hardwareMap.dcMotor.get("left_front");

        motors[0] = leftBack;
        motors[1] = rightBack;
        motors[2] = leftFront;
        motors[3] = rightFront;
    }

    protected void initOtherStuff() {
        lift = hardwareMap.get(DcMotor.class, "lift");
        latch = hardwareMap.get(CRServo.class, "latch");
        leftPivot = hardwareMap.get(DcMotor.class, "left_pivot");
        rightPivot = hardwareMap.get(DcMotor.class, "right_pivot");
        bigBertha = hardwareMap.get(DcMotor.class, "big Bertha");

        lift.setDirection(DcMotor.Direction.REVERSE);
        rightPivot.setDirection(DcMotor.Direction.REVERSE);
        rightPivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftPivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

}
