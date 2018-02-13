package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro; //need this
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;



/**
 * Created by ivane on 11/28/2016.
 */

@Autonomous(name = "Robot2_Blue_Spot2_Encoder", group = "Linear Opmode")
public class Robot2_Blue_Spot2_Encoder extends LinearOpMode {

    // Define class members
    private DcMotor MotorFrontLeft;
    private DcMotor MotorFrontRight;
    private DcMotor MotorBackLeft;
    private DcMotor MotorBackRight;
    private DcMotor GlyphMotor;     //up and down very quickly
    private Servo GlyphServoRight;  // close and open
    private Servo GlyphServoLeft;   // close and open
    private DcMotor RelicMotor;     //Extending arm
    private Servo BigRelicServo;    // Vertical and Horizontal lift
    //    private Servo SmallRelicServo;  //Open and close claw
    private Servo SlideServo;
    private Servo JewelServo;       //Jewel Servo
    private ColorSensor ColorSensor; // Jewel Color Sensor


    ModernRoboticsI2cGyro GyroSensor; //need this

    //Vuforia  Variables
    protected VuforiaLocalizer vuforia;
    protected VuforiaTrackable relicTemplate;
    protected int glyphLocation = 0; //LEFT=0, CENTER=1, RIGHT=2


    private ElapsedTime runtime = new ElapsedTime();


    static final double COUNTS_PER_MOTOR_REV = 1120;    // eg: Andymark Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 1.0;
    static final double TURN_SPEED = 1.0;


    static final double HEADING_THRESHOLD = 3;      // As tight as we can make it with an integer gyro
    static final double P_TURN_COEFF = 0.1;     // Larger is more responsive, but also less stable
    static final double P_DRIVE_COEFF = 0.15;     // Larger is more responsive, but also less stable

    @Override
    public void runOpMode() throws InterruptedException {
// Wheels Motors
        MotorFrontLeft = hardwareMap.dcMotor.get("motor1");
        MotorFrontRight = hardwareMap.dcMotor.get("motor2");
        MotorFrontRight.setDirection(DcMotor.Direction.REVERSE);
        MotorBackLeft = hardwareMap.dcMotor.get("motor3");
        MotorBackRight = hardwareMap.dcMotor.get("motor4");
        MotorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);

        //Glyph arm Motors and Servos
        GlyphMotor = hardwareMap.dcMotor.get("motor5");
        GlyphServoLeft = hardwareMap.servo.get("servo5");
        GlyphServoRight = hardwareMap.servo.get("servo6");

        //Slide Servo
        SlideServo = hardwareMap.servo.get("servo3");

        //Jewel Servo
        JewelServo = hardwareMap.servo.get("servo4");

        //Relic arm motors and Servos
        RelicMotor = hardwareMap.dcMotor.get("motor6");
        BigRelicServo = hardwareMap.servo.get("servo1");
//        SmallRelicServo = hardwareMap.servo.get("servo2");

        //Sensors
        ColorSensor = hardwareMap.colorSensor.get("colorSensor");
        GyroSensor = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyroSensor");


        //initialize servos
        GlyphServoRight.setPosition(0.7);
        GlyphServoLeft.setPosition(0.4);
        JewelServo.setPosition(0.68);
        SlideServo.setPosition(1.0);
        BigRelicServo.setPosition(0.49);

        boolean turnright = false;

        /////////////////////////////////////////////////////////////////////////////
        VuMarkInit();
        resetEncoder();

        //calibrate gyro
        GyroSensor.calibrate();
        telemetry.addData(">", "Gyro Calibrating. Do Not move!");
        telemetry.update();

        // make sure the gyro is calibrated before continuing
        while (GyroSensor.isCalibrating()) {
            Thread.sleep(50);
            telemetry.addData(">", "time reseted");
            telemetry.update();
            runtime.reset();
            idle();
        }

        telemetry.addData(">", "Gyro Calibrated.  Press Start.");
        telemetry.addData("GyroZ value.", "%d", GyroSensor.getIntegratedZValue());
        telemetry.addData("Gyro heading.", "%d", GyroSensor.getHeading());
        telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
        telemetry.update();

        runWithEncoder();
        waitForStart();
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 30.0)) {
            telemetry.addData("I see", glyphLocation);
            telemetry.update();

            if (runtime.seconds() > 0.00 && runtime.seconds() < 0.50) { // bring servo to relic
                JewelServo.setPosition(0.0);
            }
            if (runtime.seconds() > 0.5 && runtime.seconds() < 1.00){ //algo to decide which ball to knock off
                //if turnright=false it will turn left
                checkVuMarkPosition();
                telemetry.addData("", glyphLocation);
                telemetry.update();
                if (ColorSensor.blue() >= ColorSensor.red() ){
                    turnright = true;
                }
                else {
                    turnright = false;
                }
            }
            if (runtime.seconds() > 1.00 && runtime.seconds() <1.50) {
                if (turnright == false) {
                    goLeft(.15);
                }
                if (turnright == true ){
                    goRight(.15);
                }
            }
            if (runtime.seconds() > 1.50 && runtime.seconds() <2.00) {
                JewelServo.setPosition(0.68);
                if (turnright == false) {
                    goRight(.15);
                }
                if (turnright == true ){
                    goLeft(.15);
                }
            }
            if (runtime.seconds() > 2.00 && runtime.seconds() <3.00) {
                stopRobot();
            }


            if (runtime.seconds() > 3.0 && runtime.seconds() < 7.5) {

                double targetHeading = 85.00;

                double currentHeading = GyroSensor.getHeading();
                double leftSpeed, rightSpeed;

                while (currentHeading < targetHeading || currentHeading == 359 || currentHeading == 358 || currentHeading == 357 || currentHeading == 356 || currentHeading == 355 || currentHeading == 354 || currentHeading == 353 || currentHeading == 352 || currentHeading == 351 || currentHeading == 350) {
                    leftSpeed = -0.1; // to turn left
                    rightSpeed = 0.1; // to turn left
                    MotorFrontLeft.setPower(leftSpeed);
                    MotorBackLeft.setPower(leftSpeed);
                    MotorFrontRight.setPower(rightSpeed);
                    MotorBackRight.setPower(rightSpeed);

                    currentHeading = GyroSensor.getHeading();
                    telemetry.addData("Target", "%5.2f", targetHeading);
                    telemetry.addData("Heading", "%d", GyroSensor.getHeading());
                    telemetry.addData("Current Heading", "%5.2f", currentHeading);

                    telemetry.addData("GyroZ value.", "%d", GyroSensor.getIntegratedZValue());
                    telemetry.update();
                }
                stopRobot();
            }
            if (runtime.seconds() > 7.5  && runtime.seconds() < 12.5) { //All of them off by +3 in?
                if(glyphLocation==0) {    //LEFT
                    encoderDrive(0.4, 31.0, 31.0, 11.5); // left
                    sleep(1000);
                }
                else if (glyphLocation==1) { //CENTER
                    encoderDrive(0.4, 38, 38, 12.0); // left
                    sleep(500);
                }
                else if (glyphLocation==2) {  //RIGHT
                    encoderDrive(0.4, 45.5, 45.5, 12.0); // left
                    sleep(750);
                }
            }
            if (runtime.seconds() > 12.5 && runtime.seconds() < 17.0) {

                double targetHeading = 175.00;

                double currentHeading = GyroSensor.getHeading();
                double leftSpeed, rightSpeed;

                while (currentHeading < targetHeading) {
                    leftSpeed = -0.1; // to turn left
                    rightSpeed = 0.1; // to turn left
                    MotorFrontLeft.setPower(leftSpeed);
                    MotorBackLeft.setPower(leftSpeed);
                    MotorFrontRight.setPower(rightSpeed);
                    MotorBackRight.setPower(rightSpeed);

                    currentHeading = GyroSensor.getHeading();
                    telemetry.addData("Target", "%5.2f", targetHeading);
                    telemetry.addData("Heading", "%d", GyroSensor.getHeading());
                    telemetry.addData("Current Heading", "%5.2f", currentHeading);

                    telemetry.addData("GyroZ value.", "%d", GyroSensor.getIntegratedZValue());
                    telemetry.update();
                }
                stopRobot();
            }
            if (runtime.seconds() > 17.0  && runtime.seconds() < 18.0) {
                stopRobot();
            }
            if (runtime.seconds() > 18.0  && runtime.seconds() < 19.0) { // following the white line
                goStraight(.425);
            }
            if (runtime.seconds() > 19.0  && runtime.seconds() < 20.0) { // following the white line
                stopRobot();
            }
            if (runtime.seconds() > 20.0  && runtime.seconds() < 20.5) { // following the white line
                GlyphServoRight.setPosition(1.0);
                GlyphServoLeft.setPosition(0.0);
            }
            if (runtime.seconds() > 20.5  && runtime.seconds() < 21.0) { // following the white line
                goBack(.3);
            }
            if (runtime.seconds() > 21.0  && runtime.seconds() < 21.5) { // following the white line
                GlyphServoRight.setPosition(0.7);
                GlyphServoLeft.setPosition(0.4);
            }
            if (runtime.seconds() > 21.5  && runtime.seconds() < 22.0) { // following the white line
                goStraight(.55);
            }
            if (runtime.seconds() > 22.0  && runtime.seconds() < 22.5) { // following the white line
                goLeft(.3);
            }
            if (runtime.seconds() > 22.5  && runtime.seconds() < 23.00) { // following the white line
                goBack(.3);
            }
            if (runtime.seconds() > 23.00  && runtime.seconds() < 30.00) { // following the white line
                stopRobot();
            }
            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }

        GyroSensor.resetZAxisIntegrator();
        telemetry.addData("GyroZ value.", "%d", GyroSensor.getIntegratedZValue());
        telemetry.addData("Gyro heading.", "%d", GyroSensor.getHeading());
        telemetry.update();
    }

    protected void VuMarkInit(){
        VuforiaTrackables relicTrackables;

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AXHqge7/////AAAAGenGHENAYE1qoSAsuGF8810cJsPkYPfPvtKoveVcdLDzK46GCUu14Tc8oGqzdynacNWZNLD1guU8EZbFPoQNdH2SucwUFBo/bOsW0mlHtXp/eYmyAtuLjwVp4HtvH0Hit5G4YthBouXAa89954OBiG5cBPCWilFG+2ZCzQ4IPEN7ams3nyksoBZxCD03XHbSEdJ+AkRC14iZmjnWKDxlsCg70C33QiDHQ0KGV8NB2uM4WUFoRrjXK87VAry9nGMzf4p58AD6SViaov0kDo6oFtIGDZ2Ot9QrhVlCtL1GaJiNeUXENmAyrGBvkB1NNaMPFUsJnlnNoR5KmaYYKHWOEtl8K/T6rXVaNnzeXhcyHK1e";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
        relicTrackables.activate();
    }

    protected RelicRecoveryVuMark VuMarkCheck(){
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
        if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
            telemetry.addData("VuMark", "%s visible", vuMark);
        }
        else {
            telemetry.addData("VuMark", "not visible");
        }
        telemetry.update();
        return vuMark;
    }

    protected void checkVuMarkPosition(){
        //Check Vumark and set glyphLocation based on Vumark seen
        if(VuMarkCheck().equals(RelicRecoveryVuMark.LEFT)){
            glyphLocation = 0;
            telemetry.addLine("Left");
        }
        else if(VuMarkCheck().equals(RelicRecoveryVuMark.CENTER)){
            glyphLocation = 1;
            telemetry.addLine("Center");
        }
        else if(VuMarkCheck().equals(RelicRecoveryVuMark.RIGHT)){
            glyphLocation = 2;
            telemetry.addLine("Right");
        }
        else{   //default to center if no relic found
            glyphLocation = 1;
            telemetry.addLine("No VuMark found Found");
        }
        telemetry.update();
    }

    public void gyroTurnleft(double speed, double angle)
            throws InterruptedException {

        boolean onTarget = false;
        double leftSpeed, rightSpeed;
        //get the heading here

        //loop until the correct angle within a threshold
        do {
            double variance;
            variance = GyroSensor.getHeading() - angle;

            if (Math.abs(variance) < HEADING_THRESHOLD) {
                onTarget = true; // Add stop robot here?
            } else {
                leftSpeed = -0.1; // to turn left
                rightSpeed = 0.1; // to turn left
                MotorFrontLeft.setPower(leftSpeed);
                MotorBackLeft.setPower(leftSpeed);
                MotorFrontRight.setPower(rightSpeed);
                MotorBackRight.setPower(rightSpeed);
            }
            telemetry.addData("Target", "%5.2f", angle);
            telemetry.addData("Heading", "%d", GyroSensor.getHeading());
            telemetry.addData("Variance", "%5.2f", variance);

            telemetry.addData("GyroZ value.", "%d", GyroSensor.getIntegratedZValue());
            telemetry.update();
        } while (!onTarget); // Add stop robot or here?

    }
    public void gyroTurnRight(double speed, double angle)
            throws InterruptedException {

        boolean onTarget = false;
        double leftSpeed, rightSpeed;
        //get the heading here

        //loop until the correct angle within a threshold
        do {
            double variance;
            variance = GyroSensor.getHeading() - angle;

            if (Math.abs(variance) < HEADING_THRESHOLD) {
                onTarget = true;
            } else {
                leftSpeed = 0.1; // to turn right
                rightSpeed = -0.1; // to turn right
                MotorFrontLeft.setPower(leftSpeed);
                MotorBackLeft.setPower(leftSpeed);
                MotorFrontRight.setPower(rightSpeed);
                MotorBackRight.setPower(rightSpeed);
            }
            telemetry.addData("Target", "%5.2f", angle);
            telemetry.addData("Heading", "%d", GyroSensor.getHeading());
            telemetry.addData("Variance", "%5.2f", variance);

            telemetry.addData("GyroZ value.", "%d", GyroSensor.getIntegratedZValue());
            telemetry.update();
        } while (!onTarget);

    }

    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) throws InterruptedException {
        int newFrontLeftTarget;
        int newBackLeftTarget;
        int newFrontRightTarget;
        int newBackRightTarget;
        resetEncoder();
        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newFrontLeftTarget = MotorFrontLeft.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newBackLeftTarget = MotorBackLeft.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newFrontRightTarget = MotorFrontRight.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            newBackRightTarget = MotorBackRight.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);

            MotorFrontLeft.setTargetPosition(newFrontLeftTarget);
            MotorBackLeft.setTargetPosition(newBackLeftTarget);
            MotorFrontRight.setTargetPosition(newFrontRightTarget);
            MotorBackRight.setTargetPosition(newBackRightTarget);


            // Turn On RUN_TO_POSITION
            MotorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            MotorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            MotorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            MotorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            //runtime.reset();

            MotorFrontLeft.setPower(Math.abs(speed));
            MotorBackLeft.setPower(Math.abs(speed));
            MotorFrontRight.setPower(Math.abs(speed));
            MotorBackRight.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (MotorFrontLeft.isBusy() && MotorFrontRight.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newFrontLeftTarget, newFrontRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        MotorFrontLeft.getCurrentPosition(),
                        MotorFrontRight.getCurrentPosition());
                telemetry.update();

                // Allow time for other processes to run.
                idle();
            }

            // Stop all motion;
            stopRobot();

            // Turn off RUN_TO_POSITION
            runWithEncoder();


            //  sleep(250);   // optional pause after each move
        }
    }

    public void resetEncoder() {
        MotorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void runWithEncoder() {
        MotorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void goStraight(double intensity) {
        MotorFrontLeft.setPower(1.0 * intensity);
        MotorFrontRight.setPower(1.0 * intensity);
        MotorBackLeft.setPower(1.0 * intensity);
        MotorBackRight.setPower(1.0 * intensity);
    }

    public void goBack(double intensity) {
        MotorFrontLeft.setPower(-1.0 * intensity);
        MotorFrontRight.setPower(-1.0 * intensity);
        MotorBackLeft.setPower(-1.0 * intensity);
        MotorBackRight.setPower(-1.0 * intensity);
    }

    public void goRight(double intensity) {
        MotorFrontLeft.setPower(1.0 * intensity);
        MotorFrontRight.setPower(-1.0 * intensity);
        MotorBackLeft.setPower(1.0 * intensity);
        MotorBackRight.setPower(-1.0 * intensity);

    }

    public void goLeft(double intensity) {
        MotorFrontLeft.setPower(-1.0 * intensity);
        MotorFrontRight.setPower(1.0 * intensity);
        MotorBackLeft.setPower(-1.0 * intensity);
        MotorBackRight.setPower(1.0 * intensity);

    }

    public void stopRobot() {
        MotorFrontLeft.setPower(0.0);
        MotorFrontRight.setPower(0.0);
        MotorBackLeft.setPower(0.0);
        MotorBackRight.setPower(0.0);
    }

    public void goLeftCurved(double intensity) {
        MotorFrontLeft.setPower(-1.0 * intensity);
        MotorBackLeft.setPower(-1.0 * intensity);
    }

    public void goRightCurved(double intensity) {
        MotorFrontRight.setPower(-1.0 * intensity);
        MotorBackRight.setPower(-1.0 * intensity);
    }


}