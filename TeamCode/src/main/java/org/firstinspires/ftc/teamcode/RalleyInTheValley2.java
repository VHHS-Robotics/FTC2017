import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;


/**
 * Created by ivane on 11/28/2016.
 */

@Autonomous(name = "RallyInTheValley2", group = "Linear Opmode")
public class RalleyInTheValley2 extends LinearOpMode {

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



        runWithEncoder();
        waitForStart();
        runtime.reset();
        telemetry.addLine("I am ready");
        telemetry.update();
        while (opModeIsActive()) {
            //positive is to turn right
//            encoderDrive(0.6,24,24);
//            JewelServo.setPosition(0.2);
//            sleep(500);
//            JewelServo.setPosition(0.68);
//            encoderDrive(0.6, 72, 72);
            encoderTurn(0.6,-90);
//            encoderDrive(0.6, 36, 36);
//            encoderTurn(0.6,-90);
//            encoderDrive(0.6, 12, 12);
//            encoderTurn(0.6,-45);
//            encoderDrive(0.6, 34, 34);
//            encoderTurn(0.6, 180);
//            encoderDrive(0.6, 28, 28);
//            encoderDrive(0.6, -28, -28);
//            encoderTurn(0.6, -180);
//            JewelServo.setPosition(0.2);
//            GlyphMotor.setPower(0.6);
//            sleep(500);
//            JewelServo.setPosition(0.68);
//            GlyphMotor.setPower(-0.6);
//            sleep(500);
//            GlyphMotor.setPower(0.0);
//            encoderTurn(0.6, -45);
//            encoderDrive(0.6, 6, 6);
//            encoderTurn(0.6,-90);
//            encoderDrive(0.6, 60, 60);
//            encoderTurn(0.6, -180);
//            encoderDrive(0.6,36,36);
//            JewelServo.setPosition(0.2);
//            sleep(500);
//            JewelServo.setPosition(0.68); //end of page
//            encoderTurn(0.6, 180);
//            encoderDrive(0.6,24,24);
//            encoderTurn(0.6,-90);
//            encoderDrive(0.6,6,6);
//            JewelServo.setPosition(0.2);

            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
            return;
        }

        GyroSensor.resetZAxisIntegrator();
        telemetry.addData("GyroZ value.", "%d", GyroSensor.getIntegratedZValue());
        telemetry.addData("Gyro heading.", "%d", GyroSensor.getHeading());
        telemetry.update();
    }


    public void encoderTurn(double speed,double targetHeading){
        int newFrontLeftTarget = 0;
        int newBackLeftTarget = 0;
        int newFrontRightTarget = 0;
        int newBackRightTarget = 0;
        resetEncoder();
        double leftInches = Math.abs(targetHeading*2*Math.PI*7/360);
        double rightInches = Math.abs(targetHeading*2*Math.PI*7/360);

        newFrontLeftTarget = MotorFrontLeft.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
        newBackLeftTarget = MotorBackLeft.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
        newFrontRightTarget = MotorFrontRight.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
        newBackRightTarget = MotorBackRight.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);

        MotorFrontLeft.setTargetPosition(newFrontLeftTarget);
        MotorBackLeft.setTargetPosition(newBackLeftTarget);
        MotorFrontRight.setTargetPosition(newFrontRightTarget);
        MotorBackRight.setTargetPosition(newBackRightTarget);
        if (targetHeading > 0){
            MotorFrontLeft.setPower(speed);
            MotorBackLeft.setPower(speed);
            MotorFrontRight.setPower(-speed);
            MotorBackRight.setPower(-speed);
        }
        else{
            MotorFrontLeft.setPower(-speed);
            MotorBackLeft.setPower(-speed);
            MotorFrontRight.setPower(speed);
            MotorBackRight.setPower(speed);
        }

        while(Math.abs(MotorFrontLeft.getCurrentPosition()) < MotorFrontLeft.getTargetPosition()) {
            telemetry.addLine("Target Heading: "+MotorBackLeft.getTargetPosition());
            telemetry.addLine("Current Heading: "+MotorFrontLeft.getCurrentPosition());
            telemetry.update();
        }
        stopRobot();
    }

    public void gyroTurnleft(double targetHeading)
            throws InterruptedException {


        double currentHeading = GyroSensor.getHeading();
        double leftSpeed, rightSpeed;

        while (currentHeading < targetHeading || currentHeading == 359 || currentHeading == 358 || currentHeading == 357 || currentHeading == 356 || currentHeading == 355 || currentHeading == 354 || currentHeading == 353 || currentHeading == 352 || currentHeading == 351 || currentHeading == 350) {
            leftSpeed = -0.3; // to turn left
            rightSpeed = 0.3; // to turn left
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
    public void gyroTurnRight(double targetHeading)
            throws InterruptedException {

        double currentHeading = GyroSensor.getHeading();
        double leftSpeed, rightSpeed;

        while (currentHeading < targetHeading || currentHeading == 0 || currentHeading == 1 || currentHeading == 2 || currentHeading == 3 || currentHeading == 4 || currentHeading == 5 || currentHeading == 6 || currentHeading == 7 || currentHeading == 8 || currentHeading == 9) {
            leftSpeed = 0.3; // to turn right
            rightSpeed = -0.3; // to turn right
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

    public void encoderDrive(double speed,double leftInches, double rightInches) throws InterruptedException {
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
            if (leftInches < 0){
                MotorFrontLeft.setPower(-Math.abs(speed));
                MotorBackLeft.setPower(-Math.abs(speed));
            } else {
                MotorFrontLeft.setPower(Math.abs(speed));
                MotorBackLeft.setPower(Math.abs(speed));
            }
            if (rightInches < 0) {
                MotorFrontRight.setPower(-Math.abs(speed));
                MotorBackRight.setPower(-Math.abs(speed));
            } else {
                MotorFrontRight.setPower(Math.abs(speed));
                MotorBackRight.setPower(Math.abs(speed));
            }

            if(leftInches < 0){
                while(MotorFrontLeft.getCurrentPosition() > MotorFrontLeft.getTargetPosition()) {
                    telemetry.addLine("Target Position: "+MotorBackLeft.getTargetPosition());
                    telemetry.addLine("Current Position: "+MotorFrontLeft.getCurrentPosition());
                    telemetry.update();
                }
            } else{
                while(MotorFrontLeft.getCurrentPosition() < MotorFrontLeft.getTargetPosition()) {
                    telemetry.addLine("Target Position: "+MotorBackLeft.getTargetPosition());
                    telemetry.addLine("Current Position: "+MotorFrontLeft.getCurrentPosition());
                    telemetry.update();
                }
            }
            stopRobot();
        }
    }

    public void resetEncoder() {
        MotorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        runWithEncoder();
    }

    public void runWithEncoder() {
        MotorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    public void stopRobot() {
        MotorFrontLeft.setPower(0.0);
        MotorFrontRight.setPower(0.0);
        MotorBackLeft.setPower(0.0);
        MotorBackRight.setPower(0.0);
    }


}