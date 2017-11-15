package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * TeleOp Mode
 * <p>
 * Enables control of the robot via the gamepad
 */

@SuppressWarnings({"FieldCanBeLocal", "unused"})
@TeleOp(name = "Competition_Robot", group = "Competition")
public class FTC_Robot extends LinearOpMode {

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
    private Servo SmallRelicServo;  //Open and close claw
    private Servo JewelServo;       //Jewel Servo

    //Driving
    private float throttle;
    private float direction;
    private float right;
    private float left;

    //Members for moving the GlyphMotors
    private static int GlyphMotorPosition = 0;      //Positions are 0:Down, 1:Mid, 2:Up
    private static final int        GLYPH_MOTOR_UP          = 1;
    private static final int        GLYPH_MOTOR_DOWN        = -1;
    private static final long       TIME_TO_MOVE_GLYPH      = 1000;    //Time to move the GlyphSlide one block height
    private static boolean GlyphMotorMoving = false;
    private static long glyphStartTime = 0;

    private static final double     COUNTS_PER_MOTOR_REV    = 1680 ;    // eg: TETRIX Motor Encoder
    private static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    private static final double     WHEEL_DIAMETER_INCHES   = 2.0 ;     // For figuring circumference
    private static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);
    private static final double     GLYPH_HEIGHT            = 6.25;      //6.5 inches, actual is 6 inches but we need to clear the cube
    private static final double     GLYPH_MOTOR_SPEED       = 0.8;      //this can be adjusted, possibly increase to 0.5

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

        //Jewel Servo
        JewelServo = hardwareMap.servo.get("servo4");

        //Relic arm motors and Servos
        RelicMotor = hardwareMap.dcMotor.get("motor6");
        BigRelicServo = hardwareMap.servo.get("servo1");
        SmallRelicServo = hardwareMap.servo.get("servo2");

        GlyphServoRight.setPosition(1.0);
        GlyphServoLeft.setPosition(0.0);

        JewelServo.setPosition(0.1);
        SmallRelicServo.setPosition(0.5);
        BigRelicServo.setPosition(0.5);

        waitForStart();

        // Ramp motor speeds till stop pressed.
        while(opModeIsActive()) {
            throttle = -gamepad1.left_stick_y;
            direction = -gamepad1.left_stick_x;

            right = throttle - direction;
            left = throttle + direction;

            // clip the right/left values so that the values never exceed +/- 1
            right = Range.clip(right, -1, 1);
            left = Range.clip(left, -1, 1);

            // scale the joystick value to make it easier to control
            // the robot more precisely at slower speeds
            right = (float)lowSensitivityScaleInput(right);
            left =  (float)lowSensitivityScaleInput(left);


            MotorFrontLeft.setPower(right); //motor 1
            MotorFrontRight.setPower(left); // motor 2
            MotorBackLeft.setPower(right); // motor 3
            MotorBackRight.setPower(left); // motor 4


            //Move Glyph motor UP and DOWN
            if(gamepad1.dpad_up && !GlyphMotorMoving){
                moveGlyphMotor(GLYPH_MOTOR_UP);
            }
            if(gamepad1.dpad_down && !GlyphMotorMoving){
                moveGlyphMotor(GLYPH_MOTOR_DOWN);
            }
            if(GlyphMotorMoving){
                //timeout for the glyph motor
                if(System.currentTimeMillis()-glyphStartTime>=TIME_TO_MOVE_GLYPH){
                    GlyphMotor.setPower(0.0);
                    GlyphMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    GlyphMotorMoving = false;
                }
            }


            // Glyph Servo Controller Code
            if(gamepad1.a){    //close
                GlyphServoRight.setPosition(0.20);
                GlyphServoLeft.setPosition(0.80);
            }
            else if(gamepad1.x){//open
                GlyphServoRight.setPosition(1.0);
                GlyphServoLeft.setPosition(0.0);
            }


            // Relic Linear slide motor
            while(gamepad1.dpad_left){
                RelicMotor.setPower(-0.5);
            }
            while(gamepad1.dpad_right){
                RelicMotor.setPower(0.5);
            }
            RelicMotor.setPower(0.0);


            // Relic Servo Controller Code
            if(gamepad1.b) {
                SmallRelicServo.setPosition(SmallRelicServo.getPosition()+0.05);
            }
            else if(gamepad1.y){
                SmallRelicServo.setPosition(SmallRelicServo.getPosition()-0.05);
            }


            //1 degree equals 0.00055 decimal
            if(gamepad1.right_bumper){
                BigRelicServo.setPosition(0.5);
            }
            if(gamepad1.left_bumper){
                BigRelicServo.setPosition(0.585);
            }

            //jewel servo up
            if  (gamepad1.left_stick_button) {
                JewelServo.setPosition(1.0);
            }//jewel servo down
            if  (gamepad1.right_stick_button) {
                JewelServo.setPosition(0.0);
            }
        }
    }

    private void moveGlyphMotor(int motorUpDown){
        GlyphMotorPosition += motorUpDown;
        if(GlyphMotorPosition<0 || GlyphMotorPosition>2){
            GlyphMotorPosition -= motorUpDown;
            return;
        }

        GlyphMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        GlyphMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if(motorUpDown == GLYPH_MOTOR_UP){
            GlyphMotor.setTargetPosition(GlyphMotor.getCurrentPosition() - (int)(GLYPH_HEIGHT * COUNTS_PER_INCH));
        }
        else if(motorUpDown == GLYPH_MOTOR_DOWN){
            GlyphMotor.setTargetPosition(GlyphMotor.getCurrentPosition() + (int)(GLYPH_HEIGHT * COUNTS_PER_INCH));
        }

        GlyphMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        glyphStartTime = System.currentTimeMillis();
        GlyphMotor.setPower(GLYPH_MOTOR_SPEED);
        GlyphMotorMoving = true;
    }

    private double lowSensitivityScaleInput(double joystickInputValue){
        boolean positive = true;
        if(joystickInputValue<0){
            positive = false;
        }
        joystickInputValue = Math.abs(joystickInputValue);
        double scaledValue;

        //joystickInputValue=0.4
        if(joystickInputValue<=0.8376){
            scaledValue = 0.25 * joystickInputValue;
        }
        else if(joystickInputValue>0.8375 && joystickInputValue<=0.9434){
            scaledValue = (joystickInputValue-0.38)*(joystickInputValue-0.38);
        }
        else{
            //scaledValue = (1.3*joystickInputValue*joystickInputValue) - ((0.23)*joystickInputValue);
            scaledValue = 12.5*(joystickInputValue-.92);
        }
        if(scaledValue>1.0)
            scaledValue = 1.0;

        if(!positive){
            return -scaledValue;
        }
        return scaledValue;
    }

    private double scaleInput(double dVal)  {
        double[] scaleArray = { 0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00 };

        // get the corresponding index for the scaleInput array.
        int index = (int) (dVal * 16.0);

        // index should be positive.
        if (index < 0) {
            index = -index;
        }

        // index cannot exceed size of array minus 1.
        if (index > 16) {
            index = 16;
        }

        // get value from the array.
        double dScale;
        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }

        // return scaled value.
        return dScale;
    }
}
