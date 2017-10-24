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

    //Driving
    private float throttle;
    private float direction;
    private float right;
    private float left;

    //Members for moving the GlyphMotors
    private static int GlyphMotorPosition = 0;              //Positions are 0:Down, 1:Mid, 2:UP
    private static final int GLYPH_MOTOR_UP = 1;
    private static final int GLYPH_MOTOR_DOWN = -1;
    private static final long TIME_TO_MOVE_GLYPH = 2400;    //Time to move the GlyphSlide from bottom to top
    private static boolean GlyphMotorMoving = false;
    private static long glyphStartTime = 0;

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
        GlyphServoLeft = hardwareMap.servo.get("servo3");
        GlyphServoRight = hardwareMap.servo.get("servo4");

        //Relic arm motors and Servos
        RelicMotor = hardwareMap.dcMotor.get("motor6");
        BigRelicServo = hardwareMap.servo.get("servo5");
        SmallRelicServo = hardwareMap.servo.get("servo6");

        GlyphServoRight.setPosition(0.5);
        GlyphServoLeft.setPosition(0.5);
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
                if(System.currentTimeMillis()-glyphStartTime>=(TIME_TO_MOVE_GLYPH/3)){
                    GlyphMotorMoving = false;
                    glyphStartTime = 0;
                    GlyphMotor.setPower(0.0);
                }
            }


            // Glyph Servo Controller Code
            if(gamepad1.a){    //close
                GlyphServoRight.setPosition(GlyphServoRight.getPosition()+0.05);
                GlyphServoLeft.setPosition(GlyphServoLeft.getPosition()-0.05);
            }
            else if(gamepad1.x){//open
                GlyphServoRight.setPosition(GlyphServoRight.getPosition()-0.05);
                GlyphServoLeft.setPosition(GlyphServoLeft.getPosition()+0.05);
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


            //TODO: Get rid of while statements in the runOpMode while loop
            //1 degree equals 0.00055 decimal
            while(gamepad1.right_bumper){
                BigRelicServo.setPosition(0.5);
            }
            while(gamepad1.left_bumper){
                BigRelicServo.setPosition(0.585);
            }
        }
    }

    private void moveGlyphMotor(int motorUpDown){
        GlyphMotorPosition += motorUpDown;
        if(GlyphMotorPosition<0 || GlyphMotorPosition>2){
            GlyphMotorPosition -= motorUpDown;
            return;
        }

        glyphStartTime = System.currentTimeMillis();
        if(motorUpDown == GLYPH_MOTOR_UP){
            GlyphMotor.setPower(-0.5);
        }
        else if(motorUpDown == GLYPH_MOTOR_DOWN){
            GlyphMotor.setPower(0.42);
        }
        GlyphMotorMoving = true;
    }

    private double lowSensitivityScaleInput(double joystickInputValue){
        /* For values <=0.4 we use y=(1/4)x
           For values >0.4 we use y=(1.3)x^2 - (0.23)x
         */
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
