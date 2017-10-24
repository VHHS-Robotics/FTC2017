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
@TeleOp(name = "Competition_Robot", group = "Competition")
public class FTC_Robot extends LinearOpMode {

    // Define class members
    DcMotor MotorFrontLeft;
    DcMotor MotorFrontRight;
    DcMotor MotorBackLeft;
    DcMotor MotorBackRight;
    DcMotor GlyphMotor; // up and down very quickly
    Servo GlyphServoRight; // close and open
    Servo GlyphServoLeft; // close and open
    DcMotor RelicMotor; //Extending arm
    Servo BigRelicServo; // Vertical and Horizontal lift
    Servo SmallRelicServo; //Open and close claw

    float throttle;
    float direction;
    float right;
    float left;

    //TODO get ride of extra arm throttle
    private float Armthrottle;
    float Armdirection;
    float ArmUp;
    float ArmDown;

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

        // Command_Wait for the start button
        telemetry.addData(">", "Press Start to run Motors.");
        telemetry.update();
        waitForStart();

        GlyphServoRight.setPosition(0.5);
        GlyphServoLeft.setPosition(0.5);
        SmallRelicServo.setPosition(0.5);
        BigRelicServo.setPosition(0.5);

        // Wheels Motors
        throttle = -gamepad1.left_stick_y;
        direction = gamepad1.left_stick_x;

        right = throttle - direction;
        left = throttle + direction;

        // clip the right/left values so that the values never exceed +/- 1
        right = Range.clip(right, -1, 1);
        left = Range.clip(left, -1, 1);

        // scale the joystick value to make it easier to control
        // the robot more precisely at slower speeds.
        //right = (float)scaleInput(right);
        //left =  (float)scaleInput(left);
        right = (float)lowSensitivityScaleInput(right);
        left =  (float)lowSensitivityScaleInput(left);

        // Arm Motor
        Armthrottle = gamepad1.left_trigger; //Dont think
        Armdirection = gamepad1.right_trigger;

        ArmUp = Armthrottle - Armdirection;
        ArmDown = Armthrottle + Armdirection;

        // clip the right/left values so that the values never exceed +/- 1
        ArmUp = Range.clip(ArmUp, -0.5f, 0.5f);
        ArmDown = Range.clip(ArmDown, -0.5f, 0.5f);

        // scale the joystick value to make it easier to control
        // the robot more precisely at slower speeds.
        ArmUp = (float)scaleInput(ArmUp);
        ArmDown =  (float)scaleInput(ArmDown);


        // Ramp motor speeds till stop pressed.
        while(opModeIsActive()) {
            //TODO: Remove negaitves and test joystick direction with lowSensitivityScaleInput(double)
            throttle = -gamepad1.left_stick_y;
            direction = -gamepad1.left_stick_x;

            right = throttle - direction;
            left = throttle + direction;

            // clip the right/left values so that the values never exceed +/- 1
            right = Range.clip(right, -1, 1);
            left = Range.clip(left, -1, 1);

            // scale the joystick value to make it easier to control
            // the robot more precisely at slower speeds.
            //right = (float)scaleInput(right);
            //left =  (float)scaleInput(left);
            right = (float)lowSensitivityScaleInput(right);
            left =  (float)lowSensitivityScaleInput(left);



            // Arm Motor
            Armthrottle = gamepad1.left_trigger;
            Armdirection = gamepad1.right_trigger;

            ArmUp = Armthrottle - Armdirection;
            ArmDown = Armthrottle + Armdirection;

            // clip the right/left values so that the values never exceed +/- 1
            ArmUp = Range.clip(ArmUp, -0.5f, 0.5f);
            ArmDown = Range.clip(ArmDown, -0.5f, 0.5f);

            // scale the joystick value to make it easier to control
            // the robot more precisely at slower speeds.
            ArmUp = (float)scaleInput(ArmUp);
            ArmDown =  (float)scaleInput(ArmDown);



            // End of Arm Motor
            MotorFrontLeft.setPower(right); //motor 1
            MotorFrontRight.setPower(left); // motor 2
            MotorBackLeft.setPower(right); // motor 3
            MotorBackRight.setPower(left); // motor 4

            //1.2 seconds for full movement
            // Glyph Linear slide motor
            while (gamepad1.dpad_up){
                GlyphMotor.setPower(-0.5);
            }

            while (gamepad1.dpad_down){
                GlyphMotor.setPower(0.5);
            }
            GlyphMotor.setPower(0.0);

            // Glyph Servo Controller Code
            if (gamepad1.a)//close
            {
                GlyphServoRight.setPosition(GlyphServoRight.getPosition()+0.05);
                GlyphServoLeft.setPosition(GlyphServoLeft.getPosition()-0.05);

            }
            else if (gamepad1.x){//open
                GlyphServoRight.setPosition(GlyphServoRight.getPosition()-0.05);
                GlyphServoLeft.setPosition(GlyphServoLeft.getPosition()+0.05);
            }

            // Relic Linear slide motor
            while (gamepad1.dpad_left){
                RelicMotor.setPower(-0.5);
            }

            while (gamepad1.dpad_right){
                RelicMotor.setPower(0.5);
            }
            RelicMotor.setPower(0.0);

            // Relic Servo Controller Code
            if (gamepad1.b)
            {
                SmallRelicServo.setPosition(SmallRelicServo.getPosition()+0.05);
            }
            else if (gamepad1.y){
                SmallRelicServo.setPosition(SmallRelicServo.getPosition()-0.05);
            }

            //1 degree equals 0.00055 decimal
            while(gamepad1.right_bumper){
                BigRelicServo.setPosition(0.5);
            }
            while(gamepad1.left_bumper){
                BigRelicServo.setPosition(0.585);
            }


            // Display the current value
            telemetry.addData("Motor Right", "%5.2f", right);
            telemetry.addData("Motor Left Power", "%5.2f", left);
            telemetry.addData(">", "Press Stop to end test." );
            telemetry.update();

            telemetry.addData(">", "Done");
            telemetry.update();
        }
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
        double scaledValue = 0.0;

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

    double scaleInput(double dVal)  {
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
        double dScale = 0.0;
        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }

        // return scaled value.
        return dScale;
    }
}
