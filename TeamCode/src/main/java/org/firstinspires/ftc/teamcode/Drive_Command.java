package org.firstinspires.ftc.teamcode;

import android.drm.DrmStore;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public abstract class Drive_Command implements Command{
    //Encoder Initialization
    protected static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // eg: TETRIX Motor Encoder
    protected static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    protected static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    protected static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    protected static final double     DRIVE_SPEED             = 0.5; //time to move 18in = 1 sec
    protected static final double     TURN_SPEED              = 1.0;

    protected double power = 0.3;
    protected static DcMotor MotorFrontLeft;
    protected static DcMotor MotorFrontRight;
    protected static DcMotor MotorBackLeft;
    protected static DcMotor MotorBackRight;
    public static HardwareMap hardwareMap;

    public Drive_Command(){
        initializeMotors();
    }

    private void initializeMotors(){
        MotorFrontLeft = hardwareMap.dcMotor.get("motor1");
        MotorFrontRight = hardwareMap.dcMotor.get("motor2");
        MotorFrontRight.setDirection(DcMotor.Direction.REVERSE);
        MotorBackLeft = hardwareMap.dcMotor.get("motor3");
        MotorBackRight = hardwareMap.dcMotor.get("motor4");
        MotorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);

        MotorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        MotorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    abstract void startMotors();

    protected void stopMotors(){
        MotorFrontLeft.setPower(0.0);
        MotorFrontRight.setPower(0.0);
        MotorBackLeft.setPower(0.0);
        MotorBackRight.setPower(0.0);

        MotorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}

class Drive_Straight extends Drive_Command {

    private boolean finished = false;
    private long startTime = 0;
    private double leftInches, rightInches;
    private long timeToDistance = 0;
    private boolean isPositive = true;

    protected Drive_Straight(double leftInches, double rightInches) {
        super();
        this.leftInches = leftInches;
        this.rightInches = rightInches;
        finished = false;
        //calculate time to get to distance with motor power
    }

    @Override
    public void start() {
      //  startTime = System.currentTimeMillis();
        startMotors();
        //timeToDistance = 2000;//(long) (Math.abs(leftInches/speed) * 1000);
        while(MotorBackLeft.isBusy() || MotorBackRight.isBusy() || MotorFrontLeft.isBusy() || MotorFrontRight.isBusy()){
            //wait
        }
        stopMotors();
        finished = true;
    }

    @Override
    public boolean isFinished() {
        return finished;
    }

    void startMotors() {
        /*
        double motor_power = power;
        if(!isPositive){
            motor_power = -power;
        }
        MotorFrontLeft.setPower(motor_power);
        MotorFrontRight.setPower(motor_power);
        MotorBackLeft.setPower(motor_power);
        MotorBackRight.setPower(motor_power);
        */
        MotorFrontLeft.setTargetPosition(MotorFrontLeft.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH));
        MotorBackLeft.setTargetPosition(MotorBackLeft.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH));
        MotorFrontRight.setTargetPosition(MotorFrontRight.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH));
        MotorBackRight.setTargetPosition(MotorBackRight.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH));

        MotorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        MotorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        MotorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        MotorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        MotorFrontLeft.setPower(Math.abs(DRIVE_SPEED));
        MotorBackLeft.setPower(Math.abs(DRIVE_SPEED));
        MotorFrontRight.setPower(Math.abs(DRIVE_SPEED));
        MotorBackRight.setPower(Math.abs(DRIVE_SPEED));
    }

    @Override
    protected void stopMotors() {
        super.stopMotors();
    }
}

class Drive_Turn extends Drive_Command {

    private boolean isPositive = true;
    private double degrees = 0.0;
    private boolean finished = false;
    private long startTime = 0;
    private long timeToDistance = 250;

    public Drive_Turn(double degrees){
        super();
        if(degrees<0.0){
            isPositive = false;
        }
        this.degrees = degrees;
        finished = false;
    }

    @Override
    public void start() {
        startTime = System.currentTimeMillis();
        startMotors();
        while(System.currentTimeMillis()-startTime<=timeToDistance){
            //wait
        }
        stopMotors();
        finished = true;
    }

    @Override
    public boolean isFinished() {
        return finished;
    }

    @Override
    void startMotors() {
        if(isPositive) {
            MotorFrontLeft.setPower(power);
            MotorBackLeft.setPower(power);
            MotorFrontRight.setPower(-power);
            MotorBackRight.setPower(-power);
        }
        else{
            MotorFrontLeft.setPower(-power);
            MotorBackLeft.setPower(-power);
            MotorFrontRight.setPower(power);
            MotorBackRight.setPower(power);
        }
    }

    @Override
    protected void stopMotors() {
        super.stopMotors();
    }
}