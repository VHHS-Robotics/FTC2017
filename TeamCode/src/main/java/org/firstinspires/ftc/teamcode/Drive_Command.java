package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public abstract class Drive_Command implements Command{
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
    }

    abstract void startMotors();

    protected void stopMotors(){
        MotorFrontLeft.setPower(0.0);
        MotorFrontRight.setPower(0.0);
        MotorBackLeft.setPower(0.0);
        MotorBackRight.setPower(0.0);
    }
}

class Drive_Straight extends Drive_Command {

    private boolean finished = false;
    private long startTime = 0;
    private double distance;
    private long timeToDistance = 500;
    private boolean isPositive = true;

    protected Drive_Straight(double distance) {
        super();
        if(distance<0.0){
            isPositive = false;
        }
        this.distance = distance;
        finished = false;
        //calculate time to get to distance with motor power
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

    void startMotors(){
        double motor_power = power;
        if(!isPositive){
            motor_power = -power;
        }
        MotorFrontLeft.setPower(motor_power);
        MotorFrontRight.setPower(motor_power);
        MotorBackLeft.setPower(motor_power);
        MotorBackRight.setPower(motor_power);
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