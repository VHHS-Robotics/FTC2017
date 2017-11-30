package org.firstinspires.ftc.teamcode.FTC_AUTO;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@SuppressWarnings("WeakerAccess")
public abstract class Drive_Command implements Command{
    //Gyro
    protected static ModernRoboticsI2cGyro GyroSensor;

    //Encoder Initialization
    protected static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // eg: TETRIX Motor Encoder
    protected static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    protected static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    protected static final double     CIRCUMFERENCE           = Math.PI * WHEEL_DIAMETER_INCHES;
    protected static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (CIRCUMFERENCE);
    protected static final double     DRIVE_SPEED             = 0.5;      //time to move 18in = 1 sec at 0.5 speed
    protected static final double     ROBOT_DIAMETER          = 15.0;     //inches
    protected static final double     ROBOT_CIRCUMFERENCE     = (Math.PI*ROBOT_DIAMETER);
    protected static final double     ONE_DEGREE_INCHES       = ROBOT_CIRCUMFERENCE/360.0;
    protected static boolean firstInit = true;
    protected static Telemetry telemetry;

    protected static DcMotor MotorFrontLeft;
    protected static DcMotor MotorFrontRight;
    protected static DcMotor MotorBackLeft;
    protected static DcMotor MotorBackRight;
    public static HardwareMap hardwareMap;

    protected Drive_Command(){
        if(firstInit) {
            initMotors();
            firstInit = false;
        }
    }

    static void initGyro(){
        telemetry.addLine("GETTING GYRO");
        telemetry.update();
        GyroSensor = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyroSensor");
        GyroSensor.calibrate();
        while(GyroSensor.isCalibrating()){
            //wait
        }
        telemetry.addLine("GYRO done calibrating, heading="+GyroSensor.getHeading());
        telemetry.update();
    }

    static void initMotors(){
        MotorFrontLeft = hardwareMap.dcMotor.get("motor1");
        MotorFrontRight = hardwareMap.dcMotor.get("motor2");
        MotorFrontRight.setDirection(DcMotor.Direction.REVERSE);
        MotorBackLeft = hardwareMap.dcMotor.get("motor3");
        MotorBackRight = hardwareMap.dcMotor.get("motor4");
        MotorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);
        firstInit = false;

        //resetEncoders();
        //setRunWithEncoders();
    }

    protected void setRunWithoutEncoders(){
        MotorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MotorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MotorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MotorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    protected void setRunWithEncoders(){
        MotorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    protected void resetEncoders(){
        MotorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    @SuppressWarnings("unused")
    abstract void runMotors();

    protected void stopMotors(){
        MotorFrontLeft.setPower(0.0);
        MotorFrontRight.setPower(0.0);
        MotorBackLeft.setPower(0.0);
        MotorBackRight.setPower(0.0);

        //setRunWithoutEncoders();
    }
}

@SuppressWarnings("WeakerAccess")
class Drive_Straight extends Drive_Command {

    private double distanceInches;

    protected Drive_Straight(double distanceInches) {
        super.setRunWithEncoders();
        this.distanceInches = distanceInches;
    }

    @Override
    public void startCommand() {
        long startTime = System.currentTimeMillis();
        long timeToDistance = (long) (distanceInches/CIRCUMFERENCE * 1500.0); //this is only true if DRIVE_SPEED = 0.5
        runMotors();
        while(System.currentTimeMillis()-startTime<=timeToDistance){
            //wait
        }
        stopMotors();
    }

    @Override
    public String printString() {
        return "Drive Straight "+distanceInches+" inches.";
    }

    void runMotors() {
        MotorFrontLeft.setTargetPosition(MotorFrontLeft.getCurrentPosition() + (int) (distanceInches * COUNTS_PER_INCH));
        MotorBackLeft.setTargetPosition(MotorBackLeft.getCurrentPosition() + (int) (distanceInches * COUNTS_PER_INCH));
        MotorFrontRight.setTargetPosition(MotorFrontRight.getCurrentPosition() + (int) (distanceInches * COUNTS_PER_INCH));
        MotorBackRight.setTargetPosition(MotorBackRight.getCurrentPosition() + (int) (distanceInches * COUNTS_PER_INCH));

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

@SuppressWarnings("WeakerAccess")
class Drive_Turn extends Drive_Command {

    private boolean turnRight = true;
    //private double distanceInches;
    //private double degrees;
    private double speed;
    //private double gyroStartHeading = GyroSensor.getHeading();
    private double gyroTargetHeading;
    private String leftRight = "LEFT";

    protected Drive_Turn(double targetHeading, double speed, String leftRight){
        if(targetHeading<0.0)
            return;

        super.resetEncoders();
        super.setRunWithoutEncoders();

        MotorFrontLeft = hardwareMap.dcMotor.get("motor1");
        MotorFrontRight = hardwareMap.dcMotor.get("motor2");
        MotorFrontRight.setDirection(DcMotor.Direction.REVERSE);
        MotorBackLeft = hardwareMap.dcMotor.get("motor3");
        MotorBackRight = hardwareMap.dcMotor.get("motor4");
        MotorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);

        if(leftRight.equals("LEFT")){
            turnRight = false;
        }
        else{
            turnRight = true;
        }
        this.speed = Math.abs(speed);   //no negative speed for turning

        //GYRO runs counter-clockwise from 0 to 360, GYRO does not have negative values
        gyroTargetHeading = targetHeading;
    }

    @Override
    public void startCommand() {
        //long startTime = System.currentTimeMillis();
       // long timeToDistance = 1000;//(long) (distanceInches/CIRCUMFERENCE * (-1666.67*speed+2083.33)); //time based on speed
        //runMotors();        //TODO: Try runMotors() in while loop
       // while(System.currentTimeMillis()-startTime<=timeToDistance){
      //      //wait
       // }
        telemetry.addLine("In turn before loop, targetHeading"+gyroTargetHeading+" currentHeading="+GyroSensor.getHeading());
        telemetry.update();
        while (true) {
            if(gyroTargetHeading>=2 && gyroTargetHeading<=357) {
                if (GyroSensor.getHeading() <= (gyroTargetHeading + 2) && (GyroSensor.getHeading() >= (gyroTargetHeading - 2))) {
                    telemetry.addLine("Broke in if");
                    break;
                }
            }
            else{   //in the case where 2>heading>358
                if(GyroSensor.getHeading()<=1 || GyroSensor.getHeading()>=358){
                    telemetry.addLine("Broke in else");
                    break;
                }
            }
            if(turnRight)
                telemetry.addLine("In Turn loop, RIGHT");
            else
                telemetry.addLine("In Turn loop, LEFT");
            telemetry.addLine("targetHeading=" + gyroTargetHeading + " currentHeading=" + GyroSensor.getHeading());
            telemetry.update();
            //runMotors();
            if(turnRight){
                MotorFrontLeft.setPower(speed);
                MotorBackLeft.setPower(speed);
                MotorFrontRight.setPower(-speed);
                MotorBackRight.setPower(-speed);
            }
            if(!turnRight){
                MotorFrontLeft.setPower(-speed);
                MotorBackLeft.setPower(-speed);
                MotorFrontRight.setPower(speed);
                MotorBackRight.setPower(speed);
            }
        }
        telemetry.addLine("after Turn While loop, stop motors, targetHeading="+gyroTargetHeading+" currentHeading="+GyroSensor.getHeading());
        telemetry.update();
        stopMotors();
    }

    @Override
    public String printString() {
        return "Turn "+ gyroTargetHeading +" degrees";
    }

    @Override
    void runMotors() {
        if(turnRight) {        //turn right
            /*
            MotorFrontLeft.setTargetPosition(MotorFrontLeft.getCurrentPosition() + (int) (distanceInches * COUNTS_PER_INCH));
            MotorBackLeft.setTargetPosition(MotorBackLeft.getCurrentPosition() + (int) (distanceInches * COUNTS_PER_INCH));
            MotorFrontRight.setTargetPosition(MotorFrontRight.getCurrentPosition() - (int) (distanceInches * COUNTS_PER_INCH));
            MotorBackRight.setTargetPosition(MotorBackRight.getCurrentPosition() - (int) (distanceInches * COUNTS_PER_INCH));
            */
            MotorFrontLeft.setPower(speed);
            MotorBackLeft.setPower(speed);
            MotorFrontRight.setPower(-speed);
            MotorBackRight.setPower(-speed);
        }
        else{                   //turn left
            /*
            MotorFrontLeft.setTargetPosition(MotorFrontLeft.getCurrentPosition() - (int) (distanceInches * COUNTS_PER_INCH));
            MotorBackLeft.setTargetPosition(MotorBackLeft.getCurrentPosition() - (int) (distanceInches * COUNTS_PER_INCH));
            MotorFrontRight.setTargetPosition(MotorFrontRight.getCurrentPosition() + (int) (distanceInches * COUNTS_PER_INCH));
            MotorBackRight.setTargetPosition(MotorBackRight.getCurrentPosition() + (int) (distanceInches * COUNTS_PER_INCH));
            */
            MotorFrontLeft.setPower(-speed);
            MotorBackLeft.setPower(-speed);
            MotorFrontRight.setPower(speed);
            MotorBackRight.setPower(speed);

        }
        /*
        MotorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        MotorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        MotorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        MotorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        MotorFrontLeft.setPower(Math.abs(speed));
        MotorBackLeft.setPower(Math.abs(speed));
        MotorFrontRight.setPower(Math.abs(speed));
        MotorBackRight.setPower(Math.abs(speed));
        */
    }

    @Override
    protected void stopMotors() {
        super.stopMotors();
    }
}