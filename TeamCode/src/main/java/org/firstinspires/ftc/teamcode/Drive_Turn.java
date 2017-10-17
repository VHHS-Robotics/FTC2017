package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

class Drive_Turn extends Drive_Command {

    private boolean isPositive = true;
    private double degrees = 0.0;
    private boolean finished = false;
    private long startTime = 0;
    private long timeToDistance = 250;

    public Drive_Turn(HardwareMap hardwareMap, double degrees){
        super(hardwareMap);
        if(degrees<0){
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
