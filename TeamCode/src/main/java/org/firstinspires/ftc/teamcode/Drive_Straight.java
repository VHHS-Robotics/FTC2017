package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

class Drive_Straight extends Drive_Command {

    private boolean finished = false;
    private long startTime = 0;
    private double distance;
    private long timeToDistance = 500;
    private boolean isPositive = true;

    public Drive_Straight(HardwareMap hardwareMap, double distance) {
        super(hardwareMap);
        if(distance<0){
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
        while(System.currentTimeMillis()-startTime<timeToDistance){
            //wait
        }
        stopMotors();
        finished = true;
    }

    @Override
    public boolean isFinished() {
        return finished;
    }

    private void stopMotors(){
        MotorFrontLeft.setPower(0.0);
        MotorFrontRight.setPower(0.0);
        MotorBackLeft.setPower(0.0);
        MotorBackRight.setPower(0.0);
    }

    private void startMotors(){
        double motor_power = power;
        if(isPositive){
            motor_power = -power;
        }
        MotorFrontLeft.setPower(motor_power);
        MotorFrontRight.setPower(motor_power);
        MotorBackLeft.setPower(motor_power);
        MotorBackRight.setPower(motor_power);
    }
}
