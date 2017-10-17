package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

class Drive_Straight extends Drive_Command {

    private boolean finished = false;
    private long startTime = 0;
    private double distance;
    private long timeToDistance = 500;
    private boolean isPositive = true;

    protected Drive_Straight(HardwareMap hardwareMap, double distance) {
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
