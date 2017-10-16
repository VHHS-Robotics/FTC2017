package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

class Drive_Straight extends Drive_Command {

    private boolean finished = false;
    private long startTime = 0;
    private double distance;
    private long timeToDistance;

    public Drive_Straight(HardwareMap hardwareMap, double distance) {
        super(hardwareMap);
        this.distance = distance;
        finished = false;
        start();

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
        MotorFrontLeft.setPower(power);
        MotorFrontRight.setPower(power);
        MotorBackLeft.setPower(power);
        MotorBackRight.setPower(power);
    }
}
