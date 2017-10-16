package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by Daniel on 10/13/2017.
 */

public abstract class Drive_Command {
    public double power = 0.5;
    protected static DcMotor MotorFrontLeft;
    protected static DcMotor MotorFrontRight;
    protected static DcMotor MotorBackLeft;
    protected static DcMotor MotorBackRight;
    private static HardwareMap hardwareMap;

    public Drive_Command(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
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

    abstract void start();
    abstract boolean isFinished();
}
