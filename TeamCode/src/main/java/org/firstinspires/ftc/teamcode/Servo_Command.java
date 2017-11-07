package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

@SuppressWarnings("WeakerAccess")
abstract class Servo_Command implements Command {
    protected final static String OPEN = "OPEN";
    protected final static String CLOSE = "CLOSE";
    protected final static String UP = "UP";
    protected final static String DOWN = "DOWN";

    protected DcMotor GlyphMotor;       //up and down very quickly
    protected Servo GlyphServoRight;    //close and open
    protected Servo GlyphServoLeft;     //close and open
    protected Servo JewelServo;         //Move jewel sensor up and down
    protected DcMotor RelicMotor;
    protected Servo BigRelicServo;
    protected Servo SmallRelicServo;

    public static HardwareMap hardwareMap;

    protected Servo_Command(){
        initializeServos();
    }

    private void initializeServos(){
        //Glyph arm Motors and Servos
        GlyphMotor = hardwareMap.dcMotor.get("motor5");
        GlyphServoLeft = hardwareMap.servo.get("servo5");
        GlyphServoRight = hardwareMap.servo.get("servo6");
        JewelServo = hardwareMap.servo.get("servo4");

        //Relic arm motors and Servos
        RelicMotor = hardwareMap.dcMotor.get("motor6");
        BigRelicServo = hardwareMap.servo.get("servo1");
        SmallRelicServo = hardwareMap.servo.get("servo2");


        GlyphServoRight.setPosition(0.5);
        GlyphServoLeft.setPosition(0.5);
        JewelServo.setPosition(0.1);
        BigRelicServo.setPosition(0.5);
    }
}

@SuppressWarnings({"FieldCanBeLocal", "WeakerAccess"})
class Servo_Glyph extends Servo_Command{
    private double incrementValue = 0.05;
    private boolean finished;
    private long timeToOpenClose = 550;
    private String position;

    public Servo_Glyph(String position){
        super();
        finished = false;
        this.position = position;
        GlyphServoRight.setPosition(0.5);
        GlyphServoLeft.setPosition(0.5);
    }

    @Override
    public void start() {
        long startTime = System.currentTimeMillis();

        if(position.equals(OPEN)){
            while(System.currentTimeMillis()-startTime<=timeToOpenClose){
                GlyphServoRight.setPosition(GlyphServoRight.getPosition()+incrementValue);
                GlyphServoLeft.setPosition(GlyphServoLeft.getPosition()-incrementValue);
            }
        }
        else if(position.equals(CLOSE)){
            while(System.currentTimeMillis()-startTime<=timeToOpenClose){
                GlyphServoRight.setPosition(GlyphServoRight.getPosition()-incrementValue);
                GlyphServoLeft.setPosition(GlyphServoLeft.getPosition()+incrementValue);
            }
        }

        finished = true;
    }

    @Override
    public boolean isFinished() {
        return finished;
    }

    @Override
    public String printString() {
        return "GlyphServo "+position;
    }
}

@SuppressWarnings("FieldCanBeLocal")
class Servo_Jewel_Sensor extends Servo_Command{

    private static String upDown;
    private static ColorSensor colorSensor;
    private static float[] hsvValues = new float[3];
    private static NormalizedRGBA colors;
    static final String RED = "RED";
    static final String BLUE = "BLUE";
    private static boolean finished = false;
    private static String colorDetected;

    public Servo_Jewel_Sensor(String upDown){
        super();

        //get colorSensor reference and turn the light on
        colorSensor = hardwareMap.get(ColorSensor.class, "sensor");
        colorSensor.enableLed(true);

        colorDetected = null;
        this.upDown = upDown;
    }

    public static String getColor(){
        return colorDetected;
    }

    @Override
    public void start() {

        long timeToMove = 1000;  //TODO: check the amount of time needed

        if(upDown.equals(DOWN))
            JewelServo.setPosition(0.9);

        //do not check color if moving motor up
        if(upDown.equals(UP)){
            JewelServo.setPosition(0.1);
            finished = true;
            return;
        }

        long startTime = System.currentTimeMillis();
        while(System.currentTimeMillis()-startTime<=timeToMove){
            //do nothing until sensor moves into position
        }

        Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);


        if(colorSensor.red() > colorSensor.blue())       //check values for red
            colorDetected = RED;
        else if(colorSensor.red() < colorSensor.blue())  //check values for blue
            colorDetected = BLUE;
        else            //if undetected
            colorDetected = null;

        finished = true;
    }

    @Override
    public boolean isFinished() {
        return finished;
    }

    @Override
    public String printString() {
        return "JewelServo detected "+getColor();
    }
}
