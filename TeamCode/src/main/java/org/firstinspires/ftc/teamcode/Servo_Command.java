package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@SuppressWarnings("WeakerAccess")
abstract class Servo_Command implements Command {
    protected final static String OPEN = "OPEN";
    protected final static String CLOSE = "CLOSE";
    protected final static String UP = "UP";
    protected final static String DOWN = "DOWN";

    //protected DcMotor GlyphMotor;       //up and down very quickly
    protected static Servo GlyphServoRight;    //close and open
    protected static Servo GlyphServoLeft;     //close and open
    protected Servo JewelServo;         //Move jewel sensor up and down
    protected DcMotor RelicMotor;
    protected static Servo BigRelicServo;
    protected Servo SmallRelicServo;

    public static HardwareMap hardwareMap;
    public static Telemetry telemetry;

    protected Servo_Command(){
        initializeServos();
    }

    public static void initGlyphServos(){
        GlyphServoLeft = hardwareMap.servo.get("servo5");
        GlyphServoRight = hardwareMap.servo.get("servo6");
        BigRelicServo = hardwareMap.servo.get("servo1");

        GlyphServoLeft.setPosition(0.5);
        GlyphServoRight.setPosition(0.5);
        BigRelicServo.setPosition(0.5);
    }

    private void initializeServos(){
        //Glyph arm Motors and Servos
        //GlyphMotor = hardwareMap.dcMotor.get("motor5");
        GlyphServoLeft = hardwareMap.servo.get("servo5");
        GlyphServoRight = hardwareMap.servo.get("servo6");
        JewelServo = hardwareMap.servo.get("servo4");

        //Relic arm motors and Servos
        RelicMotor = hardwareMap.dcMotor.get("motor6");
        BigRelicServo = hardwareMap.servo.get("servo1");
        SmallRelicServo = hardwareMap.servo.get("servo2");


        GlyphServoRight.setPosition(0.5);
        GlyphServoLeft.setPosition(0.5);
        JewelServo.setPosition(0.9);
        BigRelicServo.setPosition(0.5);
    }
}

@SuppressWarnings({"FieldCanBeLocal", "WeakerAccess"})
class Servo_Glyph extends Servo_Command{
    //private double incrementValue = 0.05;
    private long timeToOpenClose = 1000;
    private String position;

    public Servo_Glyph(String position){
        super();
        this.position = position;
        GlyphServoRight.setPosition(0.2);
        GlyphServoLeft.setPosition(0.8);
    }

    @Override
    public void start() {
        long startTime = System.currentTimeMillis();

        if(position.equals(OPEN)){
            GlyphServoLeft.setPosition(0.0);
            GlyphServoRight.setPosition(1.0);
            /*
            while(System.currentTimeMillis()-startTime<=timeToOpenClose){
                GlyphServoRight.setPosition(GlyphServoRight.getPosition()+incrementValue);
                GlyphServoLeft.setPosition(GlyphServoLeft.getPosition()-incrementValue);
            }
            */
        }
        else if(position.equals(CLOSE)){
            GlyphServoLeft.setPosition(0.2);
            GlyphServoRight.setPosition(0.8);
            /*
            while(System.currentTimeMillis()-startTime<=timeToOpenClose){
                GlyphServoRight.setPosition(GlyphServoRight.getPosition()-incrementValue);
                GlyphServoLeft.setPosition(GlyphServoLeft.getPosition()+incrementValue);
            }
            */
        }
        while(System.currentTimeMillis()-startTime<timeToOpenClose){
            //wait while it moves to position
        }
        return;
    }

    @Override
    public String printString() {
        return "GlyphServo "+position;
    }
}

@SuppressWarnings({"FieldCanBeLocal", "WeakerAccess"})
class Servo_Jewel_Sensor extends Servo_Command{

    private String upDown;
    private static ColorSensor colorSensor;
    private static float[] hsvValues = new float[3];
    static final String RED = "RED";
    static final String BLUE = "BLUE";
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

        long timeToMove = 1000;

        if(upDown.equals(DOWN))
            JewelServo.setPosition(0.1);

        //do not check color if moving motor up
        if(upDown.equals(UP)){
            JewelServo.setPosition(0.9);
            return;
        }

        long startTime = System.currentTimeMillis();
        while(System.currentTimeMillis()-startTime<=timeToMove){
            //do nothing until servo moves the sensor into position
        }

        Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);


        if(colorSensor.red() > colorSensor.blue())       //check values for red
            colorDetected = RED;
        else if(colorSensor.red() < colorSensor.blue())  //check values for blue
            colorDetected = BLUE;
        else            //if undetected
            colorDetected = null;
    }

    @Override
    public String printString() {
        return "JewelServo detected "+getColor();
    }
}
