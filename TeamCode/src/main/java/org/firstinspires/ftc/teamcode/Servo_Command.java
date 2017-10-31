package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.robotcore.external.Telemetry;

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

    private static int position = 0;
    private String upDown;
    private ColorSensor colorSensor;
    private float[] hsvValues = new float[3];
    private NormalizedRGBA colors;
    static final String RED = "RED";
    static final String BLUE = "BLUE";
    private boolean finished = false;
    private static String colorDetected;
    static Telemetry telemetry;


    public Servo_Jewel_Sensor(String upDown){
        super();

        //get colorSensor reference and turn the light on
        colorSensor = hardwareMap.get(ColorSensor.class, "sensor");
        colorSensor.enableLed(true);
        JewelServo.setPosition(0.75);

        colorDetected = null;
        if((position==0 && upDown.equals(UP)) || (position==1 && upDown.equals(DOWN))){
            return;
        }
        this.upDown = upDown;
    }

    public static String getColor(){
        return colorDetected;
    }

    @Override
    public void start() {
        double incrementValue = 0.05;
        long startTime = System.currentTimeMillis();
        long timeToMove = 500;  //TODO: check the amount of time needed

        //TODO: check if positive and negative values are reversed or not
        if(upDown.equals(DOWN))
            incrementValue = -incrementValue;

        //move motor down
        while(System.currentTimeMillis()-startTime<=timeToMove){
            JewelServo.setPosition(JewelServo.getPosition()+ incrementValue);
        }

        //do not check color if moving motor up
        if(upDown.equals(UP)){
            finished = true;
            return;
        }

        Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);
        telemetry.addData("Clear", colorSensor.alpha());
        telemetry.addData("Red  ", colorSensor.red());
        telemetry.addData("Green", colorSensor.green());
        telemetry.addData("Blue ", colorSensor.blue());
        telemetry.addData("Hue", hsvValues[0]);

        if(true)         //check values for red
            colorDetected = RED;
        else if(false)  //check values for blue
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
