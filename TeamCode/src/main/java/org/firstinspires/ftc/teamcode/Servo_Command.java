package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
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
    protected DcMotor RelicMotor;       //Extending arm
    protected Servo BigRelicServo;      //Vertical and Horizontal lift
    protected Servo SmallRelicServo;    //Open and close claw
    protected Servo JewelServo;         //Move jewel sensor up and down
    public static HardwareMap hardwareMap;

    protected Servo_Command(){
        initializeServos();
    }

    private void initializeServos(){
        //Glyph arm Motors and Servos
        GlyphMotor = hardwareMap.dcMotor.get("motor5");
        GlyphServoLeft = hardwareMap.servo.get("servo3");
        GlyphServoRight = hardwareMap.servo.get("servo4");

        //Relic arm motors and Servos
        RelicMotor = hardwareMap.dcMotor.get("motor6");
        BigRelicServo = hardwareMap.servo.get("servo5");
        SmallRelicServo = hardwareMap.servo.get("servo6");

       // JewelServo = hardwareMap.servo.get("servo1");
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
    private boolean finished = false;

    public Servo_Jewel_Sensor(String upDown){
        super();
        if((position==0 && upDown.equals(UP)) || (position==1 && upDown.equals(DOWN))){
            return;
        }
        this.upDown = upDown;
    }

    public String getColor(){
        return "RED";
    }

    @Override
    public void start() {
        double incrementValue = 0.05;
        long startTime = System.currentTimeMillis();
        long timeToMove = 1000;

        if(upDown.equals(DOWN))
            incrementValue = -incrementValue;

        while(System.currentTimeMillis()-startTime<=timeToMove){
            JewelServo.setPosition(GlyphMotor.getCurrentPosition()+incrementValue);
        }
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
