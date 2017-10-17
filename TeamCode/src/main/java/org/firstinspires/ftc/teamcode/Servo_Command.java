package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

abstract class Servo_Command implements Command {
    protected final static String OPEN = "OPEN";
    protected final static String CLOSE = "CLOSE";

    protected DcMotor GlyphMotor; // up and down very quickly
    protected Servo GlyphServoRight; // close and open
    protected Servo GlyphServoLeft; // close and open
    protected DcMotor RelicMotor; //Extending arm
    protected Servo BigRelicServo; // Vertical and Horizontal lift
    protected Servo SmallRelicServo; //Open and close claw
    public static HardwareMap hardwareMap;

    public Servo_Command(){
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
    }
}


class Servo_Claw extends Servo_Command{

    private double incrementValue = 0.05;
    private String position;
    private boolean finished;
    private long startTime = 0;
    private long timeToOpenClose = 200;

    /**
     *
     * @param position use Servo_Claw.OPEN or Servo_Claw.CLOSE
     */
    public Servo_Claw(String position){
        super();
        finished = false;
        this.position = position;
    }

    @Override
    public void start() {
        startTime= System.currentTimeMillis();
        if(position.equals(CLOSE))
            incrementValue = -incrementValue;

        while(System.currentTimeMillis()-startTime<timeToOpenClose){
            SmallRelicServo.setPosition(SmallRelicServo.getPosition()+incrementValue);
        }
        finished = true;
    }

    @Override
    public boolean isFinished() {
        return finished;
    }
}
