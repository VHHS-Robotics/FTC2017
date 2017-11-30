package org.firstinspires.ftc.teamcode.FTC_AUTO;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.LinkedList;
import java.util.Queue;

@SuppressWarnings({"WeakerAccess", "unused"})
public abstract class FTC_AUTO extends LinearOpMode{

    protected VuforiaLocalizer vuforia;
    protected VuforiaTrackable relicTemplate;
    protected Queue<Command> commands = new LinkedList<>();   //list of commands that will run autonomous
    protected int glyphLocation = 0; //LEFT=0, CENTER=1, RIGHT=2
    protected boolean firstTimeActive = true;
    protected boolean firstTimeInit = true;
    protected boolean weAreBlue = true;

    abstract void setCommands();

    @Override
    public void runOpMode() throws InterruptedException{
        if(firstTimeInit) {
            //Initialize VuMark Code
            VuMarkInit();

            //initialize hardwareMap in both command classes
            Drive_Command.hardwareMap = hardwareMap;
            Servo_Command.hardwareMap = hardwareMap;
            Drive_Command.telemetry = telemetry;

            //Initialize Gyro in Drive_Command Class
            Drive_Command.initGyro();

            //Initialize Motors in the Drive_Command Class
            Drive_Command.initMotors();

            //initialize Glyph servos before Autonomous begins
            Servo_Command.initGlyphServos();
        }
        firstTimeInit = false;
        telemetry.addLine("Ready to Start AUTONOMOUS");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            if(firstTimeActive) {
                //check the relic to determine which position to place the glyph
                checkRelicPosition();

                //do all the jewel commands before the drive commands
               // runJewelCommands();

                //set and run the commands for autonomous driving
                setCommands();
                runCommands();

                //drive robot backwards manually
                driveRobotBackwards();
            }
            firstTimeActive = false;
            idle();
        }
    }

    protected void runCommands(){
        Command command;

        while(!commands.isEmpty()){
            command = commands.poll();
            telemetry.addLine(command.printString());       //print what the robot is doing
            telemetry.update();
            command.startCommand();
            //always wait after a command to kill momentum
            command = new Command_Wait(200);
            command.startCommand();
        }
    }

    protected void VuMarkInit(){
        VuforiaTrackables relicTrackables;

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AXHqge7/////AAAAGenGHENAYE1qoSAsuGF8810cJsPkYPfPvtKoveVcdLDzK46GCUu14Tc8oGqzdynacNWZNLD1guU8EZbFPoQNdH2SucwUFBo/bOsW0mlHtXp/eYmyAtuLjwVp4HtvH0Hit5G4YthBouXAa89954OBiG5cBPCWilFG+2ZCzQ4IPEN7ams3nyksoBZxCD03XHbSEdJ+AkRC14iZmjnWKDxlsCg70C33QiDHQ0KGV8NB2uM4WUFoRrjXK87VAry9nGMzf4p58AD6SViaov0kDo6oFtIGDZ2Ot9QrhVlCtL1GaJiNeUXENmAyrGBvkB1NNaMPFUsJnlnNoR5KmaYYKHWOEtl8K/T6rXVaNnzeXhcyHK1e";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
        relicTrackables.activate();
    }

    protected RelicRecoveryVuMark VuMarkCheck(){
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
        if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
            telemetry.addData("VuMark", "%s visible", vuMark);
        }
        else {
            telemetry.addData("VuMark", "not visible");
        }
        telemetry.update();
        return vuMark;
    }

    protected void checkRelicPosition(){
        //Check Vumark and set glyphLocation based on Vumark seen
        if(VuMarkCheck().equals(RelicRecoveryVuMark.LEFT)){
            glyphLocation = 0;
            telemetry.addLine("Left");
        }
        else if(VuMarkCheck().equals(RelicRecoveryVuMark.CENTER)){
            glyphLocation = 1;
            telemetry.addLine("Center");
        }
        else if(VuMarkCheck().equals(RelicRecoveryVuMark.RIGHT)){
            glyphLocation = 2;
            telemetry.addLine("Right");
        }
        else{   //default to center if no relic found
            glyphLocation = 1;
            telemetry.addLine("No Relic Found");
        }
        telemetry.update();
    }

    //TODO: Fix this workaround, figure out why it does not work after a Servo_Command.OPEN
    protected void driveRobotBackwards(){
        //move backwards at 0.5 speed for 250 milliseconds
        long time = 250;
        double power = -0.5;

        DcMotor MotorFrontLeft = hardwareMap.dcMotor.get("motor1");
        DcMotor MotorFrontRight = hardwareMap.dcMotor.get("motor2");
        MotorFrontRight.setDirection(DcMotor.Direction.REVERSE);
        DcMotor MotorBackLeft = hardwareMap.dcMotor.get("motor3");
        DcMotor MotorBackRight = hardwareMap.dcMotor.get("motor4");
        MotorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);

        //startCommand the motors
        long startTime = System.currentTimeMillis();
        MotorFrontLeft.setPower(power);
        MotorBackLeft.setPower(power);
        MotorFrontRight.setPower(power);
        MotorBackRight.setPower(power);

        while(System.currentTimeMillis()-startTime<time){
            //wait to move
        }

        //stop the motors
        MotorFrontLeft.setPower(0);
        MotorBackLeft.setPower(0);
        MotorFrontRight.setPower(0);
        MotorBackRight.setPower(0);
    }

    protected void runJewelCommands(){
        //move the Servo_Jewel_Sensor DOWN to detect the color
        Servo_Command jewelDownCommand = new Servo_Jewel_Sensor(Servo_Command.DOWN);
        jewelDownCommand.startCommand();
        telemetry.addLine(jewelDownCommand.printString());
        telemetry.update();

        //get color that the sensor detected
        String color = Servo_Jewel_Sensor.getColor();
        telemetry.addLine("color = "+color+"");
        telemetry.update();

        Command jewelTurn;
        if(weAreBlue) {     //we are BLUE
            if (color != null) {
                //if we see BLUE
                if (color.equals(Servo_Jewel_Sensor.BLUE)) {
                    //rotate clockwise
                    telemetry.addLine("We are BLUE, rotate CLOCKWISE");
                    telemetry.update();

                    jewelTurn = new Drive_Turn(15.0, 0.5, "LEFT");
                    jewelTurn.startCommand();
                    telemetry.addLine(jewelTurn.printString());
                    telemetry.update();
                    jewelTurn = new Drive_Turn(0.0, 0.5, "RIGHT");
                    jewelTurn.startCommand();
                    telemetry.addLine(jewelTurn.printString());
                    telemetry.update();
                }
                else { //if we see RED
                    //rotate counter-clockwise
                    telemetry.addLine("We are BLUE, rotate COUNTER-CLOCKWISE");
                    telemetry.update();
                    jewelTurn = new Drive_Turn(345.0, 0.5, "RIGHT");
                    jewelTurn.startCommand();
                    telemetry.addLine(jewelTurn.printString());
                    telemetry.update();
                    jewelTurn = new Drive_Turn(0.0, 0.5, "LEFT");
                    jewelTurn.startCommand();
                    telemetry.addLine(jewelTurn.printString());
                    telemetry.update();
                }
            }
            else {  //if we did not detect a color
                telemetry.addLine("Do nothing for jewel turn");
            }
            telemetry.update();
        }
        else{               //we are RED    if(!weAreBlue)
            if (color != null) {
                //if we see BLUE
                if (color.equals(Servo_Jewel_Sensor.BLUE)) {
                    //rotate counter-clockwise
                    telemetry.addLine("We are RED, rotate COUNTER-CLOCKWISE");
                    telemetry.update();


                    jewelTurn = new Drive_Turn(345.0, 0.5, "RIGHT");
                    jewelTurn.startCommand();
                    telemetry.addLine(jewelTurn.printString());
                    telemetry.update();
                    jewelTurn = new Drive_Turn(0.0, 0.5, "LEFT");
                    jewelTurn.startCommand();
                    telemetry.addLine(jewelTurn.printString());
                    telemetry.update();
                }
                else {  //if we see RED
                    //rotate clockwise
                    telemetry.addLine("We are RED, rotate CLOCKWISE");
                    telemetry.update();

                    jewelTurn = new Drive_Turn(15.0, 0.5, "LEFT");
                    jewelTurn.startCommand();
                    telemetry.addLine(jewelTurn.printString());
                    telemetry.update();
                    jewelTurn = new Drive_Turn(0.0, 0.5, "RIGHT");
                    jewelTurn.startCommand();
                    telemetry.addLine(jewelTurn.printString());
                    telemetry.update();
                }
            } else {    //if we did not detect a color
                telemetry.addLine("Do nothing for jewel turn");
            }
            telemetry.update();
        }

        //move jewel servo UP
        jewelDownCommand = new Servo_Jewel_Sensor(Servo_Command.UP);
        jewelDownCommand.startCommand();
    }
}
