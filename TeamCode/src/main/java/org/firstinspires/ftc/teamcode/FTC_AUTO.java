/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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

/**
 commands.add(new Command());
 List of Commands:

 Drive_Straight(inches)               inches is a double, positive moves forward, negative moves backwards
 Drive_Turn(degrees)                  degrees is a double, positive moves clockwise, negative moves counter-clockwise
 Servo_Glyph(Servo_Command.OPEN))     opens the block holder
 Servo_Glyph(Servo_Command.CLOSE))    closes the block holder
 Servo_Jewel_Sensor(Servo_Command.UP) move the color sensor up
 Servo_Jewel_Sensor.getColor()        returns either Servo_Jewel_Sensor.BLUE or Servo_Jewel_Sensor.RED or null
 Command_Wait(milliseconds)           makes the robot wait the desired time in milliseconds (thousandths of a second)
 */

@SuppressWarnings({"WeakerAccess", "unused"})
public abstract class FTC_AUTO extends LinearOpMode{

    protected VuforiaLocalizer vuforia;
    protected VuforiaTrackable relicTemplate;
    protected Queue<Command> commands = new LinkedList<>();   //list of commands that will run autonomous
    protected int glyphLocation = 0; //LEFT=0, CENTER=1, RIGHT=2
    protected boolean firstTime = true;
    protected boolean firstTimeInit = true;

    abstract void setCommands();

    protected void runCommands(){
        Command command;

        while(!commands.isEmpty()){
            command = commands.poll();
            telemetry.addLine(command.printString());       //print what the robot is doing
            telemetry.update();
            command.start();
            //always wait after a command to kill momentum
            command = new Command_Wait(200);
            command.start();
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
        //Check Vumark and set relicPositionDistance based on Vumark seen
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
        else{
            glyphLocation = 1;
            telemetry.addLine("No Relic Found");
        }
        telemetry.update();
    }

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

        //start the motors
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
        jewelDownCommand.start();

        //get color that the sensor detected
        String color = Servo_Jewel_Sensor.getColor();
        telemetry.addLine("color"+color+"");
        telemetry.update();

        //assuming we are BLUE
        Command jewelTurn;
        if(color != null){
            //if we see BLUE
            if(color.equals(Servo_Jewel_Sensor.BLUE)){
                //rotate clockwise
                jewelTurn = new Drive_Turn(30.0, 0.2);
                jewelTurn.start();
                jewelTurn = new Drive_Turn(-30.0, 0.2);
                jewelTurn.start();
            }//if we see RED
            else{
                //rotate counter-clockwise
                jewelTurn = new Drive_Turn(-30.0, 0.2);
                jewelTurn.start();
                jewelTurn = new Drive_Turn(30.0, 0.2);
                jewelTurn.start();
            }
        }
        else{
            //if we did not detect a color
            telemetry.addLine("Do nothing for jewel turn");
        }
        telemetry.update();

        //move jewel servo UP
        jewelDownCommand = new Servo_Jewel_Sensor(Servo_Command.UP);
        jewelDownCommand.start();
    }

}

@SuppressWarnings({"FieldCanBeLocal","unused"})
@Autonomous(name="AUTO", group ="Competition")
class FTC_AUTO_BLUE_1 extends FTC_AUTO {

    private double CENTER_DISTANCE = 21.0;

    @Override
    public void runOpMode() throws InterruptedException{
        if(firstTimeInit) {
            //Initialize VuMark Code
            VuMarkInit();

            //initialize hardwareMap in both command classes
            Drive_Command.hardwareMap = hardwareMap;
            Servo_Command.hardwareMap = hardwareMap;

            //initialize Glyph servos before Autonomous begins
            Servo_Command.initGlyphServos();
        }
        firstTimeInit = false;
        waitForStart();

        while (opModeIsActive()) {
            if(firstTime) {
                //check the relic to determine which position to place the glyph
                checkRelicPosition();

                //do all the jewel commands before the drive commands
                runJewelCommands();

                //set and run the commands for autonomous driving
                setCommands();
                runCommands();

                //drive robot backwards manually
                driveRobotBackwards();
            }
            firstTime = false;
            idle();
        }
    }

    @Override
    void setCommands(){
        //Drive commands
        commands.add(new Drive_Straight(34.0));
        commands.add(new Drive_Turn(-95.0, 0.5));   //turns out 95.0 degrees equates to about 90.0 degrees on Robot_1
        commands.add(new Drive_Straight(26.0));
        commands.add(new Drive_Turn(-95.0, 0.5));

        if(glyphLocation==0)        //LEFT
            commands.add(new Drive_Straight(CENTER_DISTANCE+7.0));
        else if (glyphLocation==1)  //CENTER
            commands.add(new Drive_Straight(CENTER_DISTANCE));
        else if (glyphLocation==2)  //RIGHT
            commands.add(new Drive_Straight(CENTER_DISTANCE-7.0));

        commands.add(new Drive_Turn(90.0, 0.5));
        commands.add(new Drive_Straight(10.0));
        commands.add(new Servo_Glyph(Servo_Command.OPEN));
    }
}

@SuppressWarnings({"FieldCanBeLocal","unused"})
@Autonomous(name="AUTO", group ="Competition")
class FTC_AUTO_RED_1 extends FTC_AUTO{

    private double CENTER_DISTANCE = 21.0;

    @Override
    public void runOpMode() throws InterruptedException {
        if(firstTimeInit) {
            //Initialize VuMark Code
            VuMarkInit();

            //initialize hardwareMap in both command classes
            Drive_Command.hardwareMap = hardwareMap;
            Servo_Command.hardwareMap = hardwareMap;

            //initialize Glyph servos before Autonomous begins
            Servo_Command.initGlyphServos();
        }
        firstTimeInit = false;
        waitForStart();

        while (opModeIsActive()) {
            if(firstTime) {
                //check the relic to determine which position to place the glyph
                checkRelicPosition();

                //do all the jewel commands before the drive commands
                runJewelCommands();

                //set and run the commands for autonomous driving
                setCommands();
                runCommands();

                //drive robot backwards manually
                driveRobotBackwards();
            }
            firstTime = false;
            idle();
        }
    }

    @Override
    void setCommands() {
        //Drive commands
        commands.add(new Drive_Straight(34.0));
        commands.add(new Drive_Turn(95.0, 0.5));   //turns out 95.0 degrees equates to about 90.0 degrees on Robot_1
        commands.add(new Drive_Straight(26.0));
        commands.add(new Drive_Turn(95.0, 0.5));

        if(glyphLocation==0)        //LEFT
            commands.add(new Drive_Straight(CENTER_DISTANCE-7.0));
        else if (glyphLocation==1)  //CENTER
            commands.add(new Drive_Straight(CENTER_DISTANCE));
        else if (glyphLocation==2)  //RIGHT
            commands.add(new Drive_Straight(CENTER_DISTANCE+7.0));

        commands.add(new Drive_Turn(-90.0, 0.5));
        commands.add(new Drive_Straight(10.0));
        commands.add(new Servo_Glyph(Servo_Command.OPEN));
    }
}

@SuppressWarnings("unused")
@Autonomous(name="AUTO", group ="Competition")
class FTC_AUTO_BLUE_2 extends FTC_AUTO{

    //TODO: CENTER_DISTANCE for BLUE_2 and RED_2 are not 21.0
    private double CENTER_DISTANCE = 21.0;

    @Override
    public void runOpMode() throws InterruptedException {
        if(firstTimeInit) {
            //Initialize VuMark Code
            VuMarkInit();

            //initialize hardwareMap in both command classes
            Drive_Command.hardwareMap = hardwareMap;
            Servo_Command.hardwareMap = hardwareMap;

            //initialize Glyph servos before Autonomous begins
            Servo_Command.initGlyphServos();
        }
        firstTimeInit = false;
        waitForStart();

        while (opModeIsActive()) {
            if(firstTime) {
                //check the relic to determine which position to place the glyph
                checkRelicPosition();

                //do all the jewel commands before the drive commands
                runJewelCommands();

                //set and run the commands for autonomous driving
                setCommands();
                runCommands();

                //drive robot backwards manually
                driveRobotBackwards();
            }
            firstTime = false;
            idle();
        }
    }

    @Override
    void setCommands() {
        //Drive Commands

    }
}

@SuppressWarnings("unused")
@Autonomous(name="AUTO", group ="Competition")
class FTC_AUTO_RED_2 extends FTC_AUTO{

    //TODO: CENTER_DISTANCE for BLUE_2 and RED_2 are not 21.0
    private double CENTER_DISTANCE = 21.0;

    @Override
    public void runOpMode() throws InterruptedException {
        if(firstTimeInit) {
            //Initialize VuMark Code
            VuMarkInit();

            //initialize hardwareMap in both command classes
            Drive_Command.hardwareMap = hardwareMap;
            Servo_Command.hardwareMap = hardwareMap;

            //initialize Glyph servos before Autonomous begins
            Servo_Command.initGlyphServos();
        }
        firstTimeInit = false;
        waitForStart();

        while (opModeIsActive()) {
            if(firstTime) {
                //check the relic to determine which position to place the glyph
                checkRelicPosition();

                //do all the jewel commands before the drive commands
                runJewelCommands();

                //set and run the commands for autonomous driving
                setCommands();
                runCommands();

                //drive robot backwards manually
                driveRobotBackwards();
            }
            firstTime = false;
            idle();
        }
    }

    @Override
    void setCommands() {
        //Drive Commands

    }
}
