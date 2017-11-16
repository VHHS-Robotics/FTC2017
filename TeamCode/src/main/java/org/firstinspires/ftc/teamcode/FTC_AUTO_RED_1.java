package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Drive_Command;
import org.firstinspires.ftc.teamcode.Drive_Straight;
import org.firstinspires.ftc.teamcode.Drive_Turn;
import org.firstinspires.ftc.teamcode.FTC_AUTO;
import org.firstinspires.ftc.teamcode.Servo_Command;
import org.firstinspires.ftc.teamcode.Servo_Glyph;

@SuppressWarnings({"FieldCanBeLocal","unused"})
@Autonomous(name="AUTO_RED_1", group ="Competition")
public class FTC_AUTO_RED_1 extends FTC_AUTO {

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