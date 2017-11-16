package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Drive_Command;
import org.firstinspires.ftc.teamcode.FTC_AUTO;
import org.firstinspires.ftc.teamcode.Servo_Command;

@SuppressWarnings("unused")
@Autonomous(name="AUTO_BLUE_2", group ="Competition")
public class FTC_AUTO_BLUE_2 extends FTC_AUTO {

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