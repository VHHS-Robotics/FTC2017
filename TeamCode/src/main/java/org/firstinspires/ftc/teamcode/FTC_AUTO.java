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

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
//import org.firstinspires.ftc.robotcore.internal.network.RecvLoopRunnable;

import java.util.LinkedList;
import java.util.Queue;

@SuppressWarnings({"FieldCanBeLocal", "unused"})
@Autonomous(name="AUTO", group ="Competition")
public class FTC_AUTO extends LinearOpMode {

    private VuforiaLocalizer vuforia;
    private VuforiaTrackable relicTemplate;
    private Queue<Command> commands = new LinkedList<>();
    private int relicPosition;  //0=LEFT 1=CENTER 2=RIGHT
    private boolean once = false;

    @Override public void runOpMode() throws InterruptedException{

        //Initialize VuMark Code
        VuMarkInit();

        waitForStart();

        while (opModeIsActive()) {
            if(!once) {
                checkRelicPosition();
                //initialize hardwareMap in both command classes
                Drive_Command.hardwareMap = hardwareMap;
                Servo_Command.hardwareMap = hardwareMap;

                setCommands();
                runCommands();
            }
            once = true;
            idle();
        }
    }

    /**
       Enter all Autonomous Commands here
       List of Commands:

       Drive_Straight(inches)               inches is a double, positive moves forward, negative moves backwards
       Drive_Turn(degrees)                  degrees is a double, positive moves clockwise, negative moves counter-clockwise
       Servo_Glyph(Servo_Command.OPEN))     opens the block holder
       Servo_Glyph(Servo_Command.CLOSE))    closes the block holder
       Command_Wait(milliseconds)           makes the robot wait the desired time in milliseconds (thousandths of a second)
     */
    private void setCommands(){

        //do jewel operations

        commands.add(new Drive_Straight(34.0));

        if(relicPosition == 0){   //LEFT
            commands.add(new Drive_Turn(-140));
            commands.add(new Drive_Straight(30.0));
            commands.add(new Drive_Turn(50));
        }
        else if(relicPosition == 1){  //CENTER
            commands.add(new Drive_Turn(-125));
            commands.add(new Drive_Straight(26.0));
            commands.add(new Drive_Turn(35));
        }
        else if(relicPosition == 2){  //RIGHT
            commands.add(new Drive_Turn(-110));
            commands.add(new Drive_Straight(22.0));
            commands.add(new Drive_Turn(20));
        }

        commands.add(new Drive_Straight(6));
        commands.add(new Servo_Glyph(Servo_Command.OPEN));
        commands.add(new Drive_Straight(-6));
    }

    private void runCommands(){
        Command command;

        while(!commands.isEmpty()){
            command = commands.poll();
            command.start();
            //always wait after a command to kill momentum
            command = new Command_Wait(200);
            command.start();
        }
    }

    private void checkRelicPosition(){
        //Check Vumark and act based on Vumark seen
        if(VuMarkCheck().equals(RelicRecoveryVuMark.LEFT)){
            relicPosition = 0;
            telemetry.addLine("Left");
        }
        else if(VuMarkCheck().equals(RelicRecoveryVuMark.CENTER)){
            relicPosition = 1;
            telemetry.addLine("Center");
        }
        else if(VuMarkCheck().equals(RelicRecoveryVuMark.RIGHT)){
            relicPosition = 2;
            telemetry.addLine("Right");
        }
        telemetry.update();
    }

    private void VuMarkInit(){
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

    private RelicRecoveryVuMark VuMarkCheck(){
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
}
