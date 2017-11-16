package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 commands.add(new Command());
 List of Commands:

 Drive_Straight(inches)               inches is a double, positive moves forward, negative moves backwards
 Drive_Turn(degrees, speed)           degrees is a double, positive moves clockwise, negative moves counter-clockwise, speed is a double between 0.0 and 1.0
 Servo_Glyph(Servo_Command.OPEN))     opens the block holder
 Servo_Glyph(Servo_Command.CLOSE))    closes the block holder
 Servo_Jewel_Sensor(Servo_Command.UP) move the color sensor up
 Servo_Jewel_Sensor.getColor()        returns either Servo_Jewel_Sensor.BLUE or Servo_Jewel_Sensor.RED or null
 Command_Wait(milliseconds)           makes the robot wait the desired time in milliseconds (thousandths of a second)
 */
@SuppressWarnings({"FieldCanBeLocal","unused"})
@Autonomous(name="AUTO_RED_1", group ="Competition")
public class FTC_AUTO_RED_1 extends FTC_AUTO {

    public FTC_AUTO_RED_1(){
        weAreBlue = false;
    }

    private double CENTER_DISTANCE = 21.0;

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