package org.firstinspires.ftc.teamcode.FTC_AUTO;

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
//@Autonomous(name="AUTO_BLUE_2", group ="Competition")
public class FTC_AUTO_BLUE_2 extends FTC_AUTO {

    public FTC_AUTO_BLUE_2(){
        weAreBlue = true;
    }

    //TODO: CENTER_DISTANCE for BLUE_2 and RED_2 are not 21.0
    private double CENTER_DISTANCE = 41.5;

    @Override
    void setCommands() {
        //Drive Commands
        commands.add(new Drive_Straight(27.5));
        commands.add(new Drive_Turn(125.0, 0.5, "LEFT"));

        if(glyphLocation==0)        //LEFT
            commands.add(new Drive_Straight(CENTER_DISTANCE-7.0));
        else if (glyphLocation==1)  //CENTER
            commands.add(new Drive_Straight(CENTER_DISTANCE));
        else if (glyphLocation==2)  //RIGHT
            commands.add(new Drive_Straight(CENTER_DISTANCE+7.0));

        commands.add(new Drive_Turn(180.0, 0.5, "LEFT"));
        commands.add(new Drive_Straight(25.0));
        commands.add(new Servo_Glyph(Servo_Command.OPEN));
    }
}