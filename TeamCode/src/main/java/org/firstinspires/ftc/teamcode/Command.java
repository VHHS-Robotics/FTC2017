package org.firstinspires.ftc.teamcode;

@SuppressWarnings("unused")
interface Command {
    void start();
    String printString();
}

@SuppressWarnings("WeakerAccess")
class Command_Wait implements Command {

    private long waitTime = 0;
    private boolean finished = true;

    public Command_Wait(long timeInMillis){
        waitTime = timeInMillis;
    }

    @Override
    public void start() {
        long startTime = System.currentTimeMillis();
        while(System.currentTimeMillis()-startTime<waitTime){
            //wait
        }
        finished = true;
    }

    @Override
    public String printString() {
        return "Waiting "+waitTime;
    }
}

