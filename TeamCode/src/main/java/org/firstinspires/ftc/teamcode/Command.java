package org.firstinspires.ftc.teamcode;

interface Command {
    void start();
    boolean isFinished();
}

class Command_Wait implements Command {

    private long waitTime = 0;
    private boolean finished = true;
    private long startTime = 0;

    public Command_Wait(long timeInMillis){
        waitTime = timeInMillis;
    }
    @Override
    public void start() {
        startTime = System.currentTimeMillis();
        while(System.currentTimeMillis()-startTime<waitTime){
            //wait
        }
        finished = true;
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

