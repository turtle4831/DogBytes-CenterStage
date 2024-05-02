package org.firstinspires.ftc.teamcode.Pedrio.Util;

import org.firstinspires.ftc.teamcode.Pedrio.MecanumBot.Comms.Hardware;

public class RobotHardware {
    private Hardware instance = null;

    public RobotHardware getInstance(){
        if(instance == null){
            instance = new Hardware();
        }
        return this.instance;
    }
}
