// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Add your docs here. */
public class Launchpad extends GenericHID
{
    public Launchpad(int port) {super(port);}

    /** btn should be in range [0..63]. Don't ask for -1, you'll make me sad :( */
    public Trigger getBtn(int btn) 
    {
        return new Trigger(() -> {
            if (btn >= 32)  
                return super.getRawButton(btn - 32);
            else if (btn < 16) 
                return super.getPOV(1) == btn;
            else if (btn < 24) 
                return super.getPOV(2) == btn - 16;
            else if (btn < 32)
                return super.getPOV(3) == btn - 24;
            else 
                return false;
        });
    }

    /** btn should be in range [0..7]. Don't ask for -1, you'll make me sad :( */
    public Trigger getModeBtn(int modeBtn) 
        {return new Trigger(() -> super.getPOV(0) == modeBtn);}
}
