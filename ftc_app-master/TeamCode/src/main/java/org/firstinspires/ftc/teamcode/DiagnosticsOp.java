package org.firstinspires.ftc.teamcode;

import java.util.ArrayList;

/**
 * Created by jorda on 9/24/2017.
 */

public class DiagnosticsOp extends vv_OpMode {
    private vv_Lib robotLibrary;
    private ArrayList<String> errorTracker;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
    }

    private void initialize() throws InterruptedException {
        telemetryShow("Initializing...", false);
        robotLibrary = new vv_Lib(this);
        errorTracker = new ArrayList<>();
    }

    // This method gets user input on whether a test succeeded or not
    private boolean didItRun(String elementName) {
        telemetryShow(String.format("Did the %s work? A for yes, B for no", elementName), true);

        // Wait for an input from A or B
        while (!gamepad1.a && !gamepad2.b) ;

        // If A is pressed, that means the test succeeded
        return gamepad1.a;
    }

    // TODO: Create actual tests once common class methods are completed
    private boolean vuforiaDetection() {
        return true;
    }

    private boolean cubeArm() {
        return didItRun("cube arm");
    }

    private boolean cubeHand() {
        return didItRun("cube hand");
    }

    private boolean cubeHandGyro() {
        return didItRun("cube hand gyro");
    }

    private boolean relicArm() {
        return didItRun("relic arm");
    }

    private boolean relicHand() {
        return didItRun("relic hand");
    }

    // Combine telemetryAddData and telemetryUpdate into one method for convenience
    private void telemetryShow(String s, boolean isInput) {
        telemetryAddData("DiagnosticsOp", (isInput ? "Input" : "Output"), s);
        telemetryUpdate();
    }
}
