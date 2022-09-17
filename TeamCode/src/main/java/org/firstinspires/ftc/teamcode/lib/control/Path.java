package org.firstinspires.ftc.teamcode.lib.control;

import java.util.ArrayList;

public class Path {

    ArrayList<Runnable> path;

    public Path(Runnable stop) {
        path = new ArrayList<Runnable>();
        path.add(stop);
    }

    public void add(Runnable command) {
        path.add(path.size()-1, command);
    }

    public void run(int pathStep) {
        path.get(pathStep).run();
    }
}
