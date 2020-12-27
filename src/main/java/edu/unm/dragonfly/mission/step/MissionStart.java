package edu.unm.dragonfly.mission.step;

/**
 * @author John Ericksen
 */
public class MissionStart implements MissionStep {

    @Override
    public String toString() {
        return "Start{}";
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        return o != null && getClass() == o.getClass();
    }
}
