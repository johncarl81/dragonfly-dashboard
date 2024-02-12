package edu.unm.dragonfly;

import edu.unm.dragonfly.msgs.*;

/**
 * @author John Ericksen
 */
public class NamedPlume {

    public interface RemoveCallback{
        void remove();
    }

    private final String name;
    private final Plume plume;
    private final RemoveCallback deleteCallback;

    public NamedPlume(String name, Plume plume, RemoveCallback deleteCallback) {
        this.name = name;
        this.plume = plume;
        this.deleteCallback = deleteCallback;
    }

    public String getName() {
        return name;
    }

    public Plume getPlume() {
        return plume;
    }

    public void delete() {
        deleteCallback.remove();
    }
}
