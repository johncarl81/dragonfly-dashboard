package edu.unm.dragonfly;

import com.esri.arcgisruntime.mapping.view.SceneView;

/**
 * @author John Ericksen
 */
public interface Fixture {

    String getIcon();

    interface RemoveCallback{
        void remove();
    }

    void remove();

    void center(SceneView sceneView);

    String getName();
}
