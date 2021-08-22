package edu.unm.dragonfly;

import com.google.inject.AbstractModule;
import com.google.inject.name.Names;
import com.google.inject.util.Providers;
import javafx.stage.Stage;
import ros.RosBridge;

public class DashboardModule extends AbstractModule {

    public static final String MAP_OVERRIDE = "mapOverride";

    private final RosBridge bridge;
    private final Stage stage;
    private final String mapOverride;

    public DashboardModule(Stage stage, String mapOverride) {
        bridge = new RosBridge();
        bridge.connect("ws://0.0.0.0:9090", true);

        this.stage = stage;
        this.mapOverride = mapOverride;
    }

    @Override
    protected void configure() {
        bind(RosBridge.class).toInstance(bridge);
        bind(Stage.class).toInstance(stage);
        bind(String.class).annotatedWith(Names.named(MAP_OVERRIDE)).toProvider(Providers.of(mapOverride));
    }
}
