package edu.unm.dragonfly;

import com.google.inject.Guice;
import com.google.inject.Injector;
import javafx.application.Application;
import javafx.fxml.FXMLLoader;
import javafx.scene.Parent;
import javafx.scene.Scene;
import javafx.stage.Stage;

import java.util.Arrays;
import java.util.List;


public class DashboardApplication extends Application {

    private static DashboardController controller;

    private String getParameter(List<String> keys) {
        for(String key : keys) {
            if(getParameters().getNamed().containsKey(key)) {
                return getParameters().getNamed().get(key);
            }
        }
        return null;
    }

    @Override
    public void start(Stage stage) throws Exception {

        String map = getParameter(Arrays.asList("m", "map"));

        final Injector injector = Guice.createInjector(new DashboardModule(stage, map));

        FXMLLoader loader = new FXMLLoader(getClass().getResource("/dashboard.fxml"));
        loader.setControllerFactory(injector::getInstance);

        Parent root = loader.load();
        controller = loader.getController();

        Scene scene = new Scene(root);

        // set title, size, and add JavaFX scene to stage
        stage.setTitle("Dragonfly Dashboard");
        stage.setWidth(800);
        stage.setHeight(700);
        stage.setScene(scene);
        stage.show();
    }

    /**
     * Stops and releases all resources used in application.
     */
    @Override
    public void stop() {

        controller.terminate();
    }

    /**
     * Opens and runs application.
     *
     * @param args arguments passed to this application
     */
    public static void main(String[] args) {
//        ArcGISRuntimeEnvironment.setLicense("runtimelite,1000,rud2004642504,none,4N5X0H4AH76FTK118137");

        Application.launch(args);
    }

}