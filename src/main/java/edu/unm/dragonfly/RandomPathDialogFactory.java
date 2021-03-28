package edu.unm.dragonfly;

import edu.unm.dragonfly.mission.GridUtil;
import edu.unm.dragonfly.mission.Waypoint;
import javafx.collections.FXCollections;
import javafx.scene.control.ButtonType;
import javafx.scene.control.ComboBox;
import javafx.scene.control.Dialog;
import javafx.scene.control.ProgressBar;
import javafx.scene.control.TextField;
import javafx.scene.layout.GridPane;

import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.stream.Collectors;

public class RandomPathDialogFactory {

    public interface DialogCallback {
        void call(String boundaryName, float minAltitude, float maxAltitude, int size, float waitTime, float distanceThreshold);
    }

    public static void create(Map<String, List<Waypoint>> boundaries, DialogCallback callback) {

        Dialog<ButtonType> dialog = new Dialog<>();
        dialog.setTitle("Random Path Parameters");

        GridPane grid = new GridPane();

        ComboBox<String> boundarySelection = new ComboBox<>(FXCollections.observableList(boundaries.keySet().stream().sorted().collect(Collectors.toList())));
        TextField minAltitude = new TextField();
        TextField maxAltitude = new TextField();
        TextField size = new TextField();
        TextField waitTimeField = new TextField();
        TextField distanceThreshold = new TextField();
        ProgressBar progressBar = new ProgressBar(0);

        // Set Defaults
        minAltitude.setText("10");
        maxAltitude.setText("20");
        size.setText("100");
        waitTimeField.setText("3");
        distanceThreshold.setText("1");

        GridUtil.builder(grid)
                .add("Boundary:", boundarySelection)
                .add("Min Altitude:", minAltitude)
                .add("Max Altitude:", maxAltitude)
                .add("Size:", size)
                .add("Wait Time:", waitTimeField)
                .add("Distance Threshold:", distanceThreshold)
                .add(progressBar);

        dialog.getDialogPane().setContent(grid);

        dialog.getDialogPane().getButtonTypes().add(ButtonType.CANCEL);
        dialog.getDialogPane().getButtonTypes().add(ButtonType.OK);

        Optional<ButtonType> result = dialog.showAndWait();

        if(result.isPresent() && result.get() == ButtonType.OK) {
            callback.call(boundarySelection.getSelectionModel().getSelectedItem(),
                    Float.parseFloat(minAltitude.getText()),
                    Float.parseFloat(maxAltitude.getText()),
                    Integer.parseInt(size.getText()),
                    Float.parseFloat(waitTimeField.getText()),
                    Float.parseFloat(distanceThreshold.getText()));
        }
    }
}
