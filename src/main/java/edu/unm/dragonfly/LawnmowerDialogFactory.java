package edu.unm.dragonfly;


import edu.unm.dragonfly.mission.GridUtil;
import edu.unm.dragonfly.mission.Waypoint;
import javafx.collections.FXCollections;
import javafx.scene.control.ButtonType;
import javafx.scene.control.CheckBox;
import javafx.scene.control.ComboBox;
import javafx.scene.control.Dialog;
import javafx.scene.control.TextField;
import javafx.scene.layout.GridPane;

import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.stream.Collectors;

public class LawnmowerDialogFactory {

    public enum Walk {
        WALK(1),
        RANGE(2);

        public final int id;

        Walk(int id) {
            this.id = id;
        }
    }

    public interface DialogCallback {
        void call(String boundaryName, float stepLength, float altitude, int stacks, boolean walkBoundary, Walk selectedItem, float waitTime, float distanceThreshold);
    }

    public static void create(Map<String, List<Waypoint>> boundaries, DialogCallback callback) {

        Dialog<ButtonType> dialog = new Dialog<>();
        dialog.setTitle("Lawnmower Parameters");

        ComboBox<String> boundarySelection = new ComboBox<>(FXCollections.observableList(boundaries.keySet().stream().sorted().collect(Collectors.toList())));
        TextField stepLengthField = new TextField();
        TextField stacksField = new TextField();
        TextField altitudeField = new TextField();
        CheckBox walkBoundaryField = new CheckBox();
        ComboBox<Walk> walkComboBox = new ComboBox<>();
        walkComboBox.getItems().addAll(Walk.values());
        TextField waitTimeField = new TextField();
        TextField distanceThreshold = new TextField();

        // Set Defaults
        stepLengthField.setText("1");
        stacksField.setText("1");
        altitudeField.setText("10");
        walkBoundaryField.setSelected(true);
        walkComboBox.getSelectionModel().select(Walk.RANGE);
        waitTimeField.setText("0");
        distanceThreshold.setText("1");

        GridPane grid = new GridPane();
        GridUtil.builder(grid)
                .add("Boundary:", boundarySelection)
                .add("Step Length:", stepLengthField)
                .add("Altitude:", altitudeField)
                .add("Stacks:", stacksField)
                .add("Walk Boundary:", walkBoundaryField)
                .add("Walk:", walkComboBox)
                .add("Wait Time:", waitTimeField)
                .add("Distance Threshold:", distanceThreshold);

        dialog.getDialogPane().setContent(grid);

        dialog.getDialogPane().getButtonTypes().add(ButtonType.CANCEL);
        dialog.getDialogPane().getButtonTypes().add(ButtonType.OK);

        Optional<ButtonType> result = dialog.showAndWait();

        if(result.isPresent() && result.get() == ButtonType.OK) {
            callback.call(boundarySelection.getSelectionModel().getSelectedItem(),
                    Float.parseFloat(stepLengthField.getText()),
                    Float.parseFloat(altitudeField.getText()),
                    Integer.parseInt(stacksField.getText()),
                    walkBoundaryField.isSelected(),
                    walkComboBox.getSelectionModel().getSelectedItem(),
                    Float.parseFloat(waitTimeField.getText()),
                    Float.parseFloat(distanceThreshold.getText()));
        }
    }
}
