package edu.unm.dragonfly;

import edu.unm.dragonfly.mission.GridUtil;
import javafx.scene.control.ButtonType;
import javafx.scene.control.ComboBox;
import javafx.scene.control.Dialog;
import javafx.scene.control.TextField;
import javafx.scene.layout.GridPane;

import java.util.Optional;

public class DDSADialogFactory {

    public interface DialogCallback {
        void call(float radius, float stepLength, float altitude, int loops, int stacks, Walk walk, float waitTime, float distanceThreshold);
    }

    public static void create(DialogCallback callback) {

        Dialog<ButtonType> dialog = new Dialog<>();
        dialog.setTitle("DDSA Parameters");

        GridPane grid = new GridPane();

        TextField radiusField = new TextField();
        TextField stepLengthField = new TextField();
        TextField loopsField = new TextField();
        TextField stacksField = new TextField();
        TextField altitudeField = new TextField();
        ComboBox<Walk> walkComboBox = new ComboBox<>();
        walkComboBox.getItems().addAll(Walk.values());
        TextField waitTimeField = new TextField();
        TextField distanceThreshold = new TextField();

        // Set Defaults
        radiusField.setText("1");
        stepLengthField.setText("1");
        loopsField.setText("5");
        stacksField.setText("1");
        altitudeField.setText("10");
        walkComboBox.getSelectionModel().select(Walk.RANGE);
        waitTimeField.setText("0");
        distanceThreshold.setText("1");

        GridUtil.builder(grid)
                .add("Radius:", radiusField)
                .add("Step Length:", stepLengthField)
                .add("Altitude:", altitudeField)
                .add("Loops:", loopsField)
                .add("Stacks:", stacksField)
                .add("Walk:", walkComboBox)
                .add("Wait Time:", waitTimeField)
                .add("Distance Threshold:", distanceThreshold);

        dialog.getDialogPane().setContent(grid);

        dialog.getDialogPane().getButtonTypes().add(ButtonType.CANCEL);
        dialog.getDialogPane().getButtonTypes().add(ButtonType.OK);

        Optional<ButtonType> result = dialog.showAndWait();

        if(result.isPresent() && result.get() == ButtonType.OK) {
            callback.call(Float.parseFloat(radiusField.getText()),
                    Float.parseFloat(stepLengthField.getText()),
                    Float.parseFloat(altitudeField.getText()),
                    Integer.parseInt(loopsField.getText()),
                    Integer.parseInt(stacksField.getText()),
                    walkComboBox.getSelectionModel().getSelectedItem(),
                    Float.parseFloat(waitTimeField.getText()),
                    Float.parseFloat(distanceThreshold.getText()));
        }
    }
}
