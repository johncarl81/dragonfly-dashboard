package edu.unm.dragonfly;

import javafx.scene.control.ListCell;
import javafx.scene.control.ListView;
import javafx.scene.control.Tooltip;
import javafx.scene.image.Image;
import javafx.scene.image.ImageView;
import javafx.util.Callback;

/**
 * @author John Ericksen
 */
public class IconCellFixtureDecorator implements Callback<ListView<Fixture>, ListCell<Fixture>> {

    @Override
    public ListCell<Fixture> call(ListView<Fixture> param) {
        return new ListCell<Fixture>() {
            final Tooltip tooltip = new Tooltip();
            final ImageView imageView = new ImageView();
            @Override
            protected void updateItem(Fixture item, boolean empty) {

                super.updateItem(item, empty);

                if (item == null || empty) {
                    setGraphic(null);
                    setText(null);
                    setTooltip(null);
                } else {
                    imageView.setImage(new Image(item.getIcon()));
                    imageView.setFitHeight(24);
                    imageView.setFitWidth(24);
                    setGraphic(imageView);
                    setText(item.toString());
                    tooltip.setText(item.toString());
                    setTooltip(tooltip);
                }
            }
        };
    }
}
