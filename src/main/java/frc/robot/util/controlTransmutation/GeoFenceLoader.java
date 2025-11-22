package frc.robot.util.controlTransmutation;

import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Optional;
import java.util.function.Function;
import org.json.JSONArray;
import org.json.JSONObject;

import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.util.controlTransmutation.geoFence.Box;
import frc.robot.util.controlTransmutation.geoFence.Fence;
import frc.robot.util.controlTransmutation.geoFence.Line;
import frc.robot.util.controlTransmutation.geoFence.Point;
import frc.robot.util.controlTransmutation.geoFence.Polygon;

public class GeoFenceLoader {
    public static ObjectLists load() {
        var resourceDir = Filesystem.getDeployDirectory().getPath();

        String contents;
        try {
            contents = Files.readString(Path.of(resourceDir, "geofence.json"));
        } catch (Exception e) {
            contents = "";
        }

        var json = new JSONObject(contents);

        Function<String, ObjectList> getList = (name) -> parseObjectList(json.getJSONArray(name));

        // let getList = (getJSONArray json) >> parseObjectList

        return new ObjectLists(getList.apply("blueField"), getList.apply("redField"), getList.apply("sharedField"));
    }

    private static ObjectList parseObjectList(JSONArray json) {
        var objects = new ObjectList();

        for (Object object : json) 
            if (object instanceof JSONObject jsonObject) 
                parseFieldObject(jsonObject).ifPresent(objects::add);

        return objects;
    }

    private static Optional<FieldObject> parseFieldObject(JSONObject json) {
        return switch (json.getString("type")) {
            case "point" -> Optional.of(new Point(
                json.getFloat("x"), 
                json.getFloat("y"), 
                json.getFloat("radius"), 
                json.getFloat("buffer")
            ));
            case "box" -> Optional.of(new Box(
                json.getFloat("xA"), 
                json.getFloat("yA"), 
                json.getFloat("xB"), 
                json.getFloat("yB"),
                json.getFloat("radius"), 
                json.getFloat("buffer")
            ));
            case "fence" -> Optional.of(new Fence(
                json.getFloat("xA"), 
                json.getFloat("yA"), 
                json.getFloat("xB"), 
                json.getFloat("yB"),
                json.getFloat("radius"), 
                json.getFloat("buffer")
            ));
            case "line" -> Optional.of(new Line(
                json.getFloat("xA"), 
                json.getFloat("yA"), 
                json.getFloat("xB"), 
                json.getFloat("yB"),
                json.getFloat("radius"), 
                json.getFloat("buffer")
            ));
            case "polygon" -> Optional.of(new Polygon(
                json.getFloat("x"), 
                json.getFloat("y"), 
                json.getFloat("radius"), 
                json.getFloat("buffer"),
                json.getFloat("theta"),
                json.getInt("sides")
            ));
            default -> Optional.empty();
        };
    }

    record ObjectLists(ObjectList blueField, ObjectList redField, ObjectList sharedField) {}
}
