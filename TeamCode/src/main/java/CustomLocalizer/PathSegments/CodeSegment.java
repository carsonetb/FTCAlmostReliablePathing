package CustomLocalizer.PathSegments;

import Util.CustomCallable;
import Util.Pose2D;

public class CodeSegment extends PathSegment {
    public CodeSegment(Pose2D pos, CustomCallable code) {
        super(pos, pos);
        code.apply();
        isFinished = true;
    }

    public void update() {}
}
