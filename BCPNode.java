import java.util.ArrayList;
import java.util.List;

public class BCPNode {
    public List<BranchDecision> decisions = new ArrayList<>();
    public int depth = 0;
    public double parentBound = Double.NEGATIVE_INFINITY;   // NEW

    public BCPNode() {}

    public BCPNode copy() {
        BCPNode n = new BCPNode();
        n.decisions   = new ArrayList<>(decisions);
        n.depth       = depth;
        n.parentBound = parentBound;
        return n;
    }

    @Override
    public String toString() {
        return "BCPNode{depth=" + depth + ", |dec|=" + decisions.size() + "}";
    }
}