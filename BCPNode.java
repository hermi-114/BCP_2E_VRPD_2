import java.math.BigInteger;
import java.util.*;

public class BCPNode {

    // ---- customer-based branching state ----
    public BigInteger forcedTruck = BigInteger.ZERO;     // c MUST be truck-visited
    public BigInteger forcedDrone = BigInteger.ZERO;     // c MUST be drone-served
    public Map<Integer,Integer> droneFromNode = new HashMap<>(); // c -> required launch node u

    // ---- legacy branching (kept for hybrid strategies / cut diversity) ----
    public List<Route> forcedRoutes  = new ArrayList<>();
    public Set<String> forbiddenSigs = new HashSet<>();

    public int depth = 0;

    public BCPNode() {}

    public BCPNode copy() {
        BCPNode n = new BCPNode();
        n.forcedTruck   = forcedTruck;
        n.forcedDrone   = forcedDrone;
        n.droneFromNode = new HashMap<>(droneFromNode);
        n.forcedRoutes  = new ArrayList<>(forcedRoutes);
        n.forbiddenSigs = new HashSet<>(forbiddenSigs);
        n.depth         = depth;
        return n;
    }

    /** true if customer c already has a resolved service mode in this node */
    public boolean isModeFixed(int c) {
        return forcedTruck.testBit(c)
            || forcedDrone.testBit(c)
            || droneFromNode.containsKey(c);
    }

    @Override
    public String toString() {
        return "BCPNode{depth=" + depth
             + ", |truck|=" + forcedTruck.bitCount()
             + ", |drone|=" + forcedDrone.bitCount()
             + ", |fromNode|=" + droneFromNode.size() + "}";
    }
}