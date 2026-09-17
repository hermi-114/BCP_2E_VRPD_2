import com.gurobi.gurobi.GRBException;
import java.math.BigInteger;
import java.util.*;

public class BranchAndPrice {

    private List<Route> bestSolution = null;
    private double      minTime      = Double.MAX_VALUE;
    private final ColumnGeneration cg = new ColumnGeneration();

    private int nodesExplored = 0;
    private int branchesMade  = 0;

    private static final int MAX_NODES = 200;

    public void run() throws GRBException {
        System.out.println("============ Branch and Price (customer-based) ============");

        Deque<BCPNode> stack = new ArrayDeque<>();
        stack.push(new BCPNode());

        while (!stack.isEmpty() && nodesExplored < MAX_NODES) {
            BCPNode node = stack.pop();
            nodesExplored++;

            long t0 = System.currentTimeMillis();
            ColumnGeneration.NodeResult lp = cg.solveForNode(node);
            long t1 = System.currentTimeMillis();

            System.out.println("--- Node #" + nodesExplored
                    + " depth=" + node.depth
                    + " |truck|=" + node.forcedTruck.bitCount()
                    + " |drone|=" + node.forcedDrone.bitCount()
                    + "  time=" + (t1 - t0) + "ms"
                    + "  obj=" + String.format("%.4f", lp.objective));

            if (!lp.lpOptimal) continue;
            if (lp.objective >= minTime - Constant.EPSILON) continue;

            if (lp.allCovered && isIntegral(lp.lambda)) {
                if (lp.objective < minTime) {
                    minTime      = lp.objective;
                    bestSolution = extractSolution(lp);
                    System.out.println("    new incumbent: " + minTime);
                }
                continue;
            }

            int branchCustomer = pickBranchCustomer(lp, node);
            if (branchCustomer < 0) continue;

            double truckFrac = truckFraction(branchCustomer, lp);
            double droneFrac = 1.0 - truckFrac;

            BCPNode childTruck = node.copy();
            childTruck.forcedTruck = childTruck.forcedTruck.setBit(branchCustomer);
            childTruck.depth = node.depth + 1;

            BCPNode childDrone = node.copy();
            childDrone.forcedDrone = childDrone.forcedDrone.setBit(branchCustomer);
            childDrone.depth = node.depth + 1;

            stack.push(childTruck);   // popped second
            stack.push(childDrone);   // popped first
            branchesMade++;
        }

        System.out.println("Nodes explored: " + nodesExplored
                + "  (cap = " + MAX_NODES + ")");
    }

    // ------------------------------------------------------------------
    // Candidate selection
    // ------------------------------------------------------------------

    /**
     * Pick the customer whose service mode is closest to a 50/50 split.
     * Falls back to the most‑uncovered customer if no fractional mode exists.
     */
    private int pickBranchCustomer(ColumnGeneration.NodeResult lp, BCPNode node) {
        int n = Constant.TOTAL_CUSTOMER;

        double[] truckFrac = new double[n + 1];
        double[] droneFrac = new double[n + 1];

        for (int r = 0; r < lp.columns.size(); r++) {
            double lam = lp.lambda[r];
            if (lam < Constant.EPSILON) continue;
            Route route = lp.columns.get(r);

            BigInteger tMask = route.truckServedMask();
            for (BigInteger t = tMask; t.signum() != 0; ) {
                int c = t.getLowestSetBit();
                truckFrac[c] += lam;
                t = t.clearBit(c);
            }

            BigInteger dMask = route.droneServedMask();
            for (BigInteger t = dMask; t.signum() != 0; ) {
                int c = t.getLowestSetBit();
                droneFrac[c] += lam;
                t = t.clearBit(c);
            }
        }

        int bestC = -1;
        double bestScore = 0.0;
        for (int c = 1; c <= n; c++) {
            if (node.isModeFixed(c)) continue;
            double t = truckFrac[c];
            double d = droneFrac[c];
            // skip not-covered and already-integral
            if (t < Constant.EPSILON && d < Constant.EPSILON) continue;
            if (t > 1 - Constant.EPSILON || d > 1 - Constant.EPSILON) continue;
            double score = Math.min(t, d);
            if (score > bestScore) { bestScore = score; bestC = c; }
        }
        return bestC;
    }

    private double truckFraction(int c, ColumnGeneration.NodeResult lp) {
        double sum = 0.0;
        for (int r = 0; r < lp.columns.size(); r++) {
            if (lp.lambda[r] < Constant.EPSILON) continue;
            if (lp.columns.get(r).truckServedMask().testBit(c))
                sum += lp.lambda[r];
        }
        return sum;
    }

    private boolean isIntegral(double[] lambda) {
        for (double v : lambda) {
            double f = Math.abs(v - Math.round(v));
            if (f > 1e-5) return false;
        }
        return true;
    }

    // ------------------------------------------------------------------
    // Solution extraction (unchanged from your original logic)
    // ------------------------------------------------------------------
    private List<Route> extractSolution(ColumnGeneration.NodeResult lp) {
        List<Route> sol = new ArrayList<>();
        for (int i = 0; i < lp.lambda.length; i++) {
            if (lp.lambda[i] > 1 - Constant.EPSILON)
                sol.add(lp.columns.get(i));
        }
        return sol;
    }

    public List<Route> getBestSolution() { return bestSolution; }
    public double      getBestObjective() { return minTime; }
}