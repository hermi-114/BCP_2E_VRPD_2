import java.math.BigInteger;
import java.util.*;

import com.gurobi.gurobi.GRBException;

public class BranchAndPrice {

    private List<Route> bestSolution = null;
    private double      minTime     = Double.MAX_VALUE;
    private final ColumnGeneration cg = new ColumnGeneration();

    public void run() throws GRBException {
        System.out.println("=================== Branch and Price ====================");

        Deque<BCPNode> queue = new ArrayDeque<>();
        BCPNode root = new BCPNode();
        queue.add(root);

        int nodesExplored = 0;

        while (!queue.isEmpty()) {
            BCPNode node = queue.poll();
            nodesExplored++;
            System.out.println("--- Node #" + nodesExplored
                             + "  depth=" + node.depth
                             + "  forced=" + node.forcedRoutes.size()
                             + "  forbidden=" + node.forbiddenSigs.size());

            ColumnGeneration.NodeResult lp =
                cg.solveForNode(node.forcedRoutes, node.forbiddenSigs);

            // prune by bound
            if (lp.objective >= minTime - Constant.EPSILON) {
                System.out.println("    prune by bound (" + lp.objective
                                 + " ≥ " + minTime + ")");
                continue;
            }

            // prune by infeasibility (uncovered customers with no available columns)
            if (!lp.feasible) {
                System.out.println("    prune by infeasibility");
                continue;
            }

            // integral?
            int fracIdx = mostFractional(lp.lambda);
            if (fracIdx < 0) {
                System.out.println("    integral leaf, obj=" + lp.objective);
                if (lp.objective < minTime) {
                    minTime = lp.objective;
                    bestSolution = new ArrayList<>(node.forcedRoutes);
                    for (int i = 0; i < lp.lambda.length; i++)
                        if (lp.lambda[i] > 0.5) bestSolution.add(lp.columns.get(i));
                }
                continue;
            }

            // branch on the most fractional column
            Route branchRoute = lp.columns.get(fracIdx);
            double branchValue = lp.lambda[fracIdx];
            System.out.println("    branch on route id=" + branchRoute.id
                             + "  λ=" + String.format("%.4f", branchValue));

            // Child A — forbid branchRoute
            BCPNode childA = new BCPNode();
            childA.forcedRoutes  = new ArrayList<>(node.forcedRoutes);
            childA.forbiddenSigs = new HashSet<>(node.forbiddenSigs);
            childA.forbiddenSigs.add(branchRoute.getSignature());
            childA.depth         = node.depth + 1;

            // Child B — force branchRoute (only if compatible with existing forced)
            BigInteger forcedMask = BigInteger.ZERO;
            for (Route r : node.forcedRoutes)
                forcedMask = forcedMask.or(r.customerServedHashed);
            boolean compatible = forcedMask.and(branchRoute.customerServedHashed)
                                           .equals(BigInteger.ZERO);
            if (compatible) {
                BCPNode childB = new BCPNode();
                childB.forcedRoutes  = new ArrayList<>(node.forcedRoutes);
                childB.forcedRoutes.add(branchRoute);
                childB.forbiddenSigs = new HashSet<>(node.forbiddenSigs);
                childB.depth         = node.depth + 1;
                queue.add(childB);
            }

            queue.add(childA);
        }

        System.out.println("=================== Done ===================");
        System.out.println("Nodes explored: " + nodesExplored);
        System.out.println("Best objective: " + minTime);
    }

    private int mostFractional(double[] lambda) {
        int idx = -1;
        double best = 1e-6;
        for (int i = 0; i < lambda.length; i++) {
            double f = Math.abs(lambda[i] - Math.round(lambda[i]));
            if (f > best) { best = f; idx = i; }
        }
        return idx;
    }

    public List<Route> getBestSolution() { return bestSolution; }
    public double      getBestObjective() { return minTime; }
}