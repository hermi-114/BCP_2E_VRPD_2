import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

public class CutGeneration {
    private static final int    MAX_CUTS_PER_ROUND   = 50;
    private static final double VIOLATION_THRESHOLD  = 1e-2;
    private static final double EPS                  = 1e-7;

    public CutGeneration() {}

    public List<ICut> separateCuts(List<Route> routes, double[] lambda) {
        List<ICut> newCuts = new ArrayList<>();
        newCuts.addAll(separateARCC(routes, lambda));
        newCuts.addAll(separateR1C (routes, lambda));
        return newCuts;
    }

    // -----------------------------------------------------------------
    // ARCC
    // -----------------------------------------------------------------
    private List<ICut> separateARCC(List<Route> routes, double[] lambda) {
        List<ICut> cuts = new ArrayList<>();
        Set<Set<Integer>> added = new HashSet<>();          // ← avoid duplicates

        int totalCustomer = Constant.TOTAL_CUSTOMER;
        int k = (int) Math.floor((double) Constant.TRUCK_PAYLOAD
                                / Constant.DRONE_AND_EQUIPMENT_WEIGHT + EPS);

        for (int seed = 1; seed <= totalCustomer; seed++) {
            Set<Integer> C = new HashSet<>();
            C.add(seed);

            while (C.size() < totalCustomer) {
                int best = -1;
                double bestViolation = -Double.MAX_VALUE;

                for (int candidate = 1; candidate <= totalCustomer; candidate++) {
                    if (C.contains(candidate)) continue;

                    Set<Integer> test = new HashSet<>(C);
                    test.add(candidate);
                    double vio = computeARCCViolation(test, routes, lambda, k);

                    if (vio > bestViolation) {
                        bestViolation = vio;
                        best = candidate;
                    }
                }

                if (best == -1) break;
                C.add(best);

                double vio = computeARCCViolation(C, routes, lambda, k);
                if (vio > VIOLATION_THRESHOLD && !added.contains(C)) {
                    cuts.add(new ARCCut(new HashSet<>(C)));
                    added.add(new HashSet<>(C));
                    if (cuts.size() >= MAX_CUTS_PER_ROUND) return cuts;
                }
            }
        }
        return cuts;
    }

    private double computeARCCViolation(Set<Integer> C, List<Route> routes,
                                        double[] lambda, int k) {
        double totalDemand = C.stream()
                              .mapToInt(c -> VRPInstance.nodes.get(c).demand)
                              .sum();

        double rhs = Math.ceil(totalDemand / Constant.DRONE_AND_EQUIPMENT_WEIGHT - EPS);
        double lhs = 0.0;

        for (int r = 0; r < routes.size(); r++) {
            if (lambda[r] < Constant.EPSILON) continue;
            Route route = routes.get(r);
            int h = countEntries(C, route);
            lhs += lambda[r] * h * (k - route.getNumDrone());
        }
        return rhs - lhs;
    }

    private int countEntries(Set<Integer> C, Route route) {
        int h_r = 0;

        for (int i = 1; i < route.sequence.size(); i++) {
            int pre = route.sequence.get(i - 1).id;
            int cur = route.sequence.get(i).id;
            if (!C.contains(pre) && C.contains(cur)) h_r++;
        }

        for (var entry : route.customerDroneSchedule.entrySet()) {
            for (int c : entry.getValue().customerServed) {
                if (C.contains(c)) h_r++;
            }
        }
        return h_r;
    }

    // -----------------------------------------------------------------
    // R1C (Subset-Row, rho = 0.5, M = V)
    // -----------------------------------------------------------------
    private List<ICut> separateR1C(List<Route> routes, double[] lambda) {
        List<ICut> cuts = new ArrayList<>();
        Set<Set<Integer>> added = new HashSet<>();          // ← avoid duplicates

        int n = Constant.TOTAL_CUSTOMER;
        for (int i = 1; i <= n; i++) {
            for (int j = i + 1; j <= n; j++) {
                for (int k = j + 1; k <= n; k++) {
                    Set<Integer> C = new HashSet<>(Arrays.asList(i, j, k));
                    if (added.contains(C)) continue;

                    double vio = computeR1CViolation(C, routes, lambda);
                    if (vio > VIOLATION_THRESHOLD) {
                        cuts.add(new R1Cut(new HashSet<>(C)));
                        added.add(new HashSet<>(C));
                        if (cuts.size() >= MAX_CUTS_PER_ROUND) return cuts;
                    }
                }
            }
        }
        return cuts;
    }

    private double computeR1CViolation(Set<Integer> C, List<Route> routes,
                                       double[] lambda) {
        double rhs = Math.floor(C.size() * 0.5 + EPS);
        double lhs = 0.0;

        for (int r = 0; r < routes.size(); r++) {
            if (lambda[r] < Constant.EPSILON) continue;
            Route route = routes.get(r);
            double alpha = computeAlphaR1C(C, route);
            lhs += lambda[r] * alpha;
        }
        return lhs - rhs;
    }

    private double computeAlphaR1C(Set<Integer> C, Route route) {
        // Path: depot (0) at start, depot-copy (-1) at end
        List<Integer> path = new ArrayList<>();
        path.add(0);
        for (int i = 1; i < route.sequence.size() - 1; i++) {
            path.add(route.sequence.get(i).id);
        }
        path.add(-1);

        double s = 0.0;
        for (int node : path) {
            if (C.contains(node)) s += 0.5;
        }
        return Math.floor(s + EPS);
    }
}