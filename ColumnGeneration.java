import com.gurobi.gurobi.GRBConstr;
import com.gurobi.gurobi.GRBException;
import java.math.BigInteger;
import java.util.*;

public class ColumnGeneration {

    private double[] smoothedDuals;
    private double   smoothedTruck = Double.NaN;
    private double   smoothedDrone = Double.NaN;
    private double   smoothingAlpha = Constant.SMOOTHING_ALPHA_INIT;

    public static class NodeResult {
        public double       objective;
        public double[]     lambda;
        public List<Route>  columns;
        public double[]     artificial;
        public boolean      lpOptimal;
        public boolean      allCovered;
        public boolean      solvedAsMip;
    }

    public NodeResult solveForNode(BCPNode node) throws GRBException {

        NodeResult res = new NodeResult();
        res.lpOptimal  = false;
        res.allCovered = false;
        res.lambda     = new double[0];
        res.columns    = new ArrayList<>();
        res.artificial = new double[Constant.TOTAL_CUSTOMER];
        res.objective  = Double.POSITIVE_INFINITY;
        res.solvedAsMip = false;

        smoothedDuals = null;
        smoothedTruck = Double.NaN;
        smoothedDrone = Double.NaN;
        smoothingAlpha = Constant.SMOOTHING_ALPHA_INIT;

        BigInteger forcedCustomers = BigInteger.ZERO;
        int vehCap   = Constant.MAX_VEHICLE;
        int droneCap = Constant.MAX_DRONE;

        long nodeStart = System.currentTimeMillis();
        boolean isRoot = (node.depth == 0);
        long nodeTimeLimit = isRoot ? Constant.NODE_TIME_LIMIT_ROOT_MS
                                    : Constant.NODE_TIME_LIMIT_CHILD_MS;
        int  cgRounds      = isRoot ? Constant.MAX_COL_ROUNDS_ROOT
                                    : Constant.MAX_COL_ROUNDS_CHILD;

        MasterProblem master = new MasterProblem(Constant.TOTAL_CUSTOMER);
        CuttingPlanes cuttingPlanes = new CuttingPlanes();
        Set<String> signatures = new HashSet<>();

        PricingProblem pricing = new PricingProblem(cuttingPlanes, signatures);
        CutGeneration cutGeneration = new CutGeneration();

        master.setResourceCaps(vehCap, droneCap);
        master.setUncoveredMask(forcedCustomers);

        seedInitialColumns(master, cuttingPlanes, node);

        for (BranchDecision dec : node.decisions) master.addBranchRow(dec);

        // ================== CG LOOP ==================
        for (int it = 0; it < cgRounds; it++) {

            if (System.currentTimeMillis() - nodeStart > nodeTimeLimit) {
                System.out.println("    [CG] node time limit hit at iter " + it);
                break;
            }

            // ---- solve the restricted master, get duals (either Pi or Farkas) ----
            boolean feasible = master.solveReturnFeasibility();

            double[] rawPi;
            double   rawTruck;
            double   rawDrone;

            if (feasible) {
                cuttingPlanes.updateDuals(master);
                rawPi    = master.getDuals();
                rawTruck = master.getDualVehicle();
                rawDrone = master.getDualDrone();
            } else {
                // LP infeasible under the current pool + branch rows.
                // Use zero duals: pricing will produce the cheapest routes,
                // any of which might fix the infeasibility.
                rawPi    = new double[Constant.TOTAL_CUSTOMER];
                rawTruck = 0.0;
                rawDrone = 0.0;
            }

            
            // ---- smoothed pass (heuristic pricing) ----
            applySmoothing(rawPi, rawTruck, rawDrone);
            
            pricing.runThreeStagePricing(smoothedDuals, smoothedTruck, smoothedDrone,
            forcedCustomers, Collections.emptySet(),
            node.decisions,
            false,          // no enumeration yet
            isRoot);        // exact only at root
            
            List<Route> cand = pricing.getNewRoutes();
            
            // ---- raw-dual pass (on empty result or last iteration) ----
            boolean lastIter = (it == cgRounds - 1);
            if (cand.isEmpty() || lastIter) {
                pricing.runThreeStagePricing(rawPi, rawTruck, rawDrone,
                forcedCustomers, Collections.emptySet(),
                                            node.decisions,
                                            isRoot && lastIter,   // enumerate only at root's last iter
                                            isRoot);              // exact only at root
                                            cand = pricing.getNewRoutes();
                                        }
                                        
                                        // ---- stop if pricing produced nothing ----
                                        if (cand.isEmpty()) {
                                            if (!feasible) {
                                                System.out.println("    [CG] LP infeasible & no new columns → prune");
                                            }
                break;
            }
            
            // ---- MIP shortcut at root (only when feasible) ----
            if (isRoot && feasible && pricing.isEnumerationComplete()) {
                int total = pricing.getEnumeratedTotalCount();
                if (total > 0 && total <= Constant.ROUTE_ENUM_THRESHOLD) {
                    if (master.addAllAndSolveMip(pricing.getEnumeratedByD())) {
                        double[] art = master.extractArtificialVariableValues();
                        boolean ok = true;
                        for (double a : art) if (a > Constant.EPSILON) { ok = false; break; }
                        if (ok) {
                            res.solvedAsMip = true;
                            res.lpOptimal   = true;
                            res.lambda      = master.getPrimes();
                            res.columns     = new ArrayList<>(master.getRealRoutes());
                            res.artificial  = art;
                            res.objective   = master.getObjectiveValue();
                            res.allCovered  = true;
                            master.dispose();
                            return res;
                        }
                    }
                }
            }
            
            // ---- add columns ----
            boolean added = false;
            int cnt = 0;
            for (Route r : cand) {
                if (cnt >= Constant.MAX_ADD_PER_ROUND) break;
                
                // When LP was feasible: require negative reduced cost (usual rule)
                // When LP was infeasible: accept any column pricing returned —
                //   its Farkas-reduced cost may be < 0, but the sign convention
                //   is different, so we don't filter.
                if (feasible && r.reducedCost >= -Constant.EPSILON) continue;

                master.addColumn(r, cuttingPlanes);
                added = true;
                cnt++;
            }

            if (!added) {
                if (!feasible) {
                    System.out.println("    [CG] no useful column for Farkas certificate → prune");
                }
                smoothingAlpha = Math.max(Constant.SMOOTHING_ALPHA_MIN,
                                        smoothingAlpha * Constant.SMOOTHING_ALPHA_DECAY);
                break;
            }
        }

        try { master.solve(); cuttingPlanes.updateDuals(master); }
        catch (GRBException e) { master.dispose(); return res; }

        // ================== CUTS ==================
        int cutRounds   = isRoot ? Constant.CUT_ROUNDS_ROOT : Constant.CUT_ROUNDS_CHILD;
        int maxCutsPerR = isRoot ? Constant.MAX_CUTS_PER_ROUND_ROOT
                                 : Constant.MAX_CUTS_PER_ROUND_CHILD;

        for (int cr = 0; cr < cutRounds; cr++) {
            if (System.currentTimeMillis() - nodeStart > nodeTimeLimit * 2) break;

            double[] lambda = master.getPrimes();
            List<ICut> newCuts = cutGeneration.separateCuts(master.getRealRoutes(), lambda);

            // filter by violation
            newCuts.removeIf(c -> c.getViolation() < Constant.CUT_VIOLATION_THRESHOLD);
            if (newCuts.size() > maxCutsPerR) newCuts = newCuts.subList(0, maxCutsPerR);
            if (newCuts.isEmpty()) break;

            List<GRBConstr> addedC = new ArrayList<>();
            try {
                for (ICut cut : newCuts) {
                    cuttingPlanes.addCut(cut);
                    addedC.add(master.addCutAndReturn(cut));
                }
                master.solve();
                cuttingPlanes.updateDuals(master);
            } catch (GRBException e) {
                for (int k = addedC.size() - 1; k >= 0; k--)
                    try { master.removeConstraint(addedC.get(k)); } catch (GRBException ig) {}
                for (int k = 0; k < addedC.size(); k++)
                    if (!cuttingPlanes.cuts.isEmpty())
                        cuttingPlanes.cuts.remove(cuttingPlanes.cuts.size() - 1);
                try { master.solve(); } catch (GRBException ig) {}
                break;
            }
        }

        // ================== FINAL SOLVE ==================
        try { master.solve(); }
        catch (GRBException e) { master.dispose(); return res; }

        res.lpOptimal  = true;
        res.lambda     = master.getPrimes();
        res.columns    = new ArrayList<>(master.getRealRoutes());
        res.artificial = master.extractArtificialVariableValues();
        res.objective  = master.getObjectiveValue();

        res.allCovered = true;
        for (double a : res.artificial)
            if (a > Constant.EPSILON) { res.allCovered = false; break; }

        master.dispose();
        return res;
    }

    // ------------------------------------------------------------------
    // Single LP solve for strong branching — no CG, no cuts, no state.
    // ------------------------------------------------------------------
    public NodeResult solveForNodeWithoutCG(BCPNode node, List<Route> parentColumns) throws GRBException {
        NodeResult res = new NodeResult();
        res.lpOptimal = false;
        res.objective = Double.POSITIVE_INFINITY;
        res.columns   = new ArrayList<>();
        res.lambda    = new double[0];

        MasterProblem master = new MasterProblem(Constant.TOTAL_CUSTOMER);
        master.setResourceCaps(Constant.MAX_VEHICLE, Constant.MAX_DRONE);

        for (BranchDecision dec : node.decisions) master.addBranchRow(dec);
        if (parentColumns != null) {
            for (Route r : parentColumns) {
                try { master.addColumn(r, null); } catch (GRBException ignore) {}
            }
        }

        try {
            double obj = master.solveReturnObjective();
            if (!Double.isInfinite(obj)) {
                res.lpOptimal  = true;
                res.objective  = obj;
                res.lambda     = master.getPrimes();
                res.columns    = new ArrayList<>(master.getRealRoutes());
                res.artificial = master.extractArtificialVariableValues();
                res.allCovered = true;
                for (double a : res.artificial)
                    if (a > Constant.EPSILON) { res.allCovered = false; break; }
            }
        } catch (GRBException e) {
            res.lpOptimal = false;
        } finally {
            master.dispose();
        }
        return res;
    }

    /**
     * Solve the given column pool as a MIP with the node's branch rows.
     * Used for root MIP diagnostic and as a safety-net incumbent.
     */
    public NodeResult solveForNodeAsMip(BCPNode node, List<Route> columns) throws GRBException {

        NodeResult res = new NodeResult();
        res.lpOptimal   = false;
        res.allCovered  = false;
        res.solvedAsMip = true;
        res.lambda      = new double[0];
        res.columns     = new ArrayList<>();
        res.artificial  = new double[Constant.TOTAL_CUSTOMER];
        res.objective   = Double.POSITIVE_INFINITY;

        MasterProblem master = new MasterProblem(Constant.TOTAL_CUSTOMER);
        master.setResourceCaps(Constant.MAX_VEHICLE, Constant.MAX_DRONE);

        // branch rows first (empty coefficients)
        for (BranchDecision dec : node.decisions) master.addBranchRow(dec);

        // parent columns — addColumn extends every branch row
        if (columns != null) {
            for (Route r : columns) {
                try { master.addColumn(r, null); } catch (GRBException ignore) {}
            }
        }

        try {
            boolean found = master.solveAsMip();
            if (found) {
                double[] art = master.extractArtificialVariableValues();
                boolean allZero = true;
                for (double a : art)
                    if (a > Constant.EPSILON) { allZero = false; break; }

                if (allZero) {
                    res.lpOptimal  = true;
                    res.allCovered = true;
                    res.lambda     = master.getPrimes();
                    res.columns    = new ArrayList<>(master.getRealRoutes());
                    res.artificial = art;
                    res.objective  = master.getObjectiveValue();
                }
            }
        } catch (GRBException e) {
            res.lpOptimal = false;
        } finally {
            master.dispose();
        }
        return res;
    }

    private void applySmoothing(double[] pi, double dT, double dD) {
        if (smoothedDuals == null) {
            smoothedDuals = pi.clone();
            smoothedTruck = dT;
            smoothedDrone = dD;
            return;
        }
        double a = smoothingAlpha;
        for (int i = 0; i < pi.length; i++)
            smoothedDuals[i] = a * pi[i] + (1.0 - a) * smoothedDuals[i];
            
        smoothedTruck = a * dT + (1.0 - a) * smoothedTruck;
        smoothedDrone = a * dD + (1.0 - a) * smoothedDrone;
    }

    private void seedInitialColumns(MasterProblem master, CuttingPlanes cuts, BCPNode node) throws GRBException {

        int attempted = 0;
        int addedTruck = 0;
        int addedDrone = 0;
        int rejected   = 0;

        // ---- (A) single-customer truck routes ----
        for (int c = 1; c <= Constant.TOTAL_CUSTOMER; c++) {
            attempted++;
            List<Node> seq = new ArrayList<>();
            seq.add(VRPInstance.nodes.get(c));
            Route r;
            try {
                r = new Route(seq, new HashMap<>());
            } catch (Exception e) {
                rejected++;
                continue;
            }
            if (passesBranch(r, node)) { master.addColumn(r, cuts); addedTruck++; }
        }

        // ---- (B) greedy nearest-neighbour truck routes of length 2..6 ----
        // Covers multiple customers per route so the LP can respect MAX_VEHICLE.
        for (int seed = 1; seed <= Constant.TOTAL_CUSTOMER; seed++) {
            if (VRPInstance.nodes.get(seed).demand > Constant.TRUCK_PAYLOAD) continue;

            List<Node> seq = new ArrayList<>();
            seq.add(VRPInstance.nodes.get(seed));
            BigInteger used = BigInteger.ONE.shiftLeft(seed);

            // extend by nearest unvisited customer
            for (int len = 2; len <= 6; len++) {
                int last = seq.get(seq.size() - 1).id;
                int best = -1;
                double bestDist = Double.MAX_VALUE;
                for (int c = 1; c <= Constant.TOTAL_CUSTOMER; c++) {
                    if (used.testBit(c)) continue;
                    if (VRPInstance.nodes.get(c).demand
                            + seqDemand(seq) > Constant.TRUCK_PAYLOAD) continue;
                    double d = VRPInstance.distMatrix[last][c];
                    if (d < bestDist) { bestDist = d; best = c; }
                }
                if (best < 0) break;
                seq.add(VRPInstance.nodes.get(best));
                used = used.setBit(best);

                // try to construct and add this prefix as a route
                try {
                    Route r = new Route(new ArrayList<>(seq), new HashMap<>());
                    if (passesBranch(r, node)) {
                        master.addColumn(r, cuts);
                        addedTruck++;
                    }
                } catch (Exception e) {
                    // prefix is infeasible (time window or depot deadline) — stop extending
                    break;
                }
            }
        }

        // ---- (C) single-customer drone seeds ----
        for (int u = 1; u <= Constant.TOTAL_CUSTOMER; u++) {
            if (u >= DroneScheduleEnumeration.paretoMap.size()) continue;
            var byD = DroneScheduleEnumeration.paretoMap.get(u);
            if (byD.size() < 2) continue;

            for (var entry : byD.get(1).entrySet()) {          // 1-drone schedules
                for (DroneSchedule sch : entry.getValue().nonDominatedSchedules) {
                    List<Node> seq = new ArrayList<>();
                    seq.add(VRPInstance.nodes.get(u));
                    Map<Integer, DroneSchedule> drones = new HashMap<>();
                    drones.put(u, sch);
                    try {
                        Route r = new Route(seq, drones);
                        if (passesBranch(r, node)) {
                            master.addColumn(r, cuts);
                            addedDrone++;
                        }
                    } catch (Exception e) { /* skip infeasible seed */ }
                }
            }
        }

        System.out.println("    [SEED] attempted=" + attempted
                        + " truck=" + addedTruck
                        + " drone=" + addedDrone
                        + " rejected=" + rejected
                        + " pool=" + master.getNumRealVars());
    }

    private boolean passesBranch(Route r, BCPNode node) {
        for (BranchDecision dec : node.decisions) {
            double coef = dec.candidate.coefficient(r);
            if (dec.upperBound && coef > dec.rhs + Constant.EPSILON) return false;
            if (!dec.upperBound && coef < dec.rhs - Constant.EPSILON) return false;
        }
        return true;
    }

    private double seqDemand(List<Node> seq) {
        double d = 0;
        for (Node n : seq) d += n.demand;
        return d;
    }
}