import java.math.BigInteger;
import java.util.*;

import com.gurobi.gurobi.GRBConstr;
import com.gurobi.gurobi.GRBException;

public class ColumnGeneration {

    private static final int MAX_CUT_ROUNDS = 5;

    private double[] smoothedDuals = null;
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

    public void solve() throws GRBException {
        NodeResult res = solveForNode(new BCPNode());
        System.out.println("Root LP objective = " + res.objective
                         + "  lpOptimal = " + res.lpOptimal
                         + "  allCovered = " + res.allCovered);
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

        smoothedDuals  = null;
        smoothedTruck  = Double.NaN;
        smoothedDrone  = Double.NaN;
        smoothingAlpha = Constant.SMOOTHING_ALPHA_INIT;

        BigInteger forcedCustomers = BigInteger.ZERO;
        double forcedCost = 0.0;
        int forcedVehicles = 0, forcedDrones = 0;
        // no more forced routes; BCPNode holds decisions only
        int vehCap   = Constant.MAX_VEHICLE - forcedVehicles;
        int droneCap = Constant.MAX_DRONE   - forcedDrones;
        if (vehCap < 0 || droneCap < 0) return res;

        MasterProblem master = new MasterProblem(Constant.TOTAL_CUSTOMER);
        CuttingPlanes cuttingPlanes = new CuttingPlanes();
        Set<String> signatures = new HashSet<>();

        PricingProblem pricing = new PricingProblem(cuttingPlanes, signatures);
        CutGeneration  cutGeneration = new CutGeneration();

        master.setResourceCaps(vehCap, droneCap);
        master.setUncoveredMask(forcedCustomers);

        // apply node decisions as permanent rows on this node's master
        // System.out.println("    [DBG] node decisions = " + node.decisions);
        for (BranchDecision dec : node.decisions) {
            master.addBranchRow(dec);
        }

        int maxColRounds   = 100;
        int maxAddPerRound = 40;
        boolean converged  = false;

        for (int it = 0; it < maxColRounds; it++) {
            pricing.setCgIteration(it);
            try { master.solve(); } catch (GRBException e) { master.dispose(); return res; }
            cuttingPlanes.updateDuals(master);

            double[] rawPi  = master.getDuals();
            double rawTruck = master.getDualVehicle();
            double rawDrone = master.getDualDrone();
            applySmoothing(rawPi, rawTruck, rawDrone);

            pricing.runThreeStagePricing(smoothedDuals, smoothedTruck, smoothedDrone,
                                         forcedCustomers, Collections.emptySet(),
                                         node.decisions, false);

            List<Route> cand = pricing.getNewRoutes();
            boolean lastIter = (it == maxColRounds - 1);

            // Trigger enumeration on the raw duals either when the smoothed pass
            // produced nothing, OR on the last iteration of the round cap.
            if (cand.isEmpty() || lastIter) {
                pricing.runThreeStagePricing(rawPi, rawTruck, rawDrone,
                                            forcedCustomers, Collections.emptySet(),
                                            node.decisions, true);
                cand = pricing.getNewRoutes();
            }
            if (cand.isEmpty()) { converged = true; break; }

            // MIP shortcut
            if (pricing.isEnumerationComplete() && node.depth <= 2) {
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

            boolean added = false;
            int addedCount = 0;
            for (Route r : cand) {
                if (addedCount >= maxAddPerRound) break;
                if (r.reducedCost >= -Constant.EPSILON) continue;
                master.addColumn(r, cuttingPlanes);
                added = true;
                addedCount++;
            }
            if (!added) {
                smoothingAlpha = Math.max(Constant.SMOOTHING_ALPHA_MIN,
                                          smoothingAlpha * Constant.SMOOTHING_ALPHA_DECAY);
                converged = true;
                break;
            }
        }

        try { master.solve(); cuttingPlanes.updateDuals(master); }
        catch (GRBException e) { master.dispose(); return res; }

        for (int cr = 0; cr < MAX_CUT_ROUNDS; cr++) {
            double[] lambda = master.getPrimes();
            List<ICut> newCuts = cutGeneration.separateCuts(master.getRealRoutes(), lambda);
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

            double[] pi = master.getDuals();
            double dT   = master.getDualVehicle();
            double dD   = master.getDualDrone();
            pricing.runThreeStagePricing(pi, dT, dD,
                                         forcedCustomers, Collections.emptySet(),
                                         node.decisions, false);
            List<Route> cand = pricing.getNewRoutes();
            boolean addedCol = false;
            int cnt = 0;
            for (Route r : cand) {
                if (cnt++ >= maxAddPerRound) break;
                if (r.reducedCost >= -Constant.EPSILON) continue;
                master.addColumn(r, cuttingPlanes);
                addedCol = true;
            }
            if (addedCol) {
                try { master.solve(); } catch (GRBException e) { break; }
                cuttingPlanes.updateDuals(master);
            }
        }

        // ---- Final enumeration attempt on raw duals ----
        if (!pricing.isEnumerationComplete() && node.depth <= 1) {
            try {
                master.solve();
                double[] rawPi  = master.getDuals();
                double rawTruck = master.getDualVehicle();
                double rawDrone = master.getDualDrone();
                pricing.runThreeStagePricing(rawPi, rawTruck, rawDrone,
                                            forcedCustomers, Collections.emptySet(),
                                            node.decisions, true);

                // If enumeration succeeded, add ALL enumerated routes and solve as MIP.
                if (pricing.isEnumerationComplete()) {
                    int total = pricing.getEnumeratedTotalCount();
                    if (total > 0 && total <= Constant.ROUTE_ENUM_THRESHOLD) {
                        System.out.println("    [ENUM] using " + total
                                        + " enumerated routes for MIP shortcut.");
                        if (master.addAllAndSolveMip(pricing.getEnumeratedByD())) {
                            double[] art = master.extractArtificialVariableValues();
                            boolean ok = true;
                            for (double a : art)
                                if (a > Constant.EPSILON) { ok = false; break; }
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
            } catch (GRBException ig) {
                // swallow — fall through to LP result
            }
        }

        try { master.solve(); }
        catch (GRBException e) { master.dispose(); return res; }

        res.lpOptimal  = true;
        res.lambda     = master.getPrimes();
        res.columns    = new ArrayList<>(master.getRealRoutes());
        res.artificial = master.extractArtificialVariableValues();
        res.objective  = forcedCost + master.getObjectiveValue();

        res.allCovered = true;
        for (double a : res.artificial)
            if (a > Constant.EPSILON) { res.allCovered = false; break; }

        master.dispose();
        return res;
    }

    // ------------------------------------------------------------------
    //  Dual price smoothing
    // ------------------------------------------------------------------
    private void applySmoothing(double[] pi, double dualTruck, double dualDrone) {
        if (smoothedDuals == null) {
            smoothedDuals = pi.clone();
            smoothedTruck = dualTruck;
            smoothedDrone = dualDrone;
            return;
        }
        double a = smoothingAlpha;
        for (int i = 0; i < pi.length; i++)
            smoothedDuals[i] = a * pi[i] + (1.0 - a) * smoothedDuals[i];
        smoothedTruck = a * dualTruck + (1.0 - a) * smoothedTruck;
        smoothedDrone = a * dualDrone + (1.0 - a) * smoothedDrone;
    }

    public NodeResult solveForNodeAsMip(BCPNode node, List<Route> columns) throws GRBException {
        NodeResult res = new NodeResult();
        res.lpOptimal = false;
        res.allCovered = false;
        res.solvedAsMip = false;
        res.lambda = new double[0];
        res.columns = new ArrayList<>();
        res.artificial = new double[Constant.TOTAL_CUSTOMER];
        res.objective = Double.POSITIVE_INFINITY;

        MasterProblem master = new MasterProblem(Constant.TOTAL_CUSTOMER);
        master.setResourceCaps(Constant.MAX_VEHICLE, Constant.MAX_DRONE);

        for (BranchDecision dec : node.decisions) master.addBranchRow(dec);
        for (Route r : columns) {
            try { master.addColumn(r, null); } catch (GRBException ignore) {}
        }

        if (master.solveAsMip()) {
            double[] art = master.extractArtificialVariableValues();
            boolean ok = true;
            for (double a : art) if (a > Constant.EPSILON) { ok = false; break; }
            if (ok) {
                res.lpOptimal   = true;
                res.allCovered  = true;
                res.solvedAsMip = true;
                res.lambda      = master.getPrimes();
                res.columns     = new ArrayList<>(master.getRealRoutes());
                res.artificial  = art;
                res.objective   = master.getObjectiveValue();
            }
        }
        master.dispose();
        return res;
    }

    /**
     * Evaluate one branch side using ONLY the parent's column pool.
     * No column generation, no cuts. Single LP solve with the branch row added.
     *
     * Used by strong branching phases 2 and 3.
     */
    public NodeResult solveForNodeWithoutCG(BCPNode node, List<Route> parentColumns)
            throws GRBException {

        NodeResult res = new NodeResult();
        res.lpOptimal  = false;
        res.allCovered = false;
        res.solvedAsMip = false;
        res.lambda     = new double[0];
        res.columns    = new ArrayList<>();
        res.artificial = new double[Constant.TOTAL_CUSTOMER];
        res.objective  = Double.POSITIVE_INFINITY;

        MasterProblem master = new MasterProblem(Constant.TOTAL_CUSTOMER);
        master.setResourceCaps(Constant.MAX_VEHICLE, Constant.MAX_DRONE);

        // 1. branch rows first (empty coefficients)
        for (BranchDecision dec : node.decisions) {
            master.addBranchRow(dec);
        }

        // 2. add parent columns — addColumn extends every branch row
        if (parentColumns != null) {
            for (Route r : parentColumns) {
                try { master.addColumn(r, null); } catch (GRBException ignore) {}
            }
        }

        // 3. single LP solve
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
}