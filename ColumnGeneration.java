import java.math.BigInteger;
import java.util.*;

import com.gurobi.gurobi.GRBConstr;
import com.gurobi.gurobi.GRBException;

public class ColumnGeneration {

    private static final int MAX_CUT_ROUNDS = 5;

    // ---- dual price smoothing state (Pessoa 2018) ----
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
        public boolean      solvedAsMip;    // NEW: MIP shortcut was used
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

        // reset smoothing per node
        smoothedDuals = null;
        smoothedTruck = Double.NaN;
        smoothedDrone = Double.NaN;
        smoothingAlpha = Constant.SMOOTHING_ALPHA_INIT;

        // ---------- 1. forced state ----------
        BigInteger forcedCustomers = BigInteger.ZERO;
        double forcedCost = 0.0;
        int forcedVehicles = 0, forcedDrones = 0;
        for (Route r : node.forcedRoutes) {
            forcedCustomers = forcedCustomers.or(r.customerServedHashed);
            forcedCost     += r.totalTime;
            forcedVehicles += 1;
            forcedDrones   += r.getNumDrone();
        }

        int vehCap   = Constant.MAX_VEHICLE - forcedVehicles;
        int droneCap = Constant.MAX_DRONE   - forcedDrones;
        if (vehCap < 0 || droneCap < 0) return res;

        // ---------- 2. master / cuts / pricing ----------
        MasterProblem master = new MasterProblem(Constant.TOTAL_CUSTOMER);
        CuttingPlanes cuttingPlanes = new CuttingPlanes();
        Set<String> signatures = new HashSet<>(node.forbiddenSigs);

        PricingProblem pricing = new PricingProblem(cuttingPlanes, signatures);
        CutGeneration  cutGeneration = new CutGeneration();

        master.setResourceCaps(vehCap, droneCap);
        master.setUncoveredMask(forcedCustomers);

        int maxColRounds   = 50;
        int maxAddPerRound = Constant.MAX_COLUMNS_EXACT
                           + Constant.MAX_COLUMNS_LIGHT
                           + Constant.MAX_COLUMNS_HEURISTIC;
        boolean converged  = false;

        // ---------- 3. column generation ----------
        for (int it = 0; it < maxColRounds; it++) {

            pricing.setCgIteration(it);

            try { master.solve(); }
            catch (GRBException e) { master.dispose(); return res; }

            cuttingPlanes.updateDuals(master);

            double[] rawPi   = master.getDuals();
            double rawTruck  = master.getDualVehicle();
            double rawDrone  = master.getDualDrone();

            // ---- dual smoothing ----
            applySmoothing(rawPi, rawTruck, rawDrone);

            pricing.runThreeStagePricing(
                    smoothedDuals, smoothedTruck, smoothedDrone,
                    forcedCustomers, node.forbiddenSigs,
                    node.forcedTruck, node.forcedDrone, node.droneFromNode);

            // ---- MIP shortcut from enumeration ----
            if (pricing.isEnumerationComplete()) {
                int totalEnum = pricing.getEnumeratedTotalCount();
                if (totalEnum > 0 && totalEnum <= Constant.ROUTE_ENUM_THRESHOLD) {
                    System.out.println("    [CG] enumeration complete with "
                                     + totalEnum + " routes → solving as MIP.");
                    boolean ok = master.addAllAndSolveMip(pricing.getEnumeratedByD());
                    if (ok) {
                        res.solvedAsMip = true;
                        res.lpOptimal   = true;
                        res.lambda      = master.getPrimes();
                        res.columns     = new ArrayList<>(master.getRealRoutes());
                        res.artificial  = master.artificialValues.clone();
                        res.objective   = forcedCost + master.getObjectiveValue();
                        res.allCovered  = true;
                        for (double a : res.artificial)
                            if (a > Constant.EPSILON) { res.allCovered = false; break; }
                        master.dispose();
                        return res;
                    }
                }
                // If enumeration didn't pay off (master blew up), fall back below
            }

            List<Route> candidates = pricing.getNewRoutes();
            if (candidates.isEmpty()) {
                System.out.println("    [CG] iteration " + it + ": converged.");
                converged = true;
                break;
            }

            boolean added = false;
            int addedCount = 0;
            for (Route r : candidates) {
                if (addedCount >= maxAddPerRound) break;
                if (r.reducedCost >= -Constant.EPSILON) continue;
                if (!compatible(r, node)) continue;
                master.addColumn(r, cuttingPlanes);
                added = true;
                addedCount++;
            }
            System.out.println("    [CG] iteration " + it + ": added " + addedCount
                             + " columns (" + candidates.size() + " candidates).");

            if (!added) {
                // pricing failed to produce a *new* improving column → decrease smoothing
                smoothingAlpha = Math.max(Constant.SMOOTHING_ALPHA_MIN,
                                          smoothingAlpha * Constant.SMOOTHING_ALPHA_DECAY);
                System.out.printf("    [CG] smoothing α → %.2f%n", smoothingAlpha);
                converged = true;
                break;
            }
        }

        // ---------- refresh primals ----------
        try {
            master.solve();
            cuttingPlanes.updateDuals(master);
        } catch (GRBException e) { master.dispose(); return res; }

        // ---------- 4. cut generation with rollback ----------
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
                for (int k = addedC.size() - 1; k >= 0; k--) {
                    try { master.removeConstraint(addedC.get(k)); } catch (GRBException ig) {}
                }
                for (int k = 0; k < addedC.size(); k++) {
                    if (!cuttingPlanes.cuts.isEmpty())
                        cuttingPlanes.cuts.remove(cuttingPlanes.cuts.size() - 1);
                }
                try { master.solve(); } catch (GRBException ig) {}
                break;
            }

            double[] rawPi  = master.getDuals();
            double rawTruck = master.getDualVehicle();
            double rawDrone = master.getDualDrone();
            applySmoothing(rawPi, rawTruck, rawDrone);

            pricing.runThreeStagePricing(
                    smoothedDuals, smoothedTruck, smoothedDrone,
                    forcedCustomers, node.forbiddenSigs,
                    node.forcedTruck, node.forcedDrone, node.droneFromNode);

            List<Route> cand = pricing.getNewRoutes();
            boolean addedCol = false;
            int cnt = 0;
            for (Route r : cand) {
                if (cnt++ >= maxAddPerRound) break;
                if (r.reducedCost >= -Constant.EPSILON) continue;
                if (!compatible(r, node)) continue;
                master.addColumn(r, cuttingPlanes);
                addedCol = true;
            }
            if (addedCol) {
                try { master.solve(); } catch (GRBException e) { break; }
                cuttingPlanes.updateDuals(master);
            }
        }

        // ---------- 5. final solve ----------
        try { master.solve(); }
        catch (GRBException e) { master.dispose(); return res; }

        res.lpOptimal  = true;
        res.lambda     = master.getPrimes();
        res.columns    = new ArrayList<>(master.getRealRoutes());
        res.artificial = master.artificialValues.clone();
        res.objective  = forcedCost + master.getObjectiveValue();

        res.allCovered = true;
        for (double a : res.artificial) {
            if (a > Constant.EPSILON) { res.allCovered = false; break; }
        }

        master.dispose();
        return res;
    }

    // ------------------------------------------------------------------
    //  Dual price smoothing (Pessoa et al. 2018)
    // ------------------------------------------------------------------
    private void applySmoothing(double[] pi, double dualTruck, double dualDrone) {
        if (smoothedDuals == null) {
            smoothedDuals = pi.clone();
            smoothedTruck = dualTruck;
            smoothedDrone = dualDrone;
            return;
        }
        double a = smoothingAlpha;
        for (int i = 0; i < pi.length; i++) {
            smoothedDuals[i] = a * pi[i] + (1.0 - a) * smoothedDuals[i];
        }
        smoothedTruck = a * dualTruck + (1.0 - a) * smoothedTruck;
        smoothedDrone = a * dualDrone + (1.0 - a) * smoothedDrone;
    }

    // ------------------------------------------------------------------
    //  Branch-compatibility safety net
    // ------------------------------------------------------------------
    private static boolean compatible(Route r, BCPNode node) {
        BigInteger seqMask   = r.truckServedMask();
        BigInteger droneMask = r.droneServedMask();

        if (!seqMask.and(node.forcedTruck).equals(node.forcedTruck)) return false;
        if (!droneMask.and(node.forcedDrone).equals(node.forcedDrone)) return false;
        if (seqMask.and(node.forcedDrone).signum() != 0) return false;

        for (Map.Entry<Integer,Integer> e : node.droneFromNode.entrySet()) {
            int c = e.getKey(), u = e.getValue();
            if (seqMask.testBit(c)) return false;
            if (!r.isDroneServedFrom(c, u)) return false;
        }
        return true;
    }
}