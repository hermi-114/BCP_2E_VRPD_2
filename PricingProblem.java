import java.math.BigInteger;
import java.util.*;

public class PricingProblem {

    public enum PricingStage { LIGHT, HEURISTIC, EXACT }

    CuttingPlanes cuttingPlanes;
    List<RCSPGraph> graphs;
    List<Label>[][][][] buckets;
    Label[][][][] dominatingLabels;

    public List<Route> newRoutes = new ArrayList<>();
    Set<String> signatures;
    BigInteger[] ngNeighborhood;
    private List<R1Cut> activeR1Cuts = new ArrayList<>();

    private BigInteger blockedCustomers = BigInteger.ZERO;
    private Set<String> forbiddenSigs = new HashSet<>();
    private final Set<Long> bannedArcs = new HashSet<>();
    private long requiredArc = -1;

    private List<List<Route>> enumeratedByD = null;
    private int     enumeratedTotalCount = 0;
    private boolean enumerationComplete  = false;

    static final int NUM_NODES  = 2 * (Constant.TOTAL_CUSTOMER + 1);
    static final int NUM_D      = Constant.MAX_DRONE_PER_VEHICLE + 1;
    static final int NUM_BUCKET = Config.SIZE_BUCKET;

    double timeOrigin, timeStep, capaStep;

    public PricingProblem(CuttingPlanes cuttingPlanes, Set<String> usedRoutes) {
        this.cuttingPlanes = cuttingPlanes;
        this.signatures    = usedRoutes;
        initNgNeighborhoods();
        initBucketSteps();
        allocateBuckets();
    }

    private static class RCSPGraph { List<List<RCSPArc>> adjacencyList = new ArrayList<>(); }

    private void initNgNeighborhoods() {
        if (VRPInstance.ngNeighborhood != null) {
            ngNeighborhood = VRPInstance.ngNeighborhood;
            return;
        }
        int n = Constant.TOTAL_CUSTOMER;
        ngNeighborhood = new BigInteger[n + 1];
        BigInteger all = BigInteger.ZERO;
        for (int u = 0; u <= n; u++) all = all.setBit(u);
        for (int v = 0; v <= n; v++) ngNeighborhood[v] = (v == 0) ? BigInteger.ZERO : all;
    }

    private void initBucketSteps() {
        double minStart = Double.POSITIVE_INFINITY, maxEnd = Double.NEGATIVE_INFINITY;
        for (Node node : VRPInstance.nodes) {
            if (node.tw_a < minStart) minStart = node.tw_a;
            if (node.tw_b > maxEnd)   maxEnd   = node.tw_b;
        }
        timeOrigin = Double.isInfinite(minStart) ? 0.0 : minStart;
        double horizon = Math.max(maxEnd - timeOrigin, 1.0);
        timeStep = horizon / NUM_BUCKET;
        capaStep = (double) Constant.TRUCK_PAYLOAD / NUM_BUCKET;
    }

    @SuppressWarnings("unchecked")
    private void allocateBuckets() {
        buckets = (List<Label>[][][][]) new List[NUM_D][NUM_NODES][NUM_BUCKET][NUM_BUCKET];
        dominatingLabels = new Label[NUM_D][NUM_NODES][NUM_BUCKET][NUM_BUCKET];
        for (int d = 0; d < NUM_D; d++)
            for (int v = 0; v < NUM_NODES; v++)
                for (int i = 0; i < NUM_BUCKET; i++)
                    for (int j = 0; j < NUM_BUCKET; j++)
                        buckets[d][v][i][j] = new ArrayList<>();
    }

    // ------------------------------------------------------------------
    // Entry point with runExact flag
    // ------------------------------------------------------------------
    public void runThreeStagePricing(double[] pi,
                                     double dualTruck,
                                     double dualDrone,
                                     BigInteger blockedCustomers,
                                     Set<String> forbiddenSigs,
                                     List<BranchDecision> decisions,
                                     boolean allowEnumeration,
                                     boolean runExact) {

        this.blockedCustomers = blockedCustomers;
        this.forbiddenSigs = forbiddenSigs;
        this.bannedArcs.clear();
        this.requiredArc = -1;
        if (decisions != null) {
            for (BranchDecision d : decisions) {
                if (d.candidate.type != BranchCandidate.Type.TRUCK_ARC) continue;
                long key = arcKey(d.candidate.arcI, d.candidate.arcJ);
                if (d.upperBound) bannedArcs.add(key);
                else              requiredArc = key;
            }
        }

        newRoutes.clear();
        activeR1Cuts.clear();
        for (ICut cut : cuttingPlanes.cuts)
            if (cut instanceof R1Cut r) activeR1Cuts.add(r);

        if (enumerationComplete) {
            inspectEnumeratedRoutes(pi, dualTruck, dualDrone);
            return;
        }

        buildGraphs(pi);

        // LIGHT
        solveRCSP(dualTruck, dualDrone, PricingStage.LIGHT, Constant.MAX_COLUMNS_LIGHT);

        // HEURISTIC (only if LIGHT was short)
        if (newRoutes.size() < Constant.MAX_COLUMNS_LIGHT) {
            solveRCSP(dualTruck, dualDrone, PricingStage.HEURISTIC, Constant.MAX_COLUMNS_HEURISTIC);
        }

        // EXACT (only when caller explicitly requested it — i.e. at the root)
        if (runExact
            && newRoutes.size() < Constant.MAX_COLUMNS_LIGHT + Constant.MAX_COLUMNS_HEURISTIC) {
            solveRCSP(dualTruck, dualDrone, PricingStage.EXACT, Constant.MAX_COLUMNS_EXACT);
        }

        // ENUMERATION (only at root by caller flag)
        if (allowEnumeration) tryRouteEnumeration(pi, dualTruck, dualDrone);
    }

    /** Compat overload. */
    public void runThreeStagePricing(double[] pi, double dualTruck, double dualDrone) {
        runThreeStagePricing(pi, dualTruck, dualDrone,
                             BigInteger.ZERO, new HashSet<>(),
                             Collections.emptyList(), false, true);
    }

    // ------------------------------------------------------------------
    // RCSP
    // ------------------------------------------------------------------
    private void solveRCSP(double dualTruck, double dualDrone,
                           PricingStage stage, int columnBudget) {
        int targetTotal = newRoutes.size() + columnBudget;
        for (int d = 0; d < NUM_D; d++)
            for (int v = 0; v < NUM_NODES; v++)
                for (int i = 0; i < NUM_BUCKET; i++)
                    for (int j = 0; j < NUM_BUCKET; j++) {
                        buckets[d][v][i][j].clear();
                        dominatingLabels[d][v][i][j] = null;
                    }

        int sizeV = Constant.TOTAL_CUSTOMER + 1;
        outer:
        for (int d = 0; d < NUM_D; d++) {
            RCSPGraph graph = graphs.get(d);
            Deque<Label> queue = new ArrayDeque<>();

            Label src = new Label(sizeV);
            src.ngSet = BigInteger.ZERO;
            src.d = d;
            src.droneUsed = 0;
            src.r1cState = new double[activeR1Cuts.size()];
            if (addLabelToBucket(src, d, stage)) queue.add(src);

            while (!queue.isEmpty()) {
                Label label = queue.poll();
                if (label.node == 0) {
                    if (satisfiesRequiredArc(label) && label.reducedCost < 0) {
                        Route route = reconstructRoute(label);
                        route.reducedCost = label.reducedCost - dualTruck
                                          - label.droneUsed * dualDrone;
                        String sig = route.getSignature();
                        if (route.reducedCost >= Constant.ROUTE_REDUCED_COST_THRESHOLD
                                || signatures.contains(sig)
                                || forbiddenSigs.contains(sig)) continue;
                        newRoutes.add(route);
                        signatures.add(sig);
                        if (newRoutes.size() >= targetTotal) break outer;
                    }
                    continue;
                }
                for (RCSPArc arc : graph.adjacencyList.get(label.node)) {
                    if (isArcBanned(arc)) continue;
                    Label nl = extendLabel(label, arc, d);
                    if (nl == null) continue;
                    if (addLabelToBucket(nl, d, stage)) queue.add(nl);
                }
            }
        }
    }

    private boolean satisfiesRequiredArc(Label label) {
        return requiredArc == -1 || label.requiredArcUsed;
    }

    private boolean isArcBanned(RCSPArc arc) {
        return !bannedArcs.isEmpty() && bannedArcs.contains(arcKey(arc.src, arc.dst));
    }

    private static long arcKey(int src, int dst) {
        return ((long) src << 32) | (dst & 0xFFFFFFFFL);
    }

    private Label extendLabel(Label label, RCSPArc arc, int d) {
        int dst = arc.dst;
        int origDst = dst % (Constant.TOTAL_CUSTOMER + 1);
        Node dstNode = VRPInstance.nodes.get(origDst);

        for (int cust : arc.servedCustomersInOrder)
            if (blockedCustomers.testBit(cust)) return null;

        boolean newReq = label.requiredArcUsed;
        if (requiredArc != -1 && !newReq) {
            int srcOrig = arc.src % (Constant.TOTAL_CUSTOMER + 1);
            int dstOrig = arc.dst;
            if (srcOrig == (int)(requiredArc >>> 32)
                    && dstOrig == (int)(requiredArc & 0xFFFFFFFFL))
                newReq = true;
        }

        BigInteger newNgSet = label.ngSet;
        for (int cust : arc.servedCustomersInOrder) {
            if (newNgSet.testBit(cust)) return null;
            newNgSet = newNgSet.and(ngNeighborhood[cust])
                               .or(BigInteger.ONE.shiftLeft(cust));
        }

        BigInteger newServed = label.customerServed;
        for (int cust : arc.servedCustomersInOrder) {
            if (newServed.testBit(cust)) return null;
            newServed = newServed.setBit(cust);
        }

        int sizeV = Constant.TOTAL_CUSTOMER + 1;
        double arrival = label.duration + arc.duration;
        double newDuration;
        if (dst < sizeV) {
            double startService = Math.max(arrival, dstNode.tw_a);
            if (startService > dstNode.tw_b + Constant.EPSILON) return null;
            newDuration = startService;
        } else newDuration = arrival;

        double newCapacity = label.capacity + arc.capacity;
        if (newCapacity > Constant.TRUCK_PAYLOAD + Constant.EPSILON) return null;

        int newDroneUsed = label.droneUsed;
        if (arc.schedule != null && !arc.servedCustomersInOrder.isEmpty())
            newDroneUsed = Math.max(newDroneUsed, d);

        double[] newState = label.r1cState.clone();
        double extraPenalty = 0.0;
        if (arc.schedule == null) {
            for (int c = 0; c < activeR1Cuts.size(); c++)
                extraPenalty += activeR1Cuts.get(c).visitNode(arc.dst, newState, c);
        } else {
            for (int c = 0; c < activeR1Cuts.size(); c++) {
                R1Cut cut = activeR1Cuts.get(c);
                for (int cust : arc.servedCustomersInOrder)
                    extraPenalty += cut.visitNode(cust, newState, c);
            }
        }

        Label nl = new Label(arc.dst);
        nl.arc = arc;
        nl.duration = newDuration;
        nl.capacity = newCapacity;
        nl.reducedCost = label.reducedCost + arc.reducedCost + extraPenalty;
        nl.predecessor = label;
        nl.ngSet = newNgSet;
        nl.customerServed = newServed;
        nl.requiredArcUsed = newReq;
        nl.d = d;
        nl.droneUsed = newDroneUsed;
        nl.r1cState = newState;
        return nl;
    }

    // ---- bucket management ----

    private boolean addLabelToBucket(Label newLabel, int d, PricingStage stage) {
        int v = newLabel.node;
        int Bc = (int) Math.floor(newLabel.capacity / capaStep);
        int Bd = (int) Math.floor((newLabel.duration - timeOrigin) / timeStep);
        if (Bd < 0) Bd = 0;
        if (Bc < 0) Bc = 0;
        if (Bd >= NUM_BUCKET || Bc >= NUM_BUCKET) return false;

        List<Label> bucket = buckets[d][v][Bd][Bc];
        switch (stage) {
            case LIGHT:     return addLabelLight(newLabel, bucket);
            case HEURISTIC: return addLabelWithDominance(newLabel, d, v, Bd, Bc, bucket, false);
            case EXACT:     return addLabelWithDominance(newLabel, d, v, Bd, Bc, bucket, true);
        }
        return false;
    }

    private boolean addLabelLight(Label newLabel, List<Label> bucket) {
        // bucket is a List; first half tracks "with truck", second half "drones only"
        boolean usesTruck = newLabel.customerServed.and(
                VRPInstance.truckOnlyMask).signum() != 0;

        Label best = null;
        for (Label l : bucket) {
            if (((l.customerServed.and(VRPInstance.truckOnlyMask).signum() != 0)
                    == usesTruck) && (best == null || l.reducedCost < best.reducedCost))
                best = l;
        }
        if (best == null) { bucket.add(newLabel); return true; }
        if (newLabel.reducedCost < best.reducedCost - Constant.EPSILON) {
            bucket.remove(best);
            bucket.add(newLabel);
            return true;
        }
        return false;
    }

    private boolean addLabelWithDominance(Label newLabel, int d, int v, int Bd, int Bc,
                                          List<Label> bucket, boolean checkNgAndCuts) {
        boolean removedBest = false;
        Label curBest = dominatingLabels[d][v][Bd][Bc];
        Iterator<Label> it = bucket.iterator();
        while (it.hasNext()) {
            Label label = it.next();
            if (isDominates(label, newLabel, checkNgAndCuts)) return false;
            if (isDominates(newLabel, label, checkNgAndCuts)) {
                if (label == curBest) removedBest = true;
                it.remove();
            }
        }
        if (removedBest) dominatingLabels[d][v][Bd][Bc] = null;

        for (int i = 0; i <= Bd; i++) {
            for (int j = 0; j <= Bc; j++) {
                if (i == Bd && j == Bc) continue;
                Label best = dominatingLabels[d][v][i][j];
                if (best != null && best.reducedCost > newLabel.reducedCost + Constant.EPSILON)
                    continue;
                for (Label label : buckets[d][v][i][j])
                    if (isDominates(label, newLabel, checkNgAndCuts)) return false;
            }
        }

        bucket.add(newLabel);
        Label best = dominatingLabels[d][v][Bd][Bc];
        if (best == null || isDominates(newLabel, best, checkNgAndCuts))
            dominatingLabels[d][v][Bd][Bc] = newLabel;
        return true;
    }

    private boolean isDominates(Label a, Label b, boolean checkNgAndCuts) {
        if (a == null || b == null || a.node != b.node) return false;
        if (a.duration > b.duration + Constant.EPSILON) return false;
        if (a.capacity > b.capacity + Constant.EPSILON) return false;
        if (a.droneUsed > b.droneUsed) return false;
        if (a.requiredArcUsed != b.requiredArcUsed) return false;
        if (checkNgAndCuts) {
            if (!b.customerServed.and(a.customerServed).equals(a.customerServed)) return false;
            if (!b.ngSet.and(a.ngSet).equals(a.ngSet)) return false;
            for (int c = 0; c < a.r1cState.length; c++)
                if (a.r1cState[c] > b.r1cState[c] + Constant.EPSILON) return false;
        }
        return a.reducedCost <= b.reducedCost + Constant.EPSILON;
    }

    // ---- graph building ----

    private void buildGraphs(double[] pi) {
        graphs = new ArrayList<>();
        for (int d = 0; d < NUM_D; d++) graphs.add(buildGraph(d, pi));
    }

    private RCSPGraph buildGraph(int d, double[] pi) {
        RCSPGraph graph = new RCSPGraph();
        int sizeV = Constant.TOTAL_CUSTOMER + 1;
        for (int i = 0; i < NUM_NODES; i++) graph.adjacencyList.add(new ArrayList<>());

        for (int i = 0; i < sizeV; i++) {
            int from = sizeV + i;
            for (int j = 0; j < sizeV; j++) {
                if (i == j) continue;
                if (bannedArcs.contains(arcKey(from, j))) continue;
                double driving = VRPInstance.distMatrix[i][j] / Constant.TRUCK_SPEED;
                double rc = driving;
                double cap = 0.0;
                BigInteger ngSet = BigInteger.ZERO;
                List<Integer> served = new ArrayList<>();
                if (j != 0) {
                    rc -= pi[j - 1];
                    rc += cuttingPlanes.getReducedCostPenaltyForTruckArc(i, j, d);
                    cap = VRPInstance.nodes.get(j).demand;
                    ngSet = BigInteger.ONE.shiftLeft(j);
                    served.add(j);
                }
                graph.adjacencyList.get(from).add(new RCSPArc(from, j,
                        driving, cap, rc, ngSet, served, null));
            }
        }

        for (int i = 1; i <= Constant.TOTAL_CUSTOMER; i++) {
            int to = i + sizeV;
            double servingTime = VRPInstance.nodes.get(i).servingTime;
            graph.adjacencyList.get(i).add(new RCSPArc(i, to, servingTime, 0.0,
                    servingTime, BigInteger.ZERO, new ArrayList<>(), new DroneSchedule(i)));

            if (d == 0) continue;
            if (i >= DroneScheduleEnumeration.paretoMap.size()) continue;
            if (d >= DroneScheduleEnumeration.paretoMap.get(i).size()) continue;

            var schedules = DroneScheduleEnumeration.paretoMap.get(i).get(d).entrySet();
            double baseCapacity = d * Constant.DRONE_AND_EQUIPMENT_WEIGHT;
            for (var entry : schedules) {
                for (DroneSchedule s : entry.getValue().nonDominatedSchedules) {
                    double dur = Math.max(servingTime, s.makespan);
                    double cap = baseCapacity;
                    BigInteger ngSet = BigInteger.ZERO;
                    double rc = dur;
                    for (int cust : s.customerServed) {
                        rc -= pi[cust - 1];
                        cap += VRPInstance.nodes.get(cust).demand;
                        ngSet = ngSet.setBit(cust);
                    }
                    rc += cuttingPlanes.getReducedCostPenaltyForDroneArc(i, s, d);
                    s.reducedCost = rc;
                    graph.adjacencyList.get(i).add(new RCSPArc(i, to, dur,
                            cap, rc, ngSet, new ArrayList<>(s.customerServed), s));
                }
            }
        }
        return graph;
    }

    private Route reconstructRoute(Label sinkLabel) {
        List<Integer> sequence = new ArrayList<>();
        Map<Integer, DroneSchedule> droneMap = new HashMap<>();
        Label cur = sinkLabel;
        while (cur.predecessor != null) {
            RCSPArc arc = cur.arc;
            if (arc != null) {
                if (arc.schedule != null && !arc.servedCustomersInOrder.isEmpty())
                    droneMap.put(arc.src, arc.schedule);
                else if (arc.schedule == null && arc.dst != 0)
                    sequence.add(0, arc.dst);
            }
            cur = cur.predecessor;
        }
        List<Node> routeSeq = new ArrayList<>();
        for (int c : sequence) routeSeq.add(VRPInstance.nodes.get(c));
        Route route = new Route(routeSeq, droneMap);
        route.reducedCost = sinkLabel.reducedCost;
        return route;
    }

    // ---- enumeration ----

    private void tryRouteEnumeration(double[] pi, double dualTruck, double dualDrone) {
        if (enumerationComplete) return;
        List<List<Route>> byD = new ArrayList<>();
        int total = 0;
        int sizeV = Constant.TOTAL_CUSTOMER + 1;
        long tStart = System.currentTimeMillis();

        for (int d = 0; d < NUM_D; d++) {
            if (System.currentTimeMillis() - tStart > 5000) {
                System.out.println("    [ENUM] time limit hit; aborting enumeration.");
                return;
            }
            List<Route> routesD = new ArrayList<>();
            RCSPGraph graph = graphs.get(d);
            Deque<Label> queue = new ArrayDeque<>();
            for (int v = 0; v < NUM_NODES; v++)
                for (int i = 0; i < NUM_BUCKET; i++)
                    for (int j = 0; j < NUM_BUCKET; j++) {
                        buckets[d][v][i][j].clear();
                        dominatingLabels[d][v][i][j] = null;
                    }
            Label src = new Label(sizeV);
            src.ngSet = BigInteger.ZERO;
            src.d = d;
            src.droneUsed = 0;
            src.r1cState = new double[activeR1Cuts.size()];
            if (addLabelToBucket(src, d, PricingStage.EXACT)) queue.add(src);

            while (!queue.isEmpty()) {
                Label label = queue.poll();
                if (label.node == 0) {
                    if (satisfiesRequiredArc(label) && label.reducedCost < -Constant.EPSILON) {
                        Route route = reconstructRoute(label);
                        route.reducedCost = label.reducedCost - dualTruck
                                          - label.droneUsed * dualDrone;
                        if (route.reducedCost < -Constant.EPSILON) {
                            routesD.add(route);
                            total++;
                            if (total > Constant.ROUTE_ENUM_HARD_CAP) {
                                enumerationComplete = false;
                                return;
                            }
                        }
                    }
                    continue;
                }
                for (RCSPArc arc : graph.adjacencyList.get(label.node)) {
                    if (isArcBanned(arc)) continue;
                    Label nl = extendLabel(label, arc, d);
                    if (nl == null) continue;
                    if (addLabelToBucket(nl, d, PricingStage.EXACT)) queue.add(nl);
                }
            }
            byD.add(routesD);
        }

        if (total <= Constant.ROUTE_ENUM_THRESHOLD) {
            enumeratedByD = byD;
            enumeratedTotalCount = total;
            enumerationComplete = true;
            System.out.println("    [ENUM] " + total + " routes (<= "
                             + Constant.ROUTE_ENUM_THRESHOLD + ") — pricing by inspection.");
        } else {
            enumeratedByD = null;
            enumeratedTotalCount = 0;
            enumerationComplete = false;
            System.out.println("    [ENUM] aborted at " + total);
        }
    }

    private void inspectEnumeratedRoutes(double[] pi, double dualTruck, double dualDrone) {
        for (List<Route> routesD : enumeratedByD)
            for (Route r : routesD) {
                double rc = computeReducedCost(r, pi, dualTruck, dualDrone);
                if (rc < -Constant.EPSILON) { r.reducedCost = rc; newRoutes.add(r); }
            }
    }

    private double computeReducedCost(Route r, double[] pi, double dT, double dD) {
        double rc = r.totalTime;
        for (int c : r.customerServed) rc -= pi[c - 1];
        rc -= dT;
        rc -= r.getNumDrone() * dD;
        return rc;
    }

    public boolean isEnumerationComplete()    { return enumerationComplete; }
    public int     getEnumeratedTotalCount()  { return enumeratedTotalCount; }
    public List<List<Route>> getEnumeratedByD() { return enumeratedByD; }
    public List<Route> getNewRoutes()         { return newRoutes; }
}