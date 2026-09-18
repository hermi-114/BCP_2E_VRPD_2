import com.gurobi.gurobi.GRBException;
import java.util.*;

public class BranchAndPrice {

    private List<Route> bestSolution = null;
    private double      minTime      = Double.MAX_VALUE;
    private final ColumnGeneration cg = new ColumnGeneration();

    private int nodesExplored = 0;
    private int branchesMade  = 0;

    private final Map<BranchCandidate, Double> history = new HashMap<>();

    private static final int MAX_NODES = 200;

    public void run() throws GRBException {
        System.out.println("============ Branch and Price (multi-phase strong branching) ============");

        // ============================================================
        //  ROOT
        // ============================================================
        BCPNode root = new BCPNode();
        ColumnGeneration.NodeResult rootLp = cg.solveForNode(root);
        nodesExplored++;

        System.out.printf("--- Root LP: obj=%.4f  covered=%b  optimal=%b%n",
                rootLp.objective, rootLp.allCovered, rootLp.lpOptimal);

        // (a) root is already integer & covered → done
        if (rootLp.lpOptimal && rootLp.allCovered && isIntegral(rootLp.lambda)) {
            minTime      = rootLp.objective;
            bestSolution = extractSolution(rootLp);
            System.out.println("Root integral — optimal: " + minTime);
            return;
        }

        // (b) root is infeasible
        if (!rootLp.lpOptimal) {
            System.out.println("Root LP infeasible — aborting.");
            return;
        }

        // (c) root uses artificials → the model is infeasible at this configuration
        if (!rootLp.allCovered) {
            System.out.println("Root LP uses artificials — no feasible solution with current parameters.");
            return;
        }

        // (d) root is fractional & fully covered → try to get a MIP incumbent
        try {
            ColumnGeneration.NodeResult mip = cg.solveForNodeAsMip(root, rootLp.columns);
            if (mip.lpOptimal && mip.allCovered) {
                minTime      = mip.objective;
                bestSolution = extractSolution(mip);
                System.out.println("Initial incumbent from root MIP: " + minTime);
            } else {
                System.out.println("Root MIP did not find a feasible incumbent; continuing without one.");
            }
        } catch (GRBException e) {
            System.out.println("Root MIP failed: " + e.getMessage());
        }

        // ============================================================
        //  B&P LOOP
        // ============================================================
        Deque<BCPNode> stack = new ArrayDeque<>();

        // branch on the root manually so we don't re-solve it
        BranchDecision rootBranch = pickBranchByStrongBranching(rootLp, root);
        if (rootBranch == null) {
            System.out.println("Cannot branch at root — aborting.");
            return;
        }
        System.out.println("Root branch: " + rootBranch);

        BCPNode childDown = root.copy();
        childDown.decisions.add(new BranchDecision(rootBranch.candidate, true,  Math.floor(rootBranch.rhs)));
        childDown.depth = 1;

        BCPNode childUp = root.copy();
        childUp.decisions.add(new BranchDecision(rootBranch.candidate, false, Math.ceil(rootBranch.rhs)));
        childUp.depth = 1;

        // LIFO stack → push UP first so DOWN is popped first.
        // DOWN (≤ floor) is usually more likely feasible → find incumbents sooner.
        stack.push(childUp);
        stack.push(childDown);
        branchesMade++;

        while (!stack.isEmpty() && nodesExplored < MAX_NODES) {
            BCPNode node = stack.pop();
            nodesExplored++;

            long t0 = System.currentTimeMillis();
            ColumnGeneration.NodeResult lp = cg.solveForNode(node);
            long t1 = System.currentTimeMillis();

            System.out.println("--- Node #" + nodesExplored
                    + " depth=" + node.depth
                    + " dec=" + node.decisions.size()
                    + "  time=" + (t1 - t0) + "ms"
                    + "  obj=" + String.format("%.4f", lp.objective));

            if (!lp.lpOptimal) {
                System.out.println("    prune: LP infeasible");
                continue;
            }
            if (lp.objective >= minTime - Constant.EPSILON) {
                System.out.println("    prune: bound (" + lp.objective + " >= " + minTime + ")");
                continue;
            }
            if (!lp.allCovered) {
                System.out.println("    prune: artificials in use");
                continue;
            }
            if (isIntegral(lp.lambda)) {
                if (lp.objective < minTime) {
                    minTime      = lp.objective;
                    bestSolution = extractSolution(lp);
                    System.out.println("    new incumbent: " + minTime);
                }
                continue;
            }

            // ---------- branch ----------
            BranchDecision bd = pickBranchByStrongBranching(lp, node);
            if (bd == null) {
                System.out.println("    no candidate — skipping node");
                continue;
            }
            System.out.println("    strong branching picked: " + bd);

            BCPNode cd = node.copy();
            cd.decisions.add(new BranchDecision(bd.candidate, true,  Math.floor(bd.rhs)));
            cd.depth = node.depth + 1;

            BCPNode cu = node.copy();
            cu.decisions.add(new BranchDecision(bd.candidate, false, Math.ceil(bd.rhs)));
            cu.depth = node.depth + 1;

            stack.push(cu);
            stack.push(cd);
            branchesMade++;
        }

        // ============================================================
        //  FINAL MIP FALLBACK
        // ============================================================
        if (bestSolution == null) {
            System.out.println("No incumbent found in tree — attempting final MIP fallback.");
            try {
                // Retry MIP on the root's full column pool
                ColumnGeneration.NodeResult mip = cg.solveForNodeAsMip(root, rootLp.columns);
                if (mip.lpOptimal && mip.allCovered) {
                    minTime      = mip.objective;
                    bestSolution = extractSolution(mip);
                    System.out.println("Fallback MIP incumbent: " + minTime);
                }
            } catch (GRBException e) {
                System.out.println("Fallback MIP failed: " + e.getMessage());
            }
        }

        System.out.println("=================== Done ===================");
        System.out.println("Nodes explored: " + nodesExplored);
        System.out.println("Branches made:  " + branchesMade);
        System.out.println("Best objective: " + minTime);
    }

    // ------------------------------------------------------------------
    //  Three-phase strong branching  (unchanged)
    // ------------------------------------------------------------------
    private BranchDecision pickBranchByStrongBranching(ColumnGeneration.NodeResult lp,
                                                       BCPNode node) throws GRBException {
        List<BranchCandidate> all = collectCandidates(lp);

        List<BranchCandidate> phase1 = selectPhase1(all, lp);
        if (phase1.isEmpty()) return null;
        System.out.println("    [SB] phase 1 selected " + phase1.size() + " / " + all.size());

        List<ScoredCandidate> phase2 = evaluatePhase2(phase1, lp, node);
        phase2.sort((a, b) -> Double.compare(b.score, a.score));
        int keep2 = Math.min(Constant.STRONG_BRANCHING_PHASE2_KEEP, phase2.size());
        List<ScoredCandidate> phase2Top = phase2.subList(0, keep2);
        System.out.println("    [SB] phase 2 kept " + keep2);

        List<ScoredCandidate> phase3 = evaluatePhase3(phase2Top, lp, node);
        phase3.removeIf(s -> s.score < 0);
        if (phase3.isEmpty()) return null;
        phase3.sort((a, b) -> Double.compare(b.score, a.score));
        ScoredCandidate best = phase3.get(0);
        System.out.println("    [SB] phase 3 winner: " + best.candidate
                + "  score=" + String.format("%.4f", best.score));

        history.merge(best.candidate, best.score,
                      (a, b) -> a * Constant.STRONG_BRANCHING_HISTORY_DECAY + b);

        double v = currentValue(best.candidate, lp);
        return new BranchDecision(best.candidate, true, v);
    }

    private List<BranchCandidate> collectCandidates(ColumnGeneration.NodeResult lp) {
        List<BranchCandidate> list = new ArrayList<>();
        list.add(new BranchCandidate(BranchCandidate.Type.TOTAL_TRUCKS));
        list.add(new BranchCandidate(BranchCandidate.Type.TOTAL_DRONES));
        for (int d = 0; d <= Constant.MAX_DRONE_PER_VEHICLE; d++)
            list.add(new BranchCandidate(BranchCandidate.Type.TRUCKS_WITH_D, d));

        Set<Long> seenArcs = new HashSet<>();
        for (int r = 0; r < lp.columns.size(); r++) {
            if (lp.lambda[r] < Constant.EPSILON) continue;
            List<Node> seq = lp.columns.get(r).sequence;
            for (int k = 1; k < seq.size(); k++) {
                int i = seq.get(k - 1).id;
                int j = seq.get(k).id;
                if (i == 0 && j == 0) continue;
                long key = ((long) i << 32) | (j & 0xFFFFFFFFL);
                if (seenArcs.add(key)) {
                    list.add(new BranchCandidate(BranchCandidate.Type.TRUCK_ARC, i, j));
                    if (list.size() > Constant.MAX_ARC_CANDIDATES_PER_NODE) break;
                }
            }
            if (list.size() > Constant.MAX_ARC_CANDIDATES_PER_NODE) break;
        }
        return list;
    }

    private List<BranchCandidate> selectPhase1(List<BranchCandidate> all,
                                               ColumnGeneration.NodeResult lp) {
        List<ScoredCandidate> scored = new ArrayList<>();
        for (BranchCandidate c : all) {
            double v = currentValue(c, lp);
            if (v < Constant.EPSILON) continue;
            double frac = Math.abs(v - Math.round(v));
            if (frac < 1e-4) continue;
            double s = frac;
            Double hist = history.get(c);
            if (hist != null) s = Math.max(s, hist);
            scored.add(new ScoredCandidate(c, s, v));
        }
        scored.sort((a, b) -> Double.compare(b.score, a.score));

        int keep = Constant.STRONG_BRANCHING_PHASE1_KEEP;
        Set<BranchCandidate> chosen = new LinkedHashSet<>();

        int half = keep / 2;
        for (int i = 0; i < Math.min(half, scored.size()); i++)
            chosen.add(scored.get(i).candidate);

        Map<BranchCandidate.Type, Integer> typeCount = new EnumMap<>(BranchCandidate.Type.class);
        for (ScoredCandidate s : scored) {
            if (chosen.size() >= keep) break;
            if (chosen.contains(s.candidate)) continue;
            int cnt = typeCount.getOrDefault(s.candidate.type, 0);
            if (cnt >= keep / 4 + 1) continue;
            chosen.add(s.candidate);
            typeCount.merge(s.candidate.type, 1, Integer::sum);
        }
        for (ScoredCandidate s : scored) {
            if (chosen.size() >= keep) break;
            chosen.add(s.candidate);
        }
        return new ArrayList<>(chosen);
    }

    private List<ScoredCandidate> evaluatePhase2(List<BranchCandidate> cands,
                                                 ColumnGeneration.NodeResult lp,
                                                 BCPNode node) throws GRBException {
        List<ScoredCandidate> out = new ArrayList<>();
        double z = lp.objective;
        List<Route> parentColumns = lp.columns;
        for (BranchCandidate c : cands) {
            double v = currentValue(c, lp);
            double zDown = evaluateBranchSide(parentColumns, node, c, true,  Math.floor(v));
            double zUp   = evaluateBranchSide(parentColumns, node, c, false, Math.ceil(v));
            out.add(new ScoredCandidate(c, productRule(z, zDown, zUp), v, zDown, zUp));
        }
        return out;
    }

    private List<ScoredCandidate> evaluatePhase3(List<ScoredCandidate> cands,
                                                 ColumnGeneration.NodeResult lp,
                                                 BCPNode node) throws GRBException {
        List<ScoredCandidate> out = new ArrayList<>();
        double z = lp.objective;
        List<Route> parentColumns = lp.columns;
        for (ScoredCandidate s : cands) {
            double v = s.value;
            double zDown = evaluateBranchSide(parentColumns, node, s.candidate, true,  Math.floor(v));
            double zUp   = evaluateBranchSide(parentColumns, node, s.candidate, false, Math.ceil(v));
            out.add(new ScoredCandidate(s.candidate, productRule(z, zDown, zUp), v, zDown, zUp));
        }
        return out;
    }

    private double evaluateBranchSide(List<Route> parentColumns,
                                      BCPNode parent,
                                      BranchCandidate cand,
                                      boolean upperBound,
                                      double rhs) throws GRBException {
        BCPNode temp = parent.copy();
        temp.decisions.add(new BranchDecision(cand, upperBound, rhs));
        ColumnGeneration.NodeResult r = cg.solveForNodeWithoutCG(temp, parentColumns);
        return r.lpOptimal ? r.objective : Double.POSITIVE_INFINITY;
    }

    private double productRule(double z, double zDown, double zUp) {
        boolean dInf = Double.isInfinite(zDown);
        boolean uInf = Double.isInfinite(zUp);
        if (dInf && uInf) return -1.0;
        double iDown = dInf ? Constant.STRONG_BRANCHING_INFEAS_SCORE
                            : Math.max(zDown - z, Constant.EPSILON);
        double iUp   = uInf ? Constant.STRONG_BRANCHING_INFEAS_SCORE
                            : Math.max(zUp - z, Constant.EPSILON);
        return iDown * iUp;
    }

    private double currentValue(BranchCandidate c, ColumnGeneration.NodeResult lp) {
        double sum = 0.0;
        for (int r = 0; r < lp.columns.size(); r++)
            sum += lp.lambda[r] * c.coefficient(lp.columns.get(r));
        return sum;
    }

    private boolean isIntegral(double[] lambda) {
        for (double v : lambda)
            if (Math.abs(v - Math.round(v)) > 1e-5) return false;
        return true;
    }

    private List<Route> extractSolution(ColumnGeneration.NodeResult lp) {
        List<Route> sol = new ArrayList<>();
        for (int i = 0; i < lp.lambda.length; i++)
            if (lp.lambda[i] > 1 - Constant.EPSILON)
                sol.add(lp.columns.get(i));
        return sol;
    }

    public List<Route> getBestSolution() { return bestSolution; }
    public double      getBestObjective() { return minTime; }

    private static class ScoredCandidate {
        final BranchCandidate candidate;
        final double score, value, zDown, zUp;
        ScoredCandidate(BranchCandidate c, double s, double v) {
            this(c, s, v, Double.NaN, Double.NaN);
        }
        ScoredCandidate(BranchCandidate c, double s, double v, double zd, double zu) {
            candidate = c; score = s; value = v; zDown = zd; zUp = zu;
        }
    }
}