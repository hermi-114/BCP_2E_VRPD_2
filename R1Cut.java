import java.util.*;

public class R1Cut implements ICut {
    public final Set<Integer> subsetC;
    public final Set<Integer> memorySet;
    public final int rhs;
    public final Map<Integer, Double> rhoMap;
    public double dual = 0.0;
    public double violation = 0.0;

    public R1Cut(Set<Integer> C) {
        this(C, new HashSet<>(C));
    }

    public R1Cut(Set<Integer> C, Set<Integer> M) {
        this.subsetC  = new HashSet<>(C);
        this.memorySet = new HashSet<>(M);
        this.rhoMap   = new HashMap<>();
        for (int cust : C) rhoMap.put(cust, 0.5);

        double sumRho = C.size() * 0.5;
        this.rhs = (int) Math.floor(sumRho + Constant.EPSILON);
    }

    @Override public double getDual()              { return dual; }
    @Override public void   setDual(double d)      { this.dual = d; }
    @Override public double getRHS()               { return rhs; }
    @Override public Set<Integer> getSubsetC()     { return subsetC; }
    @Override public double getViolation()         { return violation; }
    @Override public void   setViolation(double v) { this.violation = v; }

    @Override
    public double getCoefficientForRoute(Route route) {
        double s = 0.0, alpha = 0.0;
        for (Node n : route.sequence) {
            int id = n.id;
            if (id == -1) continue;
            if (!memorySet.contains(id)) s = 0.0;
            Double rho = rhoMap.get(id);
            if (rho != null) s += rho;
            while (s >= 1.0 - Constant.EPSILON) { s -= 1.0; alpha += 1.0; }
        }
        return alpha;
    }

    public double visitNode(int node, double[] sHolder, int cutIndex) {
        double s = sHolder[cutIndex];
        if (!memorySet.contains(node)) s = 0.0;
        Double rho = rhoMap.get(node);
        if (rho != null) s += rho;
        double penalty = 0.0;
        while (s >= 1.0 - Constant.EPSILON) {
            s -= 1.0;
            penalty -= dual;
        }
        sHolder[cutIndex] = s;
        return penalty;
    }

    @Override
    public double getReducedCostPenaltyForTruckArc(int src, int dst, int numDrones) {
        return 0.0;
    }

    @Override
    public double getReducedCostPenaltyForDroneArc(int park, DroneSchedule schedule, int numDrones) {
        return 0.0;
    }
}