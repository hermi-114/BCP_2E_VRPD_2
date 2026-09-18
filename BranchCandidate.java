import java.util.Objects;

public class BranchCandidate {

    public enum Type { TOTAL_TRUCKS, TOTAL_DRONES, TRUCKS_WITH_D, TRUCK_ARC }

    public final Type type;
    public final int  parameter;   // d for TRUCKS_WITH_D
    public final int  arcI, arcJ;  // for TRUCK_ARC

    public BranchCandidate(Type type) {
        this(type, -1, -1, -1);
    }
    public BranchCandidate(Type type, int parameter) {
        this(type, parameter, -1, -1);
    }
    public BranchCandidate(Type type, int arcI, int arcJ) {
        this(type, -1, arcI, arcJ);
    }
    private BranchCandidate(Type type, int parameter, int arcI, int arcJ) {
        this.type = type;
        this.parameter = parameter;
        this.arcI = arcI;
        this.arcJ = arcJ;
    }

    /** Coefficient of a route in the aggregate variable this candidate represents. */
    public double coefficient(Route r) {
        switch (type) {
            case TOTAL_TRUCKS:  return r.coefficientTotalTrucks();
            case TOTAL_DRONES:  return r.coefficientTotalDrones();
            case TRUCKS_WITH_D: return r.coefficientTrucksWithD(parameter);
            case TRUCK_ARC:     return r.coefficientTruckArc(arcI, arcJ);
        }
        return 0.0;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (!(o instanceof BranchCandidate)) return false;
        BranchCandidate c = (BranchCandidate) o;
        return type == c.type && parameter == c.parameter
            && arcI == c.arcI && arcJ == c.arcJ;
    }

    @Override
    public int hashCode() {
        return Objects.hash(type, parameter, arcI, arcJ);
    }

    @Override
    public String toString() {
        switch (type) {
            case TOTAL_TRUCKS:  return "Trucks";
            case TOTAL_DRONES:  return "Drones";
            case TRUCKS_WITH_D: return "TrucksWithD(" + parameter + ")";
            case TRUCK_ARC:     return "Arc(" + arcI + "," + arcJ + ")";
        }
        return "?";
    }
}