import java.math.BigInteger;

public class BBNode {
    public BigInteger selectedRoutes;
    public int totalTruck;
    public double totalTime;
    public BigInteger customerServed;

    public BBNode(BigInteger selectedRoutes, double totalTime, int totalTruck) {
        this.selectedRoutes = selectedRoutes;
        this.totalTime = totalTime;
        this.totalTruck = totalTruck;
    }

    
    
}
