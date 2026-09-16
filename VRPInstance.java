import java.math.BigInteger;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.PriorityQueue;


public class VRPInstance {
    
    public static List<Node> nodes = new ArrayList<>();
    public static double[][] distMatrix;
    public static List<Route> routePool = new ArrayList<>();

    public static BigInteger[] ngNeighborhood;

    public static void calculateDistance() {
        distMatrix = new double[Constant.TOTAL_CUSTOMER + 1][Constant.TOTAL_CUSTOMER + 1];

        for(int i = 0; i < distMatrix.length; i++) {
            for(int j = 0; j < distMatrix.length; j++) {
                if(i == j) distMatrix[i][j] = 1e8;

                Node src = nodes.get(i);
                Node dst = nodes.get(j);

                double x = src.x - dst.x;
                double y = src.y - dst.y;

                double dist = Math.sqrt(x*x + y*y);
                distMatrix[i][j] = dist;
            }
        }
    }

    public static void initNgNeighborhoods(int eta) {
        int n = Constant.TOTAL_CUSTOMER;
        ngNeighborhood = new BigInteger[n + 1];

        for (int v = 0; v <= n; v++) {
            // Depot: only itself.
            if (v == 0) {
                ngNeighborhood[v] = BigInteger.ZERO;
                continue;
            }

            // Find the eta closest customers (by distance).
            // Fallback: all customers.
            double[][] dist = VRPInstance.distMatrix;
            final int s = v;
            PriorityQueue<Integer> queue = new PriorityQueue<>(Comparator.comparingDouble(c -> -dist[s][c]));
            for(int i = 1; i <= n; i++) {
                if(i == v) continue;
                queue.offer(i);
                if(queue.size() > eta - 1) queue.poll(); // has not contained node itself yet
            }

            BigInteger nb = BigInteger.ZERO.setBit(v);
            while(!queue.isEmpty()) {
                int u = queue.poll();
                nb = nb.setBit(u);   // replace with closest-η selection if desired
            }
            ngNeighborhood[v] = nb;
        }
    }
}
