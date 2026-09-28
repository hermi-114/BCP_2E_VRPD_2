import com.gurobi.gurobi.*;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStream;
import java.io.PrintStream;
import java.io.PrintWriter;
import java.math.BigInteger;
import java.text.SimpleDateFormat;
import java.util.*;
import java.util.logging.ConsoleHandler;
import java.util.logging.Handler;
import java.util.logging.Logger;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;

public class Main {
    static boolean test = true;
    static PrintStream logStream;

    /** Hard wall-clock limit for the whole programme, in milliseconds. */
    static final long TOTAL_TIME_LIMIT_MS = 10800_000L * 2; // 10800 s = 3 h
    /** Timestamp (ms) at which the whole run started. */
    static long globalStartTime;

    public static void main(String[] args) {

        globalStartTime = System.currentTimeMillis();

        // -------- Set up timestamped log file + tee to System.out AND System.err
        // --------
        PrintStream originalOut = System.out;
        PrintStream originalErr = System.err;
        try {
            String logName = new SimpleDateFormat("ddMMyy-HHmmss").format(new Date()) + ".log";
            logStream = new PrintStream(new FileOutputStream(logName), true);

            PrintStream teeOut = new PrintStream(new TeeOutputStream(originalOut, logStream), true);
            PrintStream teeErr = new PrintStream(new TeeOutputStream(originalErr, logStream), true);

            System.setOut(teeOut);
            System.setErr(teeErr);

            // Rebind any JUL ConsoleHandler (default JUL logs to original System.err).
            rebindJulHandlers(logStream);
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        }

        boolean checkRoute = true;

        String set = "C101";
        String inputFile = set + ".txt";

        String type = "C";
        String format = "^" + type + "1\\d{2}\\.txt$";

        boolean runByType = true;

        boolean isRunAll = true;
        String outputFile;
        String timestamp = LocalDateTime.now().format(DateTimeFormatter.ofPattern("yyMMdd-HHmmss"));
        if (isRunAll) {
            if (runByType)
                outputFile = "./output/output_" + type + "_" + Config.SIZE_CUSTOMER_DATASET + "_aut" + timestamp + ".csv";
            else
                outputFile = "./output/output_" + Config.SIZE_CUSTOMER_DATASET + "_aut_" + timestamp + ".csv";
        } else {
            outputFile = "./output/output.csv";
        }

        new File("./output").mkdirs();

        try (PrintWriter out = new PrintWriter(outputFile)) {
            out.println();
            out.println(",,,SIZE CUSTOMER DATASET = " + Config.SIZE_CUSTOMER_DATASET);
            out.println();
            out.println(",,set,obj,dual_obj,gap(%),total_time(s),drone(s),bcp(s)");

            // Top-of-run banner (also goes to the log via the tee)
            System.out.println("SIZE CUSTOMER DATASET = " + Constant.TOTAL_CUSTOMER);
            System.out.println("TOTAL TIME LIMIT       = " + (TOTAL_TIME_LIMIT_MS / 1000) + " s");
            System.out.println();

            if (isRunAll) {
                if (runByType)
                    runAll(out, format);
                else
                    runAll(out);
            } else {
                runSingle(inputFile, out);
            }

            if (checkRoute)
                printRoutePool("./output/route.txt");

        } catch (Exception e) {
            e.printStackTrace();
        }

        printParetoFront("./output/drone.txt");

        double totalElapsed = (System.currentTimeMillis() - globalStartTime) / 1000.0;
        System.out.printf("%nTotal elapsed time: %.1fs%n", totalElapsed);
        if (isTimeLimitExceeded()) {
            System.out.println("TIME LIMIT REACHED (" + (TOTAL_TIME_LIMIT_MS / 1000)
                    + " s) — some datasets may not have been run.");
        }

        System.out.flush();
        System.err.flush();
        if (logStream != null)
            logStream.close();
    }

    /** True once the whole-run wall-clock budget has been exhausted. */
    static boolean isTimeLimitExceeded() {
        return (System.currentTimeMillis() - globalStartTime) >= TOTAL_TIME_LIMIT_MS;
    }

    /**
     * Rebind java.util.logging ConsoleHandlers so their output also lands in the
     * log file.
     * JUL binds its ConsoleHandler to the original System.err at initialisation
     * time,
     * so a plain System.setErr() does not suffice.
     */
    static void rebindJulHandlers(PrintStream target) {
        try {
            Logger root = Logger.getLogger("");
            // Snapshot first, to avoid ConcurrentModificationException.
            Handler[] existing = root.getHandlers().clone();
            for (Handler h : existing) {
                if (h instanceof ConsoleHandler) {
                    root.removeHandler(h);
                    root.addHandler(new TeeConsoleHandler(target));
                }
            }
        } catch (Throwable t) {
            // Never let logging setup break the run.
            System.err.println("JUL rebind skipped: " + t);
        }
    }

    /** ConsoleHandler whose OutputStream can be set from within a subclass. */
    static class TeeConsoleHandler extends ConsoleHandler {
        TeeConsoleHandler(OutputStream os) {
            super();
            setOutputStream(os); // protected — legal here
        }
    }

    public static void runAll(PrintWriter out) throws Exception {
        File folder = new File("./data/Solomon");
        File[] files = folder.listFiles();
        if (files != null) {
            Arrays.sort(files);
            for (File file : files) {
                if (!file.isFile())
                    continue;
                if (!file.getName().endsWith(".txt"))
                    continue;
                if (file.getName().equals("capacities.txt"))
                    continue;

                if (isTimeLimitExceeded()) {
                    System.out.println(">>> TIME LIMIT reached before running " + file.getName()
                            + " — skipping remaining instances.");
                    out.println(",," + file.getName() + ",SKIPPED_TIME_LIMIT,-1,-1,-1,-1,-1");
                    continue;
                }

                System.out.println("Running " + file.getName());
                try {
                    runSingle(file.getName(), out);
                } catch (Exception e) {
                    System.err.println("FAILED on " + file.getName());
                    e.printStackTrace();
                    out.println(",," + file.getName() + ",FAILED,-1,-1,-1,-1,-1");
                }
            }
        }
    }

    public static void runAll(PrintWriter out, String format) throws Exception {
        File folder = new File("./data/Solomon");
        File[] files = folder.listFiles();
        if (files != null) {
            Arrays.sort(files);
            for (File file : files) {
                if (!file.isFile())
                    continue;
                if (!file.getName().endsWith(".txt"))
                    continue;
                if (file.getName().equals("capacities.txt"))
                    continue;
                if (!file.getName().matches(format))
                    continue;

                if (isTimeLimitExceeded()) {
                    System.out.println(">>> TIME LIMIT reached before running " + file.getName()
                            + " — skipping remaining instances.");
                    out.println(",," + file.getName() + ",SKIPPED_TIME_LIMIT,-1,-1,-1,-1,-1");
                    continue;
                }

                System.out.println("Running " + file.getName());
                try {
                    runSingle(file.getName(), out);
                } catch (Exception e) {
                    System.err.println("FAILED on " + file.getName());
                    e.printStackTrace();
                    out.println(",," + file.getName() + ",FAILED,-1,-1,-1,-1,-1");
                }
            }
        }
    }

    public static void runSingle(String dataset, PrintWriter out) {
        VRPInstance.reset();
        DroneScheduleEnumeration.paretoMap.clear();

        System.out.println("Set " + dataset);
        DataLoader.loadCustomer("./data/Solomon/" + dataset);
        DataLoader.loadFleet(dataset);

        System.out.println("INITIALIZE CONSTANTS");
        Constant.initDerivedValues();

        if (test) {
            Constant.TRUCK_PAYLOAD              = 300;
            Constant.MAX_DRONE_PER_VEHICLE      = 3;
            Constant.MAX_VEHICLE                = 10;
            Constant.MAX_DRONE                  = 30;
            Constant.DRONE_AND_EQUIPMENT_WEIGHT = Constant.TRUCK_PAYLOAD / 5;
        }
        
        System.out.println("\nTRUE CONSTANTS");
        System.out.printf("  %-32s = %s%n", "TRUCK_PAYLOAD",              Constant.TRUCK_PAYLOAD);
        System.out.printf("  %-32s = %s%n", "MAX_DRONE_PER_VEHICLE",      Constant.MAX_DRONE_PER_VEHICLE);
        System.out.printf("  %-32s = %s%n", "MAX_VEHICLE",                Constant.MAX_VEHICLE);
        System.out.printf("  %-32s = %s%n", "MAX_DRONE",                  Constant.MAX_DRONE);
        System.out.printf("  %-32s = %s%n", "DRONE_AND_EQUIPMENT_WEIGHT", Constant.DRONE_AND_EQUIPMENT_WEIGHT);
        System.out.println();


        long startTime = System.currentTimeMillis();

        VRPInstance.calculateDistance();
        VRPInstance.initNgNeighborhoods(Constant.LABEL_MAX_NG_SIZE);

        DroneScheduleEnumeration droneSchedulesEnum = new DroneScheduleEnumeration();
        droneSchedulesEnum.solve();

        long end_drone_enum = System.currentTimeMillis();

        // Flush everything before B&P so timestamps in the log line up cleanly.
        System.out.flush();
        System.err.flush();

        BranchAndPrice bap = new BranchAndPrice();
        try {
            bap.run();
        } catch (GRBException e) {
            e.printStackTrace();
        } finally {
            // Make sure B&P's streamed output hits the log immediately.
            System.out.flush();
            System.err.flush();
        }

        long end_bap = System.currentTimeMillis();

        double time_drone = (end_drone_enum - startTime) / 1000.0;
        double time_branch = (end_bap - end_drone_enum) / 1000.0;
        double time_whole = (end_bap - startTime) / 1000.0;

        // ---- Console/log section header for this dataset ----
        System.out.println("=================== Best solution ===================");

        List<Route> best = bap.getBestSolution();
        double primal = bap.getBestObjective(); // +∞ if no incumbent
        double dual = bap.getDualObjective(); // +∞ if nothing to bound / NaN if invalid
        boolean hasPrimal = (best != null) && Double.isFinite(primal);
        boolean hasDual = Double.isFinite(dual);

        double gap = -1.0;
        if (hasPrimal && hasDual && Math.abs(primal) > Constant.EPSILON) {
            gap = Math.abs(primal - dual) / Math.abs(primal) * 100.0;
        }

        if (hasPrimal) {
            for (Route r : best)
                System.out.println(r);
            System.out.println("Objective = " + primal);
        } else {
            System.out.println("(no integer solution found)");
        }
        System.out.println();

        System.out.printf("Drone Schedules Enumeration: %.1fs%n", time_drone);
        System.out.printf("Branch and price:           %.1fs%n", time_branch);
        System.out.printf("%nProgramme runs in %.1fs%n%n", time_whole);

        // ---- CSV output (unchanged) ----
        if (!hasPrimal) {
            out.println(",," + dataset + ",NO_SOL,"
                    + (hasDual ? String.valueOf(dual) : "NaN") + ","
                    + (gap >= 0 ? String.valueOf(gap) : "NaN") + ","
                    + time_whole + "," + time_drone + "," + time_branch);
        } else {
            out.println(",," + dataset + "," + primal + ","
                    + (hasDual ? String.valueOf(dual) : "NaN") + ","
                    + (gap >= 0 ? String.valueOf(gap) : "NaN") + ","
                    + time_whole + "," + time_drone + "," + time_branch);
        }
        out.flush();
    }

    public static void printRoutePool(String fileName) {
        try (PrintWriter out = new PrintWriter(fileName)) {
            for (Route r : VRPInstance.routePool)
                out.println(r);
        } catch (FileNotFoundException e) {
            System.err.println("File not found " + e.getMessage());
        }
    }

    public static void printParetoFront(String fileName) {
        int counter = 0;
        try (PrintWriter out = new PrintWriter(fileName)) {
            for (int customer = 1; customer <= Constant.TOTAL_CUSTOMER; customer++) {
                out.println("\n==== Customer " + customer + " ====");
                for (int drone = 1; drone <= Constant.MAX_DRONE_PER_VEHICLE; drone++) {
                    for (var entry : DroneScheduleEnumeration.paretoMap.get(customer).get(drone).entrySet()) {
                        out.println("\n-" + customer + "- " + getCustomerServedSet(entry.getKey()));
                        for (DroneSchedule s : entry.getValue().nonDominatedSchedules) {
                            out.println(s + s.sequences.toString());
                            counter++;
                        }
                    }
                }
            }
            System.out.println("TOTAL DRONE SCHEDULES: " + counter);
        } catch (FileNotFoundException e) {
            System.err.println("File not found " + e.getMessage());
        }
    }

    public static List<Integer> getCustomerServedSet(BigInteger customerServed) {
        List<Integer> indices = new ArrayList<>();
        BigInteger temp = customerServed;
        while (!temp.equals(BigInteger.ZERO)) {
            int index = temp.getLowestSetBit();
            indices.add(index);
            temp = temp.clearBit(index);
        }
        return indices;
    }

    /** Writes every byte to two underlying streams (e.g. console + log file). */
    static class TeeOutputStream extends OutputStream {
        private final OutputStream a;
        private final OutputStream b;

        TeeOutputStream(OutputStream a, OutputStream b) {
            this.a = a;
            this.b = b;
        }

        @Override
        public void write(int bb) throws IOException {
            a.write(bb);
            b.write(bb);
        }

        @Override
        public void write(byte[] buf, int off, int len) throws IOException {
            a.write(buf, off, len);
            b.write(buf, off, len);
        }

        @Override
        public void flush() throws IOException {
            a.flush();
            b.flush();
        }

        @Override
        public void close() throws IOException {
            // Only flush; callers close the underlying streams themselves.
            flush();
        }
    }
}