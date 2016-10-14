/* Change log:
2016-10-06 : Launch of Round 2
*/

import javax.imageio.ImageIO;
import javax.swing.*;
import java.awt.*;
import java.awt.image.BufferedImage;
import java.awt.image.DataBufferByte;
import java.awt.event.KeyAdapter;
import java.awt.event.KeyEvent;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;
import java.io.*;
import java.util.Arrays;
import java.util.*;
import java.io.BufferedReader;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.PrintWriter;
import java.security.SecureRandom;

public class RobonautVisionVis {

    public static long seed = 1;
    public static boolean debug = true;
    public static boolean visualize = true;
    public static boolean renderFaces = false;
    public static String execCommand = null;
    public static String testingFile = null;
    public static String trainingFile = null;
    public static String[] modelFiles = new String[] {
        "drill_scaled.ply", "handrail_scaled.ply", "rfid_reader_scaled.ply", "soft_goods_box_scaled.ply"};
    public static String folder = "";
    public SecureRandom rnd;
    public static int W,H;
    public static double D; // Longest dimension of object
    public static Process solution;
    public final Object worldLock = new Object();
    public Transform TX = new Transform();

    public void printMessage(String s) {
        if (debug) {
            System.out.println(s);
        }
    }

    class Vertex {
        double x,y,z;
        public Vertex(double x, double y, double z) {
            this.x = x;
            this.y = y;
            this.z = z;
        }
    }

    class Face {
        int[] vi = null;
        public Face(int[] vi) {
            this.vi = vi;
        }
    }

    Vertex[] vertexData = null;
    Face[] faceData = null;

    public void parseModel(String[] plydata)
    {
        int numVertex = 0;
        int numFaces = 0;
        int idx = 0;
        while (!plydata[idx].equals("end_header"))
        {
            String[] tok = plydata[idx].split(" ");
            if (tok[0].equals("element"))
            {
                if (tok[1].equals("vertex"))
                {
                    numVertex = Integer.parseInt(tok[2]);
                } else if (tok[1].equals("face"))
                {
                    numFaces = Integer.parseInt(tok[2]);
                }
            }
            idx++;
        }
        idx++;
        printMessage("Model contains " + numVertex + " vertices and " + numFaces + " faces.");
        // Read vertices
        vertexData = new Vertex[numVertex];
        for (int i=0;i<numVertex;i++)
        {
            String[] tok = plydata[idx++].split(" ");
            vertexData[i] = new Vertex(Double.parseDouble(tok[0]), Double.parseDouble(tok[1]), Double.parseDouble(tok[2]));
        }
        // Read faces
        faceData = new Face[numFaces];
        for (int i=0;i<numFaces;i++)
        {
            String[] tok = plydata[idx++].split(" ");
            int[] vi = new int[Integer.parseInt(tok[0])];
            for (int j=0;j<vi.length;j++)
                vi[j] = Integer.parseInt(tok[1+j]);
            faceData[i] = new Face(vi);
        }
        // Calculate longest dimension
        D = 1e-4;
        for (int i1=0;i1<numVertex;i1++)
            for (int i2=i1+1;i2<numVertex;i2++)
            {
                double dx = vertexData[i1].x-vertexData[i2].x;
                double dy = vertexData[i1].y-vertexData[i2].y;
                double dz = vertexData[i1].z-vertexData[i2].z;
                double d = dx*dx+dy*dy+dz*dz;
                D = Math.max(D, d);
            }
        D = Math.sqrt(D);
        printMessage("Longest Dimension = " + D);
    }

    class Position {
        // x,y,z;
        // qr,qi,qj,qk;
        double[] dt = new double[7];
        public Position(double x, double y, double z, double qr, double qi, double qj, double qk) {
            dt[0] = x;
            dt[1] = y;
            dt[2] = z;
            dt[3] = qr;
            dt[4] = qi;
            dt[5] = qj;
            dt[6] = qk;
            normalize();
        }
        public Position(double[] _dt) {
            this.dt  = _dt;
            normalize();
        }
        public void normalize() {
            double sqlen = dt[3]*dt[3] + dt[4]*dt[4] + dt[5]*dt[5] + dt[6]*dt[6];
            sqlen = Math.sqrt(sqlen);
            dt[3] /= sqlen;
            dt[4] /= sqlen;
            dt[5] /= sqlen;
            dt[6] /= sqlen;
        }
    }

    class Transform
    {
        // vector from the center of the left camera to the center of the right camera
        public double[] T = new double[] {145.626654332161,1.65379695634088,-3.65966860066967};
        // quaternion parameters, correcting the right camera direction
        public double[] Q = new double[] {0.999985, 0.0000680463, -7.05536E-7, 0.00540286};
        // transition coefficients = tangent of half of the camera view angle divided by a mm-size of the pixel cell on the camera element.
        public double CxL = 0.000529185;
        public double CyL = 0.000528196;
        public double CxR = 0.000529527;
        public double CyR = 0.000528341;
        // Distortion coefficients
        public double KL = -2.41479E-8;
        public double KR = -2.21532E-8;
        // Coordinate Center
        public double[] C0 = new double[] {800, 600};

        public double[] crossProduct(double[] lhs, double[] rhs)
        {
            return new double[]{lhs[1]*rhs[2]-lhs[2]*rhs[1], lhs[2]*rhs[0]-lhs[0]*rhs[2], lhs[0]*rhs[1]-lhs[1]*rhs[0]};
        }

        public double[] rotate(double x, double y, double z, double qr, double qi, double qj, double qk)
        {
            double[] v = new double[] {x,y,z};
            double[] qvec = new double[]{qi, qj, qk};
            double[] uv = crossProduct(qvec, v);
            double[] uuv = crossProduct(qvec, uv);
            uv[0] *= 2.0 * qr;
            uv[1] *= 2.0 * qr;
            uv[2] *= 2.0 * qr;
            uuv[0] *= 2.0;
            uuv[1] *= 2.0;
            uuv[2] *= 2.0;
            return new double[] {v[0]+uv[0]+uuv[0], v[1]+uv[1]+uuv[1], v[2]+uv[2]+uuv[2]};
        }

        public double[] transform3Dto2D(double x, double y, double z)
        {

            double[] imageCoord = new double[4];
            // Left Camera
            //1. Correction on the camera position.
            double[] R1 = new double[] { x + T[0]/2.0, y + T[1]/2.0, z + T[2]/2.0 };
            //2. 3D Coordinates into the pixels in ideal (undistorted) image with the coordinate origin in the middle of the image
            double[] r1 = new double[] {(R1[0]/R1[2])/CxL , (R1[1]/R1[2])/CyL};
            //3. Distortion Applied and the coordinate origin is set to the image corner
            double[] r2 = new double[] {r1[0]*(1.0 + KL * (r1[0]*r1[0]+r1[1]*r1[1]))+C0[0],
                                        r1[1]*(1.0 + KL * (r1[0]*r1[0]+r1[1]*r1[1]))+C0[1]};
            //4. Finally, the quadratic-form fit is applied
            imageCoord[0] = r2[0];
            imageCoord[0] += -4.72664 + 0.00264009*r2[0] + 3.68547E-7*r2[0]*r2[0] - 0.003594*r2[1] +  4.59175E-7*r2[0]*r2[1] + 5.21369E-6*r2[1]*r2[1];
            imageCoord[1] = r2[1];
            imageCoord[1] += 9.60826 - 0.00203106*r2[0] - 3.99045E-6*r2[0]*r2[0] - 0.0121255*r2[1] +  5.57403E-6*r2[0]*r2[1] + 1.79152E-6*r2[1]*r2[1];

            // Right Camera
            //1. Correction on the camera position.
            R1 = new double[] { x - T[0]/2.0, y - T[1]/2.0, z - T[2]/2.0 };
            //2. Rotation correct between left and right camera
            R1 = rotate(R1[0], R1[1], R1[2], Q[0], Q[1], Q[2], Q[3]);
            //3. 3D Coordinates into the pixels in ideal (undistorted) image with the coordinate origin in the middle of the image
            r1 = new double[] {(R1[0]/R1[2])/CxR , (R1[1]/R1[2])/CyR};
            //4. Distortion Applied and the coordinate origin is set to the image corner
            r2 = new double[] {r1[0]*(1.0 + KR * (r1[0]*r1[0]+r1[1]*r1[1]))+C0[0],
                               r1[1]*(1.0 + KR * (r1[0]*r1[0]+r1[1]*r1[1]))+C0[1]};
            //5. Finally, the quadratic-form fit is applied
            imageCoord[2] = r2[0];
            imageCoord[2] += -11.058 + 0.0103577*r2[0] - 4.38807E-6*r2[0]*r2[0] + 0.0126752*r2[1] +  7.04229E-7*r2[0]*r2[1] - 5.02606E-6*r2[1]*r2[1];
            imageCoord[3] = r2[1];
            imageCoord[3] += 1.05562 + 0.0044762*r2[0] - 1.55134E-6*r2[0]*r2[0] + 0.00646057*r2[1] -  9.63543E-6*r2[0]*r2[1] - 3.60981E-6*r2[1]*r2[1];

            return imageCoord;
        }
    };

    class Drawer extends JFrame {

        public DrawerPanel panel;
        public int width, height;
        public boolean pauseMode = false;

        class DrawerKeyListener extends KeyAdapter {
            public void keyPressed(KeyEvent e) {
                synchronized (keyMutex) {
                    if (e.getKeyChar() == ' ') {
                        pauseMode = !pauseMode;
                        keyPressed = true;
                    }
                    keyMutex.notifyAll();
                }
            }
        }

        public BufferedImage imgLeft = null;
        public BufferedImage imgRight = null;

        public Position userPos, gtfPos;

        public void update(String leftFile, String rightFile, Position userPos, Position gtfPos) throws Exception {
           synchronized (worldLock) {
             imgLeft = ImageIO.read(new File(leftFile));
             imgRight = ImageIO.read(new File(rightFile));
             this.gtfPos = gtfPos;
             this.userPos = userPos;
           }
        }

        class DrawerPanel extends JPanel {

            public void renderModel(Graphics g, Position pos)
            {
                double[][] p2d = new double[vertexData.length][];
                for (int i=0;i<vertexData.length;i++)
                {
                    double[] p3 = TX.rotate(vertexData[i].x, vertexData[i].y, vertexData[i].z,
                                            pos.dt[3], pos.dt[4], pos.dt[5], pos.dt[6]);
                    p2d[i] = TX.transform3Dto2D(p3[0]+pos.dt[0], p3[1]+pos.dt[1], p3[2]+pos.dt[2]);
                }
                // render points
                for (int i=0;i<vertexData.length;i++)
                {
                    if (p2d[i][0]>=0 && p2d[i][0]<1600 && p2d[i][1]>=0 && p2d[i][1]<1200)
                        g.fillOval((int)p2d[i][0]/2-1, (int)p2d[i][1]/2-1, 2, 2);
                    if (p2d[i][2]>=0 && p2d[i][2]<1600 && p2d[i][3]>=0 && p2d[i][3]<1200)
                        g.fillOval(800+(int)p2d[i][2]/2-1, (int)p2d[i][3]/2-1, 2, 2);
                    p2d[i][0] /= 2;
                    p2d[i][1] /= 2;
                    p2d[i][2] /= 2;
                    p2d[i][3] /= 2;
                }
                // Render faces
                if (renderFaces)
                for (int f=0;f<faceData.length;f++)
                {
                    for (int k=0;k<2;k++)
                    {
                        g.setClip(800*k,0,800,600);
                        double sum = 0;
                        for (int j=0;j<faceData[f].vi.length;j++)
                        {
                            int v0 = faceData[f].vi[j];
                            int v1 = faceData[f].vi[(j+1)%faceData[f].vi.length];
                            sum += (p2d[v1][k*2]-p2d[v0][k*2])*(p2d[v1][k*2+1]+p2d[v0][k*2+1]);
                        }
                        // render only front facing faces
                        if (sum>0)
                        for (int j=0;j<faceData[f].vi.length;j++)
                        {
                            int v0 = faceData[f].vi[j];
                            int v1 = faceData[f].vi[(j+1)%faceData[f].vi.length];
                            g.drawLine(800*k+(int)p2d[v0][k*2], (int)p2d[v0][k*2+1], 800*k+(int)p2d[v1][k*2], (int)p2d[v1][k*2+1]);
                        }
                    }
                    g.setClip(0,0,1600,600);
                }
            }

            public void paint(Graphics g) {
                synchronized (worldLock) {

                    if (imgLeft!=null)
                    {
                        g.drawImage(imgLeft, 0, 0, imgLeft.getWidth()/2, imgLeft.getHeight()/2, 0, 0, imgLeft.getWidth(), imgLeft.getHeight(), null);
                    }
                    if (imgRight!=null)
                    {
                        g.drawImage(imgRight,800, 0, imgRight.getWidth(), imgRight.getHeight()/2, 0, 0, imgRight.getWidth(), imgRight.getHeight(), null);
                    }

                    // draw model
                    g.setColor(Color.GREEN);
                    if (gtfPos!=null)
                        renderModel(g, gtfPos);
                    g.setColor(Color.RED);
                    if (userPos!=null)
                        renderModel(g, userPos);
                }
            }
        }

        class DrawerWindowListener extends WindowAdapter {
            public void windowClosing(WindowEvent event) {
                RobonautVisionVis.stopSolution();
                System.exit(0);
            }
        }

        final Object keyMutex = new Object();
        boolean keyPressed;

        public void processPause() {
            synchronized (keyMutex) {
                if (!pauseMode) {
                    return;
                }
                keyPressed = false;
                while (!keyPressed) {
                    repaint();
                    try {
                        keyMutex.wait();
                    } catch (InterruptedException e) {
                        // do nothing
                    }
                }
            }
        }

        public Drawer() {
            super();

            panel = new DrawerPanel();
            getContentPane().add(panel);

            addWindowListener(new DrawerWindowListener());

            width = 800*2;
            height = 600;

            addKeyListener(new DrawerKeyListener());

            setSize(width, height);
            setTitle("Robonaut Vision Tool Manipulation Marathon Match");

            setResizable(false);
            setVisible(true);
        }
    }


    public int[] imageToArray(String imageFile) throws Exception {
        printMessage("Reading image from " + imageFile);
        BufferedImage img = ImageIO.read(new File(imageFile));

        H = img.getHeight();
        W = img.getWidth();
        int[] res = new int[H * W];

        int pos = 0;
        byte[] pixels = ((DataBufferByte) img.getRaster().getDataBuffer()).getData();
        for (int i = 0; i < pixels.length; i+=3) {
            int v0 = (int)(pixels[i]);
            if (v0<0) v0 += 256;
            int v1 = (int)(pixels[i+1]);
            if (v1<0) v1 += 256;
            int v2 = (int)(pixels[i+2]);
            if (v2<0) v2 += 256;
            res[pos++] = v0 | (v1<<8) | (v2<<16);
        }
        return res;
    }


    class SingleFrame {
        int[] leftImg;
        int[] rightImg;
        Position gtf;
        Position gtf2 = null;
        String leftFile, rightFile;

        public SingleFrame(String fileNameLeft, String fileNameRight, double[] gtfdt) {
            leftFile = fileNameLeft;
            rightFile = fileNameRight;
            gtf = new Position(gtfdt);
            if (gtfdt.length>7)
                gtf2 = new Position(gtfdt[7], gtfdt[8], gtfdt[9], gtfdt[10], gtfdt[11], gtfdt[12], gtfdt[13]);
        }

        public void Load() throws Exception {
            leftImg = imageToArray(leftFile);
            rightImg = imageToArray(rightFile);
        }
    }

    public ArrayList<SingleFrame> loadPairs(String filename) throws Exception {
        ArrayList<SingleFrame> frames = new ArrayList<SingleFrame>();
        BufferedReader br = new BufferedReader(new FileReader(filename));
        int n = Integer.parseInt(br.readLine());
        for (int i=0;i<n;i++) {
            String s = br.readLine();
            if (s==null) break;
            String[] token = s.split(",");
            int iseed = Integer.parseInt(token[0]);
            if (iseed==seed)
            {
                double[] dt = null;
                if (token.length>10)
                    dt = new double[14];
                else
                    dt = new double[7];
                for (int j=0;j<dt.length;j++)
                    dt[j] = Double.parseDouble(token[3+j]);
                SingleFrame frm = new SingleFrame(folder+token[1], folder+token[2], dt);
                frames.add(frm);
            }
        }
        br.close();
        return frames;
    }

    public double calcScore(Position userData, Position gtf, int TotalUsed)
    {
        double AverageT = 0; // Average positional error
        for (int i=0;i<vertexData.length;i++)
        {
            // estimate
            double[] e = TX.rotate(vertexData[i].x, vertexData[i].y, vertexData[i].z,
                                   userData.dt[3], userData.dt[4], userData.dt[5], userData.dt[6]);
            e[0] += userData.dt[0];
            e[1] += userData.dt[1];
            e[2] += userData.dt[2];
            // ground truth
            double[] o = TX.rotate(vertexData[i].x, vertexData[i].y, vertexData[i].z,
                                   gtf.dt[3], gtf.dt[4], gtf.dt[5], gtf.dt[6]);
            o[0] += gtf.dt[0];
            o[1] += gtf.dt[1];
            o[2] += gtf.dt[2];
            // Add positional error
            AverageT += Math.sqrt( ((o[0]-e[0])*(o[0]-e[0])) + ((o[1]-e[1])*(o[1]-e[1])) + ((o[2]-e[2])*(o[2]-e[2])) );
        }
        AverageT /= vertexData.length;
        printMessage("AverageT = " + AverageT);
        double AD =  1.0 / (1.0 + (AverageT / (0.1*D)));
        printMessage("AD = " + AD);
        printMessage("Image pairs training on = " + TotalUsed);
        double scorePair = 1000000.0*AD / (1.0 + 0.01 * TotalUsed);
        return scorePair;
    }

    public double doExec() throws Exception {

        try {
            rnd = SecureRandom.getInstance("SHA1PRNG");
        } catch (Exception e) {
            System.err.println("ERROR: unable to generate test case.");
            System.exit(1);
        }
        rnd.setSeed(seed);        
        
        // launch solution
        printMessage("Executing your solution: " + execCommand);
        solution = Runtime.getRuntime().exec(execCommand);

        BufferedReader reader = new BufferedReader(new InputStreamReader(solution.getInputStream()));
        PrintWriter writer = new PrintWriter(solution.getOutputStream());
        new ErrorStreamRedirector(solution.getErrorStream()).start();

        String[] plyData = null;
        ArrayList<SingleFrame> trainingFrames = loadPairs(trainingFile);
        ArrayList<SingleFrame> testingFrames = loadPairs(testingFile);

        // Load model file
        if (modelFiles[(int)seed-1]!=null)
        {
            ArrayList<String> temps = new ArrayList<String>();
            BufferedReader br = new BufferedReader(new FileReader(modelFiles[(int)seed-1]));
            String s = br.readLine();
            while (s!=null)
            {
                temps.add(s);
                s = br.readLine();
            }
            br.close();
            plyData = temps.toArray(new String[0]);
            parseModel(plyData);
        }

        int TotalUsed = 0;

        // call: int trainingModel(String[] model)
        writer.println(plyData.length);
        for (String s : plyData)
        {
            writer.println(s);
        }
        writer.flush();
        int ret = Integer.parseInt(reader.readLine());
        if (ret==0)
        {
            // Training images
            if (trainingFile != null) {
                printMessage("Available training images: " + trainingFrames.size());
                writer.println(trainingFrames.size());
                writer.flush();
                for (int i=0;i<trainingFrames.size();i++)
                {
                    SingleFrame aFrame = trainingFrames.get(i);
                    aFrame.Load();
                    for (int v : aFrame.leftImg) {
                        writer.println(v);
                    }
                    writer.flush();
                    for (int v : aFrame.rightImg) {
                        writer.println(v);
                    }
                    writer.flush();
                    writer.println(aFrame.gtf.dt.length);
                    for (double d : aFrame.gtf.dt) {
                        writer.println(d);
                    }
                    writer.flush();
                    TotalUsed++; // used a training image

                    // call training function
                    // ret = training(aFrame.leftImg, aFrame.rightImg, position);
                    ret = Integer.parseInt(reader.readLine());
                    if (ret==1)
                    {
                        // stop receiving training images
                        break;
                    }
                }
            } else
            {
                System.out.println("ERROR: Training file not provided");
                System.exit(0);
            }
        }

        // call doneTesting function

        double score = 0;
        String[] rawData;

        Drawer drawer = null;
        if (visualize) {
            drawer = new Drawer();
            drawer.pauseMode = false;
        }

        if (testingFile != null) {
            printMessage("Testing with " + testingFrames.size() + " image pairs");
            rawData = new String[testingFrames.size()];

            writer.println(testingFrames.size());
            writer.flush();
            for (int f=0;f<testingFrames.size();f++)
            {
                SingleFrame aFrame = testingFrames.get(f);
                aFrame.Load();
                for (int v : aFrame.leftImg) {
                    writer.println(v);
                }
                writer.flush();
                for (int v : aFrame.rightImg) {
                    writer.println(v);
                }
                writer.flush();

                // call testing function
                // ret = testing(aFrame.leftImg, aFrame.rightImg);
                Position userData = new Position(
                     Double.parseDouble(reader.readLine()),
                     Double.parseDouble(reader.readLine()),
                     Double.parseDouble(reader.readLine()),
                     Double.parseDouble(reader.readLine()),
                     Double.parseDouble(reader.readLine()),
                     Double.parseDouble(reader.readLine()),
                     Double.parseDouble(reader.readLine()) );

                // --------- Calculate score
                double scorePair = calcScore(userData, aFrame.gtf, TotalUsed);
                if (aFrame.gtf2!=null)
                {
                    scorePair = Math.max(scorePair, calcScore(userData, aFrame.gtf2, TotalUsed));
                }
                System.err.println("Score for image pair = " + scorePair + "\n");
                score += scorePair;

                // Visualize
                if (visualize) {
                    drawer.update(aFrame.leftFile, aFrame.rightFile, userData, aFrame.gtf);
                    drawer.pauseMode = true;
                    drawer.repaint();
                    drawer.processPause();
                }

            }
        } else
        {
            System.out.println("ERROR: Testing file not provided");
            System.exit(0);
        }
        stopSolution();
        return score;
    }

    
    public static void stopSolution() {
        if (solution != null) {
            try {
                solution.destroy();
            } catch (Exception e) {
                // do nothing
            }
        }
    }

    public static void main(String[] args) throws Exception {

       for (int i = 0; i < args.length; i++) {
            if (args[i].equals("-folder")) {
                folder = args[++i];
            } else if (args[i].equals("-exec")) {
                execCommand = args[++i];
            } else if (args[i].equals("-seed")) {
                seed = Long.parseLong(args[++i]);
            } else if (args[i].equals("-train")) {
                trainingFile = args[++i];
            } else if (args[i].equals("-test")) {
                testingFile = args[++i];
            } else if (args[i].equals("-silent")) {
                debug = false;
            } else if (args[i].equals("-faces")) {
                renderFaces = true;
            } else if (args[i].equals("-novis")) {
                visualize = false;
            } else {
                System.out.println("WARNING: unknown argument " + args[i] + ".");
            }
        }

        RobonautVisionVis vis = new RobonautVisionVis();
        try {
            double score = vis.doExec();
            System.out.println("Score  = " + score);
            if (visualize) System.exit(0);
        } catch (Exception e) {
            System.out.println("FAILURE: " + e.getMessage());
            e.printStackTrace();
            RobonautVisionVis.stopSolution();
        }
    }

    class ErrorStreamRedirector extends Thread {
        public BufferedReader reader;

        public ErrorStreamRedirector(InputStream is) {
            reader = new BufferedReader(new InputStreamReader(is));
        }

        public void run() {
            while (true) {
                String s;
                try {
                    s = reader.readLine();
                } catch (Exception e) {
                    // e.printStackTrace();
                    return;
                }
                if (s == null) {
                    break;
                }
                System.out.println(s);
            }
        }
    }
}
