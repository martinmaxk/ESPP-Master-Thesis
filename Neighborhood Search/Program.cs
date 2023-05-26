using System;
using System.Linq;
using System.Collections.Generic;
using System.IO;
using System.Diagnostics;
using MathTools;
using Pathfinding.Internal.TiledUtils;
using Pathfinding.VG;
using PolyBoolOpMartinez;
using Triangulation;
using pgraph.grid;
using AnyaAStar;
using DiagnosticsUtils;
using System.Threading;
using System.Globalization;
using CGALDotNet;
using CGALDotNet.Polygons;
using CGALDotNetGeometry.Numerics;

namespace Pathfinding.VG
{
    class Program
    {
        private const int NumQueries = 1000;
        private const float CharSize = 1;

        static void Main(string[] args)
        {
            //args = new string[] { "scenarios/rooms", "hog2" };
            const string BasePath = "";//"../../../";
            /*using var br = new BinaryReader(File.OpenRead("Output/dao/ost100dReduced.vg"));
            VisibilityGraphSerializer.Deserialize(br,
                out var vg, out var _, out var _, out var _);
            Console.WriteLine(vg.visVertices.vs.Count);
            int c = 0;
            for (int i = 0; i < vg.allEdges.Count; i++)
                c += vg.allEdges[i].Count;
            Console.WriteLine(c);
            throw new Exception();*/
            string mapTypePath = Path.Combine(BasePath, args[0]);
            if (args[0] == "aggregate" && args[1] == "hog2")
            {
                BenchmarkAggregator.AggregateDir(BasePath, args[2], false);
                return;
            }
            if (args[0] == "aggregate")
            {
                BenchmarkAggregator.AggregateFile(BasePath, Path.Combine(BasePath, args[1]), args[2], false);
                return;
            }
            //ThreadPool.SetMaxThreads(11, 1);
            if (Directory.Exists(mapTypePath) && args[1] == "build")
            {
                Directory.CreateDirectory(Path.Combine(BasePath, "Output"));
                //int numFiles = Directory.EnumerateDirectories(Path.Combine(BasePath, "Input"))
                //.Select(p => Directory.EnumerateFiles(p, "*.").Count()).Sum();
                int numFiles = Directory.EnumerateFiles(mapTypePath, "*.").Count();
                using var resetEvent = new ManualResetEvent(false);
                //string mapTypePath = Path.Combine("Input", "random");
                //foreach (string mapTypePath in Directory.EnumerateDirectories(Path.Combine(BasePath, "Input")))
                string gameDir = Path.GetFileName(mapTypePath);
                string outDirPath = Path.Combine(BasePath, "Output", gameDir);
                OverwriteDir(outDirPath);
                foreach (string polyPath in Directory.EnumerateFiles(mapTypePath, "*."))
                {
                    var allPolys = new Polygon();
                    Console.WriteLine(polyPath);
                    allPolys.ReadFromLines(File.ReadAllLines(polyPath));
                    ThreadPool.QueueUserWorkItem((s) =>
                    {
                        BuildVg(polyPath, outDirPath, GetCharacter(), allPolys, true, true,
                        out Bbox _, out IVgConnector _, out AStarPathfinder _);
                        if (Interlocked.Decrement(ref numFiles) == 0)
                            resetEvent.Set();
                    });
                }
                resetEvent.WaitOne();
                return;
            }
            string mapName = args[0];
            if (Directory.Exists(mapTypePath) && args[1] == "hog2")
            {
                //int numFiles = Directory.EnumerateDirectories(Path.Combine(BasePath, "scenarios"))
                //    .Select(p => Directory.EnumerateFiles(p, "*.map.scen").Count()).Sum();
                int numFiles = Directory.EnumerateFiles(mapTypePath, "*.map.scen").Count();
                using var resetEvent = new ManualResetEvent(false);
                //foreach (string mapTypePath in Directory.EnumerateDirectories(Path.Combine(BasePath, "scenarios")))
                //{
                string gameDir = Path.GetFileName(mapTypePath);
                string outDirPath = Path.Combine(BasePath, "Output", gameDir);
                foreach (string scenPath in Directory.EnumerateFiles(mapTypePath, "*.map.scen"))
                {
                    string curMapName = Path.GetFileNameWithoutExtension(Path.GetFileNameWithoutExtension(scenPath));
                    Console.WriteLine(curMapName + " " + scenPath);
                    string polyPath = Path.Combine(BasePath, "Input", gameDir, curMapName);
                    var allPolys = new Polygon();
                    allPolys.ReadFromLines(File.ReadAllLines(polyPath));
                    var union = new Polygon();
                    union.ReadFromLines(File.ReadAllLines(Path.Combine(outDirPath, curMapName) + ".union"));
                    string vgPath = Path.Combine(outDirPath, curMapName) + "Reduced.vg";
                    var scenario = ReadScenario(scenPath);
                    using var binaryReader = new BinaryReader(File.OpenRead(vgPath));
                    VisibilityGraphSerializer.Deserialize(binaryReader,
                        out var visibilityGraph, out var levelBbox, out var connector, out var pathfinder);
                    List<double> times;
                    for (int i = 0; i < 3; i++)
                    {
                        using var vgStream = new StreamWriter(File.Create(Path.Combine(outDirPath, curMapName) + "Bench" + args[2] + ".txt"));
                        if (args[2] == "Vg")
                        {
                            BenchmarkVg(polyPath, scenario, vgStream, visibilityGraph, levelBbox, connector, pathfinder,
                                GetCharacter(), union, allPolys, true, true, out times);
                        }
                        else
                        {
                            BenchmarkTriExp(polyPath, scenario, vgStream, union, out times);
                        }
                        if (!times.Exists(t => t > 5))
                            break;
                    }
                    //using var gridStream = new StreamWriter(File.Create(Path.Combine(outDirPath, curMapName) + "BenchGrid.txt"));
                    //RunAnyaAStarExperiments(polyPath + ".bin", scenario, gridStream);
                    /*if (Interlocked.Decrement(ref numFiles) == 0)
                        resetEvent.Set();*/
                }
                //}
                //resetEvent.WaitOne();
                return;
            }
            var character = GetCharacter();
            var experiments = new List<Experiment>(); //{ new Vector2(277, 13) * 32, new Vector2(52, 41) * 32 };
            var consoleOut = Console.Out;
            Console.SetOut(TextWriter.Null);
            string vgFileName = Path.Combine(BasePath, "TestMaps", mapName);
            string gridFileName = Path.Combine(BasePath, "Grid", "BinTestMaps", mapName + ".bin");
            //Console.SetOut(consoleOut);
            GenerateExperiments(vgFileName, gridFileName, character, experiments);
            // First run is for warmup (JIT compiler)
            for (int i = 0; i < 2; i++)
            {
                if (i == 1)
                    Console.SetOut(consoleOut);
                BenchmarkVg(vgFileName, Path.GetDirectoryName(vgFileName), character, experiments, Console.Out);
                RunAnyaAStarExperiments(gridFileName, experiments, Console.Out);
                GC.Collect();
            }
        }

        private static void OverwriteDir(string path)
        {
            if (Directory.Exists(path))
                Directory.Delete(path, true);
            Directory.CreateDirectory(path);
        }

        private static List<Vector2> GetCharacter()
        {
            return new List<Vector2>() { new Vector2(0, 0), new Vector2(CharSize, 0),
                new Vector2(CharSize, CharSize), new Vector2(0, CharSize) };
        }

        private static void GenerateExperiments(string vgFileName, string gridFileName, 
            List<Vector2> character, List<Experiment> experiments)
        {
            var allPolys = new Polygon();
            allPolys.ReadFromLines(File.ReadAllLines(vgFileName));
            string outDir = Path.GetDirectoryName(vgFileName);
            var testVg = BuildVg(vgFileName, outDir, character, allPolys, true, true, out Bbox levelBbox,
                out IVgConnector _, out AStarPathfinder pathfinder);
            var testVg2 = BuildVg(vgFileName, outDir, character, allPolys, true, false, out Bbox _,
                out IVgConnector _, out AStarPathfinder pathfinder2);
            var testGrid = LoadGridGraph(gridFileName, character, out Vector2Int characterSize);
            var testGridPathfinder = new AStarPathfinder(testGrid.GraphSize);
            var random = new Random(0);
            var tempPath = new List<Vector2>();
            var tempPath2 = new List<Vector2>();
            var tempGridPath = new List<Vector2Int>();
            int m = 0;
            while (experiments.Count < NumQueries)
            {
                var randStart = new Vector2Int(random.Next((int)levelBbox.xmin / 32, (int)levelBbox.xmax / 32),
                    random.Next((int)levelBbox.ymin / 32, (int)levelBbox.ymax / 32)) * 32;
                var randGoal = new Vector2Int(random.Next((int)levelBbox.xmin / 32, (int)levelBbox.xmax / 32),
                    random.Next((int)levelBbox.ymin / 32, (int)levelBbox.ymax / 32)) * 32;
                bool res1 = testVg.FindPath(pathfinder, randStart, randGoal, tempPath);
                bool res2 = testVg2.FindPath(pathfinder2, randStart, randGoal, tempPath2);
                var startGrid = WorldToGrid(randStart);
                var goalGrid = WorldToGrid(randGoal);
                if (!MathExt.Approximately(MathExt.PathCost(tempPath), MathExt.PathCost(tempPath2),
                    1f) || res1 != res2)
                {
                    Console.WriteLine(m);
                    Console.WriteLine(res1);
                    Console.WriteLine(tempPath.Count + " " + tempPath2.Count);
                    Console.WriteLine(randStart / 32 + " " + randGoal / 32);
                    foreach (var p in tempPath)
                        Console.WriteLine(p / 32f);
                    foreach (var p in tempPath2)
                        Console.WriteLine(p / 32f);
                    Console.WriteLine(MathExt.PathCost(tempPath));
                    Console.WriteLine(MathExt.PathCost(tempPath2));
                    throw new Exception();
                }
                if (res1 && testGrid.FindPath(testGridPathfinder, startGrid, goalGrid, characterSize.x,
                    characterSize.y, tempGridPath, false))
                {
                    experiments.Add(new Experiment(randStart, randGoal, MathExt.PathCost(tempPath)));
                }
                if (m % 1000 == 0)
                    Console.WriteLine(m);
                m++;
            }
        }

        private static List<Experiment> ReadScenario(string path)
        {
            var experiments = new List<Experiment>();
            var lines = File.ReadLines(path).Skip(1);
            foreach (string line in lines)
            {
                if (line == "")
                    continue;
                var split = line.Split(' ', '\t');
                // (optimalGridPathCost / 4) rounded down
                // "Reporting results in buckets reduces the variance and
                // makes it easier to compare to similar problems across maps"
                int bucket = int.Parse(split[0]); 
                string mapName = split[1]; // Redundant, all mapNames are equal in file
                int mapWidth = int.Parse(split[2]);
                int mapHeight = int.Parse(split[3]);
                var start = new Vector2Int(int.Parse(split[4]), int.Parse(split[5])) * 32;
                var goal = new Vector2Int(int.Parse(split[6]), int.Parse(split[7])) * 32;
                start += new Vector2Int(2, 2);
                goal += new Vector2Int(2, 2);
                // The optimal path length is measured with diagonals having cost
                // sqrt(2), and cardinal movement having cost 1
                double optimalGridPathCost = double.Parse(split[8], NumberStyles.Any, CultureInfo.InvariantCulture);
                experiments.Add(new Experiment(start, goal, optimalGridPathCost));
            }
            return experiments;
        }

        private static void RunAnyaAStarExperiments(string fileName, List<Experiment> experiments, TextWriter stream)
        {
            var tileGrid = LoadTileGrid(fileName);
            var grid = new BitpackedGrid(tileGrid);
            var expander = new BitpackedGridExpansionPolicy(grid);
            var astar = new FastAStar<Vector2Int>(expander, grid.get_num_cells());
            //astar.verbose = ScenarioRunner.verbose;
            
            var sw = new Stopwatch();
            stream.WriteLine("start;target;optimalGridCost;gridCost;timeMilli;expanded");
            for (int i = 0; i < experiments.Count; i++)
            {
                sw.Reset();
                sw.Start();
                var exp = experiments[i];
                astar.mb_start_ = expander.getGridVertex(exp.start.x / 32, exp.start.y / 32);
                astar.mb_target_ = expander.getGridVertex(exp.goal.x / 32, exp.goal.y / 32);
                astar.run();
                sw.Stop();
                double cost = astar.mb_cost_;

                stream.WriteLine(string.Format("({0},{1});({2},{3});{4};{5};{6};{7}", 
                    exp.start.x / 32, exp.start.y / 32, exp.goal.x / 32, exp.goal.y / 32, exp.optimalCost,
                    Math.Round(cost, 2, MidpointRounding.AwayFromZero), sw.Elapsed.TotalMilliseconds, astar.expanded));
            }
        }

        private static void BenchmarkVg(string inFileName, string outDir, List<Vector2> character, 
            List<Experiment> experiments, TextWriter stream)
        {
            var allPolys = new Polygon();
            allPolys.ReadFromLines(File.ReadAllLines(inFileName));
            var union = new Polygon();
            union.ReadFromLines(File.ReadAllLines(inFileName + ".union"));
            //BenchmarkVg(fileName, experiments, stream, character, allPolys, false, false);
            //BenchmarkVg(fileName, experiments, stream, character, allPolys, false, true);
            //BenchmarkVg(fileName, experiments, stream, character, allPolys, true, false);
            BenchmarkVg(inFileName, outDir, experiments, stream, character, union, allPolys, true, true);
        }

        private static VisibilityGraph BuildVg(string inFileName, string outDir, List<Vector2> character, Polygon allPolys,
            bool reduced, bool storeTriangles, out Bbox levelBbox, 
            out IVgConnector connector, out AStarPathfinder pathfinder)
        {
            Console.WriteLine("Reduced: " + reduced + ", Use triangles: " + storeTriangles);
            var sw = new ExecutionStopwatch();
            sw.Start();
            /*character = new List<Vector2>(character);
            for (int i = 0; i < character.Count; i++)
                character[i] = -character[i];
            character.Reverse();
            int charBlIndex = MathExt.BottomLeftIndex(character);

            var triangulator = new PolygonTriangulator();
            var triangles = new List<List<Vector2>>();*/
            var triangulator = new PolygonTriangulator();
            var individualPolys = new List<Polygon>();
            levelBbox = new Bbox(float.MaxValue, float.MaxValue, float.MinValue, float.MinValue);
            /*var vertexToContours = new Dictionary<Vector2D, List<Contour>>();
            var vertexHash = new HashSet<Vector2D>();
            for (int i = 0; i < allPolys.Ncontours; i++)
            {
                var contour = allPolys.Contour(i);
                for (int j = 0; j < contour.Nvertices; j++)
                {
                    var vertex = contour.Vertex(j);
                    if (!vertexToContours.TryGetValue(vertex, out var contours))
                        contours = new List<Contour>();
                    contours.Add(contour);
                }
            }*/

            //int trianglesStart = 0;
            for (int i = 0; i < allPolys.Ncontours; i++)
            {
                var contour = allPolys.Contour(i);
                levelBbox = levelBbox.Union((Bbox)contour.Bbox());
                var polygon = new Polygon();
                var newContour = polygon.AddNewContour();
                for (int j = 0; j < contour.Nvertices; j++)
                    newContour.Add(contour.Vertex(j));
                individualPolys.Add(polygon);
                //triangles.Add(polygon);
                //polygon = new List<Vector2>();
                //triangulator.Triangulate(polygon, triangles, ref trianglesStart);
                //polygon.Clear();
            }
            sw.Stop();
            double triTime = sw.Elapsed.TotalMilliseconds;

            /*var levelMid = new Vector2Int(MathExt.NextPowerOfTwo(levelBbox.xmax + levelBbox.xmin) / 2,
                MathExt.NextPowerOfTwo(levelBbox.ymax + levelBbox.ymin) / 2);*/
            var levelMid = new Vector2(levelBbox.xmax + levelBbox.xmin, levelBbox.ymax + levelBbox.ymin) / 2;

            /*sw.Start();
            var expanded = new List<Polygon>(triangles.Count);
            var tempList = new List<Vector2>();

            for (int i = 0; i < triangles.Count; i++)
            {
                Debugging.Debug.Assert(MathExt.IsPolygonClockwise(triangles[i]));
                triangles[i].Reverse();
                MathExt.MinkowskiSum(triangles[i], character, tempList,
                    MathExt.BottomLeftIndex(triangles[i]), charBlIndex);
                tempList.Reverse();
                var expandedPoly = new Polygon();
                var expandedContour = expandedPoly.AddNewContour();
                for (int j = 0; j < tempList.Count; j++)
                    expandedContour.Add((Vector2D)tempList[j]);
                expanded.Add(expandedPoly);
                triangles[i].Reverse();
                tempList.Clear();
            }
            sw.Stop();
            double minkowskiTime = sw.Elapsed.TotalMilliseconds;*/

            sw.Start();
            var union = new Polygon();
            var polyBoolOp = new PolyBoolOp();
            polyBoolOp.UnionAll(individualPolys, union);
            var polygons = new List<List<Vector2>>(union.Ncontours);
            for (int i = 0; i < union.Ncontours; i++)
            {
                var contour = union.Contour(i);
                var poly = new List<Vector2>(contour.Nvertices);
                for (int j = 0; j < contour.Nvertices; j++)
                    poly.Add((Vector2)contour.Vertex(j));
                // Union can merge polygons that overlap in a vertex,
                // split these illegal non-simple polygons here
                triangulator.SplitDuplicateVertices(poly, polygons);
            }
            sw.Stop();
            double polyUnionTime = sw.Elapsed.TotalMilliseconds;
            Console.WriteLine("Triangulation time ms: " + triTime);
            //Console.WriteLine("Minkowski sum time ms: " + minkowskiTime);
            Console.WriteLine("Poly union time ms: " + polyUnionTime);
            File.WriteAllText(Path.Combine(outDir, Path.GetFileNameWithoutExtension(inFileName)) + ".union", 
                union.ToString());
            sw.Start();
            long GC_MemoryStart = System.GC.GetTotalMemory(true);
            var rotPlaneSweep = new RotationalPlaneSweep(reduced, storeTriangles);
            //var triExpansion = new TriangularExpansion(reduced, storeTriangles);
            var visVertices = new VisVertices() { vs = new DynamicArray<Vector2>(4) };
            var allEdges = new DynamicArray<DynamicArray<Edge>>(4);
            var allVisTriEdges = new DynamicArray<DynamicArray<VisTriEdge>>(4);
            var visBboxes = new DynamicArray<Bbox>(4);
            rotPlaneSweep.VisibilityGraph(polygons, ref visVertices, ref allEdges,
                ref allVisTriEdges, ref visBboxes);
            /*triExpansion.VisibilityGraph(union, polygons, ref visVertices, ref allEdges, 
                ref allVisTriEdges, ref visBboxes);*/
            long GC_MemoryEnd = System.GC.GetTotalMemory(true);
            Console.WriteLine("Vg memory usage: " + (double)(GC_MemoryEnd - GC_MemoryStart) / (1024 * 1024));
            sw.Stop();
            double vgBuildTime = sw.Elapsed.TotalMilliseconds;
            Console.WriteLine("Vg build time ms: " + vgBuildTime);
            Console.WriteLine("Total preprocessing time ms: " + (triTime + polyUnionTime +
                vgBuildTime));
            int numVertices = 0, numEdges = 0;
            for (int i = 0; i < visVertices.vs.Count; i++)
            {
                if (visVertices.IsDescriptor(i))
                    continue;
                numVertices++;
                numEdges += allEdges[i].Count;
            }
            Console.WriteLine("Num vertices: " + numVertices);
            Console.WriteLine("Num edges: " + numEdges);
            sw.Reset();

            var outPolyVg = new Polygon();
            var charContour = outPolyVg.AddNewContour();
            for (int i = character.Count - 1; i >= 0; i--)
                charContour.Add((Vector2D)(-character[i]));
            for (int i = 0; i < visVertices.vs.Count; i++)
            {
                if (visVertices.IsDescriptor(i))
                    continue;
                var contour = outPolyVg.AddNewContour();
                contour.Add((Vector2D)visVertices.V(i));
                for (int j = 0; j < allEdges[i].Count; j++)
                    contour.Add((Vector2D)visVertices.V(allEdges[i][j].vertexId));
            }
            string fileNameEnding = Path.GetFileNameWithoutExtension(inFileName) + (reduced ? "Reduced" : "");
            File.WriteAllText(Path.Combine(outDir, "outPolyVg" + fileNameEnding), outPolyVg.ToString());

            float maxLevelDim = Math.Max(levelBbox.xmax - levelBbox.xmin, levelBbox.ymax - levelBbox.ymin);
            if (storeTriangles)
            {
                Console.WriteLine(Path.Combine(outDir, fileNameEnding));
                using var binaryWriter = new BinaryWriter(File.Create(Path.Combine(outDir, fileNameEnding) + ".vg"));
                VisibilityGraphSerializer.Serialize(binaryWriter, visVertices, allEdges,
                    allVisTriEdges, levelMid, levelBbox);
            }

            if (storeTriangles)
            {
                var dfsConnector = new TimeDfsVgConnector();
                dfsConnector.Initialize(visVertices, allVisTriEdges, levelMid,
                    maxLevelDim);
                connector = dfsConnector;
            }
            else
            {
                var rpsConnector = new TimePlaneSweepConnector();
                rpsConnector.Initialize(rotPlaneSweep);
                connector = rpsConnector;
            }
            var visibilityGraph = new VisibilityGraph();
            visibilityGraph.Initialize(visVertices, allEdges, connector);
            pathfinder = new AStarPathfinder(visVertices.vs.Count + 2);
            return visibilityGraph;
        }

        private static void BenchmarkVg(string inFileName, string outDir, List<Experiment> experiments,
            TextWriter stream, List<Vector2> character, Polygon union, Polygon allPolys,
            bool reduced, bool storeTriangles)
        {
            Bbox levelBbox;
            IVgConnector connector;
            AStarPathfinder pathfinder;
            var visibilityGraph = BuildVg(inFileName, outDir, character, allPolys, reduced, storeTriangles,
                out levelBbox, out connector, out pathfinder);
            BenchmarkVg(inFileName, experiments, stream, visibilityGraph,
                levelBbox, connector, pathfinder, character, union, allPolys, reduced, storeTriangles, out var _);
        }

        private static void BenchmarkVg(string fileName, List<Experiment> experiments,
            TextWriter stream, VisibilityGraph visibilityGraph, Bbox levelBbox, IVgConnector connector, 
            AStarPathfinder pathfinder, List<Vector2> character, Polygon union, Polygon allPolys,
            bool reduced, bool storeTriangles, out List<double> times)
        {
            /*var prevConnector = connector;
            PolygonUtils.GetTraversablePolygonsAndTEV(union, out var traversablePolys, out var triExpPtrs);
            var triExpConnector = new TimeTriangularExpansionConnector();
            triExpConnector.Initialize(new TriangularExpansion(reduced, storeTriangles), 
                traversablePolys, triExpPtrs, visibilityGraph.GetVertexToIdMap());
            connector = triExpConnector;*/
            visibilityGraph.SetConnector(connector);
            var path = new List<Vector2>();
            var path2 = new List<Vector2>();
            /*double totalConnectTime = 0, totalPathLengths = 0;
            long totalNumExpansions = 0, totalLosChecks = 0;
            int worstTimeIdx = 0;*/
            var sw = new Stopwatch();
            var sw2 = new Stopwatch();
            int numPaths = 0;
            times = new List<double>(experiments.Count);
            stream.WriteLine("start;target;optimalGridCost;cost;connectTimeMilli;searchTimeMilli;expanded;losChecks");
            var outputPoints = new List<Point2d>();
            double avgDiff = 0;
            for (int i = 0; i < experiments.Count; i++)
            {
                var exp = experiments[i];
                var start = exp.start;
                var goal = exp.goal;
                /*var unionCppPoly = new PolygonWithHoles2<EIK>(new Polygon2<EIK>(new Point2d[] { 
                    new Point2d(0, 0), new Point2d(10, 0), new Point2d(10, 10), new Point2d(0, 10) }));//unionCppPolys[j];
                unionCppPoly.AddHole(new Polygon2<EIK>(new Point2d[] { new Point2d(1, 1), new Point2d(2, 1), new Point2d(2, 2), new Point2d(1, 2) }.ToArray()));
                var locator = PolygonVisibility<EIK>.Instance.GetLocatorAndTEV(unionCppPoly);
                PolygonVisibility<EIK>.Instance.ComputeVisibilityTEVCached(new Point2d(2, 2), locator, out var res);
                var points = new List<Point2d>();
                res.GetAllPoints(points);
                Console.WriteLine(string.Join(";", points.Select(p => new Vector2D(p.x, p.y))));
                throw new Exception();*/
                //var unionCppPoly = unionCppPolys[j];
                //bool succ = PolygonVisibility<EIK>.Instance.ComputeVisibilityRSV(new Point2d(start.x, start.y), unionCppPoly, out var res);
                /*Console.WriteLine(succ);
                var points = new List<Point2d>();
                res.GetBoundary().GetPoints(points);
                Console.WriteLine(string.Join(";", points.Select(p => new Vector2D(p.x, p.y) / 32)));*/
                sw.Reset();
                sw.Start();
                bool success = visibilityGraph.FindPath(pathfinder, start, goal, path);
                sw.Stop();
                /*if (!success)
                    throw new Exception((Vector2)start / 32 + " " + (Vector2)goal / 32);*/
                times.Add(sw.Elapsed.TotalMilliseconds);
                /*if (times[^1] > times[worstTimeIdx])
                    worstTimeIdx = i;*/
                double connectStartTime, connectGoalTime;
                int losChecks = -1;
                if (connector is TimeDfsVgConnector timeDfsConnector)
                {
                    connectStartTime = timeDfsConnector.startTime;
                    connectGoalTime = timeDfsConnector.goalTime;
                    //totalLosChecks += timeDfsConnector.numLosChecks;
                    losChecks = timeDfsConnector.numLosChecks;
                    timeDfsConnector.numLosChecks = 0;
                }
                else if (connector is TimeTriangularExpansionConnector timeTriExpConnector)
                {
                    connectStartTime = timeTriExpConnector.startTime;
                    connectGoalTime = timeTriExpConnector.goalTime;

                    //visibilityGraph.SetConnector(prevConnector);
                    bool success2 = visibilityGraph.FindPath(pathfinder, start, goal, path2);
                    if (success != success2)
                        throw new Exception("One found a path, the other did not");
                    double cost1 = Math.Round(MathExt.PathCost(path) / 32, 2, MidpointRounding.AwayFromZero);
                    double cost2 = Math.Round(MathExt.PathCost(path2) / 32, 2, MidpointRounding.AwayFromZero);
                    if (Math.Abs(cost1 - cost2) > 1)
                        throw new Exception("Path cost diff is too large");
                    avgDiff += Math.Abs(cost1 - cost2);
                    visibilityGraph.SetConnector(connector);
                }
                else if (connector is TimePlaneSweepConnector timeRpsConnector)
                {
                    connectStartTime = timeRpsConnector.startTime;
                    connectGoalTime = timeRpsConnector.goalTime;
                }
                else
                    throw new Exception("Invalid connector");
                //double oldConn = connectStartTime + connectGoalTime;
                //Console.WriteLine("Start " + start);
                var startP = new Point2d(start.x, start.y);
                //Console.WriteLine(PolygonVisibility<EIK>.Instance.CdtContains(new Point2d(81 * 32, 195 * 32), triExpPtrs[0]));
                //throw new Exception();
                /*var ps = new List<Point2d>();
                PolygonVisibility<EIK>.Instance.ComputeVisibilityTEVCached(new Point2d(86 * 32, 191 * 32), triExpPtrs[0], ps);
                Console.WriteLine(string.Join(";", ps.Select(p => new Vector2D(p.x, p.y) / 32)));
                throw new Exception();*/
                /*var startPolyIndex = PolygonVisibility<EIK>.Instance.FindIndexContainsPoint(startP, traversablePolys);
                sw2.Restart();
                PolygonVisibility<EIK>.Instance.ComputeVisibilityTEVCached(startP, triExpPtrs[startPolyIndex], outputPoints);
                sw2.Stop();
                connectStartTime = sw2.Elapsed.TotalMilliseconds;
                //Console.WriteLine("Goal " + goal);
                var goalP = new Point2d(goal.x, goal.y);
                var goalPolyIndex = PolygonVisibility<EIK>.Instance.FindIndexContainsPoint(goalP, traversablePolys);
                sw2.Restart();
                PolygonVisibility<EIK>.Instance.ComputeVisibilityTEVCached(goalP, triExpPtrs[goalPolyIndex], outputPoints);
                sw2.Stop();
                connectGoalTime = sw2.Elapsed.TotalMilliseconds;*/
                /*totalConnectTime += connectStartTime + connectGoalTime;
                totalNumExpansions += pathfinder.numExpansions;
                totalPathLengths += MathExt.PathCost(path);*/
                double connectTime = connectStartTime + connectGoalTime;
                stream.WriteLine(string.Format("({0},{1});({2},{3});{4};{5};{6};{7};{8};{9}",
                    exp.start.x / 32, exp.start.y / 32, exp.goal.x / 32, exp.goal.y / 32, exp.optimalCost,
                    Math.Round(MathExt.PathCost(path) / 32, 2, MidpointRounding.AwayFromZero), 
                    connectTime, sw.Elapsed.TotalMilliseconds - connectTime, 
                    pathfinder.numExpansions, losChecks));
                numPaths++;
            }
            /*double totalTime = totalSw.Elapsed.TotalMilliseconds;
            double meanTime = totalTime / numPaths;
            Console.WriteLine("Avg time ms: " + meanTime);
            Console.WriteLine("Avg connect time ms: " + (totalConnectTime / numPaths));
            Console.WriteLine("Avg search time ms: " + ((totalTime - totalConnectTime) / numPaths));
            double worstTime = times[worstTimeIdx];
            Console.WriteLine("Worst time ms: " + worstTime + 
                " with start tile: " + experiments[worstTimeIdx].start / 32 + " and goal tile: " + 
                experiments[worstTimeIdx].goal / 32);
            Console.WriteLine("Standard deviation time ms: " + StandardDeviation(times, meanTime));
            Console.WriteLine("Avg path length: " + (totalPathLengths / numPaths));
            Console.WriteLine("Avg node expansions: " + (totalNumExpansions / (double)numPaths));
            if (storeTriangles)
                Console.WriteLine("Avg LOS checks: " + totalLosChecks / (double)numPaths);
            Console.WriteLine();*/
        }

        private static void BenchmarkTriExp(string fileName, List<Experiment> experiments,
            TextWriter stream, Polygon union, out List<double> times)
        {
            PolygonUtils.GetTraversablePolygonsAndTEV(union, out var traversablePolys, out var triExpPtrs);

            var sw = new Stopwatch();
            var sw2 = new Stopwatch();
            int numPaths = 0;
            times = new List<double>(experiments.Count);
            stream.WriteLine("start;target;optimalGridCost;cost;connectTimeMilli;searchTimeMilli;expanded;losChecks");
            var outputPoints = new List<Point2d>();
            for (int i = 0; i < experiments.Count; i++)
            {
                var exp = experiments[i];
                var start = exp.start;
                var goal = exp.goal;
                /*if (times[^1] > times[worstTimeIdx])
                    worstTimeIdx = i;*/
                double connectStartTime, connectGoalTime;
                var startP = new Point2d(start.x, start.y);
                var startPolyIndex = PolygonVisibility<EIK>.Instance.FindIndexContainsPoint(startP, traversablePolys);
                sw2.Restart();
                if (startPolyIndex != -1)
                    PolygonVisibility<EIK>.Instance.ComputeVisibilityTEVCached(startP, triExpPtrs[startPolyIndex], outputPoints);
                sw2.Stop();
                connectStartTime = sw2.Elapsed.TotalMilliseconds;
                //Console.WriteLine("Goal " + goal);
                var goalP = new Point2d(goal.x, goal.y);
                var goalPolyIndex = PolygonVisibility<EIK>.Instance.FindIndexContainsPoint(goalP, traversablePolys);
                sw2.Restart();
                if (goalPolyIndex != -1)
                    PolygonVisibility<EIK>.Instance.ComputeVisibilityTEVCached(goalP, triExpPtrs[goalPolyIndex], outputPoints);
                sw2.Stop();
                connectGoalTime = sw2.Elapsed.TotalMilliseconds;
                /*totalConnectTime += connectStartTime + connectGoalTime;
                totalNumExpansions += pathfinder.numExpansions;
                totalPathLengths += MathExt.PathCost(path);*/
                double connectTime = connectStartTime + connectGoalTime;
                stream.WriteLine(string.Format("({0},{1});({2},{3});{4};{5};{6};{7};{8};{9}",
                    exp.start.x / 32, exp.start.y / 32, exp.goal.x / 32, exp.goal.y / 32, exp.optimalCost,
                    0, connectTime, 0, 0, -1));
                numPaths++;
            }
            /*double totalTime = totalSw.Elapsed.TotalMilliseconds;
            double meanTime = totalTime / numPaths;
            Console.WriteLine("Avg time ms: " + meanTime);
            Console.WriteLine("Avg connect time ms: " + (totalConnectTime / numPaths));
            Console.WriteLine("Avg search time ms: " + ((totalTime - totalConnectTime) / numPaths));
            double worstTime = times[worstTimeIdx];
            Console.WriteLine("Worst time ms: " + worstTime + 
                " with start tile: " + experiments[worstTimeIdx].start / 32 + " and goal tile: " + 
                experiments[worstTimeIdx].goal / 32);
            Console.WriteLine("Standard deviation time ms: " + StandardDeviation(times, meanTime));
            Console.WriteLine("Avg path length: " + (totalPathLengths / numPaths));
            Console.WriteLine("Avg node expansions: " + (totalNumExpansions / (double)numPaths));
            if (storeTriangles)
                Console.WriteLine("Avg LOS checks: " + totalLosChecks / (double)numPaths);
            Console.WriteLine();*/
        }

        private static TileGrid LoadTileGrid(string fileName)
        {
            var binaryReader = new BinaryReader(File.OpenRead(fileName));
            TileExtensions.Initialize(binaryReader.ReadInt32());
            int mapSizeX = binaryReader.ReadInt32();
            int mapSizeY = binaryReader.ReadInt32();
            var tileGrid = new TileGrid(mapSizeX, mapSizeY);
            for (int i = 0; i < tileGrid.raw.Length; i++)
                tileGrid[i] = (TileShape)binaryReader.ReadByte();
            binaryReader.Dispose();
            return tileGrid;
        }

        private static GridGraph LoadGridGraph(string fileName, 
            List<Vector2> character, out Vector2Int characterSize)
        {
            var tileGrid = LoadTileGrid(fileName);
            var charPoly = new Polygon();
            charPoly.FromEnumerable(character);
            var charBbox = charPoly.Bbox();
            characterSize = new Vector2Int(
                (int)Math.Ceiling((charBbox.xmax - charBbox.xmin) / TileExtensions.TileSize),
                (int)Math.Ceiling((charBbox.ymax - charBbox.ymin) / TileExtensions.TileSize));

            return new GridGraph(tileGrid);
        }

        private static void BenchmarkGrid(string fileName, List<Vector2> character, List<Experiment> experiments)
        {
            var gridGraph = LoadGridGraph(fileName, character, out Vector2Int characterSize);
            BenchmarkGrid(gridGraph, new AStarPathfinder(gridGraph.GraphSize), experiments,
                characterSize);
            BenchmarkGrid(gridGraph, new AStarPathfinder(gridGraph.GraphSize), experiments,
                characterSize, true);
            BenchmarkGrid(gridGraph, new ThetaStarPathfinder(gridGraph.GraphSize), experiments,
                characterSize);
            BenchmarkGrid(gridGraph, new LazyThetaStarPathfinder(gridGraph.GraphSize), experiments,
                characterSize);
        }

        private static void BenchmarkGrid(GridGraph gridGraph, AStarPathfinder pathfinder, 
            List<Experiment> experiments,
            Vector2Int characterSize, bool smooth = false)
        {
            var gridPath = new List<Vector2Int>();
            var sw = new ExecutionStopwatch();
            decimal totalPathLengths = 0;
            long totalNumExpansions = 0, totalLosChecks = 0;
            int worstTimeIdx = 0;
            int numPaths = 0;
            var times = new List<double>();
            for (int i = 0; i < experiments.Count; i++)
            {
                var start = WorldToGrid(experiments[i].start);
                var goal = WorldToGrid(experiments[i].goal);
                double timeBef = sw.Elapsed.TotalMilliseconds;
                sw.Start();
                bool success = gridGraph.FindPath(pathfinder, start, goal,
                    characterSize.x, characterSize.y, gridPath, smooth);
                sw.Stop();
                times.Add(sw.Elapsed.TotalMilliseconds - timeBef);
                if (times[^1] > times[worstTimeIdx])
                    worstTimeIdx = i;
                totalNumExpansions += pathfinder.numExpansions;
                totalLosChecks += gridGraph.numLosChecks;
                totalPathLengths += (decimal)MathExt.PathCost(gridPath);
                numPaths++;
                if (experiments[i].optimalCost > (MathExt.PathCost(gridPath) * 32) + 1)
                {
                    Console.WriteLine(experiments[i].optimalCost + " " + MathExt.PathCost(gridPath) * 32);
                    Console.WriteLine(start + " " + goal);
                    var poly = new Polygon();
                    var cont = poly.AddNewContour();
                    cont.Add(new Vector2D(0, 0));
                    cont.Add(new Vector2D(characterSize.x, 0) * 32);
                    cont.Add(new Vector2D(characterSize.x, characterSize.y) * 32);
                    cont.Add(new Vector2D(0, characterSize.y) * 32);
                    cont = poly.AddNewContour();
                    for (int j = 0; j < gridPath.Count; j++)
                        cont.Add(new Vector2D(gridPath[j].x * 32, gridPath[j].y * 32));
                    File.WriteAllText("path", poly.ToString());
                    throw new Exception();
                }
            }
            Console.WriteLine(pathfinder + " Smoothing: " + smooth);
            double meanTime = (sw.Elapsed.TotalMilliseconds / numPaths);
            Console.WriteLine("Avg time ms: " + meanTime);
            double worstTime = times[worstTimeIdx];
            Console.WriteLine("Worst time ms: " + worstTime +
                " with start tile: " + experiments[worstTimeIdx].start / 32 + " and goal tile: " +
                experiments[worstTimeIdx + 1].goal / 32);
            Console.WriteLine("Standard deviation time ms: " + BenchmarkAggregator.StandardDeviation(times));
            Console.WriteLine("Avg path length: " + (totalPathLengths / numPaths) * TileExtensions.TileSize);
            Console.WriteLine("Avg node expansions: " + (totalNumExpansions / (double)numPaths));
            if (pathfinder is ThetaStarPathfinder)
                Console.WriteLine("Avg LOS checks: " + (totalLosChecks / (double)numPaths));
            /*if (pathfinder is ThetaStarPathfinder)
            {
                var poly = new Polygon();
                var cont = poly.AddNewContour();
                cont.Add(new Vector2D(0, 0));
                cont.Add(new Vector2D(characterSize.x, 0) * 32);
                cont.Add(new Vector2D(characterSize.x, characterSize.y) * 32);
                cont.Add(new Vector2D(0, characterSize.y) * 32);
                cont = poly.AddNewContour();
                for (int j = 0; j < gridPath.Count; j++)
                    cont.Add(new Vector2D(gridPath[j].x * 32, gridPath[j].y * 32));
                File.WriteAllText("path", poly.ToString());
            }*/
            Console.WriteLine();
        }

        private static Vector2Int WorldToGrid(Vector2 world)
        {
            return new Vector2Int((int)Math.Floor(world.x), (int)Math.Floor(world.y)) / TileExtensions.TileSize;
        }

        private struct Experiment
        {
            public Vector2Int start, goal;
            public double optimalCost;

            public Experiment(Vector2Int start, Vector2Int goal, double optimalCost)
            {
                this.start = start;
                this.goal = goal;
                this.optimalCost = optimalCost;
            }
        }

        public class TimePlaneSweepConnector : PlaneSweepConnector
        {
            private Stopwatch sw = new Stopwatch();
            public double startTime, goalTime;

            public override bool FindEdges(int sourceId, int goalId, VisVertices visVertices,
                DynamicArray<DynamicArray<Edge>> allEdges)
            {
                sw.Reset();
                sw.Start();
                bool connected = base.FindEdges(sourceId, goalId, visVertices, allEdges);
                sw.Stop();
                if (sourceId == goalId)
                    goalTime = sw.Elapsed.TotalMilliseconds;
                else
                    startTime = sw.Elapsed.TotalMilliseconds;
                return connected;
            }
        }

        public class TimeTriangularExpansionConnector : TriangularExpansionConnector
        {
            private Stopwatch sw = new Stopwatch();
            public double startTime, goalTime;

            public override bool FindEdges(int sourceId, int goalId, VisVertices visVertices,
                DynamicArray<DynamicArray<Edge>> allEdges)
            {
                triangularExpansion.sw = sw;
                sw.Reset();
                sw.Start();
                bool connected = base.FindEdges(sourceId, goalId, visVertices, allEdges);
                sw.Stop();
                if (sourceId == goalId)
                    goalTime = sw.Elapsed.TotalMilliseconds;
                else
                    startTime = sw.Elapsed.TotalMilliseconds;
                return connected;
            }
        }

        public class TimeDfsVgConnector : DfsVgConnector
        {
            private Stopwatch sw = new Stopwatch();
            public int numLosChecks;
            public double startTime, goalTime;

            public override bool FindEdges(int sourceId, int goalId, VisVertices visVertices, 
                DynamicArray<DynamicArray<Edge>> allEdges)
            {
                sw.Reset();
                sw.Start();
                bool connected = base.FindEdges(sourceId, goalId, visVertices, allEdges);
                sw.Stop();
                if (sourceId == goalId)
                    goalTime = sw.Elapsed.TotalMilliseconds;
                else
                    startTime = sw.Elapsed.TotalMilliseconds;
                return connected;
            }

            public override bool LineOfSight(int fromId, int toId)
            {
                numLosChecks++;
                return base.LineOfSight(fromId, toId);
            }
        }
    }
}
