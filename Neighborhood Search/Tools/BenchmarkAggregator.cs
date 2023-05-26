using MathTools;
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;

namespace Pathfinding.VG
{
    public static class BenchmarkAggregator
    {
        public static void AggregateFile(string BasePath, string mapPath, string benchType, bool useGrid = true)
        {
            string gridBenchPath = mapPath + "BenchGrid.txt";
            string vgBenchPath = mapPath + "Bench" + benchType + ".txt";
            var outLines = new List<string>();
            var gridTimes = new List<double>();
            var gridExpansions = new List<int>();
            double gridMeanTime = 0;
            if (useGrid)
            {
                AggregateGridBenchFile(gridBenchPath, gridTimes, gridExpansions);
                GetGridAggregateOutput(gridTimes, gridExpansions, outLines, out gridMeanTime);
            }

            var vgTimes = new List<double>();
            var vgConnectTimes = new List<double>();
            var vgSearchTimes = new List<double>();
            var vgExpansions = new List<int>();
            var vgLosChecks = new List<int>();
            int worstIndex = -1;
            string worstStart = null, worstGoal = null;
            AggregateVgBenchFile(vgBenchPath, vgTimes, vgConnectTimes, vgSearchTimes, vgExpansions, vgLosChecks,
                ref worstIndex, ref worstStart, ref worstGoal);
            GetVgAggregateOutput(vgTimes, vgConnectTimes, vgSearchTimes, vgExpansions, vgLosChecks,
                worstIndex, worstStart, worstGoal, "", gridMeanTime, outLines);
            File.WriteAllLines(Path.Combine(BasePath, mapPath + "Aggregate" + benchType + ".txt"), outLines);
        }

        public static void AggregateDir(string BasePath, string benchType, bool useGrid = true)
        {
            var outLines = new List<string>();
            foreach (string mapDirPath in Directory.EnumerateDirectories(Path.Combine(BasePath, "Output")))
            {
                string gameDir = Path.GetFileName(mapDirPath);
                outLines.Add(gameDir);
                string outDirPath = Path.Combine(BasePath, "Output", gameDir);
                var gridTimes = new List<double>();
                var gridExpansions = new List<int>();
                double gridMeanTime = 0;
                if (useGrid)
                {
                    foreach (string gridBenchPath in Directory.EnumerateFiles(mapDirPath, "*Grid.txt"))
                    {
                        AggregateFile(BasePath, gridBenchPath.Substring(0, gridBenchPath.Length - "BenchGrid.txt".Length), benchType);
                        AggregateGridBenchFile(gridBenchPath, gridTimes, gridExpansions);
                    }
                    GetGridAggregateOutput(gridTimes, gridExpansions, outLines, out gridMeanTime);
                }

                var vgTimes = new List<double>();
                var vgConnectTimes = new List<double>();
                var vgSearchTimes = new List<double>();
                var vgExpansions = new List<int>();
                var vgLosChecks = new List<int>();
                int worstIndex = -1;
                string worstStart = null, worstGoal = null;
                string worstMap = "";
                foreach (string vgBenchPath in Directory.EnumerateFiles(mapDirPath, "*" + benchType + ".txt"))
                {
                    int prevWorstIndex = worstIndex;
                    AggregateVgBenchFile(vgBenchPath, vgTimes, vgConnectTimes, vgSearchTimes, vgExpansions, vgLosChecks,
                        ref worstIndex, ref worstStart, ref worstGoal);
                    if (worstIndex != prevWorstIndex)
                        worstMap = vgBenchPath;
                }
                GetVgAggregateOutput(vgTimes, vgConnectTimes, vgSearchTimes, vgExpansions, vgLosChecks, 
                    worstIndex, worstStart, worstGoal, worstMap, gridMeanTime, outLines);
                outLines.Add("");
            }
            File.WriteAllLines(Path.Combine(BasePath, "Output", "aggregate" + benchType + ".txt"), outLines);
        }

        private static void AggregateGridBenchFile(string gridBenchPath, 
            List<double> gridTimes, List<int> gridExpansions)
        {
            var gridLines = File.ReadLines(gridBenchPath).Skip(1);
            foreach (string gridLine in gridLines)
            {
                var split = gridLine.Split(';');
                gridTimes.Add(double.Parse(split[4]));
                gridExpansions.Add(int.Parse(split[5]));
            }
        }

        private static void AggregateVgBenchFile(string vgBenchPath,
            List<double> vgTimes, List<double> vgConnectTimes, List<double> vgSearchTimes,
            List<int> vgExpansions, List<int> vgLosChecks,
            ref int worstIndex, ref string worstStart, ref string worstGoal)
        {
            var vgLines = File.ReadLines(vgBenchPath).Skip(1);
            foreach (string vgLine in vgLines)
            {
                var split = vgLine.Split(';');
                double connectTime = double.Parse(split[4]);
                double searchTime = double.Parse(split[5]);
                vgTimes.Add(connectTime + searchTime);
                vgConnectTimes.Add(connectTime);
                vgSearchTimes.Add(searchTime);
                vgExpansions.Add(int.Parse(split[6]));
                vgLosChecks.Add(int.Parse(split[7]));
                if (worstIndex == -1 || vgTimes[^1] > vgTimes[worstIndex] + MathExt.EpsilonD)
                {
                    worstIndex = vgTimes.Count - 1;
                    worstStart = split[0];
                    worstGoal = split[1];
                }
            }
        }

        private static void GetGridAggregateOutput(List<double> gridTimes, 
            List<int> gridExpansions, List<string> outLines, out double gridMeanTime)
        {
            outLines.Add("Grid");
            gridMeanTime = gridTimes.Average();
            outLines.Add("Avg time ms: " + gridMeanTime);
            outLines.Add("Avg node expansions: " + gridExpansions.Average());
            outLines.Add("Standard deviation time ms: " + StandardDeviation(gridTimes));
        }

        private static void GetVgAggregateOutput(List<double> vgTimes, List<double> vgConnectTimes, List<double> vgSearchTimes,
            List<int> vgExpansions, List<int> vgLosChecks,
            int worstIndex, string worstStart, string worstGoal, string worstMap,
            double gridMeanTime, List<string> outLines)
        {
            outLines.Add("VisibilityGraph");
            double vgMeanTime = vgTimes.Average();
            outLines.Add("Avg time ms: " + vgMeanTime);
            double vgConnectMeanTime = vgConnectTimes.Average();
            outLines.Add("Avg connect time ms: " + vgConnectMeanTime);
            outLines.Add("Std connect time ms: " + StandardDeviation(vgConnectTimes));
            outLines.Add("Avg search time ms: " + vgSearchTimes.Average());
            outLines.Add("Worst time ms: " + vgTimes[worstIndex] +
                    " with start tile: " + worstStart + " and goal tile: " + worstGoal + (worstMap == "" ? "" : (" in map " + worstMap)));
            outLines.Add("Avg node expansions: " + vgExpansions.Average());
            outLines.Add("Avg LOS checks: " + vgLosChecks.Average());
            outLines.Add("Standard deviation time ms: " + StandardDeviation(vgTimes));

            outLines.Add("Average speedup over grid: " + (gridMeanTime / vgMeanTime));
        }

        /*public static double StandardDeviation(List<double> data, double mean)
        {
            decimal squaredSum = 0;
            for (int i = 0; i < data.Count; i++)
            {
                decimal diff = (decimal)(data[i] - mean);
                squaredSum += diff * diff;
            }
            double variance = (double)(squaredSum / data.Count);
            return Math.Sqrt(variance);
        }*/

        public static double StandardDeviation(List<double> values)
        {
            double avg = values.Average();
            return Math.Sqrt(values.Average(v => Math.Pow(v - avg, 2)));
        }
    }
}
