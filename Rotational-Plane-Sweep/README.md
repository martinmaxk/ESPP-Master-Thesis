# Neighborhood Search and Triangular Expansion
This is an implementation of Neighborhood Search,
which is an algorithm that can be used to connect a query point 
to a visibility graph.

Included is also the CGAL implementation of Triangular Expansion.
The six map types used (bgmaps, da2, dao, rooms, sc1 and wc3) are retrieved from MovingAI (https://movingai.com).
To perform pre-processing on all maps of a map type, which involves
building the visibility graph using Rotational Plane Sweep, in the terminal execute
```console
dotnet run Input/[BENCHMARK_SUITE] build --configuration Release
``` 

To perform benchmarking with Neighborhood Search on a map type
(replace "[BENCHMARK_SUITE]" with e.g. dao), in the terminal execute 
```console
dotnet run scenarios/[BENCHMARK_SUITE] hog2 Vg
``` 

To perform benchmarking with Triangular Expansion on a map type
(replace "[BENCHMARK_SUITE]" with e.g. dao), in the terminal execute 
```console
dotnet run scenarios/[BENCHMARK_SUITE] hog2 TriExp
``` 

The results of the benchmarks will be in the folder "Output/[BENCHMARK_SUITE]".