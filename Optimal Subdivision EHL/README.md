# Optimal Subdivision EHL

## Introduction

This codebase is largely based off of the EHL implementation
found here: https://github.com/goldi1027/EHL 
and therefore the prerequisites are the same and the experiments
for EHL can also be reproduced with this repository.
Therefore some of the original instructions from that project
are also included here.
EHL is licensed under MIT license and so is this project,
however the license for EHL was kept anonymous, and as such cannot be included here.

EHL can be applied to any mesh, not just grids.
This extension of the original EHL implementation experiments with
different meshes as well as a form of tree compression to reduce
the size of the files outputted by the pre-processing.

The primary contribution is the implementation of an optimal
subdivision in the sense that no other subdivision could
yield a lower average number of via labels per cell.
Intuitively, if one took a cell in this subdivision and divided
it further in any which way, each of the pieces would contain
the exact same via labels as the original cell.
It also has the property that for any cell and convex vertex pair,
either the entire cell is within a taut region of the vertex or none of it is,
which means visibility checks can be omitted in query time.
There may be a large overlap of via labels among neighboring
cells, however this can be mitigated by using the aforementioned
compression, which is made more effective by that property.

The results from experiments can be found as .csv files under dataset/result/Query.
meshLambda[X]Comp[Y] contain results for lambda [X] with compression depth [Y].
Lambda -1 is optimal subdivision. The .csv files currently there
are the results from our experiment run.

## Running

To reproduce the result, use the two following bash scripts.

bash preprocessing_mesh_EHL.sh [BENCHMARK_SUITE] [LAMBDA] [COMPRESSION_DEPTH]
e.g., run "bash preprocessing_mesh_EHL.sh dao 1 0" This bash command creates mesh and via labels for all the maps in the benchmark suite (dao).
It is required to run preprocessing_EHL.sh before doing this.

Lambda is the number of pieces each m-cdt polygon is be subdivided into.
Specify "-1" for optimal subdivision.
Specify compression depth as "-1" for maximum depth.

bash benchmark_mesh_EHL.sh [BENCHMARK_SUITE] 
e.g., run "bash benchmark_EHL.sh dao" This bash command runs queries for mesh EHL for all the maps in the benchmark suite (dao) using the queries taken from MovingAI.

The following is the original description of the EHL implementation which is still
valid for this repository.

# Euclidean Hub Labeling (EHL)

## Introduction

Implementation of Euclidean Hub Labeling (EHL). EHL is an ultrafast optimal path finding algorithm used for finding shortest paths in Euclidean space.
EHL utilises visibility graph and extends hub labeling[1] to build the index used for online pathfinding. The algorithm and details of our implementation of EHL is available in our paper. EHL is licensed under MIT license, which the license is kept anonymous for this submission.


## Dataset

The four benchmarks used by EHL (dao, da2, sc1, and bgmaps) are retrieved from MovingAI (https://movingai.com).
The merged-meshes are provided by the authors of Polyanya [2] and available from repository (https://bitbucket.org/%7B3c286763-d509-45c2-b036-75814ce8955f%7D/)
We have also provided the scenarios used for testing and experiments in the dataset to reproduce our results.
## Requirements
The following libraries need to be installed in order to reproduce the implementation results.
For installation of the libraries, please follow the guidelines provided by the links.

- CMake: https://cmake.org
- OpenMP: https://www.openmp.org
- Google Sparsehash: https://github.com/justinsb/google-sparsehash
- Boost geometry - EHL uses boost geometry library to find and check visibility area as required in the algorithm. However, the latest version doesn't handle some precise cases and only version 1.64 is accurate to our precision needs. Boost version 1.64 can be downloaded from here (https://www.boost.org/users/history/version_1_64_0.html)

After installing the libraries, as EHL uses Cmake to compile, certain changes and modifications should be made to CMakeLists.txt based on different machine settings.

Then with the Makefile which we have provided, the code could be compiled using "make fast".

## Running

Currently, we provide three bash scripts to quickly reproduce the experimental results reported in our paper.

bash preprocessing_EHL.sh [BENCHMARK_SUITE] 
e.g., run "bash preprocessing_EHL.sh dao" This bash command creates all the indexes (visibility graph, hub label and EHL) needed for EHL for all the maps in the benchmark suite (dao).

bash benchmark_EHL.sh [BENCHMARK_SUITE] 
e.g., run "bash benchmark_EHL.sh dao" This bash command runs queries for EHL for all the maps in the benchmark suite (dao) using the queries taken from MovingAI.

bash clean_index.sh [BENCHMARK_SUITE]
e.g., run "bash clean_index.sh". This bash command deletes and cleans all indexes of EHL directories.
