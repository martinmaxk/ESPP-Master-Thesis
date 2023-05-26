/*
 * Computes EHL query for a given scenario
 */
#include <stdio.h>
#include <iostream>
#include "point.h"
#include "consts.h"
#include "scenario.h"
#include "edge.h"
#include "searchinstance.h"
#include "visibleSearchInstance.h"

using namespace std;
namespace pl = polyanya;
pl::MeshPtr mp;
pl::visibleSearchInstance* vs;
int runtimes = 5;


//main query run
int benchmark_map(string dir_name, string map_name, vector<double>& p_time, vector<int>& p_expansions, vector<int>& p_num_visible) {
    //std::cout << "Current version: Final Experiment" << std::endl;
    vector<polyanya::Scenario> out;
    string mesh_path = "dataset/merged-mesh/" + dir_name + "/" + map_name + "-merged.mesh";
    ifstream meshfile(mesh_path);
    mp = new pl::Mesh(meshfile);
    //test with polyanya
    vs = new pl::visibleSearchInstance(mp);

    string grid_path = "dataset/grid/" + dir_name + "/" + map_name + ".map";

    // mark obstacle edge;
    mp->pre_compute_obstacle_edge_on_vertex();
    //wrap it in class later?
    mp->mark_turning_point(grid_path.c_str());

    string scenario_path = "dataset/scenarios/" + dir_name + "/" + map_name + ".map.scen";
    ifstream scenariofile(scenario_path);
    polyanya::load_scenarios(scenariofile, out);
    warthog::timer timer = warthog::timer();

    p_time.resize(out.size());
    fill(p_time.begin(), p_time.end(), 0);

    p_expansions.resize(out.size());
    fill(p_expansions.begin(), p_expansions.end(), 0);

    p_num_visible.resize(out.size());
    fill(p_num_visible.begin(), p_num_visible.end(), 0);

    for (int i = 0; i < runtimes; i++) {
        int query_number = 0;
        for (const polyanya::Scenario& s : out) {
            timer.start();
            std::vector<int> visible_vertices;
            vs->search_visible_vertices(s.start, visible_vertices);
            p_expansions[query_number] = p_expansions[query_number] + vs->successor_calls;
            p_num_visible[query_number] = p_num_visible[query_number] + visible_vertices.size();
            visible_vertices.clear();
            /*std::cout << s.start << std::endl;
            for (const int v : visible_vertices)
            {
                std::cout << mp->mesh_vertices[v].p << std::endl;
            }
            std::cout << "end" << std::endl;
            return 0;*/
            vs->search_visible_vertices(s.goal, visible_vertices);
            p_expansions[query_number] = p_expansions[query_number] + vs->successor_calls;
            p_num_visible[query_number] = p_num_visible[query_number] + visible_vertices.size();
            timer.stop();

            p_time[query_number] = p_time[query_number] + timer.elapsed_time_micro();
            query_number++;
        }
    }
    double totalTime = 0;
    for (double d_time : p_time) {
        totalTime = totalTime + d_time;
    }
    double meanTime = totalTime / out.size() / runtimes;

    int totalExp = 0;
    for (int exp : p_expansions) {
        totalExp = totalExp + exp;
    }
    double meanExp = ((double)totalExp) / out.size() / runtimes;

    int totalVis = 0;
    for (int numVis : p_num_visible) {
        totalVis = totalVis + numVis;
    }
    double meanVis = ((double)totalVis) / out.size() / runtimes;

    //std::cout << "average time: " << mean << " micro secs" << std::endl;
    std::cout << meanTime << ", " << meanExp << ", " << meanVis << std::endl;

    delete mp;
    delete vs;
    return 0;
}

void benchmark_map(string dir_name, string map_name) {
    vector<double> p_time;
    vector<int> p_expansions;
    vector<int> p_num_visible;
    benchmark_map(dir_name, map_name, p_time, p_expansions, p_num_visible);
}


int main(int argc, char* argv[]) {
    if (argc > 1) {
        string dir_name = string(argv[1]);
        string map_name = string(argv[2]);
        benchmark_map(dir_name, map_name);
    }
}