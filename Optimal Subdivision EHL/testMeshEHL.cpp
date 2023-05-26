/*
 * Computes EHL query for a given scenario
 */
#include <stdio.h>
#include <iostream>
#include "point.h"
#include "consts.h"
#include "scenario.h"
#include "edge.h"
#include "coverage_ordering_path.h"
#include "ebhl_poly.h"
#include "ebhl_poly_query_v2.h"
#include "searchinstance.h"
#include <sys/stat.h>

using namespace std;
namespace pl = polyanya;
pl::MeshPtr mp;
pl::SearchInstance* si;
pl:: EBHL_poly_query_v2* ebhlQueryV2;
int runtimes = 5;

bool file_exists(const std::string& filename)
{
    struct stat buf;
    return stat(filename.c_str(), &buf) != -1;
}

//main query ran for EHL
int final_path_adjacent( string dir_name, string map_name, vector<double> &p_time, vector<double> &p_distance, vector<int> &path_length,
    int lambda, int compression){
    srand(2);
    std::cout<<"Current version: Final Experiment" << std::endl;
    vector<polyanya::Scenario> out;
    string mesh_path = "dataset/merged-mesh/"+dir_name+"/"+map_name +"-merged.mesh";
    ifstream meshfile(mesh_path);
    mp = new pl::Mesh(meshfile);
    //test with polyanya
    si = new pl::SearchInstance(mp);

    string grid_path = "dataset/grid/"+dir_name+"/"+map_name +".map";

    // mark obstacle edge;
    mp->pre_compute_obstacle_edge_on_vertex();
    //wrap it in class later?
    mp->mark_turning_point(grid_path.c_str());
    //turning point is the actual point which would be used in search
    vector<pl::Point> turning_point;
    //corresponding vertices number in mesh_vertices
    vector<int> turning_vertices;
    //vertice location in polygon?
    vector<pl::PointLocation> turning_vertices_location;
    vector<pl::Point> obstacle_middle_point;
    int id = 0;
    int poly = 0;
    int num_vertices = 0;
    for( pl::Vertex& v : mp->mesh_vertices){
        if (v.is_corner)
            num_vertices++;
        if(v.is_turning_vertex && !v.is_ambig) {
            for (int polygon: v.polygons) {
                if (polygon != -1) {
                    //                    p.polygons.insert(polygon);
                    poly = polygon;
                }
            }
            // there is some issue here, old implementation assume that these vertex always inside one polygon only. it doesnt
            // use the correct type actually, I fix it here manually assign a random adjacent polygon
            pl::PointLocation location = {pl::PointLocation::ON_CORNER_VERTEX_UNAMBIG, poly, -1, id, -1};
            turning_vertices_location.push_back(location);
            turning_vertices.push_back(id);
            turning_point.push_back(mp->mesh_vertices[id].p);
            pl::Vertex o1 = mp->mesh_vertices[mp->mesh_vertices[id].obstacle_edge[0]];
            pl::Vertex o2 = mp->mesh_vertices[mp->mesh_vertices[id].obstacle_edge[1]];
            pl::Point m  = {(o1.p.x+ o2.p.x)/2,
                            (o1.p.y+ o2.p.y)/2
            };
            obstacle_middle_point.push_back(m);
        }
        id ++;
    }
    turning_point.shrink_to_fit();
    obstacle_middle_point.shrink_to_fit();

    //load mesh
    string ebhl_mesh_path = "dataset/ebhl-mesh/" + dir_name + "/" + map_name + "-merged.mesh";
    ifstream ebhl_meshfile(ebhl_mesh_path);
    pl::MeshPtr ebhl_mp = new pl::Mesh(ebhl_meshfile);
    // mark obstacle edge;
    ebhl_mp->pre_compute_obstacle_edge_on_vertex();

    auto ebhl = new pl::EBHL_poly(ebhl_mp);

    string triangles_path = "dataset/ehl/" + dir_name + "/" + map_name + ".nt_mesh_triangles";
    ebhl->load_non_taut_triangles(triangles_path.c_str());

    ebhlQueryV2 = new pl::EBHL_poly_query_v2(mp, ebhl, turning_point, obstacle_middle_point);
    string label = "dataset/hub_label/" + dir_name + "/" + map_name + ".label";
    string order = "dataset/hub_label/" + dir_name + "/" + map_name + ".order";
    ebhlQueryV2->load_hub_label_on_convext_vertex(label, order);

    string grid_label_path = "dataset/ehl/" + dir_name + "/" + map_name + ".nt_mesh_adj";
    ebhl->load_adjacent_list(grid_label_path.c_str());
    if (lambda != -100 && ebhl->lambda != lambda)
        exit(0);
    if (compression != -100 && ebhl->compression != compression)
        exit(0);

    string scenario_path = "dataset/scenarios/"+dir_name+"/"+map_name +".map.scen";
    ifstream scenariofile(scenario_path);
    polyanya::load_scenarios(scenariofile, out);
    warthog::timer timer =  warthog::timer ();

    p_distance.resize(out.size());
    path_length.resize(out.size());
    p_time.resize(out.size());
    fill(p_time.begin(), p_time.end(), 0);

    double unique_label_size;
    double error_count = 0;
    vector<double> pl_distance(out.size());
    fill(pl_distance.begin(),pl_distance.end(),0);

    int pl_query = 0;
    for(const polyanya::Scenario& s : out){
        si->set_start_goal(s.start,s.goal);
        si->search();
        double poly_search_cost =  si->get_cost();
        pl_distance[pl_query] = poly_search_cost;
        pl_query++;
    }
    for ( int i = 0; i < runtimes; i++) {
        int query_number = 0;
        for (const polyanya::Scenario& s : out) {
            ebhlQueryV2->set_start_goal(s.start,s.goal);
            ebhlQueryV2->search_path();
            double distance = ebhlQueryV2->get_cost();
            auto current_path = ebhlQueryV2->get_path();
            double time = ebhlQueryV2->visible_timer.elapsed_time_micro() + ebhlQueryV2->location_timer.elapsed_time_micro() + 
                ebhlQueryV2->inner_timer.elapsed_time_micro();
            p_time[query_number] = p_time[query_number] + time;
            p_distance[query_number] = distance;
            path_length[query_number] = current_path.size();


            double epsilon = 0.01f;
            if (fabs(distance - pl_distance[query_number])>EPSILON*10) {
                // dont want to handle ambiguous case

                if(!ebhlQueryV2->is_start_or_target_ambiguous()){
//                                        vector<pl::Point> path;
//                                        si->get_path_points(path);
//                                        for(const auto& p : path){
//                                            std::cout<<p<<std::endl;
//                                        }
                    std::cout<<std::endl;
                    for(int i = 0; i < current_path.size(); ++i){
                        cout << current_path[i] << endl;
                    }
                    cout << "Error: "<< query_number << s.start << s.goal << pl_distance[query_number] << " " << distance << endl;
                    ++error_count;
                }
            }
            query_number++;
        }


    }
    double total = 0 ;
    for(double d_time  :p_time){
        total = total + d_time;
    }
    double avg_vis_time = (ebhlQueryV2->visible_time) / out.size() / runtimes;
    double avg_pl_time = (ebhlQueryV2->location_time) / out.size() / runtimes;
    double avg_inner_time = (ebhlQueryV2->inner_time) / out.size() / runtimes;
    double avg_total_time = avg_vis_time + avg_pl_time + avg_inner_time;
    std::cout << "Num via labels and visible checks: " << ebhlQueryV2->num_via << " " << ebhlQueryV2->num_visible_checks << std::endl;
    std::cout << "Num via labels pruned: " << ebhlQueryV2->num_via_pruned << std::endl;
    std::cout << "ebhl: average total time: " << avg_total_time << " micro secs" << std::endl;
    std::cout << "ebhl: average visibility time: " << avg_vis_time << " micro secs" << std::endl;
    std::cout<<"ebhl: average point location time: "<< avg_pl_time <<" micro secs" <<std::endl;
    std::cout << "ebhl: average via label time: " << avg_inner_time << " micro secs" << std::endl;
    cout << "Error count: " << error_count/runtimes << " out of " << out.size() << endl;
    
    ebhl_meshfile = ifstream(ebhl_mesh_path);
    ebhl_meshfile.ignore(std::numeric_limits<std::streamsize>::max());
    std::streamsize ebhl_mesh_size = ebhl_meshfile.gcount();
    ebhl_meshfile.clear();   //  Since ignore will have set eof.
    ebhl_meshfile.seekg(0, std::ios_base::beg);
    unsigned long total_space = ebhl_mesh_size + ebhl->hub_space + ebhl->via_space + ebhl->aux_space;

    string query_output = "dataset/result/query/meshLambda" + to_string(ebhl->lambda) + "Comp" + 
        to_string(ebhl->compression) + "/" + dir_name + ".csv";
    bool did_file_exist = file_exists(query_output);
    std::ofstream q_file;
    q_file.open(query_output, std::ios_base::app);
    double avg_num_via_checked = ebhlQueryV2->num_via / (double)out.size() / (double)runtimes;
    double avg_visible_checks = ebhlQueryV2->num_visible_checks / (double)out.size() / (double)runtimes;
    double avg_num_pruned = ebhlQueryV2->num_via_pruned / (double)out.size() / (double)runtimes;
    double mb_div = 1024 * 1024;
    if (!did_file_exist)
        q_file << "Map,NumVertices,Regions,M-CdtRegions,VisTime,PointLocationTime,ViaLabelTime,TotalTime,ViaLabelsCheckedAvg,VisibleChecksAvg,ViaLabelsPrunedAvg,NumViaLabels,HubSpaceMB,ViaSpaceMB,AuxSpaceMB,MeshSpaceMB,TotalSpaceMB,Cost/ViaLabels\n";
    q_file << std::fixed << setprecision(8) << map_name << "," << num_vertices << "," << ebhl_mp->mesh_polygons.size() << "," <<
        mp->mesh_polygons.size() << "," << avg_vis_time << "," <<
        avg_pl_time << "," << avg_inner_time << "," << avg_total_time << "," << 
        avg_num_via_checked << "," << avg_visible_checks << "," << avg_num_pruned << "," << 
        ebhl->convex_label_set.size() << "," << 
        (ebhl->hub_space / mb_div) << "," << (ebhl->via_space / mb_div) << "," << (ebhl->aux_space / mb_div) << "," << 
        (ebhl_mesh_size / mb_div) << "," << 
        (total_space / mb_div) << "," << ebhl->cost_ratio << std::endl;
    q_file.close();

    std::cout << "Wrote to " << query_output << std::endl;

    delete mp;
    delete ebhl;
    delete ebhlQueryV2;
    delete si;
    return 0;
}

void benchmark_map(string dir_name, string map_name, int lambda, int compression){


    vector<double> p_time;
    vector<double> p_distance;
    vector<int> path_length;


   final_path_adjacent(dir_name, map_name, p_time, p_distance, path_length, lambda, compression);
}


int main(int argc, char* argv[]) {
    if (argc > 1) {
        string dir_name = string (argv[1]);
        string map_name = string(argv[2]);
        int expected_lambda = -100;
        int expected_compression = -100;
        if (argc > 3) {
            expected_lambda = atoi(argv[3]);
            expected_compression = atoi(argv[4]);
        }
        benchmark_map(dir_name, map_name, expected_lambda, expected_compression);

    }
}