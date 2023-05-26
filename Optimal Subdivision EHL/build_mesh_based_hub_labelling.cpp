/*
 * EHL construction and implementation
 */
#undef NDEBUG
#include <assert.h>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <expansion.h>
#include "visibleSearchInstance.h"
#include "visibleAreaSearchInstance.h"
#include "point.h"
#include <omp.h>
#include <iomanip>
#include "geometry.h"
#include "ebhl_poly.h"
#include <map>
#include <set>
#include <searchinstance.h>
#include <helper.h>
#include "coverage_ordering_path.h"
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include "structs/graph.h"
#include "ebhl_poly_query_v2.h"
#include <math.h>


typedef boost::geometry::model::d2::point_xy<double> point_type;
typedef boost::geometry::model::polygon<point_type> polygon_type;
typedef boost::geometry::model::box<point_type> box_type;
typedef boost::geometry::model::linestring<point_type> linestring_type;
typedef boost::geometry::model::multi_linestring<linestring_type> multi_linestring_type;
typedef boost::geometry::model::segment<point_type> segment_type;
using namespace std;
namespace pl = polyanya;
pl::MeshPtr mp;
pl::visibleAreaSearchInstance* vs;
pl::SearchInstance* se;
namespace bg = boost::geometry;

struct vis_poly {
    polygon_type  boost_poly;
    box_type bounding_box;
};


//output triangles into an entire polygon for visible area
void output_non_taut_triangle(const vector<pair<vis_poly, vis_poly>>& boost_poly_set, int& size, string output_file) {
    cout << "Saving triangles file to " << output_file << endl;
    ofstream outputFile(output_file);
    outputFile << boost_poly_set.size() << std::endl;
    for (int i = 0; i < size; ++i) {
        for (int j = 0; j < boost_poly_set[i].first.boost_poly.outer().size(); ++j) {
            outputFile << std::fixed << setprecision(8) << i * 2 << " " << boost_poly_set[i].first.boost_poly.outer()[j].x() << " " << boost_poly_set[i].first.boost_poly.outer()[j].y() << "\n";
        }
        for (int j = 0; j < boost_poly_set[i].second.boost_poly.outer().size(); ++j) {
            outputFile << std::fixed << setprecision(8) << i * 2 + 1 << " " << boost_poly_set[i].second.boost_poly.outer()[j].x() << " " << boost_poly_set[i].second.boost_poly.outer()[j].y() << "\n";
        }
    }
    outputFile.close();

}

//checks whether a grid is within a rectangle or not
/*bool is_within_rectangle(const point_type& check_point, const point_type& min, const point_type& max) {

    return min.x()<= check_point.x() && check_point.x() <= max.x() && min.y()<= check_point.y() && check_point.y() <= max.y();

}

bool polygon_within_grid(const polygon_type& poly1, const polygon_type& poly2) {
    for(const auto& p : poly1.outer()){
        if(!is_within_rectangle(p,poly2.outer()[0],poly2.outer()[2] )){
            return false;
        }
    }
    return true;
}*/




bool are_rectangles_overlap(const point_type& r1_min, const point_type& r1_max, const point_type& r2_min, const point_type& r2_max) {

    bool overlap_case = r1_min.x() <= r2_max.x() && r2_min.x() <= r1_max.x() &&
        r1_min.y() <= r2_max.y() && r2_min.y() <= r1_max.y();

    bool contain_case1 = r1_min.x() >= r2_min.x() && r1_max.x() <= r2_max.x() &&
        r1_min.y() >= r2_min.y() && r1_max.y() <= r2_max.y();

    bool contain_case2 = r2_min.x() >= r1_min.x() && r2_max.x() <= r1_max.x() &&
        r2_min.y() >= r1_min.y() && r2_max.y() <= r1_max.y();

    return overlap_case || contain_case1 || contain_case2;

}

bool grid_within_polygon(const polygon_type& grid, const polygon_type& poly2) {
    polygon_type max_grid;
    bg::append(max_grid, point_type{ grid.outer()[0].x() - EPSILON, grid.outer()[0].y() - EPSILON });
    bg::append(max_grid, point_type{ grid.outer()[1].x() - EPSILON, grid.outer()[1].y() + EPSILON });
    bg::append(max_grid, point_type{ grid.outer()[2].x() + EPSILON, grid.outer()[2].y() + EPSILON });
    bg::append(max_grid, point_type{ grid.outer()[3].x() + EPSILON, grid.outer()[3].y() - EPSILON });
    bg::append(max_grid, point_type{ grid.outer()[4].x() - EPSILON, grid.outer()[4].y() - EPSILON });

    polygon_type min_grid;
    bg::append(min_grid, point_type{ grid.outer()[0].x() + EPSILON, grid.outer()[0].y() + EPSILON });
    bg::append(min_grid, point_type{ grid.outer()[1].x() + EPSILON, grid.outer()[1].y() - EPSILON });
    bg::append(min_grid, point_type{ grid.outer()[2].x() - EPSILON, grid.outer()[2].y() - EPSILON });
    bg::append(min_grid, point_type{ grid.outer()[3].x() - EPSILON, grid.outer()[3].y() + EPSILON });
    bg::append(min_grid, point_type{ grid.outer()[4].x() + EPSILON, grid.outer()[4].y() + EPSILON });


    return   bg::within(max_grid, poly2) && bg::within(min_grid, poly2) && bg::within(grid, poly2);

    ;

}


//checks whether a given path is a non taut path could be pruned or not
bool is_non_taut_path(pl::Point start, const pl::Vertex& vertex, const pl::Point& target) {
    // for testing grid only
    const vector<pl::Vertex>& mesh_vertices = mp->mesh_vertices;
    const pl::Orientation& v_right_ori = get_orientation(start, vertex.p, target);
    if (v_right_ori == pl::Orientation::COLLINEAR) {
        // collinear is an ambiguious case, set it to true;
        /*if (is_left_bottom) {
            return !(start.distance(target) >= vertex.p.distance(target)
                     && start.distance(target) >= start.distance(vertex.p));
        }else{
            return true;
        }*/
        return true;
    }
    pl::Point mid_point = { (mesh_vertices[vertex.obstacle_edge[0]].p.x + mesh_vertices[vertex.obstacle_edge[1]].p.x) / 2,
                           (mesh_vertices[vertex.obstacle_edge[0]].p.y + mesh_vertices[vertex.obstacle_edge[1]].p.y) / 2
    };
    const pl::Orientation& start_v_ori = get_orientation(start, vertex.p, mid_point);

    if (v_right_ori == start_v_ori) {
        //same side;
        const pl::Orientation& o1 = get_orientation(mid_point, vertex.p, start);
        const pl::Orientation& o2 = get_orientation(mid_point, vertex.p, target);
        return o1 == o2;
    }
    else {
        return true;
    }
}

bool poly_to_label_is_non_taut(const pl::EBHL_poly* ebhl,
    const pl::Point& predecessor, const pl::Vertex& convex_vertex, const pl::Poly_label& poly) {
    bool isNonTaut = true;
    pl::Polygon plPoly = ebhl->mp->mesh_polygons[poly.poly];
    for (int v : plPoly.vertices) {
        isNonTaut = isNonTaut && is_non_taut_path(predecessor, convex_vertex, ebhl->mp->mesh_vertices[v].p);
    }
    return isNonTaut;
}

/*bool grid_to_label_is_non_taut(const pl::Point& predecessor, const pl::Vertex& convex_vertex, const pl::Grid_label& grid) {

    return is_non_taut_path(predecessor, convex_vertex, grid.a_p,true) && is_non_taut_path(predecessor, convex_vertex, grid.b_p,false) &&
           is_non_taut_path(predecessor, convex_vertex, grid.c_p,false) && is_non_taut_path(predecessor, convex_vertex, grid.d_p,false);

}*/


bool is_turn_edge(int source_vertex, int target_vertex) {
    pl::Point start = mp->mesh_vertices[source_vertex].p;
    const pl::Vertex& v = mp->mesh_vertices[target_vertex];
    const pl::Vertex& v1 = mp->mesh_vertices[v.obstacle_edge[0]];
    const pl::Vertex& v2 = mp->mesh_vertices[v.obstacle_edge[1]];
    const pl::Orientation  o1 = get_orientation(start, v.p, v1.p);
    const pl::Orientation  o2 = get_orientation(start, v.p, v2.p);
    if (o1 == pl::Orientation::COLLINEAR || o2 == pl::Orientation::COLLINEAR) {
        return true;
    }
    else if (o1 == o2) {
        return true;
    }
    return false;
}

bool is_valid_turn_edge(int source_vertex, int target_vertex) {
    if (!mp->mesh_vertices[target_vertex].is_ambig) {
        if (!is_turn_edge(source_vertex, target_vertex)) {
            return false;
        }
    }

    if (!mp->mesh_vertices[source_vertex].is_ambig) {
        if (!is_turn_edge(target_vertex, source_vertex)) {
            return false;
        }
    }

    return true;

}

double min_distance_to_poly(pl::Point p, polygon_type poly) {
    point_type gp = point_type(p.x, p.y);
    return bg::distance(gp, poly);
    /*double minSqrdDist = std::numeric_limits<double>::min();
    for (const auto& gp : poly.outer()) {
        pl::Point p2;
        p2.x = gp.x;
        p2.y = gp.y;
        minSqrdDist = std::min(minSqrdDist, p.distance_sq(p, p2));
    }
    return std::sqrt(minSqrdDist);*/
}

double max_distance_to_poly(pl::Point p, polygon_type poly) {
    double maxSqrdDist = std::numeric_limits<double>::max();
    for (const auto& gp : poly.outer()) {
        pl::Point p2;
        p2.x = gp.x();
        p2.y = gp.y();
        maxSqrdDist = std::max(maxSqrdDist, p.distance_sq(p2));
    }
    return std::sqrt(maxSqrdDist);
}

void create_half_split_vertices(pl::MeshPtr mp, int polygonId, int siblingPolyId,
    int hitEIndex, int otherHitEIndex, int hitVId, int otherHitVId,
    bool intersectsOnVertex, bool otherIntersectsOnVertex, vector<int>& vertices)
{
    pl::Polygon polygon = mp->mesh_polygons[polygonId];
    vertices.reserve(2 * (2 + std::abs(otherHitEIndex - hitEIndex)));

    vertices.push_back(hitVId);
    if (!otherIntersectsOnVertex) {
        vertices.push_back(otherHitVId);
    }
    for (int i = otherHitEIndex + 1; i != (hitEIndex + 1); i++) {
        if (i == polygon.vertices.size())
            i = 0;
        vertices.push_back(polygon.vertices[i]);
        if (!intersectsOnVertex && i == hitEIndex)
            break;
    }
}

void create_half_split_polygon(pl::MeshPtr mp, int polygonId, int siblingPolyId,
    int hitEIndex, int otherHitEIndex, int hitVId, int otherHitVId,
    bool intersectsOnVertex, bool otherIntersectsOnVertex)
{
    int childPolyId;
    if (siblingPolyId == polygonId)
        childPolyId = mp->mesh_polygons.size();
    else
        childPolyId = polygonId;

    pl::Polygon polygon = mp->mesh_polygons[polygonId];
    std::vector<int> vertices;
    vertices.reserve(2 * (2 + std::abs(otherHitEIndex - hitEIndex)));
    std::vector<int> polygons;
    polygons.reserve(vertices.capacity() + 1);
    polygons.push_back(-100);

    vertices.push_back(hitVId);
    polygons.push_back(siblingPolyId);
    if (!otherIntersectsOnVertex) {
        vertices.push_back(otherHitVId);
        polygons.push_back(polygon.polygons[(otherHitEIndex + 1) % polygon.vertices.size()]);
    }
    for (int i = otherHitEIndex + 1; i != (hitEIndex + 1); i++) {
        if (i == polygon.vertices.size())
            i = 0;
        vertices.push_back(polygon.vertices[i]);
        int neighborId = polygon.polygons[(i + 1) % polygon.vertices.size()];
        polygons.push_back(neighborId);
        if (!intersectsOnVertex && i == hitEIndex)
            break;

        if (neighborId == -1)
            continue;
        int vId = polygon.vertices[(i + 1) % polygon.vertices.size()];
        bool foundVertex = false;
        pl::Polygon& neighborPoly = mp->mesh_polygons[neighborId];
        for (int j = 0; j < neighborPoly.vertices.size(); j++) {
            int neighborVId = neighborPoly.vertices[j];
            if (neighborVId != vId)
                continue;
            assert(neighborPoly.vertices[(j + 1) % neighborPoly.vertices.size()] ==
                polygon.vertices[i]);
            neighborPoly.polygons[(j + 1) % neighborPoly.vertices.size()] = childPolyId;
            foundVertex = true;
            break;
        }
        assert(foundVertex);
    }
    int lastNeighbor = polygons.back();
    polygons.pop_back();
    polygons[0] = lastNeighbor;
    pl::Polygon childPoly;
    childPoly.vertices = vertices;
    childPoly.polygons = polygons;
    childPoly.min_x = childPoly.min_y = std::numeric_limits<double>::max();
    childPoly.max_x = childPoly.max_y = std::numeric_limits<double>::min();
    for (int i = 0; i < vertices.size(); i++) {
        childPoly.min_x = std::fmin(childPoly.min_x, mp->mesh_vertices[vertices[i]].p.x);
        childPoly.min_y = std::fmin(childPoly.min_y, mp->mesh_vertices[vertices[i]].p.y);
        childPoly.max_x = std::fmax(childPoly.max_x, mp->mesh_vertices[vertices[i]].p.x);
        childPoly.max_y = std::fmax(childPoly.max_y, mp->mesh_vertices[vertices[i]].p.y);
    }
    if (siblingPolyId == polygonId) {
        mp->mesh_polygons.push_back(childPoly);
    }
    else {
        mp->mesh_polygons[childPolyId] = childPoly;
    }

    int endpointIndex = (hitEIndex + 1) % polygon.vertices.size();
    if (!intersectsOnVertex && (polygon.polygons[endpointIndex] != -1)) {
        // Polygon on opposite side of edge has the order of the two endpoints reversed, 
        // so the vertex id defining the edge is also opposite
        int endpointVId = polygon.vertices[endpointIndex];

        bool foundVertex = false;
        pl::Polygon& neighborPoly = mp->mesh_polygons[polygon.polygons[endpointIndex]];
        for (int i = 0; i < neighborPoly.vertices.size(); i++) {
            int neighborVId = neighborPoly.vertices[i];
            if (neighborVId != endpointVId)
                continue;
            int nextIndex = (i + 1) % neighborPoly.vertices.size();
            // Next vertex should be other vertex on edge
            assert(neighborPoly.vertices[nextIndex] ==
                polygon.vertices[hitEIndex]);
            neighborPoly.vertices.insert(neighborPoly.vertices.begin() + nextIndex, hitVId);
            // Edge is split, so neighbor now has 2 neighbors instead of 1 there
            neighborPoly.polygons[nextIndex] = siblingPolyId;
            neighborPoly.polygons.insert(neighborPoly.polygons.begin() +
                ((nextIndex + 1) % neighborPoly.vertices.size()),
                childPolyId);
            foundVertex = true;
            break;
        }
        assert(foundVertex);
    }
}

void out_poly(pl::MeshPtr mp, vector<int>& vertices, std::ostream& s, bool newline) {
    s << "Polygon(";
    for (int i = 0; i < vertices.size(); i++) {
        s << mp->mesh_vertices[vertices[i]].p;
        if (i != (vertices.size() - 1))
            s << ", ";
    }
    s << ")";
    if (newline)
        s << std::endl;
}

void out_poly(pl::MeshPtr mp, vector<int>& vertices) {
    out_poly(mp, vertices, std::cout, true);
}

void out_poly(polygon_type poly) {
    std::cout << "Polygon(";
    for (int i = 0; i < poly.outer().size(); i++) {
        point_type p = poly.outer().at(i);
        std::cout << "(" << p.x() << ", " << p.y() << ")";
        if (i != (poly.outer().size() - 1))
            std::cout << ", ";
    }
    std::cout << ")" << std::endl;
}

void vertices_to_bg_poly(pl::MeshPtr mp, vector<int>& vertices, polygon_type& poly)
{
    for (int i = 0; i < vertices.size(); i++) {
        pl::Point& polyP = mp->mesh_vertices[vertices[i]].p;
        bg::append(poly.outer(), point_type(polyP.x, polyP.y));
    }
    bg::correct(poly);
}

inline bool skip_line(pl::MeshPtr mp, int polygonId, std::vector<std::pair<pl::Point, pl::Point>>& lines,
    queue<pair<int, vector<pair<pl::Point, pl::Point>>>>& split_queue) {
    lines.pop_back();
    if (lines.size() == 0)
        return false;
    split_queue.push(pair<int, vector<pair<pl::Point, pl::Point>>> { polygonId, lines });
    return true;
}

bool are_vertices_close(pl::MeshPtr mp, pl::Point p1, pl::Point p2) {
    std::cout.precision(10);
    double diff_x = std::abs(p1.x - p2.x);
    double diff_y = std::abs(p1.y - p2.y);
    if ((diff_x > 1e-8 && diff_x < 0.01) || (diff_y > 1e-8 && diff_y < 0.01)) {
        return true;
    }
    return false;
}

bool is_convex(pl::MeshPtr mp, vector<int> poly) {
    vector<pl::Point> temp_poly(poly.size());
    for (int i = 0; i < temp_poly.size(); i++)
        temp_poly[i] = mp->mesh_vertices[poly[i]].p;
    vector<double> cross_prods(temp_poly.size());
    bool sign = false;
    for (int i = 0; i < temp_poly.size(); i++) {
        int index2 = (i + 1) % temp_poly.size();
        int index3 = (i + 2) % temp_poly.size();
        double dx1 = temp_poly[index2].x - temp_poly[i].x;
        double dy1 = temp_poly[index2].y - temp_poly[i].y;
        double dx2 = temp_poly[index3].x - temp_poly[index2].x;
        double dy2 = temp_poly[index3].y - temp_poly[index2].y;
        double zcrossproduct = dx1 * dy2 - dy1 * dx2;
        cross_prods[i] = zcrossproduct;
        if (zcrossproduct != 0)
            sign = zcrossproduct > 0;
    }
    bool is_convex = false;
    for (int i = 1; i < temp_poly.size(); i++) {
        if (cross_prods[i] != 0 && (cross_prods[i] > 0) != sign) {
            /*std::cout.precision(20);
            out_poly(mp, poly);
            std::cout << cross_prods[i] << " " << sign << std::endl;
            exit(0);*/
            return false;
        }
    }
    return true;
}

int precision_skip_counter = 0;
void split_polygon(pl::MeshPtr mp, int polygonId, vector<pair<pl::Point, pl::Point>> lines, 
    vector<int>& parents, int max_extra_regions)
{
    int parent_id = polygonId;
    queue<pair<int, vector<pair<pl::Point, pl::Point>>>> split_queue;
    split_queue.push(pair<int, vector<pair<pl::Point, pl::Point>>> { polygonId, lines });
    int original_num_polys = mp->mesh_polygons.size();
    while (!split_queue.empty()) {
        if (max_extra_regions >= 0 && (mp->mesh_polygons.size() - original_num_polys) >= max_extra_regions)
            return;
        pair<int, vector<pair<pl::Point, pl::Point>>> pair = split_queue.front();
        polygonId = pair.first;
        lines = pair.second;
        split_queue.pop();

        pl::Polygon polygon = mp->mesh_polygons[polygonId];
        int line_index = rand() % lines.size();
        iter_swap(lines.begin() + line_index, lines.end() - 1);
        std::pair<pl::Point, pl::Point> line = lines.back();

        polygon_type poly;
        vertices_to_bg_poly(mp, polygon.vertices, poly);
        const point_type& poly_min = point_type(polygon.min_x, polygon.min_y);
        const point_type& poly_max = point_type(polygon.max_x, polygon.max_y);
        segment_type seg(point_type(line.first.x, line.first.y),
            point_type(line.second.x, line.second.y));
        if (!are_rectangles_overlap(point_type(std::fmin(seg.first.x(), seg.second.x()), std::fmin(seg.first.y(), seg.second.y())),
            point_type(std::fmax(seg.first.x(), seg.second.x()), std::fmax(seg.first.y(), seg.second.y())), poly_min, poly_max)) {
            skip_line(mp, polygonId, lines, split_queue);
            continue;
        }
        linestring_type input_ls;
        input_ls.push_back(point_type(line.first.x, line.first.y));
        input_ls.push_back(point_type(line.second.x, line.second.y));
        multi_linestring_type intersection;
        if (!bg::intersection(poly, input_ls, intersection)) {
            skip_line(mp, polygonId, lines, split_queue);
            continue;
        }
        if (intersection.size() != 1) {
            skip_line(mp, polygonId, lines, split_queue);
            continue;
        }
        assert(intersection.size() == 1);
        pl::Point entry_point{ intersection.front().front().x(), intersection.front().front().y() };
        pl::Point exit_point{ intersection.front().back().x(), intersection.front().back().y() };
        if (entry_point == exit_point) {
            skip_line(mp, polygonId, lines, split_queue);
            continue;
        }

        /*vector<int> old_vertices = polygon.vertices;
        std::cout << "Split " << polygonId << std::endl;
        std::cout << "Linjestykke(" << line.first << ", " << line.second << ")" << std::endl;
        out_poly(mp, old_vertices);*/

        int hitEIndex = -1;
        int hitEIndex2 = -1;
        bool intersectsOnVertex = false;
        bool intersectsOnVertex2 = false;
        bool modifiedVertices = false;
        //bool intersect_too_close = false;
        int original_v_count = mp->mesh_vertices.size();
        for (int i = 0; i < polygon.vertices.size(); i++) {
            int index2 = i + 1;
            if (index2 == polygon.vertices.size())
                index2 = 0;
            // Intersection is on vertex and we already have it from the other incident edge
            //if (hitEIndex == i || hitEIndex == index2)
                //continue;
            pl::Point p1 = mp->mesh_vertices[polygon.vertices[i]].p;
            pl::Point p2 = mp->mesh_vertices[polygon.vertices[index2]].p;
            pl::Point intersectP = lineLineIntersection(p1, p2, line.first, line.second);
            // The lines are parallel
            if (intersectP.x == -1)
                continue;
            bool intersectsP1 = intersectP == p1;
            // Deal with this for iteration when intersectsP2 instead
            if (intersectsP1)
                continue;
            bool intersectsP2 = intersectP == p2;
            if (!intersectsP2 && !onSegmentApprox(p1, intersectP, p2))
                continue;
            if (!intersectsP2) {
                pl::Vertex intersectV;
                intersectV.p = intersectP;
                //if (are_vertices_close(mp, p1, intersectP) || are_vertices_close(mp, p2, intersectP))
                    //intersect_too_close = true;
                //std::cout << "Intersect " << intersectP << " " << p1 << " " << p2 << std::endl;
                mp->mesh_vertices.push_back(intersectV);
                modifiedVertices = true;
            }
            //int tempHitIndex = intersectP2 ? index2 : i;
            if (hitEIndex == -1)
            {
                if (intersectsP2)
                    intersectsOnVertex = true;
                hitEIndex = i;
            }
            else
            {
                if (intersectsP2)
                    intersectsOnVertex2 = true;
                hitEIndex2 = i;
                // Now we have both intersection points
                break;
            }
        }
        if (hitEIndex == -1 || hitEIndex2 == -1) {
            assert(!modifiedVertices);
            skip_line(mp, polygonId, lines, split_queue);
            continue;
        }
        /*if (!intersectsOnVertex && !intersectsOnVertex2) {
            if (are_vertices_close(mp, mp->mesh_vertices.back().p, mp->mesh_vertices[mp->mesh_vertices.size() - 2].p)) {
                intersect_too_close = true;
            }
        }*/
        /*if (intersect_too_close) {
            precision_skip_counter++;
            if (!intersectsOnVertex)
                mp->mesh_vertices.pop_back();
            if (!intersectsOnVertex2)
                mp->mesh_vertices.pop_back();
            assert(original_v_count == mp->mesh_vertices.size());
            skip_line(mp, polygonId, lines, split_queue);
            continue;
        }*/
        // If intersection is on existing vertex set hitVId to that, otherwise add new vertex and set to that

        int hitVId = intersectsOnVertex ? polygon.vertices[(hitEIndex + 1) % polygon.vertices.size()] : (mp->mesh_vertices.size() - 1);
        int hitVId2 = intersectsOnVertex2 ? polygon.vertices[(hitEIndex2 + 1) % polygon.vertices.size()] : (mp->mesh_vertices.size() - 1);
        if (!intersectsOnVertex && !intersectsOnVertex2)
            hitVId--;

        vector<int> child_vertices1;
        vector<int> child_vertices2;
        polygon_type child_bg_poly1;
        polygon_type child_bg_poly2;
        create_half_split_vertices(mp, polygonId, polygonId,
            hitEIndex, hitEIndex2, hitVId, hitVId2,
            intersectsOnVertex, intersectsOnVertex2, child_vertices1);
        vertices_to_bg_poly(mp, child_vertices1, child_bg_poly1);

        create_half_split_vertices(mp, polygonId, mp->mesh_polygons.size() - 1,
            hitEIndex2, hitEIndex, hitVId2, hitVId,
            intersectsOnVertex2, intersectsOnVertex, child_vertices2);
        vertices_to_bg_poly(mp, child_vertices2, child_bg_poly2);
        if (max_extra_regions >= 0 && (bg::area(child_bg_poly1) < 0.01 || bg::area(child_bg_poly2) < 0.01)) {
            precision_skip_counter++;
            if (!intersectsOnVertex)
                mp->mesh_vertices.pop_back();
            if (!intersectsOnVertex2)
                mp->mesh_vertices.pop_back();
            assert(original_v_count == mp->mesh_vertices.size());
            skip_line(mp, polygonId, lines, split_queue);
            continue;
        }

        create_half_split_polygon(mp, polygonId, polygonId,
            hitEIndex, hitEIndex2, hitVId, hitVId2,
            intersectsOnVertex, intersectsOnVertex2);
        create_half_split_polygon(mp, polygonId, mp->mesh_polygons.size() - 1,
            hitEIndex2, hitEIndex, hitVId2, hitVId,
            intersectsOnVertex2, intersectsOnVertex);
        vector<int>& new_vertices1 = mp->mesh_polygons.back().vertices;
        vector<int>& new_vertices2 = mp->mesh_polygons[polygonId].vertices;
        assert(new_vertices1.size() >= 3);
        assert(new_vertices2.size() >= 3);
        parents[polygonId] = parent_id;
        parents.push_back(parent_id);
        /*std::cout << "First " << (mp->mesh_polygons.size() - 1) << std::endl;
        out_poly(mp, new_vertices1);
        std::cout << "Second " << polygonId << std::endl;
        out_poly(mp, new_vertices2);*/
        lines.pop_back();
        if (lines.size() == 0)
            continue;

        std::vector<std::pair<pl::Point, pl::Point>> copyLines;
        copyLines = lines;
        split_queue.push(std::pair<int, std::vector<std::pair<pl::Point, pl::Point>>> { mp->mesh_polygons.size() - 1, lines });
        split_queue.push(std::pair<int, std::vector<std::pair<pl::Point, pl::Point>>> { polygonId, copyLines });
    }
}

struct Convex_vertices_label_full
{
    int hub_id;
    pl::Convex_vertices_label label;
};

template<typename TValue>
struct key_value
{
    int key;
    TValue sanity;
    TValue value;
};

template<typename TValue>
inline bool operator<(const key_value<TValue>& lhs, const key_value<TValue>& rhs)
{
    if (lhs.key == rhs.key)
        return lhs.value < rhs.value;
    return lhs.key < rhs.key;
}

inline pair<int, int> ordered_pair(int value1, int value2)
{
    if (value1 < value2)
        return pair<int, int>(value1, value2);
    return pair<int, int>(value2, value1);
}

double dmin(double value1, double value2) {
    return value1 < value2 ? value1 : value2;
}

double dmax(double value1, double value2) {
    return value1 > value2 ? value1 : value2;
}

struct angle_index
{
    double angle;
    double dist;
    int index;
};

inline bool operator<(const angle_index& lhs, const angle_index& rhs)
{
    if (std::abs(lhs.angle - rhs.angle) < 1e-12)
        return lhs.dist < rhs.dist;
    return lhs.angle < rhs.angle;
}

int find_angle_index(vector<angle_index>& sorted_angles, double angle) {
    angle_index search_key = angle_index{ angle, -1 };
    vector<angle_index>::iterator lower_it = std::lower_bound(
        sorted_angles.begin(), sorted_angles.end(), search_key);
    vector<angle_index>::iterator upper_it = std::upper_bound(
        sorted_angles.begin(), sorted_angles.end(), search_key);
    for (vector<angle_index>::iterator cur_it = lower_it; cur_it <= upper_it; cur_it++) {
        if (cur_it == sorted_angles.end())
            continue;
        if (abs(angle - (*cur_it).angle) < 1e-12) {
            return (*cur_it).index;
        }
    }
    return -1;
}

bool raycast(pl::EBHL_poly_query_v2* ebhlQueryV2, pl::MeshPtr mp, pl::Point start, pl::Point goal, 
    const pl::PointLocation& start_pl, const pl::PointLocation& goal_pl, pl::Point& intersection) {
    //const pl::PointLocation& start_pl = get_point_location_in_search(start, mp, false);
    //const pl::PointLocation& goal_pl = get_point_location_in_search(goal, mp, false);
    bool intersects = false;
    bool is_visible = ebhlQueryV2->isVisible(start, goal, start_pl, goal_pl, intersects, intersection, false);
    assert(intersects);
    return is_visible;
}

/*
 * Construct of EHL as detailed in Offline Preprocessing 3-5.
 * Using pre-constructed visibility graph and hub labels, EHL is built.
 *
 */
void build_ebhl(string dir, string map, int max_extra_regions, int max_tree_depth) {
    srand(2);
    std::streamsize ss = std::cout.precision();

    std::cout << "Building EHL ..." << map << endl;
    //load mesh
    string mesh_path = "dataset/merged-mesh/" + dir + "/" + map + "-merged.mesh";
    ifstream meshfile(mesh_path);
    mp = new pl::Mesh(meshfile);
    // mark obstacle edge;
    mp->pre_compute_obstacle_edge_on_vertex();
    // mark valid turning point;
    string grid_path = "dataset/grid/" + dir + "/" + map + ".map";
    mp->mark_turning_point(grid_path.c_str());
    //mp->mark_turning_point_polygon();
    //turning point is the actual point which would be used in search
    vector<pl::Point> turning_point;
    //corresponding vertices number in mesh_vertices
    vector<int> turning_vertices;
    //vertice location in polygon?
    vector<pl::PointLocation> turning_vertices_location;

    vector<pl::Point> obstacle_vertices;
    /*auto hash = [](const pl::Point& p) { return std::hash<int>()(p.x) * 31 + std::hash<int>()(p.y); };
    auto equal = [](const pl::Point& p1, const pl::Point& p2) { return p1 == p2; };
    std::unordered_set<pl::Point, decltype(hash), decltype(equal)> concave_vertices(8, hash, equal);*/
    vector<pl::Point> concave_points;
    vector<int> concave_vertices;
    vector<pl::PointLocation> concave_vertices_location;
    warthog::timer timer = warthog::timer();
    timer.start();
    int id = 0;
    int poly = 0;
    for (pl::Vertex& v : mp->mesh_vertices) {
        if (v.is_corner)
            obstacle_vertices.push_back(v.p);
        if (v.is_corner && !v.is_turning_vertex) {
            bool is_obstacles = true;
            for (int i = v.polygons.size() - 1; i >= 0; i--) {
                int polygon = v.polygons[i];
                if (polygon != -1 && is_obstacles) {
                    poly = polygon;
                    concave_points.push_back(v.p);
                    concave_vertices.push_back(id);
                    pl::PointLocation location = { pl::PointLocation::ON_NON_CORNER_VERTEX, poly, -1, id, -1 };
                    concave_vertices_location.push_back(location);
                    is_obstacles = false;
                    if (!v.is_ambig)
                        break;
                }
                else if (polygon == -1)
                    is_obstacles = true;
            }
        }
        if (v.is_turning_vertex && !v.is_ambig) {
            for (int polygon : v.polygons) {
                if (polygon != -1) {
                    //                    p.polygons.insert(polygon);
                    poly = polygon;
                }
            }

            // there is some issue here, old implementation assume that these vertex always inside one polygon only. it doesnt
            // use the correct type actually, I fix it here manually assign a random adjacent polygon
            pl::PointLocation location = { pl::PointLocation::ON_CORNER_VERTEX_UNAMBIG, poly, -1, id, -1 };
            turning_vertices_location.push_back(location);
            turning_vertices.push_back(id);
            turning_point.push_back(mp->mesh_vertices[id].p);
        }
        id++;
    }

    //load mesh
    string ebhl_mesh_path = "dataset/merged-mesh/" + dir + "/" + map + "-merged.mesh";
    ifstream ebhl_meshfile(ebhl_mesh_path);
    pl::MeshPtr ebhl_mp = new pl::Mesh(ebhl_meshfile);
    pl::EBHL_poly_query_v2* ebhlQueryV2 = new pl::EBHL_poly_query_v2(mp, NULL, turning_point, vector<pl::Point>());

    string vg_path = "dataset/visibility_graph/" + dir + "/" + map + ".vis";
    pl::Graph vg;
    vector<vector<int>> visibility_graph;
    vg.load_graph(vg_path, visibility_graph);
    vector<pair<pl::Point, pl::Point>> all_lines;
    assert(turning_point.size() == visibility_graph.size());

    vector<pl::Point> all_point = turning_point;
    all_point.insert(all_point.end(), concave_points.begin(), concave_points.end());
    vector<int> all_vertices = turning_vertices;
    all_vertices.insert(all_vertices.end(), concave_vertices.begin(), concave_vertices.end());
    vector<pl::PointLocation> all_vertices_location = turning_vertices_location;
    all_vertices_location.insert(all_vertices_location.end(), concave_vertices_location.begin(), concave_vertices_location.end());
    vector<int> vertice_mapper(mp->mesh_vertices.size());
    for (int i = 0; i < turning_vertices.size(); i++) {
        vertice_mapper[turning_vertices[i]] = i;
    }
    for (int i = 0; i < concave_vertices.size(); i++) {
        vertice_mapper[concave_vertices[i]] = turning_vertices.size() + i;
    }
    /*vector<pl::visibleSearchInstance*> visibility_search;
    for (int i = 0; i < omp_get_max_threads(); i++) {
        visibility_search.push_back(new pl::visibleSearchInstance(mp));
    }*/
    pl::visibleSearchInstance* VS = new pl::visibleSearchInstance(mp);
    for (int i = 0; i < concave_points.size(); i++) {
        vector<int> visible_vertices;
        VS->search_visible_vertices(concave_points[i], concave_vertices_location[i], visible_vertices);
        vector<int> mapped_vertices;
        for (int j = 0; j < visible_vertices.size(); j++) {
            //assert(!mp->mesh_vertices[visible_vertices[j]].is_ambig);
            if (visible_vertices[i] == concave_vertices[i])
                continue;
            mapped_vertices.push_back(vertice_mapper[visible_vertices[j]]);
            assert(all_point[turning_point.size() + i] == concave_points[i]);
            assert(visibility_graph.size() == turning_point.size() + i);
            assert(mp->mesh_vertices[visible_vertices[j]].p == all_point[mapped_vertices[j]]);
            visibility_graph[mapped_vertices[j]].push_back(turning_point.size() + i);
        }
        visibility_graph.push_back(mapped_vertices);
    }

    vector<vector<angle_index>> all_sorted_angles(visibility_graph.size());
    for (int i = 0; i < visibility_graph.size(); i++) {
        pl::Point start = mp->mesh_vertices[all_vertices[i]].p;
        vector<double> angles;
        vector<double> distances;
        for (int j = 0; j < visibility_graph[i].size(); j++) {
            int v_index2 = visibility_graph[i][j];
            if (i == v_index2) {
                angles.push_back(-100);
                distances.push_back(-1);
                continue;
            }
            pl::Point goal = mp->mesh_vertices[all_vertices[v_index2]].p;
            distances.push_back(start.distance(goal));
            double angle = atan2(goal.y - start.y, goal.x - start.x);
            if (angle < 0) {
                angle += 2.0 * M_PI;
            }
            angles.push_back(angle);
        }
        vector<angle_index>& sorted_angles = all_sorted_angles[i];
        for (int j = 0; j < angles.size(); j++)
            sorted_angles.push_back(angle_index{ angles[j], distances[j], visibility_graph[i][j] });
        sort(sorted_angles.begin(), sorted_angles.end());
    }
    for (int i = 0; i < all_sorted_angles.size(); i++) {
        pl::Point start = all_point[i];
        vector<angle_index>& angles = all_sorted_angles[i];
        double last_angle = -1;
        for (int j = 0; j < all_sorted_angles[i].size(); j++) {
            int v_index2 = all_sorted_angles[i][j].index;
            if (i == v_index2)
                continue;
            pl::Point goal = all_point[v_index2];
            double angle = angles[j].angle;
            if (std::abs(angle - last_angle) < 1e-12)
                continue;
            last_angle = angle;
            if (find_angle_index(all_sorted_angles[v_index2], angle) != -1)
                continue;
            double opp_angle = angle - M_PI;
            if (opp_angle < 0) {
                opp_angle += 2.0 * M_PI;
            }
            int opp_index = i;
            int prev_opp_index = -1;
            int num_opp = 0;
            while (true) {
                int temp_index = find_angle_index(all_sorted_angles[opp_index], opp_angle);
                if (temp_index == -1)
                    break;
                prev_opp_index = opp_index;
                opp_index = temp_index;
                num_opp++;
            }
            pl::Point intersection;
            pl::Point intersection2;
            bool is_goal_concave = v_index2 >= turning_point.size();
            if (is_goal_concave)
                intersection = mp->mesh_vertices[all_vertices[v_index2]].p;
            else
                bool is_visible = raycast(ebhlQueryV2, ebhl_mp, start, goal, 
                    all_vertices_location[i], all_vertices_location[v_index2], intersection);
            if (num_opp > 0) {
                if (angle >= M_PI)
                    continue;
                if (opp_index >= turning_point.size())
                    intersection2 = mp->mesh_vertices[all_vertices[opp_index]].p;
                else
                    raycast(ebhlQueryV2, ebhl_mp, start, all_point[opp_index],
                        all_vertices_location[i], all_vertices_location[opp_index], intersection2);
                all_lines.push_back(pair<pl::Point, pl::Point> { intersection, intersection2 });
                continue;
            }

            if (goal == intersection)
                continue;
            all_lines.push_back(pair<pl::Point, pl::Point> { goal, intersection });
        }
    }
    /*for (int i = 0; i < ebhl_mp->mesh_polygons.size(); i++)
    {
        pl::Polygon& polygon = ebhl_mp->mesh_polygons[i];
        for (int j = 0; j < polygon.polygons.size(); j++)
        {
            if (polygon.polygons[j] != -1)
                continue;
            int index1 = (j == 0 ? polygon.vertices.size() : j) - 1;
            auto line = pair<pl::Point, pl::Point>
            {
                ebhl_mp->mesh_vertices[polygon.vertices[index1]].p,
                ebhl_mp->mesh_vertices[polygon.vertices[j]].p
            };
            all_lines.push_back(line);
        }
    }*/
    std::cout << all_lines.size() << std::endl;

    /*vector<pair<pl::Point, pl::Point>> all_naive_lines;
    for (int i = 0; i < obstacle_vertices.size(); i++) {
        for (int j = i + 1; j < obstacle_vertices.size(); j++) {
            all_naive_lines.push_back(pair<pl::Point, pl::Point> 
            { obstacle_vertices[i], obstacle_vertices[j] });
        }
    }

    vector<pair<pl::Point, pl::Point>> out_lines = all_lines;
    vector<pair<pl::Point, pl::Point>> out_naive_lines = all_naive_lines;
    int obstacle_start_index = out_lines.size();
    int obstacle_naive_start_index = out_naive_lines.size();
    for (int i = 0; i < ebhl_mp->mesh_polygons.size(); i++)
    {
        pl::Polygon& polygon = ebhl_mp->mesh_polygons[i];
        for (int j = 0; j < polygon.polygons.size(); j++)
        {
            if (polygon.polygons[j] != -1)
                continue;
            int index1 = (j == 0 ? polygon.vertices.size() : j) - 1;
            auto line = pair<pl::Point, pl::Point>
            {
                ebhl_mp->mesh_vertices[polygon.vertices[index1]].p,
                ebhl_mp->mesh_vertices[polygon.vertices[j]].p
            };
            out_lines.push_back(line);
            out_naive_lines.push_back(line);
        }
    }

    int num_lines = out_lines.size();
    int num_files = 0;
    while (num_lines > 0)
    {
        ofstream subdivision_outfile("dataset/lines/" + dir + "/" + map + " " + std::to_string(num_files) + ".txt");
        //subdivision_outfile << "Execute[{";
        for (int i = num_files * 8000; i < std::min(num_lines, (num_files + 1) * 8000); i++)
        {
            subdivision_outfile << "\"a" << i << "=Segment(" << out_lines[i].first << ", " << out_lines[i].second << ")\"";
            subdivision_outfile << std::endl;//", ";
            if (i >= obstacle_start_index)
                continue;
            subdivision_outfile << "\"SetColor(a" << i << ",Red)\"";
            subdivision_outfile << std::endl;
        }
        //subdivision_outfile << "}]";
        num_files++;
        num_lines -= 8000;
    }

    num_lines = out_naive_lines.size();
    num_files = 0;
    while (num_lines > 0)
    {
        ofstream subdivision_outfile("dataset/naive-lines/" + dir + "/" + map + " " + std::to_string(num_files) + ".txt");
        //subdivision_outfile << "Execute[{";
        for (int i = num_files * 8000; i < std::min(num_lines, (num_files + 1) * 8000); i++)
        {
            subdivision_outfile << "\"a" << i << "=" << (i < obstacle_naive_start_index ? "Line" : "Segment") << 
                "(" << out_naive_lines[i].first << ", " << out_naive_lines[i].second << ")\"";
            subdivision_outfile << std::endl;//", ";
            if (i >= obstacle_naive_start_index)
                continue;
            subdivision_outfile << "\"SetColor(a" << i << ",Red)\"";
            subdivision_outfile << std::endl;
        }
        //subdivision_outfile << "}]";
        num_files++;
        num_lines -= 8000;
    }*/

    int progress = 0;
    vector<int> counts(all_lines.size());
    int num_intersects = 0;
#pragma omp parallel
    {
        const int thread_count = omp_get_num_threads();
        const int thread_id = omp_get_thread_num();
        const int node_count = all_lines.size();
        int node_begin = (node_count * thread_id) / thread_count;
        int node_end = (node_count * (thread_id + 1)) / thread_count;
        for (int i = node_begin; i < node_end; i++) {
            pair<pl::Point, pl::Point> line1 = all_lines[i];
            int count = 0;
            for (int j = i + 1; j < node_count; j++) {
                pair<pl::Point, pl::Point> line2 = all_lines[j];
                if (doIntersect(line1.first, line1.second, all_lines[j].first, all_lines[j].second)) {
                    pl::Point intersectP = lineLineIntersection(line1.first, line1.second, 
                        all_lines[j].first, all_lines[j].second);
                    if (intersectP != line1.first && intersectP != line1.second &&
                        intersectP != all_lines[j].first && intersectP != all_lines[j].second)
                        count++;
                    else if (intersectP.x == -1)
                        count--;
                }
            }
            counts[i] = count;
#pragma omp critical
            {
                ++progress;
                if (progress % 100 == 0) {
                    double ratio = (double)progress / node_count * 100.0;
                    std::cout << "Progress: [" << progress << "/" << node_count << "] "
                        << std::setprecision(3) << ratio << "% \r";
                    std::cout.flush();
                }
            }
        }
    }
    for (int i = 0; i < counts.size(); i++)
        num_intersects += counts[i];
    int estimate_regions = num_intersects + all_lines.size() + 1;
    std::cout << "Num line intersections: " << num_intersects << std::endl;
    std::cout << "Num polygons in optimal subdivision: " << estimate_regions << std::endl;
    std::cout << "Num polygons initially: " << ebhl_mp->mesh_polygons.size() << std::endl;
    std::cout << (log2(num_intersects + all_lines.size()) / log2(obstacle_vertices.size())) << std::endl;
    /*if (estimate_regions > 1000) {
        ofstream outfile2;
        outfile2.open("temp_output2.txt", std::ios_base::app);
        outfile2 << (log2(num_intersects + all_lines.size()) / log2(obstacle_vertices.size())) << std::endl;
    }*/
    /*if (estimate_regions > 100000) {
        std::cout << "Too many regions" << std::endl;
        return;
    }*/
    /*ofstream outfile2;
    outfile2.open("temp_output3.txt", std::ios_base::app);
    outfile2 << map << std::endl;*/

    progress = 0;
    vector<vector<int>> node_to_lines(ebhl_mp->mesh_polygons.size());
#pragma omp parallel
    {
        const int thread_count = omp_get_num_threads();
        const int thread_id = omp_get_thread_num();
        const int node_count = ebhl_mp->mesh_polygons.size();
        int node_begin = (node_count * thread_id) / thread_count;
        int node_end = (node_count * (thread_id + 1)) / thread_count;
        for (int i = node_begin; i < node_end; i++) {
            pl::Polygon meshPoly = ebhl_mp->mesh_polygons[i];
            polygon_type poly;
            for (int i = 0; i < meshPoly.vertices.size(); i++) {
                pl::Vertex polyV = ebhl_mp->mesh_vertices[meshPoly.vertices[i]];
                bg::append(poly.outer(), point_type(polyV.p.x, polyV.p.y));
            }
            bg::correct(poly);
            const point_type& poly_min = point_type(meshPoly.min_x, meshPoly.min_y);
            const point_type& poly_max = point_type(meshPoly.max_x, meshPoly.max_y);
            vector<int>& lines = node_to_lines[i];
            for (int j = 0; j < all_lines.size(); j++) {
                segment_type line(point_type(all_lines[j].first.x, all_lines[j].first.y),
                    point_type(all_lines[j].second.x, all_lines[j].second.y));
                if (!are_rectangles_overlap(point_type(dmin(line.first.x(), line.second.x()), dmin(line.first.y(), line.second.y())),
                    point_type(dmax(line.first.x(), line.second.x()), dmax(line.first.y(), line.second.y())), poly_min, poly_max))
                    continue;
                linestring_type input_ls;
                input_ls.push_back(point_type(line.first.x(), line.first.y()));
                input_ls.push_back(point_type(line.second.x(), line.second.y()));
                multi_linestring_type intersection;
                if (bg::intersection(poly, input_ls, intersection) && intersection.size() == 1) {
                    lines.push_back(j);
                }
            }
#pragma omp critical
            {
                ++progress;
                if (progress % 100 == 0) {
                    double ratio = (double)progress / node_count * 100.0;
                    std::cout << "Progress: [" << progress << "/" << node_count << "] "
                        << std::setprecision(3) << ratio << "% \r";
                    std::cout.flush();
                    std::cout.precision(ss);
                }
            }
        }
    }

    set<key_value<int>> subdivide_queue;
    for (int i = 0; i < ebhl_mp->mesh_polygons.size(); i++) {
        subdivide_queue.insert(key_value<int> { -node_to_lines[i].size(), 0, i });
    }

    vector<int> mesh_poly_parents(ebhl_mp->mesh_polygons.size());
    for (int i = 0; i < ebhl_mp->mesh_polygons.size(); i++) {
        mesh_poly_parents[i] = i;
    }

    //int max_num_polys = ebhl_mp->mesh_polygons.size();
    while (subdivide_queue.size() > 0) {
        set<key_value<int>>::iterator it = --subdivide_queue.end();
        key_value<int> kv = *it;
        int poly_to_split = kv.value;
        subdivide_queue.erase(it);

        vector<int>& line_indices = node_to_lines[poly_to_split];
        if (line_indices.size() == 0)
            continue;
        //if (ebhl_mp->mesh_polygons.size() + line_indices.size() > max_num_polys)
            //break;
        std::vector<std::pair<pl::Point, pl::Point>> lines(line_indices.size());
        for (int i = 0; i < line_indices.size(); i++) {
            lines[i] = all_lines[line_indices[i]];
        }

        split_polygon(ebhl_mp, poly_to_split, lines, mesh_poly_parents, max_extra_regions);
    }
    std::cout << "num_polys: " << ebhl_mp->mesh_polygons.size() << "polys" << std::endl;
    std::cout << "num skipped: " << precision_skip_counter << std::endl;
    std::cout << "left in queue: " << subdivide_queue.size() << std::endl;
    //std::cout << "ratio: " << (ebhl_mp->mesh_polygons.size() / (double)estimate_regions) << std::endl;
    /*if (ebhl_mp->mesh_polygons.size() > 2000) {
        ofstream outfile;
        outfile.open("temp_output.txt", std::ios_base::app);
        outfile << (ebhl_mp->mesh_polygons.size() / (double)estimate_regions) << std::endl;
    }*/

    /*std::vector<std::pair<pl::Point, pl::Point>> lines;
    lines.push_back(std::pair<pl::Point, pl::Point>(pl::Point{ 1.5, 31.0 }, pl::Point{ 1.5, 38.0 }));
    lines.push_back(std::pair<pl::Point, pl::Point>(pl::Point{ 1.0, 33.0 }, pl::Point{ 2.0, 33.0 }));
    std::vector<int> tempNeighbors = ebhl_mp->mesh_polygons[3].polygons;
    split_polygon(ebhl_mp, 3, lines);
    std::cout << ebhl_mp->mesh_vertices.size() << " " << ebhl_mp->mesh_polygons.size() << std::endl;
    for (int n : tempNeighbors) {
        if (n == -1)
            continue;
        pl::Polygon firstPoly = ebhl_mp->mesh_polygons[n];
        for (int i = 0; i < firstPoly.vertices.size(); i++)
        {
            std::cout << ebhl_mp->mesh_vertices[firstPoly.vertices[i]].p << " ";
        }
        std::cout << "\n";
        for (int i = 0; i < firstPoly.polygons.size(); i++)
        {
            std::cout << firstPoly.polygons[i] << " ";
        }
        std::cout << "\n";
    }
    */
    ebhl_mp->calculate_from_polygons();
    ofstream ebhl_outfile("dataset/ebhl-mesh/" + dir + "/" + map + "-merged.mesh");
    ebhl_mp->print_mesh(ebhl_outfile);
    // mark obstacle edge;
    //ebhl_mp->pre_compute_obstacle_edge_on_vertex();

    /*int num_polys = ebhl_mp->mesh_polygons.size();
    int num_files = 0;
    while (num_polys > 0)
    {
        ofstream subdivision_outfile("dataset/polys/" + dir + "/" + map + " " + std::to_string(num_files) + ".txt");
        //subdivision_outfile << "Execute[{";
        for (int i = num_files * 8000; i < std::min(num_polys, (num_files + 1) * 8000); i++)
        {
            subdivision_outfile << "\"";
            out_poly(ebhl_mp, ebhl_mp->mesh_polygons[i].vertices, subdivision_outfile, false);
            subdivision_outfile << "\"";
            if (i != ebhl_mp->mesh_polygons.size() - 1)
                subdivision_outfile << std::endl;//", ";
        }
        //subdivision_outfile << "}]";
        num_files++;
        num_polys -= 8000;
    }*/

    auto ebhl = new pl::EBHL_poly(ebhl_mp);
    ebhl->lambda = max_extra_regions + 1;
    ebhl->store_visibility = max_tree_depth == 0;
    ebhl->all_fully_visible = max_extra_regions < 0;
    ebhl->mesh_poly_parents = mesh_poly_parents;

    //loading hub label
    PLabel lab;
    string p_label = "dataset/hub_label/" + dir + "/" + map + ".label";
    lab.load_labels(p_label.c_str());
    string order = "dataset/hub_label/" + dir + "/" + map + ".order";
    ifstream order_ifs(order.c_str());
    vector<NodeID> label_rank(numOfVertices);
    vector<NodeID> label_inv(numOfVertices);
    for (int i = 0; i < numOfVertices; ++i) {
        NodeID tv;
        order_ifs >> tv;
        label_rank[tv] = i;
        label_inv[i] = tv;
    }

    //Pruning 4: filtering dead-end label;
    vector<vector<raw_label>> r_label = lab.get_raw_label_list(label_inv);
    unsigned label_size = 0;
    for (auto fl : r_label) {
        label_size += fl.size();
    }
    std::cout << "Before filtering label size: " << label_size << std::endl;

    vector<vector<raw_label>> f_label;
    for (unsigned i = 0; i < r_label.size(); i++) {
        int current_vertex_id = turning_vertices[i];
        vector<raw_label> filtered_labels;
        for (unsigned j = 0; j < r_label[i].size(); j++) {
            int current_hub_id = turning_vertices[label_inv[r_label[i][j].hub_id]];
            int current_predecessor = turning_vertices[r_label[i][j].label_predecessor];
            int current_first_node = turning_vertices[r_label[i][j].label_first_node];
            if (current_hub_id == current_vertex_id) {
                filtered_labels.push_back(r_label[i][j]);
            }
            else {
                if (current_predecessor != current_vertex_id) {
                    if (!is_valid_turn_edge(current_predecessor, current_vertex_id)) {
                        continue;
                    }
                }
                else {
                    std::cout << "current predecessor should not equal to current vertex id " << std::endl;
                }
                if (current_first_node != current_hub_id) {
                    if (!is_valid_turn_edge(current_first_node, current_hub_id)) {
                        continue;
                    }
                }
                else {
                    std::cout << "current first node should not equal to current vertex id " << std::endl;
                }
                filtered_labels.push_back(r_label[i][j]);
            }
        }
        filtered_labels.shrink_to_fit();
        f_label.push_back(filtered_labels);
    }
    f_label.shrink_to_fit();
    label_size = 0;
    for (auto fl : f_label) {
        label_size += fl.size();
    }
    std::cout << "After filtering label size: " << label_size << std::endl;

    vector<Convex_vertices_label_full> flat_label;
    vector<vector<int>> f_label_to_id(f_label.size());
    for (unsigned i = 0; i < f_label.size(); i++) {
        vector<int> inner(f_label[i].size());
        for (unsigned j = 0; j < f_label[i].size(); j++) {
            inner[j] = flat_label.size();
            polyanya::Convex_vertices_label convex_label =
                polyanya::Convex_vertices_label{ i,f_label[i][j].label_distance,false, f_label[i][j].label_predecessor };
            flat_label.push_back(Convex_vertices_label_full { f_label[i][j].hub_id, convex_label });
        }
        f_label_to_id[i] = inner;
    }

    /*int map_height;
    int map_width;
    mp->get_grid_width_height(map_width, map_height);*/
    //superimpose mesh for the whole map
    ebhl->initialize_poly_map();

    //find all the triangle successors and insert into a unique set for computation of polygon later
    vector<pl::visibleAreaSearchInstance* > thread_search(omp_get_max_threads());
    for (unsigned i = 0; i < thread_search.size(); i++) {
        thread_search[i] = new pl::visibleAreaSearchInstance(mp);
    }
    vector< pair <vis_poly, vis_poly>> boost_poly(turning_point.size());
    vector<vector<int>> all_visible(ebhl->mp->mesh_polygons.size());
    vector<vector<int>> all_partial_visible(ebhl->mp->mesh_polygons.size());
    vector<vector<int>> all_fully_visible(ebhl->mp->mesh_polygons.size());

    //finding all the visible areas for the given map and pruning 1 is applied
    progress = 0;
    {
        printf("Using %d threads\n", omp_get_max_threads());
#pragma omp parallel
        {
            const int thread_count = omp_get_num_threads();
            const int thread_id = omp_get_thread_num();
            const int node_count = turning_point.size();
            int node_begin = (node_count * thread_id) / thread_count;
            int node_end = (node_count * (thread_id + 1)) / thread_count;
            auto search = thread_search[thread_id];
            for (int source_node = node_begin; source_node < node_end; ++source_node) {
                search->search_non_taut_visible_area(turning_vertices[source_node],
                    turning_vertices_location[source_node], boost_poly[source_node].first.boost_poly,
                    boost_poly[source_node].second.boost_poly);
                //finds the two visible polygon for each convex vertex and stores it as two boost polygons
                bg::envelope(boost_poly[source_node].first.boost_poly, boost_poly[source_node].first.bounding_box);
                bg::dsv(boost_poly[source_node].first.bounding_box);
                bg::envelope(boost_poly[source_node].second.boost_poly, boost_poly[source_node].second.bounding_box);
                bg::dsv(boost_poly[source_node].second.bounding_box);
#pragma omp critical
                {
                    ++progress;
                    if (progress % 100 == 0) {
                        double ratio = (double)progress / node_count * 100.0;
                        std::cout << "Progress: [" << progress << "/" << node_count << "] "
                            << std::setprecision(3) << ratio << "% \r";
                        std::cout.flush();
                        std::cout.precision(ss);
                    }
                }
            }
        }
    }
    //find the hub nodes and via labels for each poly
    progress = 0;
    {
        printf("Using %d threads\n", omp_get_max_threads());
#pragma omp parallel
        {
            const int thread_count = omp_get_num_threads();
            const int thread_id = omp_get_thread_num();
            const int node_count = ebhl->poly_labels.size();
            int node_begin = (node_count * thread_id) / thread_count;
            int node_end = (node_count * (thread_id + 1)) / thread_count;
            for (int source_node = node_begin; source_node < node_end; ++source_node) {
                // compute visible and partial visible vertices for each poly.
                vector<int> visible_vertices;
                vector<int> partial_visible;
                pl::Polygon meshPoly = ebhl->mp->mesh_polygons[source_node];
                vector<pl::Point> temp_poly(meshPoly.vertices.size());
                pl::Point poly_center = pl::Point{ 0, 0 };
                for (int i = 0; i < meshPoly.vertices.size(); i++) {
                    temp_poly[i] = ebhl->mp->mesh_vertices[meshPoly.vertices[i]].p;
                    poly_center = poly_center + temp_poly[i];
                }
                poly_center.x /= temp_poly.size();
                poly_center.y /= temp_poly.size();
                for (int i = 0; i < temp_poly.size(); i++) {
                    double diff_length = poly_center.distance(temp_poly[i]);
                    pl::Point diff = poly_center - temp_poly[i];
                    pl::Point diff_dir = pl::Point{ diff.x / diff_length, diff.y / diff_length };
                    temp_poly[i] = temp_poly[i] + (1e-8 * diff_dir);
                }

                polygon_type poly;
                for (int i = 0; i < temp_poly.size(); i++) {
                    bg::append(poly.outer(), point_type(temp_poly[i].x, temp_poly[i].y));
                }
                bg::correct(poly);

                const point_type& poly_min = point_type(meshPoly.min_x, meshPoly.min_y);
                const point_type& poly_max = point_type(meshPoly.max_x, meshPoly.max_y);
                //checks whether it is partially or fully visible for first visible polygon
                for (unsigned j = 0; j < boost_poly.size(); j++) {
                    bool already_added_vertex = false;
                    const point_type& min = boost_poly[j].first.bounding_box.min_corner();
                    const point_type& max = boost_poly[j].first.bounding_box.max_corner();

                    if (are_rectangles_overlap(min, max, poly_min, poly_max)) {
                        if (bg::covered_by(boost_poly[j].first.boost_poly, poly)) {
                            partial_visible.push_back(j);
                            already_added_vertex = true;
                        }
                        else {
                            if (bg::intersects(poly, boost_poly[j].first.boost_poly)) {
                                bool all_points_inside_poly = true;
                                for (const auto& gp : poly.outer()) {
                                    if (!bg::covered_by(gp, boost_poly[j].first.boost_poly)) {
                                        all_points_inside_poly = false;
                                        break;
                                    }
                                }
                                if (bg::covered_by(poly, boost_poly[j].first.boost_poly) && all_points_inside_poly) {
                                    visible_vertices.push_back(j);
                                }
                                else {
                                    /*std::cout.precision(20);
                                    out_poly(ebhl->mp, meshPoly.vertices);
                                    out_poly(boost_poly[j].first.boost_poly);
                                    std::cout << ebhl->mp->mesh_vertices[turning_vertices[j]].p << std::endl;
                                    exit(0);*/
                                    partial_visible.push_back(j);
                                }
                                already_added_vertex = true;
                            }
                            else {
                                bool all_points_inside_poly = true;
                                for (const auto& gp : poly.outer()) {
                                    if (!bg::covered_by(gp, boost_poly[j].first.boost_poly)) {
                                        all_points_inside_poly = false;
                                        break;
                                    }
                                }
                                if (all_points_inside_poly) {
                                    visible_vertices.push_back(j);
                                    already_added_vertex = true;

                                }

                            }

                        }
                    }
                    if (already_added_vertex) { continue; }

                    const point_type& second_min = boost_poly[j].second.bounding_box.min_corner();
                    const point_type& second_max = boost_poly[j].second.bounding_box.max_corner();
                    //checks whether its visible for second polygon
                    if (are_rectangles_overlap(second_min, second_max, poly_min, poly_max)) {
                        if (bg::covered_by(boost_poly[j].second.boost_poly, poly)) {
                            partial_visible.push_back(j);
                        }
                        else {
                            if (bg::intersects(poly, boost_poly[j].second.boost_poly)) {
                                bool all_points_inside_poly = true;
                                for (const auto& gp : poly.outer()) {
                                    if (!bg::covered_by(gp, boost_poly[j].second.boost_poly)) {
                                        all_points_inside_poly = false;
                                        break;
                                    }
                                }
                                if (bg::covered_by(poly, boost_poly[j].second.boost_poly) && all_points_inside_poly) {
                                    visible_vertices.push_back(j);

                                }
                                else {
                                    /*std::cout.precision(20);
                                    out_poly(ebhl->mp, meshPoly.vertices);
                                    out_poly(boost_poly[j].second.boost_poly);
                                    std::cout << ebhl->mp->mesh_vertices[turning_vertices[j]].p << std::endl;
                                    exit(0);*/
                                    partial_visible.push_back(j);
                                }
                            }
                            else {
                                bool all_points_inside_poly = true;
                                for (const auto& gp : poly.outer()) {
                                    if (!bg::covered_by(gp, boost_poly[j].second.boost_poly)) {
                                        all_points_inside_poly = false;
                                        break;
                                    }
                                }
                                if (all_points_inside_poly) {
                                    visible_vertices.push_back(j);
                                }
                            }
                        }
                    }
                }


                visible_vertices.shrink_to_fit();
                partial_visible.shrink_to_fit();
                std::sort(visible_vertices.begin(), visible_vertices.end());
                std::sort(partial_visible.begin(), partial_visible.end());
                all_visible[source_node] = partial_visible;
                all_visible[source_node].insert(all_visible[source_node].end(), visible_vertices.begin(), visible_vertices.end());
                std::sort(all_visible[source_node].begin(), all_visible[source_node].end());
                all_partial_visible[source_node] = partial_visible;
                all_fully_visible[source_node] = visible_vertices;

#pragma omp critical
                {
                    ++progress;
                    if (progress % 100 == 0) {
                        double ratio = (double)progress / node_count * 100.0;
                        std::cout << "Progress: [" << progress << "/" << node_count << "] "
                            << std::setprecision(3) << ratio << "% \r";
                        std::cout.flush();
                        std::cout.precision(ss);
                    }
                }

            }
        }
    }

    int original_num_polys = all_visible.size();
    int average_num_visible = 0;
    for (int i = 0; i < all_visible.size(); i++) {
        average_num_visible += all_visible[i].size();
    }
    std::cout << "Avg num visible: " << ((double)average_num_visible / all_visible.size()) << " vertices" << std::endl;

    vector<vector<int>> all_convex_label_ids(all_visible.size());
    vector<int> original_costs(ebhl->mp->mesh_polygons.size(), 0);
    progress = 0;
    {
        printf("Using %d threads\n", omp_get_max_threads());
#pragma omp parallel
        {
            const int thread_count = omp_get_num_threads();
            const int thread_id = omp_get_thread_num();
            const int node_count = all_visible.size();
            int node_begin = (node_count * thread_id) / thread_count;
            int node_end = (node_count * (thread_id + 1)) / thread_count;
            for (int source_node = node_begin; source_node < node_end; ++source_node) {
                std::map<int, polyanya::Hub_label> label_mapper;
                std::map<int, vector<int>> label_id_mapper;

                vector<int>& visible_vertices = all_fully_visible[source_node];
                vector<int>& partial_visible = all_partial_visible[source_node];
                ebhl->poly_visible[source_node] =
                    unordered_set<int>(visible_vertices.begin(), visible_vertices.end());

                //organises and inserts visible via labels for each hub node
                for (unsigned j = 0; j < visible_vertices.size(); j++) {
                    const vector<int>& id_list = f_label_to_id[visible_vertices[j]];
                    for (unsigned k = 0; k < id_list.size(); k++) {
                        Convex_vertices_label_full& full_label = flat_label[id_list[k]];
                        const int& hub_id = full_label.hub_id;
                        polyanya::Convex_vertices_label convex_label = full_label.label;
                        convex_label.visibility = true;
                        if (label_mapper.find(hub_id) == label_mapper.end()) {
                            label_mapper.insert({ hub_id, pl::Hub_label{hub_id,vector<pl::Convex_vertices_label>(0)} });
                            label_id_mapper.insert({ hub_id, vector<int>(0)});
                        }
                        label_mapper[hub_id].convex_labels.push_back(convex_label);
                        label_id_mapper[hub_id].push_back(id_list[k]);
                    }
                }

                //organises and inserts partially visible via labels for each hub node
                for (unsigned j = 0; j < partial_visible.size(); j++) {
                    const vector<int>& id_list = f_label_to_id[partial_visible[j]];
                    for (unsigned k = 0; k < id_list.size(); k++) {
                        Convex_vertices_label_full& full_label = flat_label[id_list[k]];
                        const int& hub_id = full_label.hub_id;
                        polyanya::Convex_vertices_label convex_label = full_label.label;
                        convex_label.visibility = false;
                        if (label_mapper.find(hub_id) == label_mapper.end()) {
                            label_mapper.insert({ hub_id, pl::Hub_label{hub_id,vector<pl::Convex_vertices_label>(0)} });
                            label_id_mapper.insert({ hub_id, vector<int>(0) });
                        }
                        label_mapper[hub_id].convex_labels.push_back(convex_label);
                        label_id_mapper[hub_id].push_back(id_list[k]);
                    }
                }

                ebhl->poly_labels[source_node].hub_labels = vector<pl::Hub_label>();
                std::map<int, pl::Hub_label>::iterator it;
                for (it = label_mapper.begin(); it != label_mapper.end(); it++)
                {
                    ebhl->poly_labels[source_node].hub_labels.push_back(it->second);
                }

                //sort the labels based on hub node ID
                std::sort(std::begin(ebhl->poly_labels[source_node].hub_labels),
                    std::end(ebhl->poly_labels[source_node].hub_labels),
                    [](const pl::Hub_label& lvalue, const pl::Hub_label& rvalue) {
                    return  lvalue.hub_id < rvalue.hub_id; });

                pl::Polygon meshPoly = ebhl->mp->mesh_polygons[source_node];
                polygon_type poly;
                for (int i = 0; i < meshPoly.vertices.size(); i++) {
                    pl::Vertex polyV = ebhl->mp->mesh_vertices[meshPoly.vertices[i]];
                    bg::append(poly.outer(), point_type(polyV.p.x, polyV.p.y));
                }
                bg::correct(poly);

                vector<int>& filtered_ids = all_convex_label_ids[source_node];
                //const pl::Point &  min_corner =  ebhl->poly_labels[source_node].a_p;
                //const pl::Point &  max_corner =  ebhl->poly_labels[source_node].c_p;
                //Pruning 2 and 3
                for (unsigned j = 0; j < ebhl->poly_labels[source_node].hub_labels.size(); j++) {
                    const pl::Hub_label hl = ebhl->poly_labels[source_node].hub_labels[j];
                    const vector< pl::Convex_vertices_label>& convex_label_list = hl.convex_labels;
                    double min_upperbound = INF_WEIGHT;
                    for (const pl::Convex_vertices_label& label : convex_label_list) {
                        if (label.visibility) {
                            // distance function should already handle the containment cases.
                            double upperbound = max_distance_to_poly(turning_point[label.convex_vertex], poly) + label.distance;
                            if (min_upperbound > upperbound) {
                                min_upperbound = upperbound;
                            }
                        }
                    }

                    vector<int>& label_ids = label_id_mapper[hl.hub_id];
                    int prev_num_labels = filtered_ids.size();
                    for (int k = 0; k < label_ids.size(); k++) {
                        int convex_label_id = label_ids[k];
                        const pl::Convex_vertices_label& label = convex_label_list[k];
                        double lowerbound = min_distance_to_poly(turning_point[label.convex_vertex], poly) + label.distance;
                        if (lowerbound <= min_upperbound) {
                            //only add label is  lowerbound  <= min_upperbound
                            if (label.visibility) {
                                // prune non-taut path label ;
                                pl::Point hub = turning_point[label_inv[hl.hub_id]];
                                pl::Point predecessor = turning_point[label.predecessor];
                                const pl::Vertex& convex_vertex = mp->mesh_vertices[turning_vertices[label.convex_vertex]];
                                if (hub == convex_vertex.p) {
                                    filtered_ids.push_back(convex_label_id);
                                }
                                else {
                                    if (!poly_to_label_is_non_taut(ebhl, predecessor, convex_vertex, ebhl->poly_labels[source_node])) {
                                        filtered_ids.push_back(convex_label_id);
                                    }
                                }
                            }
                            else {
                                filtered_ids.push_back(convex_label_id);
                            }
                        }
                    }
                    if (filtered_ids.size() - prev_num_labels == 0)
                        ebhl->poly_labels[source_node].hub_labels[j].convex_labels.clear();
                    ebhl->poly_labels[source_node].hub_labels[j].convex_labels.shrink_to_fit();
                }
                original_costs[source_node] = filtered_ids.size();
                std::sort(filtered_ids.begin(), filtered_ids.end());

                ebhl->poly_labels[source_node].hub_labels.erase(std::remove_if(ebhl->poly_labels[source_node].hub_labels.begin(),
                    ebhl->poly_labels[source_node].hub_labels.end(),
                    [](const pl::Hub_label& x) {
                    return x.convex_labels.empty();
                }), ebhl->poly_labels[source_node].hub_labels.end());

                for (unsigned j = 0; j < ebhl->poly_labels[source_node].hub_labels.size(); j++) {
                    ebhl->poly_labels[source_node].hub_labels[j].convex_labels.clear();
                }
                ebhl->poly_labels[source_node].hub_labels.shrink_to_fit();

#pragma omp critical
                {
                    ++progress;
                    if (progress % 100 == 0) {
                        double ratio = (double)progress / node_count * 100.0;
                        std::cout << "Progress: [" << progress << "/" << node_count << "] "
                            << std::setprecision(3) << ratio << "% \r";
                        std::cout.flush();
                        std::cout.precision(ss);
                    }
                }
            }
        }
    }

    set<key_value<pair<int, int>>> pair_queue;
    vector<int> child_to_parent(all_convex_label_ids.size(), -1);
    //vector<vector<int>> parent_to_children(all_convex_label_ids.size());
    vector<vector<int>> parent_to_children(all_convex_label_ids.size());
    // Neighbor cells
    vector<set<int>> all_neighbors(all_convex_label_ids.size());
    vector<bool> all_ignore(all_convex_label_ids.size(), false);
    vector<int> temp_res;
    for (int i = 0; i < ebhl->mp->mesh_polygons.size(); i++) {
        pl::Polygon poly = ebhl->mp->mesh_polygons[i];
        if (poly.polygons.size() == 0) {
            continue;
        }
        set<int>& neighbors = all_neighbors[i];
        for (int j = 0; j < poly.polygons.size(); j++)
        {
            int neighbor = poly.polygons[j];
            if (neighbor == -1)
                continue;
            if (all_convex_label_ids[neighbor].size() == 0)
                continue;
            neighbors.insert(neighbor);
        }
        parent_to_children[i].push_back(i);
        for (int neighbor : neighbors) {
            if (neighbor >= i)
                break;
            temp_res.clear();
            std::set_intersection(all_convex_label_ids[i].begin(), all_convex_label_ids[i].end(),
                all_convex_label_ids[neighbor].begin(), all_convex_label_ids[neighbor].end(),
                std::back_inserter(temp_res));
            if (temp_res.size() == 0)
                continue;
            pair_queue.insert(key_value<pair<int, int>> { temp_res.size(), pair<int, int> { 1, 1 }, ordered_pair(i, neighbor) });
        }
    }

    if (!ebhl->store_visibility) {
        int tree_depth = 0;
        //int last_num_cells = ebhl->mp->mesh_polygons.size();
        int last_num_cells;
        do {
            last_num_cells = all_convex_label_ids.size();
            while (pair_queue.size() > 0) {
                set<key_value<pair<int, int>>>::iterator it = --pair_queue.end();
                key_value<pair<int, int>> kv = *it;
                pair<int, int> to_merge = kv.value;
                pair_queue.erase(it);
                if (all_ignore[to_merge.first] || all_ignore[to_merge.second])
                    continue;
                if (child_to_parent[to_merge.first] != -1 ||
                    child_to_parent[to_merge.second] != -1)
                    continue;
                if (kv.sanity.first != parent_to_children[to_merge.first].size() ||
                    kv.sanity.second != parent_to_children[to_merge.second].size())
                    continue;
                if (to_merge.first >= last_num_cells && to_merge.second >= last_num_cells)
                    continue;
                int merged = std::max(to_merge.first, to_merge.second);
                int child = std::min(to_merge.first, to_merge.second);
                if (merged < last_num_cells) {
                    int child2 = merged;
                    merged = all_convex_label_ids.size();
                    child_to_parent.push_back(-1);
                    parent_to_children.push_back(vector<int> { child2, merged });
                    all_neighbors.push_back(set<int>());
                    all_convex_label_ids.push_back(vector<int>());
                    all_ignore.push_back(false);
                }
                child_to_parent[to_merge.first] = merged;
                child_to_parent[to_merge.second] = merged;
                child_to_parent[merged] = -1;
                vector<int>& children = parent_to_children[merged];
                vector<int>& other_children = parent_to_children[child];
                if (child < last_num_cells)
                    children.push_back(child);
                else
                    children.insert(children.end(), other_children.begin(), other_children.end());
                // Delete other node if it is not original
                if (merged >= last_num_cells && child >= last_num_cells) {
                    all_ignore[child] = true;
                    for (int i = 0; i < other_children.size(); i++) {
                        assert(other_children[i] != merged);
                        child_to_parent[other_children[i]] = merged;
                    }
                }
                int num_children = children.size();
                /*vector<int> children(2);
                children.push_back(to_merge.first);
                children.push_back(to_merge.second);
                parent_to_children.push_back(children);*/

                // Update neighbors
                set<int> neighbors;
                std::merge(all_neighbors[to_merge.first].begin(), all_neighbors[to_merge.first].end(),
                    all_neighbors[to_merge.second].begin(), all_neighbors[to_merge.second].end(),
                    std::inserter(neighbors, neighbors.end()));
                neighbors.erase(to_merge.first);
                neighbors.erase(to_merge.second);

                for (int neighbor : all_neighbors[to_merge.first]) {
                    assert(neighbor != to_merge.first);
                    all_neighbors[neighbor].erase(to_merge.first);
                }
                for (int neighbor : all_neighbors[to_merge.second]) {
                    assert(neighbor != to_merge.second);
                    all_neighbors[neighbor].erase(to_merge.second);
                }
                for (int neighbor : neighbors) {
                    all_neighbors[neighbor].insert(merged);
                }
                all_neighbors[merged] = neighbors;

                vector<int> visible;
                std::set_intersection(all_convex_label_ids[to_merge.first].begin(), all_convex_label_ids[to_merge.first].end(),
                    all_convex_label_ids[to_merge.second].begin(), all_convex_label_ids[to_merge.second].end(),
                    std::back_inserter(visible));
                all_convex_label_ids[merged] = visible;
                for (int neighbor : neighbors) {
                    temp_res.clear();
                    std::set_intersection(all_convex_label_ids[merged].begin(), all_convex_label_ids[merged].end(),
                        all_convex_label_ids[neighbor].begin(), all_convex_label_ids[neighbor].end(),
                        std::back_inserter(temp_res));
                    int add = (visible.size() - temp_res.size()) * num_children;
                    int sub = temp_res.size();
                    int new_key = sub - add;
                    if (new_key <= 0)
                        continue;
                    pair<int, int> value = ordered_pair(merged, neighbor);
                    pair<int, int> sanity = pair<int, int>{ parent_to_children[value.first].size(), 
                        parent_to_children[value.second].size() };
                    pair_queue.insert(key_value<pair<int, int>> { new_key, sanity, value });
                }
            }

            pair_queue.clear();
            for (int i = last_num_cells; i < all_convex_label_ids.size(); i++) {
                set<int>& neighbors = all_neighbors[i];
                // Remove neighbors that are cells of the lower level
                neighbors.erase(neighbors.begin(), neighbors.upper_bound(last_num_cells));
                for (int neighbor : neighbors) {
                    if (neighbor >= i)
                        break;
                    temp_res.clear();
                    std::set_intersection(all_convex_label_ids[i].begin(), all_convex_label_ids[i].end(),
                        all_convex_label_ids[neighbor].begin(), all_convex_label_ids[neighbor].end(),
                        std::back_inserter(temp_res));
                    if (temp_res.size() == 0)
                        continue;
                    pair<int, int> value = ordered_pair(i, neighbor);
                    pair<int, int> sanity = pair<int, int>{ parent_to_children[value.first].size(), parent_to_children[value.second].size() };
                    pair_queue.insert(key_value<pair<int, int>> { temp_res.size(), sanity, value });
                }
            }
            tree_depth++;
        } while (pair_queue.size() > 0 && (all_convex_label_ids.size() - last_num_cells) > 1 && 
            (max_tree_depth == -1 || tree_depth < max_tree_depth));

        ebhl->compression = tree_depth;
        if (max_tree_depth != tree_depth) {
            std::cout << "Cannot reach compression depth" << std::endl;
            exit(0);
        }
    }
    /*int max_depth = 0;
    for (int i = 0; i < ebhl->mp->mesh_polygons.size(); i++) {
        int cur = i;
        int depth = 0;
        while (cur != -1) {
            cur = child_to_parent[cur];
            depth++;
        }
        if (depth > max_depth)
            max_depth = depth;
    }
    std::cout << "Depth: " << max_depth << std::endl;*/
    // Remap ids to account for deleted nodes, so we don't have to include them
    vector<int> id_to_index(all_convex_label_ids.size());
    int actual_index = ebhl->mp->mesh_polygons.size();
    int actual_size = ebhl->mp->mesh_polygons.size();
    for (int i = 0; i < ebhl->mp->mesh_polygons.size(); i++) {
        id_to_index[i] = i;
    }
    for (int i = ebhl->mp->mesh_polygons.size(); i < all_convex_label_ids.size(); i++) {
        if (all_ignore[i])
            continue;
        id_to_index[i] = actual_index++;
        actual_size++;
        //std::cout << parent_to_children[i].size() << std::endl;
    }
    // resize
    ebhl->initialize_poly_map(actual_size);
    vector<int> costs(actual_size);
    progress = 0;
    {
        printf("Using %d threads\n", omp_get_max_threads());
#pragma omp parallel
        {
            const int thread_count = omp_get_num_threads();
            const int thread_id = omp_get_thread_num();
            const int node_count = all_convex_label_ids.size();
            int node_begin = (node_count * thread_id) / thread_count;
            int node_end = (node_count * (thread_id + 1)) / thread_count;
            for (int cur_node = node_begin; cur_node < node_end; ++cur_node) {
                int source_node = cur_node;
                if (all_ignore[source_node])
                    continue;
                bool is_poly = source_node < ebhl->mp->mesh_polygons.size();
                vector<int> convex_label_ids;
                int parent = child_to_parent[source_node];
                std::map<int, int> hub_id_to_index;

                vector<int>& convex_label_ids_og = all_convex_label_ids[source_node];
                if (parent == -1) {
                    convex_label_ids = convex_label_ids_og;
                }
                else {
                    // Remove parent's vertices from child
                    std::set_difference(convex_label_ids_og.begin(), convex_label_ids_og.end(),
                        all_convex_label_ids[parent].begin(), all_convex_label_ids[parent].end(),
                        std::back_inserter(convex_label_ids));
                }
                if (!is_poly) {
                    source_node = id_to_index[source_node];
                    unordered_set<int> hub_ids;
                    for (int j = 0; j < convex_label_ids.size(); j++) {
                        hub_ids.insert(flat_label[convex_label_ids[j]].hub_id);
                    }

                    ebhl->poly_labels[source_node].hub_labels = vector<pl::Hub_label>();
                    for (const int& hub_id : hub_ids)
                    {
                        ebhl->poly_labels[source_node].hub_labels.push_back(pl::Hub_label{ hub_id, vector<pl::Convex_vertices_label>(0) });
                    }

                    //sort the labels based on hub node ID
                    std::sort(std::begin(ebhl->poly_labels[source_node].hub_labels),
                        std::end(ebhl->poly_labels[source_node].hub_labels),
                        [](const pl::Hub_label& lvalue, const pl::Hub_label& rvalue) {
                        return  lvalue.hub_id < rvalue.hub_id; });
                }

                for (int j = 0; j < ebhl->poly_labels[source_node].hub_labels.size(); j++)
                    hub_id_to_index.insert({ ebhl->poly_labels[source_node].hub_labels[j].hub_id, j });

                for (int j = 0; j < convex_label_ids.size(); j++) {
                    Convex_vertices_label_full& full_label = flat_label[convex_label_ids[j]];
                    int index = hub_id_to_index[full_label.hub_id];
                    pl::Convex_vertices_label convex_label = full_label.label;
                    convex_label.visibility = is_poly ? (ebhl->poly_visible[source_node].find(convex_label.convex_vertex) != 
                        ebhl->poly_visible[source_node].end()) : false;
                    ebhl->poly_labels[source_node].hub_labels[index].convex_labels.push_back(convex_label);
                }
                costs[source_node] = convex_label_ids.size();

                for (int j = 0; j < ebhl->poly_labels[source_node].hub_labels.size(); j++) {
                    pl::Hub_label& hl = ebhl->poly_labels[source_node].hub_labels[j];
                    std::sort(std::begin(hl.convex_labels),
                        std::end(hl.convex_labels),
                        [](const pl::Convex_vertices_label& lvalue, const pl::Convex_vertices_label& rvalue) {
                        return  lvalue.visibility == rvalue.visibility ? lvalue.convex_vertex < rvalue.convex_vertex :
                        lvalue.visibility > rvalue.visibility; });
                }

                if (parent == -1)
                    ebhl->poly_parents[source_node] = -1;
                else
                    ebhl->poly_parents[source_node] = id_to_index[parent];
                assert(!all_ignore[parent]);

                pl::Convex_vertices_label cv = pl::Convex_vertices_label{ -1,INF_WEIGHT,false, -1 };
                polyanya::Hub_label hub_label = polyanya::Hub_label{ (int)turning_point.size(),
                    vector<pl::Convex_vertices_label>{cv}, std::numeric_limits<double>::max() };
                ebhl->poly_labels[source_node].hub_labels.push_back(hub_label);

                ebhl->poly_labels[source_node].hub_labels.shrink_to_fit();

#pragma omp critical
                {
                    ++progress;
                    if (progress % 100 == 0) {
                        double ratio = (double)progress / node_count * 100.0;
                        std::cout << "Progress: [" << progress << "/" << node_count << "] "
                            << std::setprecision(3) << ratio << "% \r";
                        std::cout.flush();
                        std::cout.precision(ss);
                    }
                }
            }
        }
    }

    int num_labels = 0;
    int num_hubs = 0;
    for (int i = ebhl->mp->mesh_polygons.size(); i < ebhl->poly_labels.size(); i++) {
        for (int j = 0; j < ebhl->poly_labels[i].hub_labels.size(); j++) {
            num_hubs++;
            num_labels += ebhl->poly_labels[i].hub_labels[j].convex_labels.size();
        }
    }
    std::cout << "Average num via labels per hub label id for parents: " << (num_labels / (double)num_hubs) << std::endl;

    long total_og_cost = 0;
    long total_cost = 0;
    for (int original_cost : original_costs) {
        total_og_cost += original_cost;
    }
    for (int cost : costs) {
        total_cost += cost;
    }
    double cost_ratio = total_cost / (double)total_og_cost;
    std::cout << "Cost/via_labels: " << cost_ratio << " end" << std::endl;
    std::cout << "Original avg via labels: " << ((double)total_og_cost / original_num_polys) << std::endl;

    ebhl->cost_ratio = cost_ratio;

    progress = 0;
    {
        printf("Using %d threads\n", omp_get_max_threads());
#pragma omp parallel
        {
            const int thread_count = omp_get_num_threads();
            const int thread_id = omp_get_thread_num();
            const int node_count = ebhl->mp->mesh_polygons.size();
            int node_begin = (node_count * thread_id) / thread_count;
            int node_end = (node_count * (thread_id + 1)) / thread_count;
            for (int source_node = node_begin; source_node < node_end; ++source_node) {
                if (all_ignore[source_node])
                    continue;
                pl::Polygon meshPoly = ebhl->mp->mesh_polygons[source_node];
                polygon_type poly;
                for (int i = 0; i < meshPoly.vertices.size(); i++) {
                    pl::Vertex polyV = ebhl->mp->mesh_vertices[meshPoly.vertices[i]];
                    bg::append(poly.outer(), point_type(polyV.p.x, polyV.p.y));
                }
                bg::correct(poly);
                //Optimisation: storing the lower bound;
                for (unsigned j = 0; j < ebhl->poly_labels[source_node].hub_labels.size(); j++) {
                    int hub_id = ebhl->poly_labels[source_node].hub_labels[j].hub_id;
                    double lower_bound = std::numeric_limits<double>::max();
                    int cur_node = source_node;
                    while (cur_node != -1) {
                        vector<pl::Hub_label>& hub_labels = ebhl->poly_labels[id_to_index[cur_node]].hub_labels;
                        vector<pl::Hub_label>::iterator hub_it = std::lower_bound(
                            hub_labels.begin(), hub_labels.end(), pl::Hub_label{ hub_id },
                            [](const pl::Hub_label& lvalue, const pl::Hub_label& rvalue) {
                            return  lvalue.hub_id < rvalue.hub_id; });
                        if (hub_it == hub_labels.end() || hub_id < (*hub_it).hub_id)
                            goto skip;
                        for (auto convex_label : (*hub_it).convex_labels) {
                            lower_bound = std::min(lower_bound, min_distance_to_poly(turning_point[convex_label.convex_vertex], poly) + convex_label.distance);
                        }
                    skip:
                        int prev_node = cur_node;
                        cur_node = child_to_parent[cur_node];
                        assert(prev_node != cur_node);
                    }
                    ebhl->poly_labels[source_node].hub_labels[j].min_lower_bound = lower_bound;
                }

#pragma omp critical
                {
                    ++progress;
                    if (progress % 100 == 0) {
                        double ratio = (double)progress / node_count * 100.0;
                        std::cout << "Progress: [" << progress << "/" << node_count << "] "
                            << std::setprecision(3) << ratio << "% \r";
                        std::cout.flush();
                        std::cout.precision(ss);
                    }
                }
            }
        }
    }

    timer.stop();
    std::cout << std::fixed << setprecision(8) << "Preprocessing finished in " << timer.elapsed_time_micro() / 1000000 << " seconds" << std::endl;

    //save in adjacent lists without parallelisation
    string output_path = "dataset/ehl/" + dir + "/" + map + ".nt_mesh_adj";
    ebhl->save_adjacent_list(output_path.c_str());
    //store the visible polygons
    string output_triangle_file = "dataset/ehl/" + dir + "/" + map + ".nt_mesh_triangles";
    int size_of_turning = turning_point.size();
    output_non_taut_triangle(boost_poly, size_of_turning, output_triangle_file);

    delete ebhl;

}

int main(int argc, char* argv[]) {

    try {
        string dir;
        string map;
        int max_extra_regions, max_tree_depth;
        if (argc != 5) {
            cerr << argv[0] << "directory map" << endl;
            return 1;
        }
        else {
            dir = argv[1];
            map = argv[2];
            max_extra_regions = atoi(argv[3]) - 1;
            max_tree_depth = atoi(argv[4]);
        }

        //improved memory and preprocessing pruning version saving in adjacent lists
        build_ebhl(dir, map, max_extra_regions, max_tree_depth);

        //saving grid labels in parallel
        //build_ebhl_parallel(dir,map,stoi(grid_size));


    }
    catch (exception& err) {

        cerr << "Stopped on exception: " << err.what() << endl;
    }
}