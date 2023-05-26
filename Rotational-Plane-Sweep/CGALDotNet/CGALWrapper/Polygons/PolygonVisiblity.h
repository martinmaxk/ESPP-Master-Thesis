#pragma once

#include "../CGALWrapper.h"
#include "../Geometry/Geometry2.h"
#include "Polygon2.h"
#include "PolygonWithHoles2.h"

#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Simple_polygon_visibility_2.h>
#include <CGAL/Triangular_expansion_visibility_2.h>
#include <CGAL/Rotational_sweep_visibility_2.h>
#include <CGAL/Arrangement_2.h>
#include <CGAL/Arr_segment_traits_2.h>
#include <CGAL/Arr_naive_point_location.h>
#include <istream>
#include <vector>

#include <CGAL/boost/iterator/transform_iterator.hpp>
#include <CGAL/Constrained_Delaunay_triangulation_2.h>

template<class K>
class PolygonVisibility
{
public:

	//typedef CGAL::Exact_predicates_exact_constructions_kernel       Kernel;
	typedef CGAL::Polygon_2<K>                                        Point_2;
	typedef CGAL::Segment_2<K>                                      Segment_2;
	typedef CGAL::Arr_segment_traits_2<K>                      Traits_2;
	typedef CGAL::Arrangement_2<Traits_2>                           Arrangement_2;

	typedef CGAL::Triangular_expansion_visibility_2<Arrangement_2>  TEV;
	typedef CGAL::Rotational_sweep_visibility_2<Arrangement_2> RSV;

	typedef CGAL::Arr_walk_along_line_point_location<Arrangement_2> Locator;

	typedef typename Polygon2<K>::Polygon_2 Polygon;
	typedef typename PolygonWithHoles2<K>::Pwh_2 Pwh;

	typedef typename Arrangement_2::Geometry_traits_2    Geometry_traits_2;
	typedef typename Geometry_traits_2::Kernel            GK;
	typedef CGAL::Triangulation_vertex_base_2<GK>                          Vb;
	typedef CGAL::Constrained_triangulation_face_base_2<GK>                Fb;
	typedef CGAL::Triangulation_data_structure_2<Vb, Fb>                  TDS;
	typedef CGAL::No_constraint_intersection_requiring_constructions_tag  Itag;
	typedef CGAL::Constrained_Delaunay_triangulation_2<GK, TDS, Itag>      CDT;

	typedef typename Arrangement_2::Halfedge              Halfedge;
	typedef typename Arrangement_2::Edge_const_iterator   Edge_const_iterator;
	typedef typename GK::Point_2                           GPoint_2;
	typedef std::pair<GPoint_2, GPoint_2>                                   Constraint;

	inline static PolygonVisibility* NewPolygonVisibility()
	{
		return new PolygonVisibility();
	}

	inline static void DeletePolygonVisibility(void* ptr)
	{
		auto obj = static_cast<PolygonVisibility*>(ptr);

		if (obj != nullptr)
		{
			delete obj;
			obj = nullptr;
		}
	}

	inline static PolygonVisibility* CastToPolygonVisibility(void* ptr)
	{
		return static_cast<PolygonVisibility>(ptr);
	}

	static void GetSegments(std::vector<Segment_2>& segments, const Polygon& poly)
	{
		int count = (int)poly.size();
		for (int i = 0; i < count; i++)
		{
			if (i != count - 1)
			{
				auto p1 = poly.vertex(i);
				auto p2 = poly.vertex(i + 1);
				segments.push_back(Segment_2(p1, p2));
			}
			else
			{
				auto p1 = poly.vertex(i);
				auto p2 = poly.vertex(0);
				segments.push_back(Segment_2(p1, p2));
			}
		}
	}

	static void GetSegments(std::vector<Segment_2>& segments, const Pwh& pwh)
	{
		int count = (int)pwh.outer_boundary().size();
		for (int i = 0; i < count; i++)
		{
			if (i != count - 1)
			{
				auto p1 = pwh.outer_boundary().vertex(i);
				auto p2 = pwh.outer_boundary().vertex(i + 1);
				segments.push_back(Segment_2(p1, p2));
			}
			else
			{
				auto p1 = pwh.outer_boundary().vertex(i);
				auto p2 = pwh.outer_boundary().vertex(0);
				segments.push_back(Segment_2(p1, p2));
			}
		}

		int holes = (int)pwh.number_of_holes();
		for (int j = 0; j < holes; j++)
		{
			auto& hole = pwh.holes().at(j);
			count = (int)hole.size();
			for (int i = 0; i < count; i++)
			{
				if (i != count - 1)
				{
					auto p1 = hole.vertex(i);
					auto p2 = hole.vertex(i + 1);
					segments.push_back(Segment_2(p1, p2));
				}
				else
				{
					auto p1 = hole.vertex(i);
					auto p2 = hole.vertex(0);
					segments.push_back(Segment_2(p1, p2));
				}
			}
		}
	}

	static void* ComputeVisibilitySimple(const Point2d& point, void* polyPtr)
	{

		auto poly = Polygon2<K>::CastToPolygon2(polyPtr);

		std::vector<Segment_2> segments;
		GetSegments(segments, *poly);

		Arrangement_2 env;
		CGAL::insert_non_intersecting_curves(env, segments.begin(), segments.end());

		Locator pl(env);

		auto q = point.ToCGAL<K>();
		auto obj = pl.locate(q);
		auto face = boost::get<Arrangement_2::Face_const_handle>(&obj);

		typedef CGAL::Simple_polygon_visibility_2<Arrangement_2, CGAL::Tag_true> RSPV;

		Arrangement_2 regular_output;
		RSPV regular_visibility(env);
		regular_visibility.compute_visibility(q, *face, regular_output);

		auto result = Polygon2<K>::NewPolygon2();
		auto start = regular_output.edges_begin()->next();
		auto eit = start;

		do
		{
			auto p = eit->source()->point();
			result->push_back(p);
			eit = eit->next();

		} while (eit != start);

		return result;
	}

	static void* ComputeVisibilityTEV(const Point2d& point, void* pwhPtr)
	{
		auto pwh = PolygonWithHoles2<K>::CastToPolygonWithHoles2(pwhPtr);

		std::vector<Segment_2> segments;
		GetSegments(segments, *pwh);

		Arrangement_2 env;
		CGAL::insert_non_intersecting_curves(env, segments.begin(), segments.end());

		Locator pl(env);

		auto q = point.ToCGAL<K>();
		auto obj = pl.locate(q);
		auto face = boost::get<Arrangement_2::Face_const_handle>(&obj);

		Arrangement_2 output_arr;
		TEV tev(env);
		auto fh = tev.compute_visibility(q, *face, output_arr);

		auto result = PolygonWithHoles2<K>::NewPolygonWithHoles2();
		auto curr = fh->outer_ccb();

		result->outer_boundary().push_back(curr->source()->point());

		while (++curr != fh->outer_ccb())
		{
			result->outer_boundary().push_back(curr->source()->point());
		}
			
		return result;
	}

	static void* GetLocatorAndTEV(void* pwhPtr)
	{
		auto pwh = PolygonWithHoles2<K>::CastToPolygonWithHoles2(pwhPtr);

		std::vector<Segment_2> segments;
		GetSegments(segments, *pwh);

		Arrangement_2* env = new Arrangement_2();
		CGAL::insert_non_intersecting_curves(*env, segments.begin(), segments.end());

		TriExpansionInfo* triExp = new TriExpansionInfo();

		/*typedef typename boost::transform_iterator<Make_constraint,
			Edge_const_iterator>        Iter;
		Iter begin = boost::make_transform_iterator(env->edges_begin(),
			Make_constraint());
		Iter end = boost::make_transform_iterator(env->edges_end(),
			Make_constraint());
		triExp->p_cdt = std::shared_ptr<CDT>(new CDT(begin, end));*/

		//triExp->locator = new Locator(*env);
		triExp->tev = new TEV(*env);
		return triExp;
	}

	static BOOL CdtContains(const Point2d& point, void* triExpPtr)
	{
		TriExpansionInfo* triExp = static_cast<TriExpansionInfo*>(triExpPtr);
		std::shared_ptr<CDT> p_cdt = triExp->p_cdt;
		auto q = point.ToCGAL<K>();
		typename CDT::Face_handle fh = p_cdt->locate(q);
		return !p_cdt->is_infinite(fh);
	}

	static void* ComputeVisibilityTEVCached(const Point2d& point, void* triExpPtr)
	{
		TriExpansionInfo* triExp = static_cast<TriExpansionInfo*>(triExpPtr);
		//Locator* pl = triExp->locator;
		TEV* tev = triExp->tev;
		auto q = point.ToCGAL<K>();
		//auto obj = pl->locate(q);
		//auto face = boost::get<Arrangement_2::Face_const_handle>(&obj);

		const Arrangement_2::Face_const_handle temp = Arrangement_2::Face_const_handle();

		Arrangement_2 output_arr;
		auto fh = tev->compute_visibility(q, temp, output_arr);//*face, output_arr);

		auto result = PolygonWithHoles2<K>::NewPolygonWithHoles2();
		auto curr = fh->outer_ccb();

		result->outer_boundary().push_back(curr->source()->point());

		while (++curr != fh->outer_ccb())
		{
			result->outer_boundary().push_back(curr->source()->point());
		}

		return result;
	}

	static void* ComputeVisibilityRSV(const Point2d& point, void* pwhPtr)
	{
		auto pwh = PolygonWithHoles2<K>::CastToPolygonWithHoles2(pwhPtr);

		std::vector<Segment_2> segments;
		GetSegments(segments, *pwh);

		Arrangement_2 env;
		CGAL::insert_non_intersecting_curves(env, segments.begin(), segments.end());

		Locator pl(env);

		auto q = point.ToCGAL<K>();
		auto obj = pl.locate(q);
		auto face = boost::get<Arrangement_2::Face_const_handle>(&obj);

		Arrangement_2 output_arr;
		RSV rsv(env);
		auto fh = rsv.compute_visibility(q, *face, output_arr);

		auto result = PolygonWithHoles2<K>::NewPolygonWithHoles2();
		auto curr = fh->outer_ccb();

		result->outer_boundary().push_back(curr->source()->point());

		while (++curr != fh->outer_ccb())
		{
			result->outer_boundary().push_back(curr->source()->point());
		}

		return result;
	}

	private:
		struct TriExpansionInfo
		{
			/*typedef CGAL::Polygon_2<K>                                        Point_2;
			typedef CGAL::Segment_2<K>                                      Segment_2;
			typedef CGAL::Arr_segment_traits_2<K>                      Traits_2;
			typedef CGAL::Arrangement_2<Traits_2>                           Arrangement_2;

			typedef CGAL::Triangular_expansion_visibility_2<Arrangement_2>  TEV;
			typedef CGAL::Rotational_sweep_visibility_2<Arrangement_2> RSV;

			typedef CGAL::Arr_walk_along_line_point_location<Arrangement_2> Locator;

			typedef typename Polygon2<K>::Polygon_2 Polygon;
			typedef typename PolygonWithHoles2<K>::Pwh_2 Pwh;

			typedef CGAL::Triangulation_vertex_base_2<K>                          Vb;
			typedef CGAL::Constrained_triangulation_face_base_2<K>                Fb;
			typedef CGAL::Triangulation_data_structure_2<Vb, Fb>                  TDS;
			typedef CGAL::No_constraint_intersection_requiring_constructions_tag  Itag;
			typedef CGAL::Constrained_Delaunay_triangulation_2<K, TDS, Itag>      CDT;*/

			Locator* locator;
			mutable std::shared_ptr<CDT> p_cdt;
			TEV* tev;
		};

		// Functor to create edge constraints for the CDT out of Halfedges
		struct Make_constraint
		{
			Constraint operator()(const Halfedge& edge) const {
				return std::make_pair(edge.source()->point(),
					edge.target()->point());
			}
		};
};
