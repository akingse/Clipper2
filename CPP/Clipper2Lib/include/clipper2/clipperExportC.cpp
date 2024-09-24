
#include "clipper2/clipper.h"
#include "clipper2/clipper.core.h"
#include "clipper2/clipper.engine.h"
#include "clipper2/clipper.offset.h"
#include "clipper2/clipper.rectclip.h"
#include "clipperExportC.h" //copy from clipper.export.h
using namespace Clipper2Lib;
using namespace Clipper;

template <typename T>
static void GetPathCountAndCPathsArrayLen(const Paths<T>& paths, size_t& cnt, size_t& array_len)
{
    array_len = 2;
    cnt = 0;
    for (const Path<T>& path : paths)
    {
        if (path.size())
        {
            array_len += path.size() * 2 + 2;
            ++cnt;
        }
    }
}

//vector<T> -> double*
template <typename T> //specialize int64_t*
static T* CreateCPaths(const Paths<T>& paths)
{
    size_t cnt, array_len;
    GetPathCountAndCPathsArrayLen(paths, cnt, array_len);
    T* result = new T[array_len], * v = result;
    *v++ = array_len;
    *v++ = cnt;
    for (const Path<T>& path : paths)
    {
        if (!path.size())
            continue;
        *v++ = path.size();
        *v++ = 0;
        for (const Point<T>& pt : path)
        {
            *v++ = pt.x;
            *v++ = pt.y;
        }
    }
    return result;
}

static CPathsD CreateCPathsDFromPaths64(const Paths64& _paths, double scale)
{
#ifdef DO_SELFINTERSECT_PROCESS
    auto _isSelfIntersect = [](const Path64& path)->bool
    {
        int n = (int)path.size(); //even if n==3
        for (int i = 0; i < n - 2; ++i) //skip adjacent
        {
            for (int j = i + 2; j < n; ++j)
            {
                //include i(0,1)-j(n-1,0)
                if (SegmentsIntersect(path[i], path[i + 1], path[j], path[(j + 1) % n], false))
                    return true;
            }
        }
        return false;
    };
#endif
#ifdef USING_TOLERANCE_PROCESS
    //process tiny area and self interset
    std::vector<Path64> paths;
    for (int k = 0; k < _paths.size(); ++k)
    {
        double area = Area(_paths[k]) * scale;
        if (area < g_tolerance2)
            continue;
#ifdef DO_SELFINTERSECT_PROCESS
        //double loop
        if (3 < _paths[k].size() && _isSelfIntersect(_paths[k]))
            continue;
#endif
        paths.push_back(_paths[k]);
    }
#else
    const Paths64& paths = _paths;
#endif //USING_TOLERANCE_PROCESS
    if (!paths.size()) return nullptr;
    size_t cnt, array_len;
    GetPathCountAndCPathsArrayLen(paths, cnt, array_len);
    CPathsD result = new double[array_len], v = result;
    *v++ = (double)array_len;
    *v++ = (double)cnt;
    for (const Path64& path : paths)
    {
        if (!path.size()) continue;
        *v = (double)path.size();
        ++v; *v++ = 0;
        for (const Point64& pt : path)
        {
            *v++ = pt.x * scale;
            *v++ = pt.y * scale;
        }
    }
    return result;
}

//double* -> vector<T>
template <typename T>
static Paths<T> ConvertCPaths(const T* paths)
{
    Paths<T> result;
    if (!paths) return 
        result;
    const T* v = paths; ++v;
    size_t cnt = *v++;
    result.reserve(cnt);
    for (size_t i = 0; i < cnt; ++i)
    {
        size_t cnt2 = *v;
        v += 2;
        Path<T> path;
        path.reserve(cnt2);
        for (size_t j = 0; j < cnt2; ++j)
        {
            T x = *v++, y = *v++;
            path.push_back(Point<T>(x, y));
        }
        result.push_back(path);
    }
    return result;
}

static Paths64 ConvertCPathsDToPaths64(const CPathsD paths, double scale)
{
    Paths64 result;
    if (!paths) 
        return result;
    double* v = paths;
    ++v; // skip the first value (0)
    int64_t cnt = (int64_t)*v++;
    result.reserve(cnt);
    for (int i = 0; i < cnt; ++i)
    {
        int64_t cnt2 = (int64_t)*v;
        v += 2;
        Path64 path;
        path.reserve(cnt2);
        for (int j = 0; j < cnt2; ++j)
        {
            double x = *v++ * scale;
            double y = *v++ * scale;
            path.push_back(Point64(x, y)); //using std::round
        }
        result.push_back(path);
    }
    return result;
}

//int64_t version
int Clipper::BooleanOp64(uint8_t cliptype, uint8_t fillrule,
    const CPaths64 subjects, const CPaths64 clips, CPaths64& solution)
{
    if (cliptype > static_cast<uint8_t>(ClipType::Xor)) return -4;
    if (fillrule > static_cast<uint8_t>(FillRule::Negative)) return -3;

    Paths64 sub, clp, sol;
    sub = ConvertCPaths(subjects);
    clp = ConvertCPaths(clips);

    Clipper64 clipper;
    //clipper.PreserveCollinear = true;
    //clipper.ReverseSolution = false;
    if (sub.size() > 0) 
        clipper.AddSubject(sub);
    if (clp.size() > 0) 
        clipper.AddClip(clp);
    if (!clipper.Execute(ClipType(cliptype), FillRule(fillrule), sol))
        return -1; // clipping bug - should never happen :)
    solution = CreateCPaths(sol); //using new, remember to delete
    return 0; //success !!
}

//Double version
int Clipper::BooleanOpD(uint8_t cliptype, uint8_t fillrule,
	const CPathsD subjects, const CPathsD clips, CPathsD& solution,	int precision)
{
    if (precision < -8 || precision > 16) return -5;
    if (cliptype > static_cast<uint8_t>(ClipType::Xor)) return -4;
    if (fillrule > static_cast<uint8_t>(FillRule::Negative)) return -3;
    const double scale = std::pow(10, precision);

    Paths64 sub, clp, sol; //subjects-clips=solution
    sub = ConvertCPathsDToPaths64(subjects, scale);
    clp = ConvertCPathsDToPaths64(clips, scale);

    Clipper64 clipper;
    //clipper.preserve_collinear_ = false; //whether reserve collinear points
    //clipper.reverse_solution_ = false;
    if (sub.size() > 0) 
        clipper.AddSubject(sub);
    if (clp.size() > 0) 
        clipper.AddClip(clp);
    Paths64 open_paths;
    if (!clipper.Execute(ClipType(cliptype), FillRule(fillrule), sol, open_paths)) //reduce function nest
        return -1;
    //TrimCollinear
    //for (auto& iter: sol)
    //    iter = TrimCollinear(iter, false);
    //SimplifyPaths
    //sol = SimplifyPaths(sol, 10, true);
    solution = CreateCPathsDFromPaths64(sol, 1.0 / scale); //new pointer
    return 0;
}

//-----------------------------------------------------------------------------
// get area, copy from TestExportHeaders.cpp
//-----------------------------------------------------------------------------

//int64_t version
static bool CreatePolyPath64FromCPolyPath(CPolyPath64& v, PolyPath64& owner)
{
    int64_t poly_len = *v++, child_count = *v++;
    if (!poly_len) 
        return false;
    Path64 path;
    path.reserve(poly_len);
    for (int64_t i = 0; i < poly_len; ++i)
    {
        int64_t x = *v++, y = *v++;
        path.push_back(Point64(x, y));
    }

    PolyPath64* new_owner = owner.AddChild(path);
    for (int64_t i = 0; i < child_count; ++i)
        CreatePolyPath64FromCPolyPath(v, *new_owner);
    return true;
}

static bool BuildPolyTree64FromCPolyTree(CPolyTree64 tree, PolyTree64& result)
{
    result.Clear();
    int64_t* v = tree;
    int64_t array_len = *v++, child_count = *v++;
    for (int64_t i = 0; i < child_count; ++i)
        if (!CreatePolyPath64FromCPolyPath(v, result)) 
            return false;
    return true;
}

double Clipper::getAreaOfPolyTree64(const CPaths64 subjects)
{
    if (subjects == nullptr)
        return 0;// std::nan("0");
    PolyTree64 sol_tree;
    BuildPolyTree64FromCPolyTree(subjects, sol_tree);
    return sol_tree.Area();
}

//Double version
static bool CreatePolyPathDFromCPolyPath(CPolyPathD& v, PolyPathD& owner)
{
    int64_t poly_len = (int64_t)*v++, child_count = (int64_t)*v++;
    if (!poly_len)
        return false;
    PathD path;
    path.reserve(poly_len);
    for (int64_t i = 0; i < poly_len; ++i)
    {
        double x = *v++, y = *v++;
        path.push_back(PointD(x, y));
    }
    PolyPathD* new_owner = owner.AddChild(path);
    for (int64_t i = 0; i < child_count; ++i)
        CreatePolyPathDFromCPolyPath(v, *new_owner);
    return true;
}

static bool BuildPolyTreeDFromCPolyTree(CPolyTreeD tree, PolyTreeD& result)
{
    result.Clear();
    double* v = tree;
    int64_t array_len = (int64_t)*v++, child_count = (int64_t)*v++;
    for (int64_t i = 0; i < child_count; ++i)
        if (!CreatePolyPathDFromCPolyPath(v, result)) 
            return false;
    return true;
}

double Clipper::getAreaOfPolyTreeD(const CPathsD subjects)
{
    if (subjects == nullptr)
        return 0;// std::nan("0");
    PolyTreeD sol_tree;
    BuildPolyTreeDFromCPolyTree(subjects, sol_tree);
    return sol_tree.Area();
}
