
#include "clipper2/clipper.h"
#include "clipper2/clipper.core.h"
#include "clipper2/clipper.engine.h"
#include "clipper2/clipper.offset.h"
#include "clipper2/clipper.rectclip.h"
#include "clipperExportC.h" //copy from clipper.export.h
using namespace Clipper2Lib;
using namespace Clipper;

//namespace Clipper //convert function, copy from clipper.h
//{
//    template <typename T>
//    struct Point
//    {
//        T x;
//        T y;
//        Point() = default;
//        Point(T _x, T _y) :x(_x), y(_y) {}
//    };
//    //using PointD = Point<double>;
//    template <typename T>
//    using Path = std::vector<Point<T>>;
//    template <typename T>
//    using Paths = std::vector<Path<T>>;
//
//    using PathD = Path<double>;
//    using PathsD = std::vector<PathD>;
//
//    namespace details
//    {
//        template<typename T, typename U>
//        inline void MakePathGeneric(const T list, size_t size,
//            std::vector<U>& result)
//        {
//            for (size_t i = 0; i < size; ++i)
//#ifdef USINGZ
//                result[i / 2] = U{ list[i], list[++i], 0 };
//#else
//                result[i / 2] = U{ list[i], list[++i] };
//#endif
//        }
//
//    } // end details namespace
//
//    template<typename T, std::size_t N>
//    inline PathD MakePathD(const T(&list)[N])
//    {
//        // Make the compiler error on unpaired value (i.e. no runtime effects).
//        static_assert(N % 2 == 0, "MakePath requires an even number of arguments");
//        PathD result(N / 2);
//        details::MakePathGeneric(list, N, result);
//        return result;
//    }
//
//    template <typename T>
//    static void GetPathCountAndCPathsArrayLen(const Paths<T>& paths,
//        size_t& cnt, size_t& array_len)
//    {
//        array_len = 2;
//        cnt = 0;
//        for (const Path<T>& path : paths)
//            if (path.size())
//            {
//                array_len += path.size() * 2 + 2;
//                ++cnt;
//            }
//    }
//
//    template <typename T>
//    static T* CreateCPaths(const Paths<T>& paths)
//    {
//        size_t cnt, array_len;
//        GetPathCountAndCPathsArrayLen(paths, cnt, array_len);
//        T* result = new T[array_len], * v = result;
//        *v++ = array_len;
//        *v++ = cnt;
//        for (const Path<T>& path : paths)
//        {
//            if (!path.size()) continue;
//            *v++ = path.size();
//            *v++ = 0;
//            for (const Point<T>& pt : path)
//            {
//                *v++ = pt.x;
//                *v++ = pt.y;
//            }
//        }
//        return result;
//    }
//
//    template <typename T>
//    static Paths<T> ConvertCPaths(T* paths)
//    {
//        Paths<T> result;
//        if (!paths) return result;
//        T* v = paths; ++v;
//        size_t cnt = *v++;
//        result.reserve(cnt);
//        for (size_t i = 0; i < cnt; ++i)
//        {
//            size_t cnt2 = *v;
//            v += 2;
//            Path<T> path;
//            path.reserve(cnt2);
//            for (size_t j = 0; j < cnt2; ++j)
//            {
//                T x = *v++, y = *v++;
//                path.push_back(Point<T>(x, y));
//            }
//            result.push_back(path);
//        }
//        return result;
//    }
//}

//namespace Clipper //specialization
//{
//    struct PointD
//    {
//        double x;
//        double y;
//        PointD() = default;
//        PointD(double _x, double _y) :x(_x), y(_y) {}
//    };
//    using PathD = std::vector<PointD>; //Path<double>;
//    using PathsD = std::vector<PathD>;
//
//    //template function
//    //template<typename T, typename U>
//    inline constexpr void MakePathGeneric(const double* list, size_t size, std::vector<PointD>& result)
//    {
//        for (size_t i = 0; i < size; ++i)
//#ifdef USINGZ
//            result[i / 2] = PointD{ list[i], list[++i], 0 };
//#else
//            result[i / 2] = PointD{ list[i], list[++i] };
//#endif
//    }
//
//    //template<typename T, std::size_t N>
//    inline PathD MakePathD(const double* list, size_t N)
//    {
//        PathD result(N / 2);
//        MakePathGeneric(list, N, result);
//        return result;
//    }
//
//    //template <typename T>
//    static void GetPathCountAndCPathsArrayLen(const PathsD& paths,
//        size_t& cnt, size_t& array_len)
//    {
//        array_len = 2;
//        cnt = 0;
//        for (const PathD& path : paths)
//        {
//            if (path.size()) //!empty
//            {
//                array_len += path.size() * 2 + 2;
//                ++cnt;
//            }
//        }
//    }
//
//    //template <typename T>
//    static double* CreateCPaths(const PathsD& paths)
//    {
//        size_t cnt, array_len;
//        GetPathCountAndCPathsArrayLen(paths, cnt, array_len);
//        double* result = new double[array_len];
//        double* v = result;
//        *v++ = array_len;
//        *v++ = cnt;
//        for (const PathD& path : paths)
//        {
//            if (!path.size())
//                continue;
//            *v++ = path.size();
//            *v++ = 0;
//            for (const PointD& pt : path)
//            {
//                *v++ = pt.x;
//                *v++ = pt.y;
//            }
//        }
//        return result;
//    }
//
//    //template <typename T>
//    static PathsD ConvertCPaths(double* paths)
//    {
//        PathsD result;
//        if (!paths)
//            return result;
//        double* v = paths; ++v;
//        size_t cnt = *v++;
//        result.reserve(cnt);
//        for (size_t i = 0; i < cnt; ++i)
//        {
//            size_t cnt2 = *v;
//            v += 2;
//            PathD path;
//            path.reserve(cnt2);
//            for (size_t j = 0; j < cnt2; ++j)
//            {
//                double x = *v++, y = *v++;
//                path.push_back(PointD(x, y));
//            }
//            result.push_back(path);
//        }
//        return result;
//    }
//}

template <typename T>
static void GetPathCountAndCPathsArrayLen(const Paths<T>& paths,
    size_t& cnt, size_t& array_len)
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

static CPathsD CreateCPathsDFromPaths64(const Paths64& paths, double scale)
{
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

template <typename T>
static Paths<T> ConvertCPaths(T* paths)
{
    Paths<T> result;
    if (!paths) return 
        result;
    T* v = paths; ++v;
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

// get area, copy from TestExportHeaders.cpp

static bool CreatePolyPath64FromCPolyPath(CPolyPath64& v, PolyPath64& owner)
{
    int64_t poly_len = *v++, child_count = *v++;
    if (!poly_len) 
        return false;
    Path64 path;
    path.reserve(poly_len);
    for (size_t i = 0; i < poly_len; ++i)
    {
        int64_t x = *v++, y = *v++;
        path.push_back(Point64(x, y));
    }

    PolyPath64* new_owner = owner.AddChild(path);
    for (size_t i = 0; i < child_count; ++i)
        CreatePolyPath64FromCPolyPath(v, *new_owner);
    return true;
}

static bool BuildPolyTree64FromCPolyTree(CPolyTree64 tree, PolyTree64& result)
{
    result.Clear();
    int64_t* v = tree;
    int64_t array_len = *v++, child_count = *v++;
    for (size_t i = 0; i < child_count; ++i)
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

static bool CreatePolyPathDFromCPolyPath(CPolyPathD& v, PolyPathD& owner)
{
    int64_t poly_len = *v++, child_count = *v++;
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
    int64_t array_len = *v++, child_count = *v++;
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
