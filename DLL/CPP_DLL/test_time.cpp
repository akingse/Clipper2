#include "pch.h"
//#include "clipper2/clipper.h"
using namespace std;
using namespace Eigen;
using namespace Clipper2Lib;

static constexpr size_t N_10E_6 = (size_t)1e6;
static constexpr double eps = FLT_EPSILON; //1e-7
static const size_t totalNum = N_10E_6;

inline Matrix4d rotz(double theta)
{
    Matrix4d R;
    R << cos(theta), -sin(theta), 0, 0,
        sin(theta), cos(theta), 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;
    return R;
}

inline Matrix4d translate(double x, double y, double z = 0.0)
{
    Matrix4d T;
    T << 1, 0, 0, x,
        0, 1, 0, y,
        0, 0, 1, z,
        0, 0, 0, 1;
    return T;
}

inline Matrix4d scale(double x, double y, double z = 1.0)
{
    Matrix4d T;
    T << x, 0, 0, 0,
        0, y, 0, 0,
        0, 0, z, 0,
        0, 0, 0, 1;
    return T;
}

inline PointD operator*(const Eigen::Matrix4d& mat, const PointD& point)
{
    Eigen::Vector3d vec(point.x, point.y, 0); //convert commute
    Eigen::Vector4d res = mat * vec.homogeneous();
    return PointD(res.x(), res.y());
}

inline PathD operator*(const Eigen::Matrix4d& mat, const PathD& points)
{
    PathD res(points.size());
    for (int i = 0; i < points.size(); ++i)//(const auto& iter : points)
        res[i] = mat * points[i];
    return res;
}


void testForTime(const std::vector<PathD>& pathVct1, const std::vector<PathD>& pathVct2)
{
    //time count
    clock_t timeStart, timeEnd;
    string timecout;
    std::vector<PathsD> pathsRes;
    for (int i = 0; i < 3; i++)
    {
        timeStart = clock();
        PathsD res;
        for (int i = 0; i < totalNum; i++)
        {
            //res = BooleanOp(ClipType::Union, FillRule::EvenOdd, { pathVct1[i]}, { pathVct2[i] }, 8);
            //res = BooleanOp(ClipType::Difference, FillRule::EvenOdd, { pathVct1[i]}, { pathVct2[i] }, 8);
            res = BooleanOp(ClipType::Difference, FillRule::EvenOdd, { pathVct1[i] }, { pathVct2[i] }, 8);
            pathsRes.emplace_back(res);
        }
        timeEnd = clock(); //completed
        cout << /*"time = " <<*/ double(timeEnd - timeStart) / CLOCKS_PER_SEC << "s" << endl;
        timecout += to_string(double(timeEnd - timeStart) / CLOCKS_PER_SEC);
        timecout += "s\n";
        Sleep(1000); //1s
    }
    //write file
    std::ofstream outputFile("time_result.txt");
    if (!outputFile.is_open()) {
        std::cerr << "无法打开文件！" << std::endl;
        return;
    }
    outputFile << timecout;
    outputFile.close();
    return;
}

void testForTime(const std::vector<PathsD>& pathVct1, const std::vector<PathsD>& pathVct2)
{
    //time count
    clock_t timeStart, timeEnd;
    string timecout;
    std::vector<PathsD> pathsRes;
    for (int i = 0; i < 3; i++)
    {
        timeStart = clock();
        PathsD res;
        for (int i = 0; i < totalNum; i++)
        {
            res = BooleanOp(ClipType::Union, FillRule::EvenOdd, pathVct1[i], pathVct2[i], 8);
            //res = BooleanOp(ClipType::Difference, FillRule::EvenOdd, { pathVct1[i]}, { pathVct2[i] }, 8);
            //res = BooleanOp(ClipType::Intersection, FillRule::EvenOdd, { pathVct1[i] }, { pathVct2[i] }, 8);
            pathsRes.emplace_back(res);
        }
        timeEnd = clock(); //completed
        cout << /*"time = " <<*/ double(timeEnd - timeStart) / CLOCKS_PER_SEC << "s" << endl;
        timecout += to_string(double(timeEnd - timeStart) / CLOCKS_PER_SEC);
        timecout += "s\n";
        Sleep(1000); //1s
    }
    //write file
    std::ofstream outputFile("time_result.txt");
    if (!outputFile.is_open()) {
        std::cerr << "无法打开文件！" << std::endl;
        return;
    }
    outputFile << timecout;
    outputFile.close();
    return;
}

static void test0()
{
    // 创建两个多边形路径
    //PathD path1;
    //path1.emplace_back(PointD(10.0, 10.0));
    //path1.emplace_back(PointD(50.0, 10.0));
    //path1.emplace_back(PointD(50.0, 50.0));
    //path1.emplace_back(PointD(10.0, 50.0));

    //PathD path2;
    //path2.emplace_back(PointD(10.0, 0.0));
    //path2.emplace_back(PointD(20.0, 0.0));
    //path2.emplace_back(PointD(20.0, 10.0));
    //path2.emplace_back(PointD(10.0, 10.0));

    PathD path1;
    path1.emplace_back(PointD(10.0, 10.0));
    path1.emplace_back(PointD(-10.0, 10.0));
    path1.emplace_back(PointD(-10.0, -10.0));
    path1.emplace_back(PointD(10.0, -10.0));
    PathD path2;
    //path2.emplace_back(PointD(0.0, 0.0));
    //path2.emplace_back(PointD(20.0, 0.0));
    //path2.emplace_back(PointD(20.0, 20.0));
    //path2.emplace_back(PointD(0.0, 20.0));
    path2.emplace_back(PointD(10.0, 0.0));
    path2.emplace_back(PointD(20.0, 0.0));
    path2.emplace_back(PointD(20.0, 20.0));
    path2.emplace_back(PointD(10.0, 20.0));

    PathD path3;
    path3.push_back(PointD(-10.0, 0.0));
    path3.push_back(PointD(10.0, 0.0));
    path3.push_back(PointD(0.0, 20.0));
    PathD path4;
    path4.push_back(PointD(0.0, 0.0));
    path4.push_back(PointD(20.0, 0.0));
    path4.push_back(PointD(10.0, 20.0));

    PathsD res0 = Clipper2Lib::BooleanOp(ClipType::Intersection, FillRule::EvenOdd, { path3 }, { path4 });

    PathsD solution0;

    // 执行布尔运算
    //Clipper2Lib::ClipperD c;
    //c.AddPath(path1, ptSubject, false);
    //c.AddPath(path2, ptClip, false);
    //c.Execute(ctUnion, solution, pftNonZero, pftNonZero);
     //do boolean
    //PathsD res1 = Clipper2Lib::BooleanOp(ClipType::Union, FillRule::EvenOdd, { path1 }, { path2 });
    //PathsD res2 = Clipper2Lib::BooleanOp(ClipType::Intersection, FillRule::EvenOdd, { path1 }, { path2 });
    //PathsD res3 = Clipper2Lib::BooleanOp(ClipType::Difference, FillRule::EvenOdd, { path1 }, { path2 });
    //PathsD res4 = Clipper2Lib::BooleanOp(ClipType::Xor, FillRule::EvenOdd, { path1 }, { path2 });
    //PathsD res5 = Clipper2Lib::BooleanOp(ClipType::None, FillRule::EvenOdd, { path1 }, { path2 });
    //// 
    path2 = translate(-eps, 0) * path2;

    PathsD res1 = Clipper2Lib::BooleanOp(ClipType::Intersection, FillRule::EvenOdd, { path1 }, { path2 });
    PathsD res2 = Clipper2Lib::BooleanOp(ClipType::Intersection, FillRule::EvenOdd, { path1 }, { path2 }, 8);
    PathsD res3 = Clipper2Lib::BooleanOp(ClipType::Intersection, FillRule::EvenOdd, { path1 }, { path2 }, 6);
    // 输出结果
    for (const auto& path : solution0)
    {
        for (const auto& point : path)
        {
            // 处理结果多边形的坐标点
            std::cout << "X: " << point.x << ", Y: " << point.y << std::endl;
        }
    }

    Paths64 subject;
    subject.push_back(MakePath({ 0,0, 0,5, 5,5, 5,0 }));
    subject.push_back(MakePath({ 1,1, 1,6, 6,6, 6,1 }));

    Clipper64 clipper;
    clipper.AddSubject(subject);

    PolyTree64 solution1;
    Paths64 open_paths;
    if (IsPositive(subject[0]))
        clipper.Execute(ClipType::Union, FillRule::Positive, solution1, open_paths);
    else
    {
        //because clipping ops normally return Positive solutions
        //clipper.ReverseSolution = true;
        clipper.Execute(ClipType::Union, FillRule::Negative, solution1, open_paths);
    }
    //测试洞
    PathD outerPolygon;
    outerPolygon.push_back(PointD(10, 10));
    outerPolygon.push_back(PointD(-10, 10));
    outerPolygon.push_back(PointD(-10, -10));
    outerPolygon.push_back(PointD(10, -10));
    PathD innerPolygon = scale(0.5, 0.5) * outerPolygon;
    PathsD pathH;
    pathH.push_back(outerPolygon);
    pathH.push_back(innerPolygon);
    PathD sub;
    sub.push_back(PointD(20, 10));
    sub.push_back(PointD(0, 10));
    sub.push_back(PointD(0, -10));
    sub.push_back(PointD(20, -10));

    PathsD res = BooleanOp(ClipType::Difference, FillRule::EvenOdd, pathH, { sub }, 8);
    return;
}

//三角形
static void test5()
{
    //const size_t totalNum = N_10E_6;
    typedef std::array<PointD, 4> rectangle;
    //std::array<PointD, 4>* randData4A = new std::array<PointD, 4>[totalNum];
    //std::array<PointD, 4>* randData4B = new std::array<PointD, 4>[totalNum];
    //int a = sizeof(array<PointD,2>);
    //vector<std::array<PointD, 4>> randData4A(totalNum);
    //vector<std::array<PointD, 4>> randData4B(totalNum);

    // 随机数生成器
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dis(-100.0, 100.0);
    std::vector<PathD> paths1;
    std::vector<PathD> paths2;

    for (int i = 0; i < totalNum; i++)
    {
        PathD path1;
        PathD path2;
        PointD tmp1 = PointD(dis(gen), dis(gen));
        path1.emplace_back(tmp1);
        PointD tmp2 = PointD(dis(gen), dis(gen));
        path1.emplace_back(tmp2);
        PointD tmp3 = PointD(dis(gen), dis(gen));
        path1.emplace_back(tmp3);
        //path2
        tmp1 = PointD(dis(gen), dis(gen));
        path2.emplace_back(tmp1);
        tmp2 = PointD(dis(gen), dis(gen));
        path2.emplace_back(tmp2);
        tmp3 = PointD(dis(gen), dis(gen));
        path2.emplace_back(tmp3);
        //triangle
        paths1.push_back(path1);
        paths2.push_back(path2);
    }
    //效率测试
    testForTime(paths1, paths2);
}

//矩形
static void test1()
{
    const size_t totalNum = N_10E_6;
    typedef std::array<PointD, 4> rectangle;
    //std::array<PointD, 4>* randData4A = new std::array<PointD, 4>[totalNum];
    //std::array<PointD, 4>* randData4B = new std::array<PointD, 4>[totalNum];
    //int a = sizeof(array<PointD,2>);
    //vector<std::array<PointD, 4>> randData4A(totalNum);
    //vector<std::array<PointD, 4>> randData4B(totalNum);

    // 随机数生成器
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dis(0.0, 100.0);
    std::vector<PathD> paths1;
    std::vector<PathD> paths2;

    for (int i = 0; i < totalNum; i++)
    {
        PathD path1;
        PathD path2;
        //for (int j = 0; j < 4; ++j) 
        //{
        //    path1.emplace_back(PointD(dis(gen), dis(gen)));
        //    path2.emplace_back(PointD(dis(gen), dis(gen)));
        //}
        PointD tmp1 = PointD(dis(gen), dis(gen));
        path1.emplace_back(tmp1);
        double leng = dis(gen);
        double heig = dis(gen);
        path1.emplace_back(translate(fabs(leng), 0) * tmp1);
        path1.emplace_back(translate(fabs(leng), fabs(heig)) * tmp1);
        path1.emplace_back(translate(0, fabs(heig)) * tmp1);
        //path2
        double leng2 = dis(gen);
        double heig2 = dis(gen);
        double py = dis(gen);
        path2.emplace_back(translate(fabs(leng) - eps, 0) * tmp1);
        path2.emplace_back(translate(fabs(leng) - eps + fabs(leng2), 0) * tmp1);
        path2.emplace_back(translate(fabs(leng) - eps + fabs(leng2), fabs(heig2)) * tmp1);
        path2.emplace_back(translate(fabs(leng) - eps, fabs(heig2)) * tmp1);

        paths1.push_back(path1);
        paths2.push_back(path2);
    }
    //效率测试
    testForTime(paths1, paths2);
}

//随机 4-10边形
static void test2()
{
    //准备数据
    std::srand(std::time(nullptr));
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dis(0.0, 100.0);

    std::vector<PathD> pathVct1;
    std::vector<PathD> pathVct2;
    for (int i = 0; i < totalNum; i++)
    {
        PathD path1;
        PathD path2;
        int N1 = std::rand() % 7 + 4;
        int N2 = std::rand() % 7 + 4;
        for (int j = 0; j < N1; ++j)
        {
            PointD tmp = PointD(dis(gen), dis(gen));
            path1.emplace_back(tmp);
        }
		PathD pathSim1 = Clipper2Lib::SimplifyPath(path1, 0, true);
        pathVct1.push_back(pathSim1);
        for (int j = 0; j < N2; ++j)
        {
            PointD tmp = PointD(dis(gen), dis(gen));
            path2.emplace_back(tmp);
        }
        PathD pathSim2 = Clipper2Lib::SimplifyPath(path2, 0, true);
		pathVct2.push_back(translate(60, 0, 0) * pathSim2);
    }
    testForTime(pathVct1, pathVct2);
}

//带洞多边形
static void test3()
{
    //准备数据
    std::srand(std::time(nullptr));
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dis(0.0, 100.0);

    std::vector<PathsD> pathVct1;
    std::vector<PathsD> pathVct2;
    for (int i = 0; i < totalNum; i++)
    {
        double x = dis(gen);
        double y = dis(gen);
        PathD path1;
        path1.emplace_back(PointD(x,y));
        path1.emplace_back(PointD(-x,y));
        path1.emplace_back(PointD(-x,-y));
        path1.emplace_back(PointD(x,-y));
        PathD path2 = scale(0.6, 0.6) * path1;
        PathsD paths1 = { path1 ,path2 };
        x = dis(gen);
        y = dis(gen);
        PathD path3;
        path3.emplace_back(PointD(x, y));
        path3.emplace_back(PointD(-x, y));
        path3.emplace_back(PointD(-x, -y));
        path3.emplace_back(PointD(x, -y));
        PathD path4 = scale(0.6, 0.6) * path3;
        PathsD paths2 = { translate(60,0) * path3 ,translate(60,0) * path4 };
        pathVct1.push_back(paths1);
        pathVct2.push_back(paths2);
    }
    testForTime(pathVct1, pathVct2);
}

//测试 PolyTreeD
static void test4()
{
    //Paths64 subject, clip, solution;
    PathsD subject, clip, solution;
    subject.push_back(MakePathD({ 100, 50, 10, 79, 65, 2, 65, 98, 10, 21 }));
    clip.push_back(MakePathD({ 98, 63, 4, 68, 77, 8, 52, 100, 19, 12 }));
    solution = Intersect(subject, clip, FillRule::EvenOdd);

    PathD path1;
    path1.push_back(PointD(1, 0));
    path1.push_back(PointD(0, 2));
    path1.push_back(PointD(-1, 0));
    PathD path2;
    path2.push_back(PointD(0, 0));
    path2.push_back(PointD(2, 0));
    path2.push_back(PointD(1, 2));
    //path2.push_back(PointD(1, -1));
    //path2.push_back(PointD(0, 1));
    //path2.push_back(PointD(-1, -1));
    PolyTreeD polytree;
    Clipper2Lib::BooleanOp(ClipType::Intersection, FillRule::EvenOdd, { path1 }, { path2 }, polytree, 8);
    double area = polytree.Area();
    return;
}

static constexpr uint8_t None = 0, Intersection = 1, Union = 2, Difference = 3, Xor = 4;
static constexpr uint8_t EvenOdd = 0, NonZero = 1, Positive = 2, Negative = 3;

//#include "clipper2/clipper.export.h" //function name conflict
#include "clipper2/clipperExportC.h"//C
using namespace Clipper;

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


static void testClipperFeatrue3()
{
    PathsD subj, clip, solution;
    PathD path1;
    //path1.push_back(PointD(1, 0));
    //path1.push_back(PointD(0, 2));
    //path1.push_back(PointD(-1, 0));
    path1.push_back(PointD(11, 12));
    path1.push_back(PointD(21, 22));
    path1.push_back(PointD(31, 32));
    subj.push_back(path1);
    PathD path2;
    path2.push_back(PointD(1, -1));
    path2.push_back(PointD(0, 1));
    path2.push_back(PointD(-1, -1));
    clip.push_back(path2);

    CPathsD c_subj = CreateCPaths(subj);
    CPathsD c_clip = CreateCPaths(clip);
    CPathsD c_solu = nullptr;
    //Clipper::BooleanOpD(Intersection, EvenOdd, c_subj, c_clip, c_solu, 2);
    solution = ConvertCPaths(c_solu);
    return;
}

static void testClipperFeatrue2()
{
    PathsD subj, clip, solution;
    //subj.push_back(MakePathD({ -10.0, 0.0,/**/ 10.0, 0.0, /**/ 0.0, 20.0}));
    //clip.push_back(MakePathD({ 0.0, 0.0, /**/ 20.0, 0.0, /**/ 10.0, 20.0 }));
    PathD path1;
    path1.push_back(PointD(10, 0));
    path1.push_back(PointD(0, 20));
    path1.push_back(PointD(-10, 0));
    path1.push_back(PointD(10, 0));
    subj.push_back(path1);
    PathD path2;
    path2.push_back(PointD(10, -10));
    path2.push_back(PointD(0, 10));
    path2.push_back(PointD(-10, -10));
    clip.push_back(path2);

    double d = 20;
    //for (int i = 1; i < 6; ++i)
    //    subj.push_back(MakePathD({ -i * d,-i * d, i * d,-i * d, i * d,i * d, -i * d,i * d }));
    //clip.push_back(MakePathD({ -90.,-120.,90.,-120., 90.,120., -90.,120. }));
    CPathsD c_subj_open = nullptr, c_solu = nullptr, c_solu_open = nullptr;

    CPathsD c_subj = CreateCPaths(subj);
    CPathsD c_clip = CreateCPaths(clip);
    // open polyline
    c_subj_open = CreateCPaths(subj);

    //Clipper::BooleanOpD(Intersection, EvenOdd,
    //    c_subj, c_subj_open, c_clip,
    //    c_solu, c_solu_open);
    solution = ConvertCPaths(c_solu);
    PathsD subj_open, solu_open;
    subj_open = ConvertCPaths(c_subj_open);
    solu_open = ConvertCPaths(c_solu_open);

    return;
}


static int _enrol = []()
{
    //test0();
    //test1();
    //test2();
    //test3();
    //test4();
    test5();
    return 0;
}();

