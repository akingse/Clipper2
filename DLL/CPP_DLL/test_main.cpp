#include "pch.h"

int main()
{
	return 0;
}

#include "clipper2/clipperExportC.h"
namespace Clipper
{
    typedef std::vector<std::vector<Eigen::Vector2d>> PathsEigen;

    static const uint8_t EvenOdd = 0, Intersection = 1, Union = 2, Difference = 3;
    static const int mdp = 8;
    static constexpr double eps = 1e-8; //FLT_EPSILON; //
}

using namespace std;
using namespace Eigen;
using namespace Clipper;
using namespace Clipper2Lib;

inline double* createCPaths(const std::array<Eigen::Vector2d, 3>& path, const Eigen::Vector2d& relative = Eigen::Vector2d(0, 0))
{
    double* result = new double[10];
    double* v = result;
    *v++ = 10.0;//array_len
    *v++ = 1.0; //cnt
    *v++ = 3.0; //path.size();
    *v++ = 0.0;
    for (const auto& pt : path)//m_triangle2d
    {
        *v++ = pt[0] + relative[0];
        *v++ = pt[1] + relative[1];
    }
    return result;
}

inline double* createCPaths(const vector<vector<Eigen::Vector2d>>& paths, const Eigen::Vector2d& relative = Eigen::Vector2d(0, 0))
{
    size_t cnt, array_len;
    auto _getPathCountAndCPathsArrayLen = [&]()
    {
        array_len = 2;
        cnt = 0;
        for (const auto& path : paths)
        {
            array_len += path.size() * 2 + 2; //path.m_triangle2d.size
            ++cnt;
        }
    };
    _getPathCountAndCPathsArrayLen();
    double* result = new double[array_len];
    double* v = result;
    *v++ = (double)array_len;
    *v++ = (double)cnt;
    for (const auto& path : paths)
    {
        *v++ = (double)path.size();
        *v++ = 0;
        for (const auto& pt : path)
        {
            *v++ = pt[0] + relative[0];
            *v++ = pt[1] + relative[1];
        }
    }
    return result;
}

inline std::vector<std::vector<Eigen::Vector2d>> convertCPaths(const CPathsD paths, const Eigen::Vector2d& relative = Eigen::Vector2d(0, 0))
{
    std::vector<std::vector<Eigen::Vector2d>> result;
    if (!paths)
        return result;
    double* v = paths; ++v;
    size_t cnt = (size_t)*v++;
    result.reserve(cnt);
    for (size_t i = 0; i < cnt; ++i)
    {
        size_t cnt2 = (size_t)*v;
        v += 2;
        std::vector<Eigen::Vector2d> path;
        path.reserve(cnt2);
        for (size_t j = 0; j < cnt2; ++j)
        {
            double x = *v++, y = *v++;
            path.push_back(Eigen::Vector2d(x, y) + relative);
        }
        result.push_back(path);
    }
    return result;
}

Paths64 operator*(const double scale, const PathsD& paths)
{
    Paths64 result;
    for (const auto& path : paths)
    {
        Path64 temp;
        for (const auto& iter : path)
        {
            PointD pt = iter * scale;
            temp.push_back(Point64(pt.x, pt.y));
        }
        result.push_back(temp);
    }
    return result;
}

static PathsD ConvertCPaths(double* paths)
{
    PathsD result;
    if (!paths)
        return result;
    double* v = paths; ++v;
    size_t cnt = *v++;
    result.reserve(cnt);
    for (size_t i = 0; i < cnt; ++i)
    {
        size_t cnt2 = *v;
        v += 2;
        PathD path;
        path.reserve(cnt2);
        for (size_t j = 0; j < cnt2; ++j)
        {
            double x = *v++, y = *v++;
            path.push_back(PointD(x, y));
        }
        result.push_back(path);
    }
    return result;
}

typedef std::array<Eigen::Vector2d, 3> Triangle2d;

//四舍五入
static void test0()
{
    double a = 3.74159;
    __m128d xmm_a = _mm_set_sd(a);
    __int64 result = _mm_cvtsd_si64(xmm_a); 
    int64_t b1 = std::round(a);// 
    int64_t b2 = std::nearbyint(a);
    auto type = typeid(result).name();

    Triangle2d trigon2, trigon3;
    trigon2 = {
        //Vector2d(1.00000100,0.99999900), //limit
        //Vector2d(1.00000001,0.99999999),
        Vector2d(1.000000001,0.999999999),
        Vector2d(10,0),
        Vector2d(0,10), };
    trigon3 = {
        trigon2[0],
        Vector2d(-10,0),
        Vector2d(0,10), };
    CPathsD c_subj0 = createCPaths(trigon2);
    CPathsD c_clipI = createCPaths(trigon2);
    CPathsD c_solu = nullptr;

    BooleanOpD(Clipper::Union, EvenOdd, c_subj0, c_clipI, c_solu, mdp);
    vector<vector<Eigen::Vector2d>> solution = convertCPaths(c_solu);

    return;
}

static void test1()
{
    Point64 offPt(0, 10);
    Point64 seg1(0, 0);
    Point64 seg2(10, 10);
    Point64 cpoint = GetClosestPointOnSegment(offPt, seg1, seg2);

    Triangle2d trigon0, trigon1;
    trigon0 = {
        Vector2d(0,0),
        Vector2d(10,0),
        Vector2d(0,10), };
    trigon1 = {
        Vector2d(5,0),
        Vector2d(15,0),
        Vector2d(5,10), };
    CPathsD c_subj0 = createCPaths(trigon0);
    CPathsD c_clipI = createCPaths(trigon1);
    CPathsD c_solu = nullptr;

    BooleanOpD(Clipper::Union, EvenOdd, c_subj0, c_clipI, c_solu, mdp);
    vector<vector<Eigen::Vector2d>> solution = convertCPaths(c_solu);

    ////Hexagram
    //vector<vector<Eigen::Vector2d>> poly0 = { {
    //    Vector2d(-5,0),
    //    Vector2d(5,0),
    //    Vector2d(0,10),
    //    } };
    //vector<vector<Eigen::Vector2d>> poly1 = { {
    //    Vector2d(0,-5),
    //    Vector2d(5,5),
    //    Vector2d(-5,5),
    //    } };

// without horz_seg_list_
    vector<vector<Eigen::Vector2d>> poly0 = { {
        Vector2d(0,0),
        Vector2d(10,5),
        Vector2d(0,10),
        } };
    vector<vector<Eigen::Vector2d>> poly1 = { {
        Vector2d(5,0),
        Vector2d(5,10),
        Vector2d(-5,5),
        } };

    BooleanOpD(Clipper::Union, EvenOdd, createCPaths(poly0), createCPaths(poly1), c_solu, 8); //mdp=8/7
    vector<vector<Eigen::Vector2d>> solution0 = convertCPaths(c_solu);

    return;
}

//测试平行四边形
static void test2()
{
    vector<vector<Eigen::Vector2d>> polyOut = { {
        Vector2d(39.243835735550896, 33.800647400000003),
        Vector2d(23.160033206380934, 33.800647400000003),
        Vector2d(15.356164052838400, 13.513218000000000),
        Vector2d(31.439966657301248, 13.513218000000000),
    } };
    vector<vector<Eigen::Vector2d>> polyIn = { {
        Vector2d(36.321815876758215, 26.204380600000000),
        Vector2d(20.238013319396178, 26.204380600000000),
        Vector2d(18.278183939823158, 21.109484800000001),
        Vector2d(34.361986516093928, 21.109484800000001),
    } };


    CPathsD c_subj0 = createCPaths(polyOut);
    CPathsD c_clipI = createCPaths(polyIn);
    CPathsD c_solu = nullptr;
    //mdp=8 success | mdp=7 fail
    BooleanOpD(Clipper::Difference, EvenOdd, c_subj0, c_clipI, c_solu, 8); 
    vector<vector<Eigen::Vector2d>> solution = convertCPaths(c_solu);

    return;
}

//单坐标误差
static void test3()
{
    // from p3d
    PathsEigen trigon0 = { {
        Vector2d(-8361.9297704215405, 1500.0000000000000),
        Vector2d(-8445.5313397241498, 1504.8036798991927),
        Vector2d(-8487.6843035733509, 1500.0000000000000),
        } };

    PathsEigen trigon1 = { {
        Vector2d(-8444.6157833588732, 1519.0301168721785),
        Vector2d(-8767.6897247666548, 0.0000000000000000),
        Vector2d(-8404.0827342707416, 1504.8036798991925),
    } };

    PathsEigen trigon2 = { {
        Vector2d(-8481.9712553406735, 1542.1325969243635),
        Vector2d(-8767.6897247666548, 0.0000000000000000),
        Vector2d(-8444.6157833588732, 1519.0301168721785),
    } };

    vector<PathsEigen> trigonVct;// = { trigon0 ,trigon1 ,trigon2 };
    CPathsD c_subj0 = nullptr;
    CPathsD c_solu = nullptr;
    for (int i = 0; i < trigonVct.size(); ++i)
    {
        CPathsD c_clipI = createCPaths(trigonVct[i]);
        BooleanOpD(Clipper::Union, EvenOdd, c_subj0, c_clipI, c_solu, mdp);
        CPathsD c_subj = c_subj0;
        c_subj0 = c_solu;
        //without delete
    }
    PathsEigen solution1 = convertCPaths(c_solu);
    //共边三角形
    PathsEigen poly0;
    PathsEigen poly1;
    c_solu = nullptr;
    poly0 = { {
        Vector2d(0,0),
        Vector2d(5,5),
        Vector2d(0,10),
    } };
    poly1 = { {
        Vector2d(0,0),
        Vector2d(10,2), 
        //Vector2d(5 + eps,5), //右
        Vector2d(5 + 2 * eps,5 + eps), //右上
        //Vector2d(5,5 - eps), //下
        //Vector2d(5 - eps,5 - 2 * eps), //左下
        //Vector2d(5 + eps,5 - eps), //double dir

    } };

    //BooleanOpD(Clipper::Union, EvenOdd, createCPaths(poly0), createCPaths(poly1), c_solu, 6); //merge
    BooleanOpD(Clipper::Union, EvenOdd, createCPaths(poly0), createCPaths(poly1), c_solu, 8); //distingush
    PathsEigen solution2 = convertCPaths(c_solu);

    return;
}

//水平线误差
static void test4()
{
    PathsEigen poly0;
    PathsEigen poly1;

    //两个水平边
    poly0 = { {
        Vector2d(0,0),
        Vector2d(10,0),
        Vector2d(0,10),
    } };
    //poly0 = { {
    //    Vector2d(0,0 - eps),
    //    Vector2d(10,0 - eps),
    //    Vector2d(0,10),
    //} };

    //poly1 = { { //collinar
    //    Vector2d(0,0),
    //    Vector2d(15,0),
    //    Vector2d(0,-10),
    //} };

    //with small gap
    poly1 = { {
        Vector2d(0,0 - eps),
        Vector2d(10,0 - eps),
        Vector2d(0,-10),
    } };

    //poly1 = { {
    //    Vector2d(5,0 - eps),
    //    Vector2d(15,0 - eps),
    //    Vector2d(0,-10),
    //} };

    //poly1 = { {
    //    Vector2d(-5,0 - eps),
    //    Vector2d(15,0 - eps),
    //    Vector2d(0,-10),
    //} };

    //poly1 = { {
    //    // compat next isnot horizon
    //    Vector2d(0,0 - eps),
    //    Vector2d(10,-5),
    //    Vector2d(0,-10),
    //} };

    //两个竖直边
    //poly1 = { {
    //    Vector2d(0 - eps,0),
    //    Vector2d(0 - eps,10),
    //    Vector2d(-10,0),
    //} };
    ////临边平行
    //poly0 = { {
    //    Vector2d(0,0),
    //    Vector2d(5,5),
    //    Vector2d(0,10),
    //} };
    //poly1 = { { //x+
    //    Vector2d(0 + eps,0),
    //    Vector2d(10,0),
    //    Vector2d(5 + eps,5),
    //} };
    //poly1 = { { //y-
    //    Vector2d(0,0 - eps),
    //    Vector2d(10,0),
    //    Vector2d(5,5 - eps),
    //} };
    CPathsD c_solu = nullptr;
    BooleanOpD(Clipper::Union, EvenOdd, createCPaths(poly0), createCPaths(poly1), c_solu, 8); //distingush
    PathsD solution = ConvertCPaths(c_solu);
    return;
}

//水平线误差-复杂
static void test6()
{
    PathsEigen poly0;
    PathsEigen poly1;

    //两个水平边
    poly0 = { {
        Vector2d(0,0),
        Vector2d(10,0),
        Vector2d(0,10),
    } };
    //poly0 = { {
    //    Vector2d(0,0 + eps),
    //    Vector2d(10,0 + eps),
    //    Vector2d(0,10),
    //} };

    poly1 = { { //collinar
        Vector2d(0,0),
        Vector2d(15,0),
        Vector2d(20,10),
        Vector2d(20,0),
        Vector2d(0,-10),
    } };

    CPathsD c_solu = nullptr;
    BooleanOpD(Clipper::Union, EvenOdd, createCPaths(poly0), createCPaths(poly1), c_solu, 8); //distingush
    //PathsEigen solution2 = convertCPaths(c_solu);
    PathsD solution1 = ConvertCPaths(c_solu);
    return;
}

//X两边相交
static void test5()
{
    PathsEigen poly0;
    PathsEigen poly1;
    poly0 = { {
        Vector2d(0,0),
        Vector2d(10,10),
        Vector2d(0,10), //if horizon
    } };
    poly1 = { {
       Vector2d(0 + eps,0),
       Vector2d(10,0),
       Vector2d(10 - eps,10),
    } };
    //双向
    poly1 = { {
       Vector2d(0 + eps,0 - eps),
       Vector2d(10,0),
       Vector2d(10 - eps,10 + eps),
    } };

    CPathsD c_solu = nullptr;
    BooleanOpD(Clipper::Union, EvenOdd, createCPaths(poly0), createCPaths(poly1), c_solu, 8); //distingush
    PathsEigen solution = convertCPaths(c_solu);

    IOFormat CleanFmt(2, 0, ", ", "\n", "[", "]");
    for (const auto& path : solution)
    {
        cout << "path-" << endl;
        for (const auto& iter : path)
        {
            cout << iter.format(CleanFmt) << endl;
        }
    }
    return;
}

//两端分离
static void test7()
{
    PathsEigen poly0;
    PathsEigen poly1;
    poly0 = { { //upper
        Vector2d(7109.864100364, 189.168143176),//-
        Vector2d(7165.356935344, 298.800392896),     //up
        Vector2d(6194.089807894, 185.205059916),
        Vector2d(6194.278057694, 184.221562696),//- min scanline
    } };

    poly1 = { { //under
        Vector2d(7109.864100365, 189.168143169),
        Vector2d(6194.278057715, 184.221562699),
        Vector2d(6213.399798845, 084.321435249), //down
    } };

    CPathsD c_solu = nullptr;
    BooleanOpD(Clipper::Union, EvenOdd, createCPaths(poly0), createCPaths(poly1), c_solu, 8);
    PathsD solution = ConvertCPaths(c_solu);

    return;
}

//缺口三角形
static void test8()
{
    PathsEigen poly0;
    PathsEigen poly1;
    PathsEigen poly2;
    double k = 1e-8;
    int64_t y = 1e11;

    poly0 = { {
        k * Vector2d(0,0),
        k * Vector2d(13270919859,-69362172378+y),
        k * Vector2d(3680285322, -19242782915+y),//peak
        //k * Vector2d(3680283376, -19242772753+y),//peak
        k * Vector2d(13274767281,-69356501865+y),
        k * Vector2d(1e11,y),
        k * Vector2d(0,y),
    } };
    poly1 = { {
        k * Vector2d(13270919858,-69362172378+y),
        k * Vector2d(3680283377, -19242772756+y), //peak
        k * Vector2d(13274767280,-69356501864+y),
    } };

    CPathsD c_solu = nullptr;
    BooleanOpD(Clipper::Union, EvenOdd, createCPaths(poly0), createCPaths(poly1), c_solu, 8);
    PathsD solution = ConvertCPaths(c_solu);
    return;
}

//布尔大误差
static void test9()
{
    double k = 1e-8;
    PointD ln1a = PointD(-742.98265717825677, -7243.9818210315834);
    PointD ln1b = PointD(-647.07629236825687, -7745.1758172515838);
    PointD ln2a = PointD(-742.98265718465132, -7243.9818210284675);
    PointD ln2b = PointD(-647.03781814465128, -7745.1191121184675);
    PointD ip;
    //GetSegmentIntersectPt(ln1a, ln1b, ln2a, ln2b, ip);

    PathsEigen poly0;
    PathsEigen poly1;
    PathsEigen poly2;
    poly0 = { {
        k * Vector2d(33706423263, -39243345602),
        k * Vector2d(36721325486, -34799837850),
        k * Vector2d(36308198167, -32641998951),
        k * Vector2d(35362682849, -27698266489),
        k * Vector2d(27066658209, -21200306227),
        k * Vector2d(  -10819377 , 30836082),
        k * Vector2d(0 , 0),
        k * Vector2d( 3680283377, -19242772756), //peak
        k * Vector2d(13274767281, -69356501865), // under2
    } };
    poly1 = { {
        k * Vector2d(13270919859 , -69362172378), //under1
        k * Vector2d(3680283378 ,-19242772756), //peak
        k * Vector2d(-12744783073, 5786039672),
        k * Vector2d(-15564438717, 5456264564),
        k * Vector2d(11708028477 , -71665636789),
    } };
    //{x=36 8028 33759999999 y=-192 427 72753000000 } //peak
    //{x=36 8028 53220000003 y=-192 427 82915000001 }

    poly0 = { {
        Vector2d(0,0),
        Vector2d(10,10),
        Vector2d(5,30),//peak
        Vector2d(0,35),
    } };
    poly1 = { {
        Vector2d(10.1,10.1),
        //Vector2d(5 - eps,30 + eps),//peak
        Vector2d(5 - 2*eps,30 + eps),//peak-left
        //Vector2d(5 - eps,30 + 2*eps),//peak
        Vector2d(20,45),
    } };

    CPathsD c_solu = nullptr;
    BooleanOpD(Clipper::Union, EvenOdd, createCPaths(poly0), createCPaths(poly1), c_solu, 8);
    Paths64 solution = 1e8 * ConvertCPaths(c_solu);
    PathsD solution2 = ConvertCPaths(c_solu);
    return;
}

static void test10()
{
    PathsEigen poly0;
    PathsEigen poly1;
    //poly0 = { { //alright
    //    Vector2d(-1919.1719573000000, 1868.7277368100001), //16
    //    Vector2d(-1926.6945369499999, 1883.2844434600001), //17-
    //    Vector2d(-2206.4174202100003, 1953.2571552900001), //18-
    //    Vector2d(-2213.8008594299999, 1946.9897900100000), //19
    //} };

    //poly1 = { { //T115[size=11]
    //    Vector2d(-2200.9861930800002, 1957.8674030800000),//8
    //    Vector2d(-2206.4174202100003, 1953.2571552900001),//9-
    //    Vector2d(-1926.6945369499999, 1883.2844434600001),//10-
    //    Vector2d(-1929.5926849500001, 1888.8925582500001),//0
    //} };

    poly0 = { { //c_subj0-trigon
        Vector2d(-1772.604825370625, 600.000000000029),
        Vector2d(-0547.604825320203, 650.000000000029),
        Vector2d(-0547.604825320203, 550.000000000029),
    } };

    poly1 = { { //c_clipI-shielded
        Vector2d(-1772.604825353541, 600.000000000024),
        Vector2d(-0547.604825353526, 550.000000000024),
        Vector2d(-0547.604825353526, 650.000000000024),
    } };

    CPathsD c_solu = nullptr;
    BooleanOpD(Clipper::Difference, EvenOdd, createCPaths(poly0), createCPaths(poly1), c_solu, 8);
    Paths64 solution = 1e8 * ConvertCPaths(c_solu);
    PathsD solution2 = ConvertCPaths(c_solu);
    return;
}

static int _enrol = []()
{
    //test0();
    //test1();
    //test2();
    //test3();
    //test5();
    //test4();
    //test6();
    //test7();
    //test8();
    //test9();
    test10();
    return 0;
}();

