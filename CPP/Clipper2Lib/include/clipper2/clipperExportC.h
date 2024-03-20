#pragma once
#ifdef _WIN32
#define EXTERN_DLL_EXPORT extern "C" __declspec(dllexport)
#else
#define EXTERN_DLL_EXPORT extern "C" 
#endif

namespace Clipper
{
    typedef int64_t* CPath64;
    typedef int64_t* CPaths64;
	typedef double* CPathD;
	typedef double* CPathsD;
    typedef int64_t* CPolyPath64;
    typedef int64_t* CPolyTree64;
    typedef double* CPolyPathD; 
    typedef double* CPolyTreeD;

	// Boolean clipping:
	// cliptype: None=0, Intersection=1, Union=2, Difference=3, Xor=4
	// fillrule: EvenOdd=0, NonZero=1, Positive=2, Negative=3

    EXTERN_DLL_EXPORT int BooleanOpD(uint8_t cliptype, uint8_t fillrule,
        const CPathsD subjects, const CPathsD clips, CPathsD& solution, int precision);

    EXTERN_DLL_EXPORT int BooleanOp64(uint8_t cliptype, uint8_t fillrule, 
        const CPaths64 subjects, const CPaths64 clips, CPaths64& solution);

    EXTERN_DLL_EXPORT double getAreaOfPolyTreeD(const CPathsD subjects);

    EXTERN_DLL_EXPORT double getAreaOfPolyTree64(const CPaths64 subjects);

    EXTERN_DLL_EXPORT inline void DisposeExportedCPathsD(CPathsD& p)
    {
        delete[] p;
        p = nullptr;
    }

}
