// pch.h: This is a precompiled header file.
// Files listed below are compiled only once, improving build performance for future builds.
// This also affects IntelliSense performance, including code completion and many code browsing features.
// However, files listed here are ALL re-compiled if any one of them is updated between builds.
// Do not add files here that you will be updating frequently as this negates the performance advantage.

#ifndef PCH_H
#define PCH_H

// add headers that you want to pre-compile here
#include "framework.h"

#endif //PCH_H

#define CLIPPER_DLLEXPORT_DEFINE
#define USING_EIGEN
#include "clipper2/clipper.h" //without clipper.export
#include <random>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <Eigen/Dense> 

//config release: generate Release\Clipper2_64.lib
// without copy, Example sln set library directory