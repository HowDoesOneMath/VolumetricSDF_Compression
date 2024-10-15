#pragma once

#include "TestSuite.h"

#include <CImg.h>

#include "ArtifactCullingSuite.h"
#include "AttributeMapSuite.h"
#include "BadFileParsingSuite.h"
#include "BarycentricInterpolationSuite.h"
#include "CGAL_Marshalling_Suite.h"
#include "CGAL_RepairSuite.h"
#include "CImgSuite.h"
#include "ConvexHullSuite.h"
#include "DisplacementWaveletSuite.h"
#include "DracoSuite.h"
#include "EigenFileSavingSuite.h"
#include "ErrorMetricSuite.h"
#include "FFMPEG_Suite.h"
#include "LZ_Compression_Suite.h"
#include "MC_TestSuite.h"
#include "MeshToPointCloudSuite.h"
#include "MeshToSDF_Suite.h"
#include "MortonOrderSuite.h"
#include "PCA_Suite.h"
#include "PerfectJPEG_Suite.h"
#include "PushPullSuite.h"
#include "RaycastSuite.h"
#include "RectanglePackingSuite.h"
#include "RFind_Suite.h"
#include "RotationCaliperSuite.h"
#include "RunLengthSuite.h"
#include "SDF_SequenceSuite.h"
#include "SequenceFinderSuite.h"
#include "ShellSeparationSuite.h"
#include "SignCalculationsSuite.h"
#include "SubdivisionSuite.h"
#include "SVD_Suite.h"
#include "TemporalPCA_Suite.h"
#include "TextureDuplicatorSuite.h"
#include "TexturePaddingSuite.h"
#include "TriangleDistanceSuite.h"
#include "UnionFindSuite.h"
#include "UnsignedFieldSuite.h"
#include "UVAtlasSuite.h"
#include "VectorCompressionSuite.h"
#include "VSMC_TestSuite.h"
#include "VV_Mesh_Suite.h"
#include "VV_SDF_PCA_CompressionSuite.h"
#include "VV_SVD_TemporalSuite.h"
#include "WaveletTransformSuite.h"
#include "WavelibSuite.h"
#include "XOR_Mask_Suite.h"

class SuiteMaster
{
public:
    void run(int argc, char** argv)
    {
        cimg_library::cimg::openmp_mode(1);

        TestSuite* suite = nullptr;

        //suite = new ArtifactCullingSuite();
        //suite = new AttributeMapSuite();
        //suite = new BadFileParsingSuite();
        //suite = new BarycentricInterpolationSuite();
        //suite = new CGAL_Marshalling_Suite();
        //suite = new CGAL_RepairSuite();
        //suite = new CImgSuite();
        //suite = new ConvexHullSuite();
        //suite = new DisplacementWaveletSuite();
        //suite = new DracoSuite(); // <- Current Draco Compression
        //suite = new EigenFileSavingSuite();
        //suite = new ErrorMetricSuite();
        //suite = new FFMPEG_Suite();
        //suite = new LZ_Compression_Suite();
        //suite = new MC_TestSuite();
        //suite = new MeshToPointCloudSuite();
        //suite = new MeshToSDF_Suite();
        //suite = new MortonOrderSuite();
        //suite = new PCA_Suite();
        //suite = new PerfectJPEG_Suite();
        //suite = new PushPullSuite();
        //suite = new RaycastSuite();
        //suite = new RectanglePackingSuite();
        //suite = new RFind_Suite();
        //suite = new RotationCaliperSuite();
        //suite = new RunLengthSuite();
        //suite = new SDF_SequenceSuite();
        //suite = new SequenceFinderSuite();
        //suite = new ShellSeparationSuite();
        //suite = new SignCalculationsSuite();
        //suite = new SubdivisionSuite();
        //suite = new SVD_Suite();
        //suite = new TemporalPCA_Suite();
        //suite = new TextureDuplicatorSuite();
        //suite = new TexturePaddingSuite();
        //suite = new TriangleDistanceSuite();
        //suite = new UnionFindSuite();
        //suite = new UnsignedFieldSuite();
        //suite = new UVAtlasSuite();
        //suite = new VectorCompressionSuite();
        //suite = new VSMC_TestSuite(); // <- Current VSMC Compression
        //suite = new VV_Mesh_Suite();
        //suite = new VV_SDF_PCA_CompressionSuite();
        suite = new VV_SVD_TemporalSuite(); // <- Current SVD Compression
        //suite = new WaveletTransformSuite();
        //suite = new WavelibSuite();
        //suite = new XOR_Mask_Suite();

        suite->run(argc, argv);

        delete suite;
    }
};