#pragma once

#include <string>

inline std::string GetDrivePath()
{
	return "E:";
}

inline std::string GetWorkingDataPath()
{
	return GetDrivePath() + "/_VV_DATA";
}

inline std::string GetCompressionPath()
{
	return GetWorkingDataPath() + "/_COMPRESSIONS";
}

inline std::string GetReconstructionPath()
{
	return GetWorkingDataPath() + "/_RECONSTRUCTIONS";
}

inline std::string GetDatasetsPath()
{
	return GetWorkingDataPath() + "/_VV_DATASETS_TRIMMED";
}

inline std::string GetSDF_MeshDatasetsPath()
{
	return GetWorkingDataPath() + "/_VV_DATASETS_TRIMMED_SDF";
}