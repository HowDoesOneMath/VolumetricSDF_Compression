#pragma once

#include <vector>
#include <fstream>

#include <fftw3.h>

#include "VV_Mesh.h"
#include "MeshPartition.h"
#include "VV_SaveFileBuffer.h"
#include "VV_CGAL_Marshaller.h"

#include "GridDataStruct.h"
#include "MarchingCubesSolver.h"

#include "MortonOrderer.h"

#include <queue>
#include <set>

#include "BasicGeometry.h"

class VV_TSDF
{
public:
	enum BufferType
	{
		DEFAULT,
		BYTE_REDUC,
		DEFLATE,
	};

	//const unsigned char max_c = 63;
	const unsigned char max_c = 255;
	const unsigned char min_c = 0;

	struct RaycastResultComparator
	{
		bool operator() (const std::pair<size_t, std::pair<bool, double>>& lhs, const std::pair<size_t, std::pair<bool, double>>& rhs) const
		{
			return lhs.second.second < rhs.second.second;
		}
	};
private:
	GridDataStruct gds;

	MortonOrderer mo;

    MeshPartition mp;
    std::shared_ptr<MarchingCubesSolver> mc_solver;

	VV_CGAL_Marshaller<CGAL::Simple_cartesian<double>::Point_3, Eigen::Vector3d> vcm;

    std::vector<double> double_grid;
	std::vector<unsigned char> quantized_grid;

	bool update_quantized = false;
    //std::vector<size_t> grid_triangle_groups;

    Eigen::Vector3d grid_lower_bound = Eigen::Vector3d(0, 0, 0);
    Eigen::Vector3d grid_upper_bound = Eigen::Vector3d(0, 0, 0);

    size_t span_y = 0;
    size_t span_x = 0;

	bool initialized = false;

	void RaycastDirection(VV_Mesh &mesh, Eigen::Vector3i& grid_dims, Eigen::Vector3d grid_corner, int raycast_dim, Eigen::Vector2i axis_dims, size_t* spans);

	void GetGridDistancesCGAL(VV_Mesh& mesh);

    void FitUnsignedTriangle(VV_Mesh *mesh, size_t triangle_index, double buffer_distance);

	double FindNearestTrianglePointOnTemporaryGrid(VV_Mesh& mesh, Eigen::Vector3i& lower_bound, Eigen::Vector3i& upper_bound, Eigen::Vector3i& center);

	void CullInteriorShellsInTemporaryGrid(VV_Mesh& mesh);

	void S_TraversalFillTemporaryGrid(VV_Mesh& mesh, int reach, double buffer_distance);

	bool CheckIfInterior(size_t test_loc, size_t previous_loc, int axis, bool currently_interior); //, bool evaluating_self);//, Eigen::Vector3i grid_coords);

    void RecalculateBounds();

public:
	void ClearGrid();
	void RefreshGrid();

	bool InitializeGrid(GridDataStruct &gds);

	void QuantizeGrid();
	void ClampUnquantizedGrid();

	void SubtractFromGrid(unsigned char* data_array);
	void AddToGrid(unsigned char* data_array);

    bool MakeDebuggingSphere(Eigen::Vector3d center, double radius, std::string filename);

	std::shared_ptr<VV_Mesh> CastMeshUnsignedDistance(VV_Mesh* mesh, double buffer_distance);

	std::shared_ptr<VV_Mesh> CastMeshUnsignedDistanceSampleAll(VV_Mesh* mesh, double buffer_distance);

	void CastMeshNaiveRaycast(VV_Mesh& mesh);

	std::shared_ptr<VV_Mesh> ExtractMeshQuantized();

	std::shared_ptr<std::vector<size_t>> ExtractTriangleGroupsFromBlock(Eigen::Vector3i block_start, Eigen::Vector3i block_end);

	std::shared_ptr<std::vector<std::vector<size_t>>> ExtractTriangleGroupsByBlock(VV_Mesh& mesh, Eigen::Vector3i block_size);

	size_t CastGridDataStruct(VV_SaveFileBuffer& buffer);
	size_t RetrieveGridDataStruct(VV_SaveFileBuffer& buffer);

	void SmoothUnquantizedGrid();

	void PerformSmoothingOverUnquantizedBlock(size_t x, size_t y, size_t z, size_t len_x, size_t len_y, size_t len_z);

	void ExtractBlock(size_t x, size_t y, size_t z, size_t len_x, size_t len_y, size_t len_z, unsigned char* presized_block_array);
	void InsertBlock(size_t x, size_t y, size_t z, size_t len_x, size_t len_y, size_t len_z, unsigned char* presized_block_array);

	void InsertBlock(size_t x, size_t y, size_t z, size_t len_x, size_t len_y, size_t len_z, Eigen::MatrixXd &mat, double average);

	void ExtractBlockUnquantized(size_t x, size_t y, size_t z, size_t len_x, size_t len_y, size_t len_z, double* presized_block_array);
	void InsertBlockUnquantized(size_t x, size_t y, size_t z, size_t len_x, size_t len_y, size_t len_z, double* presized_block_array);

	void ExtractBlockMortonOrder(size_t x, size_t y, size_t z, size_t data_size, unsigned char* presized_block_array);
	void InsertBlockMortonOrder(size_t x, size_t y, size_t z, size_t data_size, unsigned char* presized_block_array);

	void ExtractBlockS_Traversal(size_t x, size_t y, size_t z, size_t len_x, size_t len_y, size_t len_z, unsigned char* presized_block_array);
	void InsertBlockS_Traversal(size_t x, size_t y, size_t z, size_t len_x, size_t len_y, size_t len_z, unsigned char* presized_block_array);

	void ExtractBlockUnquantizedS_Traversal(size_t x, size_t y, size_t z, size_t len_x, size_t len_y, size_t len_z, double* presized_block_array);
	void InsertBlockUnquantizedS_Traversal(size_t x, size_t y, size_t z, size_t len_x, size_t len_y, size_t len_z, double* presized_block_array);

	void SetQuantizedBlock(size_t x, size_t y, size_t z, size_t len_x, size_t len_y, size_t len_z, unsigned char value);

	size_t GetSpanX() { return span_x; }
	size_t GetSpanY() { return span_y; }

	unsigned char* GetQuantizedGridPointer() { return quantized_grid.data(); }
	size_t GetQuantizedGridLength() { return quantized_grid.size(); }

	size_t GetDimX() { return gds.dim_x; }
	size_t GetDimY() { return gds.dim_y; }
	size_t GetDimZ() { return gds.dim_z; }

	void HarvestSigns(std::vector<unsigned char> &sign_vector);
	void ApplySigns(std::vector<unsigned char>& sign_vector);

	void PrintCrossSectionOfSolver(size_t y_level);
	void PrintCrossSectionOfDoubleGrid(size_t y_level);
	void PrintCrossSectionOfQuantizedGrid(size_t y_level);
	Eigen::Vector3i GetDecodedGridLoc(size_t loc) { return Eigen::Vector3i((int)(loc / span_x), (int)((loc % span_x) / span_y), (int)(loc % span_y)); }

	std::shared_ptr<MarchingCubesSolver> GetSolver() { return mc_solver; }
};
