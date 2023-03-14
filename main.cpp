#include <iostream>
#include <fstream>
#include <sstream>
#include <vector> 

#include <boost/filesystem.hpp>
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/transformation_estimation_point_to_plane.h>
#include <pcl/registration/ndt.h>

using namespace std;
using namespace pcl;


template <typename T>
std::string toString(T other_type) {
	std::stringstream s;
	s << other_type;
	return s.str();
}

// Load .bin files and return point cloud object
PointCloud<PointXYZI>::Ptr MyFileOpen(const string& filename)
{
	// Input binary files
	ifstream input(filename, ios::in | ios::binary);
	if (!input.good()) {
		exit(EXIT_FAILURE);
	}
	input.seekg(0, ios::beg);

	PointCloud<PointXYZI>::Ptr points(new pcl::PointCloud<PointXYZI>);

	// The format of Velodyne data is (x , y , z , reflection) per line in .bin file.
	int i;
	for (i = 0; input.good() && !input.eof(); i++) {
		PointXYZI point;
		input.read((char *)&point.x, 3 * sizeof(float));
		input.read((char *)&point.intensity, sizeof(float));
		points->push_back(point);
	}
	input.close();
	return points;
}

void SavePointCloud(string& dir, pcl::PointCloud<pcl::PointXYZI>::Ptr pt)
{
	ofstream file_out(dir);
	for (size_t i = 0; i < pt->points.size(); ++i)
	{
		file_out << setprecision(3) << fixed
			<< pt->points[i].x << " "
			<< pt->points[i].y << " "
			<< pt->points[i].z << " "
			<< pt->points[i].intensity << " "
			<< endl;
	}
	file_out.close();

	cout << "Finish to save: " << dir << endl;
}

// Output Homogeneous matrices to text file
void MyFileOutput_Mat(const vector<Eigen::Matrix4f>& output, string& path)
{
	ofstream OutPut_File;
	OutPut_File.open(path);
	for (auto& row : output)
	{
		OutPut_File << setprecision(6) << fixed << row;
		OutPut_File << '\n';
	}
	OutPut_File.close();
}

void SavePose_Mat(const string& path, const vector<Eigen::Matrix4f>& poseMat)
{
	ofstream output(path, ios::out);
	for (size_t i = 0; i < poseMat.size(); i++)
	{
		output << setprecision(8) << fixed
			<< poseMat[i](0, 0) << " " << poseMat[i](0, 1) << " " << poseMat[i](0, 2) << " " << poseMat[i](0, 3) << " "
			<< poseMat[i](1, 0) << " " << poseMat[i](1, 1) << " " << poseMat[i](1, 2) << " " << poseMat[i](1, 3) << " "
			<< poseMat[i](2, 0) << " " << poseMat[i](2, 1) << " " << poseMat[i](2, 2) << " " << poseMat[i](2, 3) << endl;
	}
	output.close();
}

// Define a new point representation for < x, y, z, curvature >
class MyPointRepresentation : public PointRepresentation <PointNormal>
{
	using PointRepresentation<PointNormal>::nr_dimensions_;
public:
	MyPointRepresentation()
	{
		// Define the number of dimensions
		nr_dimensions_ = 4;
	}

	// Override the copyToFloatArray method to define our feature vector
	virtual void copyToFloatArray(const PointNormal &p, float * out) const
	{
		// < x, y, z, curvature >
		out[0] = p.x;
		out[1] = p.y;
		out[2] = p.z;
		out[3] = p.curvature;
	}
};

PointCloud<PointXYZI>::Ptr Trans(PointCloud<PointXYZI> points, Eigen::Matrix4f& arr)
{
	PointCloud<PointXYZI> ps;
	for (int pix = 0; pix < points.size(); pix++)
	{
		PointXYZI p;
		p.x = arr(0, 0) * points[pix].x + arr(0, 1) * points[pix].y + arr(0, 2) * points[pix].z + arr(0, 3);
		p.y = arr(1, 0) * points[pix].x + arr(1, 1) * points[pix].y + arr(1, 2) * points[pix].z + arr(1, 3);
		p.z = arr(2, 0) * points[pix].x + arr(2, 1) * points[pix].y + arr(2, 2) * points[pix].z + arr(2, 3);
		p.intensity = points[pix].intensity;
		ps.points.push_back(p);
	}
	return ps.makeShared();
}

Eigen::Matrix4f pose_Lidar2Cam(const Eigen::Matrix4f& pose_lidar)
{
	Eigen::Matrix3f rot_cam2velo;
	rot_cam2velo << 0.0, 0.0, 1.0,
		-1.0, 0.0, 0.0,
		0.0, -1.0, 0.0;
	Eigen::Matrix3f rot_velo2cam = rot_cam2velo.inverse(); // 0 -1 0; 0 0 -1; 1 0 0
	Eigen::Matrix3f pose_mat_lidar = pose_lidar.block(0, 0, 3, 3);
	Eigen::Matrix3f pose_mat_cam = rot_velo2cam * pose_mat_lidar * rot_cam2velo;

	Eigen::Matrix4f pose_cam;
	pose_cam << pose_mat_cam(0, 0), pose_mat_cam(0, 1), pose_mat_cam(0, 2), -pose_lidar(1, 3),
		pose_mat_cam(1, 0), pose_mat_cam(1, 1), pose_mat_cam(1, 2), -pose_lidar(2, 3),
		pose_mat_cam(2, 0), pose_mat_cam(2, 1), pose_mat_cam(2, 2), pose_lidar(0, 3),
		0.0, 0.0, 0.0, 1.0;

	return pose_cam;
}


//std::vector<std::string> GetDirectoryFiles(const std::string& path, const std::string& exdName)
//{
//	std::vector<std::string> _list;
//	std::string pattern(path);
//	pattern.append("/*." + exdName);
//	WIN32_FIND_DATA data;
//	HANDLE hFind;
//	if ((hFind = FindFirstFile(pattern.c_str(), &data)) != INVALID_HANDLE_VALUE)
//	{
//		do
//		{
//			//cout << data.cFileName << endl;
//			_list.push_back(getFileName(data.cFileName));             // only file name
//		//_list.push_back((path + "\\").append(data.cFileName));  //file path + name
//		} while (FindNextFile(hFind, &data) != 0);
//		FindClose(hFind);
//	}
//	return _list;
//}

std::vector<std::string> GetDirectoryFiles(const std::string& path, const std::string& exdName)
{
    std::vector<std::string> _list;
    boost::filesystem::path bt_path = (path.c_str());
    boost::filesystem::directory_iterator itr(bt_path);
    while(itr != boost::filesystem::directory_iterator()){
        if(boost::filesystem::is_directory(itr->path())){
            itr++;
            continue;
        }
        std::string file_path = itr->path().string();

        std::string stem = itr->path().stem().string();
        std::string exten = itr->path().extension().string();

        if(exten == ".bin"){
            _list.push_back(itr->path().filename().string());
        }

        ++itr;
    }

    return _list;
}


//icpPointToPlane
void
addNormal(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_with_normals)
{
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

	pcl::search::KdTree<pcl::PointXYZI>::Ptr searchTree(
		new pcl::search::KdTree<pcl::PointXYZI>);
	searchTree->setInputCloud(cloud);

	pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> normalEstimator;
	normalEstimator.setInputCloud(cloud);
	normalEstimator.setSearchMethod(searchTree);
	normalEstimator.setKSearch(15);
	normalEstimator.compute(*normals);

	pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);
}

//argv[1][0]:odometry method; argv[2]:data path; argv[3]:target path
int main(int argc, char** argv)
{
	char filename[200];
	vector<string> seqs = { "00", "01", "02", "03", "04", "05", "06", "07", "08", "09", "10" };
	//vector<string> seqs = { "Ford_1", "Ford_2" };
	int flag;
	flag = (int)(argv[1][0] - '0');

	cout << "Odometry method: ";
    if(flag == 0){
        cout<< "icp_nolinear"<< endl;
    }
    else if(flag == 1){
        cout<< "icp_p2p"<< endl;
    }
    else if (flag == 2){
        cout<< "gicp"<< endl;
    }
    else if (flag == 3){
        cout<< "icp_p2pl"<< endl;
    }
    else if (flag == 4){
        cout<< "ndt"<< endl;
    }

	cout << "Process: ";
	for (int s = 0; s < seqs.size();s++){
		cout << seqs[s] << " ";
	}
	cout << endl;

	for (int s = 0; s < seqs.size(); s++)
	{
        cout << "--------------seq" << seqs[s] << "---------------"<< std::endl;
		string seq = seqs[s];

        string path_in = argv[2] + seq + "/velodyne";

        // get all names of *.bin file
		vector<string> clouds_to_process = GetDirectoryFiles(path_in, "bin");

		int MAX_FRAME = clouds_to_process.size();
		cout << MAX_FRAME << endl;
		vector<Eigen::Matrix4f> Final_H;
		Eigen::Matrix4f Last = Eigen::Matrix4f::Identity();
		Final_H.push_back(Last);
		
		// Read .bin files and save point cloud objects to a vector	
		for (int numFrame = 0; numFrame < MAX_FRAME - 1; numFrame++)//KITTI
		//for (int numFrame = 1; numFrame < MAX_FRAME; numFrame++)//Ford
		{
		    if (numFrame % 30 == 0){
                cout <<"Frame: " << numFrame << " /" << MAX_FRAME << std::endl;
            }
			sprintf(filename, "%s/%06d.bin", path_in.c_str(), numFrame);
			PointCloud<PointXYZI>::Ptr cloud_tgt = MyFileOpen(filename);

			sprintf(filename, "%s/%06d.bin", path_in.c_str(), numFrame + 1);
			PointCloud<PointXYZI>::Ptr cloud_src = MyFileOpen(filename);

//			cout << filename << endl;

			/************************************************************************************************/
			// Apply Voxel Grid Filter to downsampling the data.
			PointCloud<PointXYZI>::Ptr cloud_tgt_filtered(new PointCloud<PointXYZI>);
			PointCloud<PointXYZI>::Ptr cloud_src_filtered(new PointCloud<PointXYZI>);

			// Create the filtering object
			VoxelGrid<PointXYZI> sor;
	
			sor.setInputCloud(cloud_tgt);
			sor.setLeafSize(0.4f, 0.4f, 0.4f);//原0.6
			sor.filter(*cloud_tgt_filtered);

			sor.setInputCloud(cloud_src);
			sor.setLeafSize(0.4f, 0.4f, 0.4f);
			sor.filter(*cloud_src_filtered);
			PointCloud<PointXYZI>::Ptr cloud_src_aligned(new PointCloud<PointXYZI>);


			//int flag = 1;
			//cout << "flag:"<<flag << endl;
			//ICP_nolinear 
			if (flag == 0)
			{
				/************************************************************************************************/
				// Compute surface normal vectors and curvature features
				PointCloud<PointNormal>::Ptr points_with_normals_src(new PointCloud<PointNormal>);
				PointCloud<PointNormal>::Ptr points_with_normals_tgt(new PointCloud<PointNormal>);
				NormalEstimation<PointXYZI, PointNormal> norm_est;

				// KD Tree Structure to search nearest neighbors
				search::KdTree<PointXYZI>::Ptr tree(new search::KdTree<PointXYZI>());
				norm_est.setSearchMethod(tree);
				norm_est.setKSearch(5);

				// Save cloud_in normal vectors 
				norm_est.setInputCloud(cloud_tgt_filtered);
				norm_est.compute(*points_with_normals_tgt);
				copyPointCloud(*cloud_tgt_filtered, *points_with_normals_tgt);

				// Save cloud_out normal vectors 
				norm_est.setInputCloud(cloud_src_filtered);
				norm_est.compute(*points_with_normals_src);
				copyPointCloud(*cloud_src_filtered, *points_with_normals_src);

				MyPointRepresentation point_representation;
				float alpha[4] = { 1.0, 1.0, 1.0, 1.0 };
				point_representation.setRescaleValues(alpha);

				/************************************************************************************************/
				// Apply non-linear ICP to align two point clouds
				IterativeClosestPointNonLinear<PointNormal, PointNormal> nonlinear_icp;
				// The maximum distance between two corresponding points
				//nonlinear_icp.setMaxCorrespondenceDistance(0.1);
				nonlinear_icp.setMaxCorrespondenceDistance(7);//原为100
				// The maximum tolerance for transformation increment
				nonlinear_icp.setTransformationEpsilon(1e-8);
				// Input normal vectors feature for calculation
				nonlinear_icp.setPointRepresentation(boost::make_shared<const MyPointRepresentation>(point_representation));
				nonlinear_icp.setInputSource(points_with_normals_src);
				nonlinear_icp.setInputTarget(points_with_normals_tgt);

				Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity(), prev;
				PointCloud<PointNormal>::Ptr nonlinear_icp_result = points_with_normals_src;

				// Keep iterating to let two clouds get closer until the error less than tolerance
				nonlinear_icp.setMaximumIterations(25);
				for (int i = 0; i < 25; ++i)
				{
					points_with_normals_src = nonlinear_icp_result;
					nonlinear_icp.setInputSource(points_with_normals_src);
					nonlinear_icp.align(*nonlinear_icp_result);

					// Accumulate transformation between each Iteration
					Ti = nonlinear_icp.getFinalTransformation() * Ti;

					// If the difference between this transformation and the previous one
					// is smaller than the threshold, refine the process by reducing
					// the maximal correspondence distance
					if (fabs((nonlinear_icp.getLastIncrementalTransformation() - prev).sum())
						< nonlinear_icp.getTransformationEpsilon())
						nonlinear_icp.setMaxCorrespondenceDistance(nonlinear_icp.getMaxCorrespondenceDistance() - 1);

					prev = nonlinear_icp.getLastIncrementalTransformation();
				}

				/************************************************************************************************/
				// Save Homogeneous matrix to vector container			
				Last = Last * Ti;
				Final_H.push_back(pose_Lidar2Cam(Last));
				

			}
			//ICP point to point
			else if (flag == 1)
			{
				pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;
				icp.setMaximumIterations(25);//25
				icp.setInputSource(cloud_src);
				icp.setInputTarget(cloud_tgt);
				icp.setMaxCorrespondenceDistance(7);
				//icp.setEuclideanFitnessEpsilon(1);
				icp.setTransformationEpsilon(1e-8);//1-e8
				//icp.align(*cloud_src_aligned);

				Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity(), prev;
				PointCloud<PointXYZI>::Ptr icp_result = cloud_src_filtered;
				PointCloud<PointXYZI>::Ptr icp_src;
				// Keep iterating to let two clouds get closer until the error less than tolerance
				for (int i = 0; i < 25; ++i)
				{
					icp_src = icp_result;
					icp.setInputSource(icp_src);
					icp.align(*icp_result);

					// Accumulate transformation between each Iteration
					Ti = icp.getFinalTransformation() * Ti;

					// If the difference between this transformation and the previous one
					// is smaller than the threshold, refine the process by reducing
					// the maximal correspondence distance
					if (fabs((icp.getLastIncrementalTransformation() - prev).sum())
						< icp.getTransformationEpsilon())
						icp.setMaxCorrespondenceDistance(icp.getMaxCorrespondenceDistance() - 1);

					prev = icp.getLastIncrementalTransformation();
				}

				Last = Last * Ti;
				Final_H.push_back(pose_Lidar2Cam(Last));
				//Final_H.push_back(Last);

			}
			//GICP
			else if (flag == 2)
			{
				pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;
				icp.setTransformationEpsilon(1e-8);
				icp.setMaxCorrespondenceDistance(7);
				icp.setMaximumIterations(25);
				icp.setRANSACIterations(10);
				icp.setInputTarget(cloud_tgt_filtered);
				icp.setInputSource(cloud_src_filtered);
				pcl::PointCloud<pcl::PointXYZI> unused_result;
				icp.align(unused_result);

				Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity();
				Ti = icp.getFinalTransformation();

				Last = Last * Ti;
				Final_H.push_back(pose_Lidar2Cam(Last));
				//Final_H.push_back(Last);

			}
			//Point To Plane
			else if (flag == 3)
			{
				pcl::PointCloud<pcl::PointNormal>::Ptr src(
					new pcl::PointCloud<pcl::PointNormal>);
				pcl::copyPointCloud(*cloud_src, *src);
				pcl::PointCloud<pcl::PointNormal>::Ptr tgt(
					new pcl::PointCloud<pcl::PointNormal>);
				pcl::copyPointCloud(*cloud_tgt, *tgt);

				pcl::NormalEstimation<pcl::PointNormal, pcl::PointNormal> norm_est;
				norm_est.setSearchMethod(pcl::search::KdTree<pcl::PointNormal>::Ptr(
					new pcl::search::KdTree<pcl::PointNormal>));
				norm_est.setKSearch(10);
				norm_est.setInputCloud(tgt);
				norm_est.compute(*tgt);

				pcl::IterativeClosestPoint<pcl::PointNormal, pcl::PointNormal> icp;
				typedef pcl::registration::TransformationEstimationPointToPlane<
					pcl::PointNormal, pcl::PointNormal>
					PointToPlane;
				boost::shared_ptr<PointToPlane> point_to_plane(new PointToPlane);
				icp.setTransformationEstimation(point_to_plane); // key

				icp.setInputSource(src);
				icp.setInputTarget(tgt);
				// icp.setRANSACOutlierRejectionThreshold(ransac_par);
				icp.setRANSACIterations(20);//原20
				icp.setMaximumIterations(50);//原100
				icp.setTransformationEpsilon(1e-8);
				pcl::PointCloud<pcl::PointNormal> output;
				icp.align(output);

				Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity();
				Ti = icp.getFinalTransformation();

				Last = Last * Ti;
				Final_H.push_back(pose_Lidar2Cam(Last));
			}
			//NDT 算法
			else if (flag == 4)
			{
				pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> ndt;
				ndt.setTransformationEpsilon(1e-8);
				ndt.setMaximumIterations(25);

				ndt.setStepSize(0.1);
				ndt.setResolution(1);

				ndt.setInputSource(cloud_src_filtered);
				ndt.setInputTarget(cloud_tgt_filtered);

				// Set initial alignment estimate found using robot odometry.
				Eigen::AngleAxisf init_rotation(0.6931, Eigen::Vector3f::UnitZ());
				Eigen::Translation3f init_translation(1.79387, 0.720047, 0);
				Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix();

				pcl::PointCloud<pcl::PointXYZI> output;
				ndt.align(output);

				Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity();
				Ti = ndt.getFinalTransformation();

				Last = Last * Ti;
				Final_H.push_back(pose_Lidar2Cam(Last));
			}
			//icpPointToPlane
			else if (flag == 5)
			{
				pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_source_normals(
					new pcl::PointCloud<pcl::PointXYZINormal>());
				pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_target_normals(
					new pcl::PointCloud<pcl::PointXYZINormal>());
				pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_source_trans_normals(
					new pcl::PointCloud<pcl::PointXYZINormal>());

				addNormal(cloud_tgt_filtered, cloud_target_normals);
				addNormal(cloud_src_filtered, cloud_source_normals);

				pcl::IterativeClosestPointWithNormals<pcl::PointXYZINormal,
					pcl::PointXYZINormal>::Ptr
					icp(new pcl::IterativeClosestPointWithNormals<pcl::PointXYZINormal,
					pcl::PointXYZINormal>());
				icp->setTransformationEpsilon(1e-8);
				icp->setMaxCorrespondenceDistance(7.0);
				icp->setMaximumIterations(25);
				icp->setRANSACIterations(20);
				icp->setInputSource(cloud_source_normals); //
				icp->setInputTarget(cloud_target_normals);
				icp->align(*cloud_source_trans_normals); //

				Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity();
				Ti = icp->getFinalTransformation();

				Last = Last * Ti;
				Final_H.push_back(pose_Lidar2Cam(Last));
				//Final_H.push_back(Last);

			}
			else {
				cout << "flag error!" << endl;
			}
		}
		string outPath;
		//string prePath = "E:\\KITTI\\";
		string prePath = argv[3];
		if(flag == 0){
			outPath = prePath + seq + "_nolinear.txt";
		}
		else if(flag == 1){
			outPath = prePath + seq + "_p2p.txt";
		}
		else if (flag == 2){
			outPath = prePath + seq + "_gicp.txt";
		}
		else if (flag == 3){
			outPath = prePath + seq + "_p2plane.txt";
		}
        else if (flag == 4){
            outPath = prePath + seq + "_ndt.txt";
        }
        //outPath = prePath + seq + ".txt";
		SavePose_Mat(outPath, Final_H);
	}

	return 0;
}


