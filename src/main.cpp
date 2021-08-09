#include <iostream>
#include <string>
#include <sstream>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/common/common.h>

#include <pcl/octree/octree.h>
#include <pcl/octree/octree_search.h>
#include <pcl/filters/extract_indices.h>

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>

typedef pcl::PointXYZRGB PointT;
namespace po = boost::program_options;

// extract sub cloud from cloud
pcl::PointCloud<PointT>::Ptr extractSubCloud(PointT min, PointT max, pcl::PointCloud<PointT>::Ptr cloud, pcl::octree::OctreePointCloudSearch<PointT> &octree)
{

    pcl::PointCloud<PointT>::Ptr sub_cloud(new pcl::PointCloud<PointT>);
    std::vector<int> pointIdxBoxSearch;
    Eigen::Vector3f min_pt(min.x, min.y, min.z);
    Eigen::Vector3f max_pt(max.x, max.y, max.z);
    octree.boxSearch(min_pt, max_pt, pointIdxBoxSearch);

    pcl::ExtractIndices<PointT> extract;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    inliers->indices = pointIdxBoxSearch;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*sub_cloud);

    return sub_cloud;
}

int main(int argc, char **argv)
{

    // boost po: variables that will store parsed command line values
    std::string input_file;
    std::string output_format;
    double resolution;
    float x_dim_sub_cloud;
    float y_dim_sub_cloud;
    float z_dim_sub_cloud;

    // boost po: setup options
    po::options_description desc("Inputs and options");
    desc.add_options()("help", "List of input arguments. Default values will be used if there is no input for an argument.")                                              // clang format
        ("input", po::value<std::string>(&input_file), "Set path to input file (.ply or .pcd). Use quotation marks (--input \"/path to file\") if file path has spaces.") // clang format
        ("output", po::value<std::string>(&output_format)->default_value("ply"), "Set output file format (--output ply or --output pcd) for sub clouds [default: ply].")  // clang format
        ("res", po::value<double>(&resolution)->default_value(0.01), "Set resolution for octree box search [default: 0.01].")                                             // clang format
        ("x-dim", po::value<float>(&x_dim_sub_cloud)->default_value(1.0), "Set sub cloud dimension in x direction [default: 1.0].")                                       // clang format
        ("y-dim", po::value<float>(&y_dim_sub_cloud)->default_value(1.0), "Set sub cloud dimension in y direction [default: 1.0].")                                       // clang format
        ("z-dim", po::value<float>(&z_dim_sub_cloud)->default_value(1.0), "Set sub cloud dimension in z direction [default: 1.0].")                                       // clang format
        ;

    // boost po: parse input
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help"))
    {
        std::cout << desc << "\n";
        return 1;
    }

    else if (vm.count("input") == 0)
    {
        std::cout << "\nError: No input file was set. Set an input file.\n";
        return 1;
    }

    else if (output_format != "ply" && output_format != "pcd")
    {
        std::cout << "\nError: Unexpected output file format. Allowed inputs are \"ply\" and \"pcd\".\n";
        return 1;
    }

    // input file path, name, extension
    boost::filesystem::path file_path(input_file);
    std::string file_name_no_ext = file_path.stem().string();
    std::string file_extension = file_path.extension().string();
    std::string file_name_with_ext = file_path.filename().string();

    // cloud object input cloud
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);

    // load input cloud
    if (file_extension == ".ply")
    {
        std::cout << "\nLoading point cloud...\n";
        pcl::io::loadPLYFile<PointT>(input_file, *cloud);
    }

    else if (file_extension == ".pcd")
    {
        std::cout << "\nLoading point cloud...\n";
        pcl::io::loadPCDFile<PointT>(input_file, *cloud);
    }

    else if (file_extension != ".ply" && file_extension != ".pcd")
    {
        std::cout << "\nError: Input is no point cloud or has unexpected file format. Allowed input formats are \"ply\" and \"pcd\".\n";
        return 1;
    }

    // create timestamp for directories where point clouds will be saved
    time_t rawtime;
    struct tm *timeinfo;
    char buffer[64];
    time(&rawtime);
    timeinfo = localtime(&rawtime);
    strftime(buffer, 64, "%Y-%m-%d-%H-%M", timeinfo); // Year, Month, Day, Hour, Minute

    // create main output dump directory
    std::string str_output_files;
    str_output_files = "../output_files";
    boost::filesystem::path output_files_path(str_output_files);

    if (boost::filesystem::is_directory(output_files_path))
    {
        std::cout << "\nOutput directory already exists. Output will be saved in \"/output_files\"\n";
    }

    else
    {
        boost::filesystem::create_directory(str_output_files);
        std::cout << "\nOutput directory was created. Output will be saved in \"/output_files\"\n";
    }

    // create save dir where sub clouds will be saved
    std::stringstream ss_dir_path_sub_cloud;
    ss_dir_path_sub_cloud << str_output_files << "/" << buffer << "_sub_clouds_" << file_name_no_ext;
    boost::filesystem::create_directory(ss_dir_path_sub_cloud.str());

    std::stringstream ss_sub_cloud_save;
    std::string str_sub_cloud_save;
    int i = 0;

    pcl::octree::OctreePointCloudSearch<PointT> octree(resolution);
    octree.setInputCloud(cloud);
    octree.addPointsFromInputCloud();
    PointT p1;
    PointT p2;
    pcl::getMinMax3D<PointT>(*cloud, p1, p2);

    float x_min_cloud = p1.x;
    float x_max_cloud = p2.x;

    float y_min_cloud = p1.y;
    float y_max_cloud = p2.y;

    float z_min_cloud = p1.z;
    float z_max_cloud = p2.z;

    // create and write some information to log file
    std::ofstream logfile("./" + ss_dir_path_sub_cloud.str() + "/" + buffer + "_" + file_name_no_ext + "_subdivide_log.csv");
    logfile << "date," << buffer << "\n";
    logfile << "cloud name," << file_name_with_ext << "\n";
    logfile << "total points cloud," << cloud->size() << "\n";
    logfile << "total size cloud x," << std::fabs(x_max_cloud - x_min_cloud) << "\n";
    logfile << "total size cloud y," << std::fabs(y_max_cloud - y_min_cloud) << "\n";
    logfile << "total size cloud z," << std::fabs(z_max_cloud - z_min_cloud) << "\n";
    logfile << "\n";
    logfile << "octree resolution," << resolution << "\n";
    logfile << "set dim x," << x_dim_sub_cloud << "\n";
    logfile << "set dim y," << y_dim_sub_cloud << "\n";
    logfile << "set dim z," << z_dim_sub_cloud << "\n";
    logfile << "\n";
    logfile << "sub cloud, n points, size x, size y, size z" << std::endl;

    // subdivide input cloud to set size sub clouds
    float x_current = x_min_cloud;

    while (x_current < x_max_cloud)
    {
        float y_current = y_min_cloud;

        while (y_current < y_max_cloud)
        {
            float z_current = z_min_cloud;

            while (z_current < z_max_cloud)
            {
                PointT min;
                PointT max;
                min.x = x_current;
                min.y = y_current;
                min.z = z_current;
                max.x = x_current + x_dim_sub_cloud;
                max.y = y_current + y_dim_sub_cloud;
                max.z = z_current + z_dim_sub_cloud;
                pcl::PointCloud<PointT>::Ptr sub_cloud = extractSubCloud(min, max, cloud, octree);

                // save sub clouds
                if (sub_cloud->size() > 0)
                {
                    // size sub clouds for logfile
                    PointT sub_p1;
                    PointT sub_p2;
                    pcl::getMinMax3D<PointT>(*sub_cloud, sub_p1, sub_p2);

                    // create save path for sub cloud
                    ss_sub_cloud_save << "./" << ss_dir_path_sub_cloud.str() << "/" << file_name_no_ext << "_sub_cloud_" << i << "." << output_format;
                    str_sub_cloud_save = ss_sub_cloud_save.str();
                    ss_sub_cloud_save.str(std::string()); // clear contents std::stringstream
                    ss_sub_cloud_save.clear();            // clear any fail and eof flags std::stringstream

                    if (output_format == "ply")
                    {
                        pcl::io::savePLYFileASCII(str_sub_cloud_save, *sub_cloud);
                    }
                    else if (output_format == "pcd")
                    {
                        pcl::io::savePCDFileASCII(str_sub_cloud_save, *sub_cloud);
                    }

                    logfile << i << "," << sub_cloud->size() << "," << std::fabs(sub_p2.x - sub_p1.x) << "," << std::fabs(sub_p2.y - sub_p1.y) << "," << std::fabs(sub_p2.z - sub_p1.z) << std::endl;
                    std::cout << "Saving sub cloud " << i << std::endl;
                    i++;
                }

                z_current += z_dim_sub_cloud;
            }

            y_current += y_dim_sub_cloud;
        }

        x_current += x_dim_sub_cloud;
    }

    logfile.close();
    std::cout << "All sub clouds were saved in /output_files." << std::endl;

    return 0;
}
