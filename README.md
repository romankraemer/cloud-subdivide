# Cloud-subdivide: Subdivide point clouds into sub point clouds
This is a tool to subdivide a .pcd/.ply point cloud into a set of smaller sub point clouds with user defined dimensions.
The tool starts at point P<sub>0</sub> = (x<sub>min</sub>, y<sub>min</sub>, z<sub>min</sub>) of the point cloud and loops through the rest of the cloud from there. The resulting sub clouds are then saved with a .csv log file with some information on the sub clouds e.g. dimensions and number of points.

## Visualization output example
![sub clouds output example](https://github.com/romankraemer/cloud-subdivide/blob/main/output_example_sub_clouds.png "sub clouds output example")

## Prerequisites
 - Build environment [tested with GCC 7 and CMake 3.10]
 - PointCloudLibrary [tested with PCL 1.8(.1)]
 - Boost filesystem and Boost program_options

## Build
```bash
$ cd path/to/build/dir
$ cmake path/to/src/dir
$ make
```

## Example usage
```bash
$ cd path/to/build/dir
$ ./cloud-subdivide --input /path/to/your/point/cloud.ply --res 0.01 --x-dim 4.0 --y-dim 4.0 --z-dim 4.0
```
The output files will be dumped into /output_files in the root directory of this tool.

## Inputs and options
Use input argument *\-\-help* to show a list of input arguments.

```bash
$ ./cloud-subdivide --help
```

|input argument   | explanation   |
| ------------ | ------------ |
|\-\-help | List of input arguments. Default values will be used if there is no input for an argument.|
|\-\-input arg | Set path to input file (.ply or .pcd). Use quotation marks (\-\-input "/path to file") if file path has spaces. |
|\-\-output arg | Set output file format (--output ply or --output pcd) for sub clouds [default: ply]. |
|\-\-res arg | Set resolution for octree box search [default: 0.01]. |
|\-\-x-dim arg | Set sub cloud dimension in x direction [default: 1.0]. |
|\-\-y-dim arg | Set sub cloud dimension in y direction [default: 1.0]. |
|\-\-z-dim arg | Set sub cloud dimension in z direction [default: 1.0]. |

## Additional explanation
The user defined dimensions are the maximum dimensions for the sub point clouds. If no more points in the current search iteration and direction (z<sub>current max</sub>, y<sub>current max</sub>, x<sub>current max</sub> based on box grid on input point cloud defined by x-dim, y-dim, z-dim) are left, the resulting sub cloud can be smaller than the defined value(s) for the respective dimension(s).

The point cloud is processed from P<sub>0</sub> &#8594; z<sub>max</sub> &#8594; y<sub>max</sub> &#8594; x<sub>max</sub>.