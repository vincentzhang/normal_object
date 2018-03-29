//Author:Camilo Perez
//Date:Oct 22 2015

#include <ros/ros.h>
#include "std_msgs/String.h"
#include "normal_object/targetObjects.h"
#include "rgb_visualization/pathData.h"

#include <pcl/filters/voxel_grid.h>


#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/octree/octree.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/integral_image_normal.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>

#include <Eigen/Geometry>
#include <Eigen/Dense>
#include "string_convertor.h"

using namespace std;

#define MAX_ITERATION    100
#define RESOLUTION       0.02   //Octree Resolution
#define SHOW_PCL        true

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

#if SHOW_PCL
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
#endif

pcl::PointXYZ robotEndEffector;
pcl::PointXYZ robotEndEffector_point;
pcl::PointXYZ kinect_robot_x_axis;
pcl::PointXYZ kinect_robot_y_axis;
pcl::PointXYZ kinect_robot_z_axis;
pcl::PointXYZ kinect_robot_zero;
std::vector<pcl::PointXYZ> xyz_path_coordinate;
//std::vector<pcl::PointXYZ> xyz_path_normal;
//std::vector<pcl::PointXYZ> xyz_path_normal_point;

pcl::PointXYZ workSpacePoint_Vis;

//pcl::PointXYZ robot; //CP

Eigen::Quaternionf robot_orientation;
Eigen::Vector3f output3_vector, z_input_vector;
Eigen::Matrix4f transformationMatrix, inverse_transformationMatrix;
Eigen::Vector4f robot_position, kinect_robot_position,kinect_robot_normal_point, robot_normal_point, robot_normal_vector , workSpacePoint , workSpacePoint_InKinectFrame; //robot_orientation,z_vector
Eigen::Vector4f kinect_robot_frame_reference_zero,kinect_robot_frame_reference_x,kinect_robot_frame_reference_y,kinect_robot_frame_reference_z,robot_frame_reference_x,robot_frame_reference_y,robot_frame_reference_z, robot_frame_reference_zero;

// for storing the bbox information
int box_size=0; // number of bounding box
std::vector<int> u_path; //x
std::vector<int> v_path; //y
std::vector<int> height_vec; //height
std::vector<int> width_vec; //width
std::vector<string> obj_vec; // vec of object names
bool new_box=false, new_box_robot=false; // flag for if there's new object

std::vector<Eigen::Vector4f> path_robot_v;
std::vector<Eigen::Vector4f> path_robot_r;

std::vector<Eigen::Vector4f> normals_robot_v;
std::vector<Eigen::Vector4f> normals_robot_r;

Eigen::Vector3f normals_robot_r_normalized;

Eigen::Matrix4f read_transform_matrix() {
    std::ifstream transform_file;
    std::string kinova_workspace = getenv("CALIBRATION_WORKSPACE");
    //std::string transform_file_path = kinova_workspace + "/kinect_wam_transform.txt"; // for kinect1
    std::string transform_file_path = kinova_workspace + "/kinect2_iiwa_transform.txt"; // for kinect2
    transform_file.open(transform_file_path.c_str(), std::ios_base::in | std::ios_base::binary);

    if(!transform_file) {
        std::cerr << "Can't open transform file" << std::endl;
        std::exit(-1);
    }

    std::string line;
    Eigen::Matrix4f transform_matrix;
    int i = 0;
    while(getline(transform_file, line) && i < 4) {
        std::istringstream in(line);
        float c1, c2, c3, c4;
        in >> c1 >> c2 >> c3 >> c4;

        transform_matrix(i, 0) = c1;
        transform_matrix(i, 1) = c2;
        transform_matrix(i, 2) = c3;
        transform_matrix(i, 3) = c4;
        ++i;
    }

    //std::cout << transform_matrix <<std::endl;
    return transform_matrix;
    //    return transform_matrix.inverse();
}

void wamPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& poseMessage) {
    robot_orientation.x() =-poseMessage->pose.orientation.x;
    robot_orientation.y() =-poseMessage->pose.orientation.y;
    robot_orientation.z() =-poseMessage->pose.orientation.z;
    robot_orientation.w() =poseMessage->pose.orientation.w;
    robot_position.x()=poseMessage->pose.position.x;
    robot_position.y()=poseMessage->pose.position.y;
    robot_position.z()=poseMessage->pose.position.z;
    robot_position.w()=1.0;

    output3_vector=robot_orientation._transformVector(z_input_vector);
    /*
     * robot_normal_vector.x()=0.2*output3_vector.x();
    robot_normal_vector.y()=0.2*output3_vector.y();
    robot_normal_vector.z()=0.2*output3_vector.z();
    robot_normal_vector.w()=1.0;
    robot_normal_point=robot_position+robot_normal_vector;
    //This is to keep homogeneous coordinates.
    robot_normal_point.w()=1.0;
    */
}

bool detect_nan(pcl::PointXYZ &point){
        if (isnan(point.x) || isnan(point.y) || isnan(point.z))
            return true;
    }

pcl::PointXYZ get_valid_pt(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int width, int cur_idx){
    pcl::PointXYZ point;
    const int inc[] = {0, 1,-1,width,-width};
    int index = 0;
    bool all_nans = true;
    for (int i=0; i<5; i++){
        index = cur_idx + inc[i];
        point = cloud->points[index];
        if (!detect_nan(point)) {
            all_nans = false;
            break;
        }
    }
    if (all_nans) {
        cout << "EVERYTHING was NAN" << endl;
    }
    return point;
}


//! callback function for extracting bounding box info from the ROS message
void bbox_callback(const std_msgs::String::ConstPtr& msg)
{
    vector<string> boxes = string_convertor::split(msg->data, ',');
    box_size = atoi(boxes[0].c_str());
    u_path.clear();
    v_path.clear();
    obj_vec.clear();
    height_vec.clear();
    width_vec.clear();
    cout << "Before inserting, the vectors are: " << endl;
    for (int i=0; i< u_path.size(); i++) {
       cout << u_path[i] << ", " << v_path[i] << ", " << obj_vec[i] << ", " << height_vec[i] << ", " << width_vec[i] << endl;
    }
    if (box_size > 0) {
        // if more than one bounding box, set the flag
        new_box = true;
        //cout << "boxes size: " << box_size << endl;
    }
    else {
        new_box = false;
        // no new box, return
        return;
    }
    float x,y,h,w;
    string cls;
    for (int i=0; i < box_size; i++){
        x = atof(boxes[i*5+1].c_str());
        y = atof(boxes[i*5+2].c_str());
        h = atof(boxes[i*5+3].c_str());
        w = atof(boxes[i*5+4].c_str());
        cls = boxes[i*5+5];
        u_path.push_back(round(x));
        v_path.push_back(round(y));
        obj_vec.push_back(cls);
        height_vec.push_back(round(h));
        width_vec.push_back(round(w));
        cout << cls << " x,y,h,w: " <<x << ", " << y << ", " << h << ", " << w << endl;
    }
    cout << "After inserting, the vectors are: " << endl;
    for (int i=0; i< u_path.size(); i++) {
       cout << u_path[i] << ", " << v_path[i] << ", " << obj_vec[i] << ", " << height_vec[i] << ", " << width_vec[i] << endl;
    }
}

void callback(const PointCloud::ConstPtr& msg) {
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree (RESOLUTION);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);  //pointcloud filter
    //pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered2(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered3(new pcl::PointCloud<pcl::PointXYZ>);

//*CP BEGIN
 /*   pcl::VoxelGrid<pcl::PointXYZ> sor;  //create downsampling filter
    sor.setInputCloud (msg);
    sor.setLeafSize (0.01f, 0.01f, 0.01f);
    sor.filter (*cloud_filtered2);

*/

//CP END
    /////////////////////////////////
    // Create the filtering object
    /////////////////////////////////
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(msg);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0, 2.0);
    pass.setKeepOrganized( true );
    pass.filter(*cloud_filtered3);

    pcl::PassThrough<pcl::PointXYZ> pass1;
    pass1.setInputCloud(cloud_filtered3);
    pass1.setFilterFieldName("y");
    pass1.setFilterLimits(-0.5, 0.5);
    pass1.setKeepOrganized( true );
    pass1.filter(*cloud_filtered2);

    pcl::PassThrough<pcl::PointXYZ> pass2;
    pass2.setInputCloud(cloud_filtered2);
    pass2.setFilterFieldName("x");
    pass2.setFilterLimits(-1.0, 1.0);
    pass2.setKeepOrganized( true );
    pass2.filter(*cloud_filtered);

    //std::cerr << "Cloud after filtering: " << std::endl;

    /*pcl::IntegralImageNormalEstimation<pcl::PointXYZ,pcl::Normal>ne;
    ne.setNormalEstimationMethod(ne.AVERAGE_3D_GRADIENT);//COVARIANCE_MATRIX
    // ne.setMaxDepthChangeFactor(0.02f);  // orignial
    ne.setMaxDepthChangeFactor(0.02f);
    ne.setNormalSmoothingSize(10.0f);    // orignial
    ne.setInputCloud(cloud_filtered);
    ne.compute(*normals);
*/
    if(new_box) {
        xyz_path_coordinate.resize(0); // clearing the vector
        xyz_path_coordinate.resize(box_size);
 //       xyz_path_normal.resize(box_size);
 //       xyz_path_normal_point.resize(box_size);

	       //resize robot vectors
        path_robot_v.resize(0);
        path_robot_r.resize(0);
        path_robot_v.resize(box_size);
        path_robot_r.resize(box_size);
  //      normals_robot_v.resize(box_size);
  //      normals_robot_r.resize(box_size);
        std::cout << "Width of point cloud: "<<cloud_filtered->width<< std::endl;

        for(int i = 0; i < box_size; i++) {
          xyz_path_coordinate[i] = get_valid_pt(cloud_filtered, cloud_filtered->width, v_path[i] * cloud_filtered->width + u_path[i]);
/*
      //	  xyz_path_normal[i]=normals->points[v_path[i]* cloud_filtered->width + u_path[i]];
          xyz_path_normal[i].x=normals->points[v_path[i]* cloud_filtered->width + u_path[i]].normal_x;
          xyz_path_normal[i].y=normals->points[v_path[i]* cloud_filtered->width + u_path[i]].normal_y;
          xyz_path_normal[i].z=normals->points[v_path[i]* cloud_filtered->width + u_path[i]].normal_z;

          //	   xyz_path_normal_point[i]=xyz_path_coordinate+0.2*xyz_path_normal;
          xyz_path_normal_point[i].x=xyz_path_coordinate[i].x+0.2*xyz_path_normal[i].x;
          xyz_path_normal_point[i].y=xyz_path_coordinate[i].y+0.2*xyz_path_normal[i].y;
          xyz_path_normal_point[i].z=xyz_path_coordinate[i].z+0.2*xyz_path_normal[i].z;

         // std::cout << "Find the normals" << std::endl;
*/
        }
//        std::cout << "after getting 3D points" << std::endl;
        //Fill the eigen vector
        for (int h = 0; h < box_size; h++) {
          path_robot_v[h].x()=xyz_path_coordinate[h].x;
          path_robot_v[h].y()=xyz_path_coordinate[h].y;
          path_robot_v[h].z()=xyz_path_coordinate[h].z;
          path_robot_v[h].w()=1.0;
          /*
          //Normals to homogeneous representation
          normals_robot_v[h].x()=xyz_path_normal[h].x;
          normals_robot_v[h].y()=xyz_path_normal[h].y;
          normals_robot_v[h].z()=xyz_path_normal[h].z;
          normals_robot_v[h].w()=1.0;
          */
        }
       // std::cout << "Converted to eigen" << std::endl;

        //transform kinect path points into robot path points, from kinect frame to robot frame
        for (int k = 0; k < box_size; k++) {
          path_robot_r[k]=transformationMatrix*path_robot_v[k];
          //std::cout << "This is the coordinate in the  robot frame of reference" << path_robot_r[k] << std::endl;

          //CP Working
          //normals_robot_r[k]=transformationMatrix*normals_robot_v[k];
          //std::cout << "this is the normal in the robot frame of reference" << normals_robot_r[k] << std::endl;
        }
        //std::cout << "transform kinect path points into robot path points" << std::endl;

        new_box_robot=true;
        new_box=false;
      }

    //std::cout << "Here is the robot Position" << robot_position << std::endl;
    kinect_robot_position=inverse_transformationMatrix*robot_position;
    //kinect_robot_normal_point=inverse_transformationMatrix*robot_normal_point; //CP
    //std::cout << "Here is the kinect robot Position" << kinect_robot_position << std::endl;

    robotEndEffector.x=kinect_robot_position.x();
    robotEndEffector.y=kinect_robot_position.y();
    robotEndEffector.z=kinect_robot_position.z();

    /*robotEndEffector_point.x=kinect_robot_normal_point.x();
    robotEndEffector_point.y=kinect_robot_normal_point.y();
    robotEndEffector_point.z=kinect_robot_normal_point.z();
*/
    std::basic_string<char> name = "arrow";
    std::basic_string<char> name2 = "arrow2";

    std::basic_string<char> name3 = "robot_frame_reference_x";
    std::basic_string<char> name4 = "robot_frame_reference_y";
    std::basic_string<char> name5 = "robot_frame_reference_z";

    #if SHOW_PCL
        viewer->addPointCloud<pcl::PointXYZ>(cloud_filtered, "sample cloud2");
        //viewer->addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(cloud_filtered,normals,25,0.01,"sample cloud2",0);
        // viewer->addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(cloud_filtered,normals,100,0.02,"sample cloud2",0);

        for (int j = 0; j < xyz_path_coordinate.size(); j++) {
          viewer->addSphere( xyz_path_coordinate[j], 0.01, 0.0, 1.0, 0.0, "point"+boost::lexical_cast<std::string>(j));
          //viewer->addSphere( xyz_path_coordinate[j], 0.005, 0.0, 1.0, 0.0, "point"+boost::lexical_cast<std::string>(j));

        //  viewer->addArrow<pcl::PointXYZ>( xyz_path_normal_point[j],xyz_path_coordinate[j],1.0,1.0,0.0,false,"normal"+boost::lexical_cast<std::string>(j));
      //    viewer->addLine<pcl::PointXYZ>(xyz_path_normal_point[j] , xyz_path_coordinate[j],1.0,1.0,0.0,"normal"+boost::lexical_cast<std::string>(j));
        }

        double rMin = 0.40;
        double rMax = 0.85;
        for (int j = 0; j < 23; j++) {
          workSpacePoint.x() = rMax*cos( (30 + 5*j) * M_PI/180 );
          workSpacePoint.y() = rMax*sin( (30 + 5*j) * M_PI/180 );
          workSpacePoint.z() = 0.0;
          workSpacePoint.w() = 1.0;
          workSpacePoint_InKinectFrame = inverse_transformationMatrix * workSpacePoint;

          workSpacePoint_Vis.x = workSpacePoint_InKinectFrame.x();
          workSpacePoint_Vis.y = workSpacePoint_InKinectFrame.y();
          workSpacePoint_Vis.z = workSpacePoint_InKinectFrame.z();

          viewer->addSphere( workSpacePoint_Vis, 0.002, 0.0, 0.0, 1.0, "sphMax"+boost::lexical_cast<std::string>(j));


          workSpacePoint.x() = rMin*cos( (30 + 5*j) * M_PI/180 );
          workSpacePoint.y() = rMin*sin( (30 + 5*j) * M_PI/180 );
          workSpacePoint.z() = 0.0; //0.2
          workSpacePoint.w() = 1.0;
          workSpacePoint_InKinectFrame = inverse_transformationMatrix * workSpacePoint;

          workSpacePoint_Vis.x = workSpacePoint_InKinectFrame.x();
          workSpacePoint_Vis.y = workSpacePoint_InKinectFrame.y();
          workSpacePoint_Vis.z = workSpacePoint_InKinectFrame.z();


          viewer->addSphere( workSpacePoint_Vis, 0.002, 0.0, 0.5, 1.0, "sphMin"+boost::lexical_cast<std::string>(j));

        }

        viewer->addSphere(robotEndEffector, 0.0, 0.0, 0.0, 0.0, "robotEndEffector");
        viewer->addArrow<pcl::PointXYZ>(robotEndEffector_point,robotEndEffector,0.0,1.0,0.0,false,name2);
        viewer->addArrow<pcl::PointXYZ>(kinect_robot_x_axis,kinect_robot_zero,1.0,0.0,0.0,false,name3);
        viewer->addArrow<pcl::PointXYZ>(kinect_robot_y_axis,kinect_robot_zero,1.0,0.0,0.0,false,name4);
        viewer->addArrow<pcl::PointXYZ>(kinect_robot_z_axis,kinect_robot_zero,1.0,0.0,0.0,false,name5);

        viewer->spinOnce(100);
        viewer->removePointCloud("sample cloud2");
        for (int j = 0; j < xyz_path_coordinate.size(); j++) {
          viewer->removeShape("point"+boost::lexical_cast<std::string>(j));
       //   viewer->removeShape("normal"+boost::lexical_cast<std::string>(j));
        }

        viewer->removeShape("robotEndEffector");
        viewer->removeShape("arrow2");
        viewer->removeShape("arrow");
        viewer->removeShape("robot_frame_reference_x");
        viewer->removeShape("robot_frame_reference_y");
        viewer->removeShape("robot_frame_reference_z");

        for (int j = 0; j < 23; j++) {
          viewer->removeShape("sphMin"+boost::lexical_cast<std::string>(j));
          viewer->removeShape("sphMax"+boost::lexical_cast<std::string>(j));
        }
    #endif
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "getPointCloudForBBox");
    ros::NodeHandle nh;

    ros::Subscriber sub_box = nh.subscribe("/bbox", 10, bbox_callback);
    // num,x,y,h,w,x,cls,y,h,w,cls..
    //ros::Subscriber sub2 = nh.subscribe("path_data", 100, chatterCallback);
    ros::Subscriber sub3 = nh.subscribe("/zeus/wam/pose", 1, wamPoseCallback);
    string topic_name = "target_bbox";
    //ros::Publisher object_pub = nh.advertise<normal_object::targetObjects>(topic_name, 100);
    ros::Publisher object_pub = nh.advertise<std_msgs::String>(topic_name, 100);

#if (SHOW_PCL)
        viewer->setBackgroundColor(0, 0, 0);
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");

        viewer->initCameraParameters();
        viewer->setCameraPosition(-0.3,-0.07,-1.57,-0.1,-1,0,0);
        viewer->setCameraFieldOfView(0.8575,0);
#endif
    //ros::Subscriber sub = nh.subscribe<PointCloud>("camera/depth_registered/points", 1, callback); //camera/depth_registered/points
    ros::Subscriber sub = nh.subscribe<PointCloud>("kinect2/hd/points", 1, callback);
    //ros::Subscriber sub = nh.subscribe<PointCloud>("kinect2/sd/points", 1, callback);

    ros::Rate loop_rate(30);
    ROS_INFO("Main loop Normal_object");

    transformationMatrix = read_transform_matrix();
    //std::cout << "Here is the transformation matrix" << transformationMatrix << std::endl;
    inverse_transformationMatrix=transformationMatrix.inverse();
    //std::cout << "Here is the inverse transformation matrix" << inverse_transformationMatrix << std::endl;
    /*z_vector.x()=0;
    z_vector.y()=0;
    z_vector.z()=1;
    z_vector.w()=0;
    */

    z_input_vector.x()=0;
    z_input_vector.y()=0;
    z_input_vector.z()=1;

    //robot_frame_reference
    robot_frame_reference_x.x()=0.2;
    robot_frame_reference_x.y()=0.0;
    robot_frame_reference_x.z()=0.0;
    robot_frame_reference_x.w()=1.0;

    robot_frame_reference_y.x()=0.0;
    robot_frame_reference_y.y()=0.2;
    robot_frame_reference_y.z()=0.0;
    robot_frame_reference_y.w()=1.0;

    robot_frame_reference_z.x()=0.0;
    robot_frame_reference_z.y()=0.0;
    robot_frame_reference_z.z()=0.2;
    robot_frame_reference_z.w()=1.0;

    robot_frame_reference_zero.x()=0.0;
    robot_frame_reference_zero.y()=0.0;
    robot_frame_reference_zero.z()=0.0;
    robot_frame_reference_zero.w()=1.0;

    kinect_robot_frame_reference_x=inverse_transformationMatrix*robot_frame_reference_x;
    kinect_robot_frame_reference_y=inverse_transformationMatrix*robot_frame_reference_y;
    kinect_robot_frame_reference_z=inverse_transformationMatrix*robot_frame_reference_z;
    kinect_robot_frame_reference_zero=inverse_transformationMatrix*robot_frame_reference_zero;

    kinect_robot_x_axis.x=kinect_robot_frame_reference_x.x();
    kinect_robot_x_axis.y=kinect_robot_frame_reference_x.y();
    kinect_robot_x_axis.z=kinect_robot_frame_reference_x.z();

    kinect_robot_y_axis.x=kinect_robot_frame_reference_y.x();
    kinect_robot_y_axis.y=kinect_robot_frame_reference_y.y();
    kinect_robot_y_axis.z=kinect_robot_frame_reference_y.z();

    kinect_robot_z_axis.x=kinect_robot_frame_reference_z.x();
    kinect_robot_z_axis.y=kinect_robot_frame_reference_z.y();
    kinect_robot_z_axis.z=kinect_robot_frame_reference_z.z();

    kinect_robot_zero.x=kinect_robot_frame_reference_zero.x();
    kinect_robot_zero.y=kinect_robot_frame_reference_zero.y();
    kinect_robot_zero.z=kinect_robot_frame_reference_zero.z();

    while(ros::ok()) {

      //normal_object::targetObjects msg_objectPoints;
      std_msgs::String msg;
      std::stringstream ss_objectPoints;

      //msg_objectPoints.objects_robot.resize(box_size);
      //msg_objectPoints.normals_robot.resize(box_size);

      ss_objectPoints << topic_name << ":";
      string separator = "";
      if(new_box_robot) {
        ss_objectPoints << box_size << ":";
        for (int i = 0; i < box_size; i++) {

            std::cout << "Ready to publish path_robot_r: " << path_robot_r[i].x() << ", " << path_robot_r[i].y() << ", " << path_robot_r[i].z() << std::endl;
            //msg_objectPoints.objects_robot[i].x=path_robot_r[i].x();
            //msg_objectPoints.objects_robot[i].y=path_robot_r[i].y();
            //msg_objectPoints.objects_robot[i].z=path_robot_r[i].z();
            ss_objectPoints << separator << obj_vec[i];
            separator = ",";
            ss_objectPoints << ","<<path_robot_r[i].x() <<","<< path_robot_r[i].y()<<"," << path_robot_r[i].z() << ","<<height_vec[i]<<","<<width_vec[i];

            //normals_robot_r_normalized = normals_robot_r[i].normalized();
     /*       normals_robot_r_normalized.x()=normals_robot_r[i].x();
            normals_robot_r_normalized.y()=normals_robot_r[i].y();
            normals_robot_r_normalized.z()=normals_robot_r[i].z();
            normals_robot_r_normalized=normals_robot_r_normalized.normalized();

            //Uncomment this if you want dynamic  normals
            msg_objectPoints.normals_robot[i].x=normals_robot_r_normalized.x();
            msg_objectPoints.normals_robot[i].y=normals_robot_r_normalized.y();
            msg_objectPoints.normals_robot[i].z=normals_robot_r_normalized.z();
            std::cout << "Ready to publish normals_robot_r normalized: " << normals_robot_r_normalized.x() << ", " << normals_robot_r_normalized.y() << ", " << normals_robot_r_normalized.z()  << std::endl;
            //std::cout << "Write it to message" << std::endl;
*/
            //Uncomment this if you want fix normals in z direction
            /* msg_targetPoints.normals_robot[i].x=0;
            msg_targetPoints.normals_robot[i].y=0;
            msg_targetPoints.normals_robot[i].z=1;
            */
        }
        //object_pub.publish(msg_objectPoint);
        msg.data = ss_objectPoints.str();
        object_pub.publish(msg);
        //std::cout << "Published the message" << std::endl;
        new_box_robot = false;
      }

      ros::spinOnce();
      loop_rate.sleep();
    }
}
