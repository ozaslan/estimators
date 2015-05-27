#include "range_based_roof_mapper.hh"

RangeBasedRoofMapper::RangeBasedRoofMapper(double res):_octree(res){
	_res = res;
}

const octomap::OcTree& RangeBasedRoofMapper::get_map(){
	return _octree;
}

bool RangeBasedRoofMapper::register_scan (const Eigen::Matrix4d &pose, const LaserProc &laser_proc, double extrude){
	octomap::point3d sensor_origin;
	Eigen::Matrix4d sensor_pose = pose * laser_proc.get_calib_params().relative_pose;
	sensor_origin.x() = pose(0, 3);
	sensor_origin.y() = pose(1, 3);
	sensor_origin.z() = pose(2, 3);
	Matrix3d dcm = sensor_pose.topLeftCorner<3, 3>();
	
	const vector<int> mask = laser_proc.get_mask();
	const vector<Eigen::Vector3d> points_3d = laser_proc.get_3d_points();

	_cloud.clear();
	_cloud.reserve(mask.size());
	octomath::Pose6D cloud_trans(0, 0, 0, 0, 0, 0);
	_cloud.transformAbsolute(cloud_trans);


	Vector4d pt;
	for(int i = 0 ; i < (int)mask.size() ; i++){
		if(mask[i] != false ){
			pt.topLeftCorner<3, 1>() = points_3d[i]; 
			pt(3) = 1;
			pt = pose * pt;
			_cloud.push_back(pt(0), pt(1), pt(2));
		}
	}

	cout << "inserting point cloud from the pose : " << pose << endl;
	cout << "extrude = " << extrude << endl;
	cout << "# of points = " << _cloud.size() << endl;

	if(extrude == 0){
		//_octree.insertPointCloud(_cloud, sensor_origin); 	
		for(int i = 0 ; i < (int)_cloud.size() ; i++)
			_octree.updateNode(	_cloud.getPoint(i), true, true);
		//_octree.updateInnerOccupancy();
	} else {
		//cout << "Q0" << endl;
		Eigen::Vector3d zvec(0, 0, 1);
		zvec = dcm * zvec;
		// Extrusion should be either in +-z-world axis or
		// along the xy-plane.
		double  z_comp = zvec(2);
		double xy_comp = sqrt(zvec(0)*zvec(0) + zvec(1)*zvec(1));
		if(z_comp > xy_comp)
			zvec(0) = zvec(1) = 0;
		else
			zvec(2) = 0;
		zvec.normalize();
		//cout << "Q1" << endl;

		cout << "zvec = " << zvec << endl;

		for(double dz = -extrude ; dz <= extrude ; dz += _res/2){
			//cout << "Q1.1" << endl;
			octomap::point3d origin = sensor_origin;
			//origin.x() += dz * zvec(0);
			//origin.y() += dz * zvec(1);
			//origin.z() += dz * zvec(2);
			//cout << "Q1.2" << endl;
			cloud_trans.x() = dz * zvec(0);
			cloud_trans.y() = dz * zvec(1);
			cloud_trans.z() = dz * zvec(2);

			cout << "cloud_trans = " << cloud_trans << endl;

			_cloud.transformAbsolute(cloud_trans);
			//cout << cloud.getPoint(0).x() << " " << cloud.getPoint(0).y() << " " << cloud.getPoint(0).z() << endl;

			_octree.insertPointCloud(_cloud, origin);
			//cout << "origin = " << origin << endl;
		}
	}

	return true;

}

bool RangeBasedRoofMapper::register_scan (const Eigen::Matrix4d &pose, const sensor_msgs::LaserScan &data, const vector<char> &mask, char cluster_id, double extrude){
	ASSERT(mask.size() == 0 || data.ranges.size() == mask.size(), "mask and data size should be the same or mask should be of size zero.");
	ASSERT(cluster_id != 0, "cluster '0' is reserved. Use another ID");
	ASSERT(extrude >= 0, "extrusion length should be >= 0");

	octomap::point3d sensor_origin;
	sensor_origin.x() = pose(0, 3);
	sensor_origin.y() = pose(1, 3);
	sensor_origin.z() = pose(2, 3);
	Matrix3d dcm = pose.topLeftCorner<3, 3>();
	_cloud.clear();
	_cloud.reserve(data.ranges.size());
	octomath::Pose6D cloud_trans(0, 0, 0, 0, 0, 0);
	_cloud.transformAbsolute(cloud_trans);

	Vector4d pt;
	double th = data.angle_min;
	for(int i = 0 ; i < (int)data.ranges.size() ; i++, th += data.angle_increment){
		if(mask.size() == 0 || mask[i] == cluster_id ){
			utils::laser::polar2euclidean(data.ranges[i], th, pt(0), pt(1));
			pt(2) = 0;
			pt(3) = 1;
			pt = pose * pt;
			_cloud.push_back(pt(0), pt(1), pt(2));
			//if(extrude)
			//	cout << pt << endl;
		}
	}

	//extrude = 1;

	cout << "inserting point cloud from the pose : " << pose << endl;

	if(extrude == 0){
		//_octree.insertPointCloud(_cloud, sensor_origin); 	
		for(int i = 0 ; i < (int)_cloud.size() ; i++)
			_octree.updateNode(	_cloud.getPoint(i), true, true);
		//_octree.updateInnerOccupancy();
	} else {
		//cout << "Q0" << endl;
		Eigen::Vector3d zvec(0, 0, 1);
		zvec = dcm * zvec;
		// Extrusion should be either in +-z-world axis or
		// along the xy-plane.
		double  z_comp = zvec(2);
		double xy_comp = sqrt(zvec(0)*zvec(0) + zvec(1)*zvec(1));
		if(z_comp > xy_comp)
			zvec(0) = zvec(1) = 0;
		else
			zvec(2) = 0;
		zvec.normalize();
		//cout << "Q1" << endl;

		cout << "zvec = " << zvec << endl;

		for(double dz = -extrude ; dz <= extrude ; dz += _res/2){
			//cout << "Q1.1" << endl;
			octomap::point3d origin = sensor_origin;
			//origin.x() += dz * zvec(0);
			//origin.y() += dz * zvec(1);
			//origin.z() += dz * zvec(2);
			//cout << "Q1.2" << endl;
			cloud_trans.x() = dz * zvec(0);
			cloud_trans.y() = dz * zvec(1);
			cloud_trans.z() = dz * zvec(2);

			cout << "cloud_trans = " << cloud_trans << endl;

			_cloud.transformAbsolute(cloud_trans);
			//cout << cloud.getPoint(0).x() << " " << cloud.getPoint(0).y() << " " << cloud.getPoint(0).z() << endl;

			_octree.insertPointCloud(_cloud, origin);
			//cout << "origin = " << origin << endl;
		}
	}

	return true;
}

bool RangeBasedRoofMapper::register_cloud(const Eigen::Matrix4d &pose, const sensor_msgs::PointCloud2 &cloud, const vector<char> &mask, char cluster_id){
	return true;
}

bool RangeBasedRoofMapper::register_ray  (const Eigen::Vector3d &from, const Eigen::Vector3d &to, double cone_angle){
	if(cone_angle == 0){

	} else {

	}
	return true;
}

bool RangeBasedRoofMapper::get_BBX(Eigen::Vector3d &min, Eigen::Vector3d &max){
	return true;
}
  
double RangeBasedRoofMapper::get_memory_usage(){
	return _octree.memoryUsage() / 1024.0;
}

bool RangeBasedRoofMapper::reset(){
	_octree.clear();
	return true;
}

