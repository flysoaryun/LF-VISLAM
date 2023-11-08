#include "keyframe.h"
#include "random_array.h"
#include "pnp_solver.h"
using namespace cv;

template <typename Derived>
static void reduceVector(vector<Derived> &v, vector<uchar> status)
{
	int j = 0;
	for (int i = 0; i < int(v.size()); i++)
		if (status[i])
			v[j++] = v[i];
	v.resize(j);
}

// create keyframe online
// KeyFrame::KeyFrame(double _time_stamp, int _index, Vector3d &_vio_T_w_i, Matrix3d &_vio_R_w_i, cv::Mat &_image,
// 				   vector<cv::Point3f> &_point_3d, vector<cv::Point2f> &_point_2d_uv, vector<cv::Point2f> &_point_2d_norm,
// 				   vector<double> &_point_id, int _sequence)
// {
// 	time_stamp = _time_stamp;
// 	index = _index;
// 	vio_T_w_i = _vio_T_w_i;
// 	vio_R_w_i = _vio_R_w_i;
// 	T_w_i = vio_T_w_i;
// 	R_w_i = vio_R_w_i;
// 	origin_vio_T = vio_T_w_i;
// 	origin_vio_R = vio_R_w_i;
// 	image = _image.clone();
// 	cv::resize(image, thumbnail, cv::Size(80, 60));
// 	point_3d = _point_3d;
// 	point_2d_uv = _point_2d_uv;
// 	point_2d_norm = _point_2d_norm;
// 	point_id = _point_id;
// 	has_loop = false;
// 	loop_index = -1;
// 	has_fast_point = false;
// 	loop_info << 0, 0, 0, 0, 0, 0, 0, 0;
// 	sequence = _sequence;
// 	computeWindowBRIEFPoint();
// 	computeBRIEFPoint();
// 	if (!DEBUG_IMAGE)
// 		image.release();
// }
// changed by wz loop closure
bool inborder(int x, int y)
{
	double center_x = 640.991301;
	double center_y = 490.937512;
	double max_r = 500;
	double min_r = 160;
	double distance_center = sqrt((x - center_x) * (x - center_x) + (y - center_y) * (y - center_y));
	if (distance_center < max_r && distance_center > min_r)
	{
		return true;
	}
	else
	{
		return false;
	}
}
// KeyFrame::KeyFrame(double _time_stamp, int _index, Vector3d &_vio_T_w_i, Matrix3d &_vio_R_w_i, cv::Mat &_image,
// 				   vector<cv::Point3f> &_point_3d, vector<cv::Point2f> &_point_2d_uv, vector<cv::Point3f> &_point_3d_norm,
// 				   vector<double> &_point_id, int _sequence)

//! changed by wz
KeyFrame::KeyFrame(double _time_stamp, int _index, Vector3d &_vio_T_w_i, Matrix3d &_vio_R_w_i, cv::Mat &_image,
				   vector<cv::Point3f> &_point_3d, vector<cv::Point2f> &_point_2d_uv, vector<cv::Point3f> &_point_3d_norm,
				   vector<double> &_point_id, int _sequence, camodocal::CameraPtr _m_camera)
{
	time_stamp = _time_stamp;
	index = _index;
	vio_T_w_i = _vio_T_w_i;
	vio_R_w_i = _vio_R_w_i;
	T_w_i = vio_T_w_i;
	R_w_i = vio_R_w_i;
	origin_vio_T = vio_T_w_i;
	origin_vio_R = vio_R_w_i;

	image = _image.clone();
	// changed by wz loop closure rotation

	int w = image.cols;
	int h = image.rows;
	double CENTER_X = 645.107791;
	double CENTER_Y = 486.025172;

	// Eigen::Vector3d ypr;
	// yaw = Utility::normalizeAngle(Utility::R2ypr(origin_vio_R).x());
	// M = getRotationMatrix2D(Point2f(CENTER_X, CENTER_Y), -yaw, 1.0);
	// warpAffine(image, rotation_image, M, Size(w, h), INTER_LINEAR, 0);
	// cv::imshow("rotation_image", rotation_image);
	// warpAffine(image, rotation_image, M, Size(w, h), INTER_LINEAR, 0);
	// cv::imshow("rotation_image", rotation_image);
	// cv::waitKey(1);

	cv::Mat remap_x = cv::Mat::zeros(image.rows, image.cols, CV_32FC1);
	cv::Mat remap_y = cv::Mat::zeros(image.rows, image.cols, CV_32FC1);

	m_camera = _m_camera;
	// std::string config_file;
	// config_file = "/home/wt/LF-VIO/loop_rotation/src/LF-VIO/config/mindvision/mindvision.yaml";
	// n.getParam("config_file", config_file);
	// cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
	// m_camera = camodocal::CameraFactory::instance()->generateCameraFromYamlFile(config_file.c_str());
	// Eigen::Vector3d ypr = Utility::R2ypr(origin_vio_R);
	// ypr(0) = -ypr(0);
	// ypr(1) = -ypr(1);
	// ypr(2) = -ypr(2);
	// Eigen::Matrix3d R_my = Utility::ypr2R(ypr);
	for (int i = 0; i < w; i++)
	{
		for (int j = 0; j < h; j++)
		{
			Eigen::Vector3d tmp_p;
			m_camera->liftProjective(Eigen::Vector2d(i, j), tmp_p);
			tmp_p = tmp_p / tmp_p.norm();
			// Eigen::Matrix3d tmp_R;
			// tmp_R << 1, 0, 0,
			// 	0, 1, 0,
			// 	0, 0, 1;
			// tmp_p = origin_vio_R * qic * tmp_R * tmp_p;
			tmp_p = qic.inverse() * origin_vio_R.transpose() * tmp_p;
			Eigen::Vector2d tmp_s;
			m_camera->spaceToPlane(tmp_p, tmp_s);
			if (tmp_s(0) >= 0 && tmp_s(0) < w && tmp_s(1) >= 0 && tmp_s(1) < h)
			{
				remap_x.at<float>(j, i) = tmp_s(0);
				remap_y.at<float>(j, i) = tmp_s(1);
			}
			else
			{
				remap_x.at<float>(j, i) = 0;
				remap_y.at<float>(j, i) = 0;
			}
		}
	}
	cv::remap(image, rotation_image, remap_x, remap_y, 0);
	// cv::imshow("rotation_image", rotation_image);
	// cv::waitKey(1);

	cv::resize(image, thumbnail, cv::Size(80, 60));
	point_3d = _point_3d;
	point_2d_uv = _point_2d_uv;
	point_3d_norm = _point_3d_norm;
	point_id = _point_id;
	has_loop = false;
	loop_index = -1;
	has_fast_point = false;
	loop_info << 0, 0, 0, 0, 0, 0, 0, 0;
	sequence = _sequence;
	computeWindowBRIEFPoint();
	computeBRIEFPoint();
	if (!DEBUG_IMAGE)
		image.release();
}

// load previous keyframe
// KeyFrame::KeyFrame(double _time_stamp, int _index, Vector3d &_vio_T_w_i, Matrix3d &_vio_R_w_i, Vector3d &_T_w_i, Matrix3d &_R_w_i,
// 				   cv::Mat &_image, int _loop_index, Eigen::Matrix<double, 8, 1> &_loop_info,
// 				   vector<cv::KeyPoint> &_keypoints, vector<cv::KeyPoint> &_keypoints_norm, vector<BRIEF::bitset> &_brief_descriptors)
// {
// 	time_stamp = _time_stamp;
// 	index = _index;
// 	// vio_T_w_i = _vio_T_w_i;
// 	// vio_R_w_i = _vio_R_w_i;
// 	vio_T_w_i = _T_w_i;
// 	vio_R_w_i = _R_w_i;
// 	T_w_i = _T_w_i;
// 	R_w_i = _R_w_i;
// 	if (DEBUG_IMAGE)
// 	{
// 		image = _image.clone();
// 		cv::resize(image, thumbnail, cv::Size(80, 60));
// 	}
// 	if (_loop_index != -1)
// 		has_loop = true;
// 	else
// 		has_loop = false;
// 	loop_index = _loop_index;
// 	loop_info = _loop_info;
// 	has_fast_point = false;
// 	sequence = 0;
// 	keypoints = _keypoints;
// 	keypoints_norm = _keypoints_norm;
// 	brief_descriptors = _brief_descriptors;
// }
// changed by wz loop closure
KeyFrame::KeyFrame(double _time_stamp, int _index, Vector3d &_vio_T_w_i, Matrix3d &_vio_R_w_i, Vector3d &_T_w_i, Matrix3d &_R_w_i,
				   cv::Mat &_image, int _loop_index, Eigen::Matrix<double, 8, 1> &_loop_info,
				   vector<cv::KeyPoint> &_keypoints, vector<cv::Point3f> &_keypoints_norm, vector<BRIEF::bitset> &_brief_descriptors)
{
	time_stamp = _time_stamp;
	index = _index;
	// vio_T_w_i = _vio_T_w_i;
	// vio_R_w_i = _vio_R_w_i;
	vio_T_w_i = _T_w_i;
	vio_R_w_i = _R_w_i;
	T_w_i = _T_w_i;
	R_w_i = _R_w_i;
	if (DEBUG_IMAGE)
	{
		image = _image.clone();
		cv::resize(image, thumbnail, cv::Size(80, 60));
	}
	if (_loop_index != -1)
		has_loop = true;
	else
		has_loop = false;
	loop_index = _loop_index;
	loop_info = _loop_info;
	has_fast_point = false;
	sequence = 0;
	keypoints = _keypoints;
	keypoints_norm = _keypoints_norm;
	brief_descriptors = _brief_descriptors;
	cout << "sssssssssssssssssssssssssssssssssssssssssssssssssssssssss" << endl;
}

// void KeyFrame::computeWindowBRIEFPoint()
// {
// 	BriefExtractor extractor(BRIEF_PATTERN_FILE.c_str());
// 	for (int i = 0; i < (int)point_2d_uv.size(); i++)
// 	{
// 		cv::KeyPoint key;
// 		key.pt = point_2d_uv[i];
// 		window_keypoints.push_back(key);
// 	}

// 	extractor(image, window_keypoints, window_brief_descriptors);
// }

void KeyFrame::computeWindowBRIEFPoint()
{
	BriefExtractor extractor(BRIEF_PATTERN_FILE.c_str());
	for (int i = 0; i < (int)point_2d_uv.size(); i++)
	{
		cv::KeyPoint key;
		Eigen::Vector3d tmp_p;
		m_camera->liftProjective(Eigen::Vector2d(point_2d_uv[i].x, point_2d_uv[i].y), tmp_p);
		tmp_p = tmp_p / tmp_p.norm();
		tmp_p = origin_vio_R * qic * tmp_p;
		Eigen::Vector2d tmp_s;
		m_camera->spaceToPlane(tmp_p, tmp_s);
		key.pt = Point2f(tmp_s(0), tmp_s(1));

		window_keypoints.push_back(key);
	}
	extractor(rotation_image, window_keypoints, window_brief_descriptors);
}

// void KeyFrame::computeWindowBRIEFPoint()
// {
// 	BriefExtractor extractor(BRIEF_PATTERN_FILE.c_str());
// 	for (int i = 0; i < (int)point_2d_uv.size(); i++)
// 	{
// 		cv::KeyPoint key;
// 		Eigen::Vector3d tmp_point(point_2d_uv[i].x, point_2d_uv[i].y, 1.0);
// 		double CENTER_X = 645.107791;
// 		double CENTER_Y = 486.025172;
// 		M = getRotationMatrix2D(Point2f(CENTER_X, CENTER_Y), -yaw, 1.0);
// 		Eigen::Matrix3d tmp_M;
// 		cv::cv2eigen(M, tmp_M);
// 		tmp_point = tmp_M * tmp_point;
// 		key.pt = Point2f(tmp_point(0), tmp_point(1));

// 		window_keypoints.push_back(key);
// 	}
// 	extractor(rotation_image, window_keypoints, window_brief_descriptors);
// }

// void KeyFrame::computeBRIEFPoint()
// {
// 	BriefExtractor extractor(BRIEF_PATTERN_FILE.c_str());
// 	const int fast_th = 20; // corner detector response threshold
// 	if (1)
// 		cv::FAST(image, keypoints, fast_th, true);
// 	else
// 	{
// 		vector<cv::Point2f> tmp_pts;
// 		cv::goodFeaturesToTrack(image, tmp_pts, 500, 0.01, 10);
// 		for (int i = 0; i < (int)tmp_pts.size(); i++)
// 		{
// 			cv::KeyPoint key;
// 			key.pt = tmp_pts[i];
// 			keypoints.push_back(key);
// 		}
// 	}
// 	extractor(image, keypoints, brief_descriptors);
// 	for (int i = 0; i < (int)keypoints.size(); i++)
// 	{
// 		Eigen::Vector3d tmp_p;
// 		m_camera->liftProjective(Eigen::Vector2d(keypoints[i].pt.x, keypoints[i].pt.y), tmp_p);
// 		// cv::KeyPoint tmp_norm;
// 		// tmp_norm.pt = cv::Point2f(tmp_p.x() / tmp_p.z(), tmp_p.y() / tmp_p.z());
// 		// keypoints_norm.push_back(tmp_norm);
// 		// changed by wz loop closure
// 		cv::Point3f tmp_norm(tmp_p.x() / tmp_p.norm(), tmp_p.y() / tmp_p.norm(), tmp_p.z() / tmp_p.norm());
// 		keypoints_norm.push_back(tmp_norm);
// 	}
// }

void KeyFrame::computeBRIEFPoint()
{
	BriefExtractor extractor(BRIEF_PATTERN_FILE.c_str());
	const int fast_th = 20; // corner detector response threshold
	if (1)
	{
		cv::FAST(rotation_image, keypoints, fast_th, true);
	}
	else
	{
		// vector<cv::Point2f> tmp_pts;
		// cv::goodFeaturesToTrack(image, tmp_pts, 500, 0.01, 10);
		// for (int i = 0; i < (int)tmp_pts.size(); i++)
		// {
		// 	cv::KeyPoint key;
		// 	Eigen::Vector3d tmp_point(point_2d_uv[i].x, point_2d_uv[i].y, 1.0);
		// 	double CENTER_X = 645.107791;
		// 	double CENTER_Y = 486.025172;
		// 	M = getRotationMatrix2D(Point2f(CENTER_X, CENTER_Y), -yaw, 1.0);
		// 	Eigen::Matrix3d tmp_M;
		// 	cv::cv2eigen(M, tmp_M);
		// 	tmp_point = tmp_M * tmp_point;

		// 	key.pt = Point2f(tmp_point(0), tmp_point(1));
		// 	tmp_keypoints.push_back(key);

		// 	key.pt = tmp_pts[i];
		// 	keypoints.push_back(key);
		// }
	}
	// vector<cv::KeyPoint> tmp_keypoints;
	// for (auto i = 0; i < keypoints.size(); i++)
	// {
	// 	cv::KeyPoint key;
	// 	Eigen::Vector3d tmp_point(keypoints[i].pt.x, keypoints[i].pt.y, 1.0);
	// 	double CENTER_X = 645.107791;
	// 	double CENTER_Y = 486.025172;
	// 	M = getRotationMatrix2D(Point2f(CENTER_X, CENTER_Y), yaw, 1.0);
	// 	Eigen::Matrix3d tmp_M;
	// 	cv::cv2eigen(M, tmp_M);
	// 	tmp_point = tmp_M * tmp_point;
	// 	key.pt = Point2f(tmp_point(0), tmp_point(1));
	// 	tmp_keypoints.push_back(key);
	// }
	extractor(rotation_image, keypoints, brief_descriptors);
	for (int i = 0; i < (int)keypoints.size(); i++)
	{
		Eigen::Vector3d tmp_p;
		m_camera->liftProjective(Eigen::Vector2d(keypoints[i].pt.x, keypoints[i].pt.y), tmp_p);
		tmp_p = tmp_p / tmp_p.norm();
		tmp_p = qic.inverse() * origin_vio_R.transpose() * tmp_p;
		Eigen::Vector2d tmp_s;
		m_camera->spaceToPlane(tmp_p, tmp_s);
		cv::KeyPoint tmp_key;
		tmp_key.pt = Point2f(tmp_s(0), tmp_s(1));
		keypoints[i] = tmp_key;
	}
	for (int i = 0; i < (int)keypoints.size(); i++)
	{
		Eigen::Vector3d tmp_p;
		m_camera->liftProjective(Eigen::Vector2d(keypoints[i].pt.x, keypoints[i].pt.y), tmp_p);
		// cv::KeyPoint tmp_norm;
		// tmp_norm.pt = cv::Point2f(tmp_p.x() / tmp_p.z(), tmp_p.y() / tmp_p.z());
		// keypoints_norm.push_back(tmp_norm);
		// changed by wz loop closure
		cv::Point3f tmp_norm(tmp_p.x() / tmp_p.norm(), tmp_p.y() / tmp_p.norm(), tmp_p.z() / tmp_p.norm());
		keypoints_norm.push_back(tmp_norm);
	}
}

void BriefExtractor::operator()(const cv::Mat &im, vector<cv::KeyPoint> &keys, vector<BRIEF::bitset> &descriptors) const
{
	m_brief.compute(im, keys, descriptors);
}

// bool KeyFrame::searchInAera(const BRIEF::bitset window_descriptor,
// 							const std::vector<BRIEF::bitset> &descriptors_old,
// 							const std::vector<cv::KeyPoint> &keypoints_old,
// 							const std::vector<cv::KeyPoint> &keypoints_old_norm,
// 							cv::Point2f &best_match,
// 							cv::Point2f &best_match_norm)
// {
// 	cv::Point2f best_pt;
// 	int bestDist = 128;
// 	int bestIndex = -1;
// 	for (int i = 0; i < (int)descriptors_old.size(); i++)
// 	{

// 		int dis = HammingDis(window_descriptor, descriptors_old[i]);
// 		if (dis < bestDist)
// 		{
// 			bestDist = dis;
// 			bestIndex = i;
// 		}
// 	}
// 	// printf("best dist %d", bestDist);
// 	if (bestIndex != -1 && bestDist < 80)
// 	{
// 		best_match = keypoints_old[bestIndex].pt;
// 		best_match_norm = keypoints_old_norm[bestIndex].pt;
// 		return true;
// 	}
// 	else
// 		return false;
// }
// changed by wz loop closure
bool KeyFrame::searchInAera(const BRIEF::bitset window_descriptor,
							const std::vector<BRIEF::bitset> &descriptors_old,
							const std::vector<cv::KeyPoint> &keypoints_old,
							// const std::vector<cv::KeyPoint> &keypoints_old_norm,
							const std::vector<cv::Point3f> &keypoints_old_norm,
							cv::Point2f &best_match,
							cv::Point3f &best_match_norm)
{
	cv::Point2f best_pt;
	int bestDist = 128;
	// int bestDist = 512;
	int bestIndex = -1;
	for (int i = 0; i < (int)descriptors_old.size(); i++)
	{

		int dis = HammingDis(window_descriptor, descriptors_old[i]);
		if (dis < bestDist)
		{
			bestDist = dis;
			bestIndex = i;
		}
	}
	// printf("best dist %d", bestDist);
	// if (bestIndex != -1 && bestDist < 80)
	// changed by wz loop closure
	if (bestIndex != -1 && bestDist < 60)
	// if (bestIndex != -1 && bestDist < 256)
	{
		best_match = keypoints_old[bestIndex].pt;
		// best_match_norm = keypoints_old_norm[bestIndex].pt;
		best_match_norm = keypoints_old_norm[bestIndex];
		return true;
	}
	else
		return false;
}

// void KeyFrame::searchByBRIEFDes(std::vector<cv::Point2f> &matched_2d_old,
// 								std::vector<cv::Point2f> &matched_2d_old_norm,
// 								std::vector<uchar> &status,
// 								const std::vector<BRIEF::bitset> &descriptors_old,
// 								const std::vector<cv::KeyPoint> &keypoints_old,
// 								const std::vector<cv::KeyPoint> &keypoints_old_norm)
// {
// 	for (int i = 0; i < (int)window_brief_descriptors.size(); i++)
// 	{
// 		cv::Point2f pt(0.f, 0.f);
// 		cv::Point2f pt_norm(0.f, 0.f);
// 		if (searchInAera(window_brief_descriptors[i], descriptors_old, keypoints_old, keypoints_old_norm, pt, pt_norm))
// 			status.push_back(1);
// 		else
// 			status.push_back(0);
// 		matched_2d_old.push_back(pt);
// 		matched_2d_old_norm.push_back(pt_norm);
// 	}
// }
// changed by wz
void KeyFrame::searchByBRIEFDes(std::vector<cv::Point2f> &matched_2d_old,
								std::vector<cv::Point3f> &matched_3d_old_norm,
								std::vector<uchar> &status,
								const std::vector<BRIEF::bitset> &descriptors_old,
								const std::vector<cv::KeyPoint> &keypoints_old,
								const std::vector<cv::Point3f> &keypoints_old_norm)
{
	for (int i = 0; i < (int)window_brief_descriptors.size(); i++)
	{
		cv::Point2f pt(0.f, 0.f);
		// cv::Point2f pt_norm(0.f, 0.f);
		// changed by wz loop clourse
		cv::Point3f pt_norm(0.f, 0.f, 0.f);
		// if (searchInAera(window_brief_descriptors[i], descriptors_old, keypoints_old, keypoints_old_norm, pt, pt_norm))
		// changed by wz loop clourse
		if (searchInAera(window_brief_descriptors[i], descriptors_old, keypoints_old, keypoints_old_norm, pt, pt_norm))
			status.push_back(1);
		else
			status.push_back(0);
		matched_2d_old.push_back(pt);
		// changed by wz loop clourse
		matched_3d_old_norm.push_back(pt_norm);
		// cout << "pt_norm: " << pt_norm << endl;
	}
}
// void KeyFrame::FundmantalMatrixRANSAC(const std::vector<cv::Point2f> &matched_2d_cur_norm,
// 									  const std::vector<cv::Point2f> &matched_2d_old_norm,
// 									  vector<uchar> &status)
// {
// 	int n = (int)matched_2d_cur_norm.size();
// 	for (int i = 0; i < n; i++)
// 		status.push_back(0);
// 	if (n >= 8)
// 	{
// 		vector<cv::Point2f> tmp_cur(n), tmp_old(n);
// 		for (int i = 0; i < (int)matched_2d_cur_norm.size(); i++)
// 		{
// 			double FOCAL_LENGTH = 460.0;
// 			double tmp_x, tmp_y;
// 			tmp_x = FOCAL_LENGTH * matched_2d_cur_norm[i].x + COL / 2.0;
// 			tmp_y = FOCAL_LENGTH * matched_2d_cur_norm[i].y + ROW / 2.0;
// 			tmp_cur[i] = cv::Point2f(tmp_x, tmp_y);

// 			tmp_x = FOCAL_LENGTH * matched_2d_old_norm[i].x + COL / 2.0;
// 			tmp_y = FOCAL_LENGTH * matched_2d_old_norm[i].y + ROW / 2.0;
// 			tmp_old[i] = cv::Point2f(tmp_x, tmp_y);
// 		}
// 		cv::findFundamentalMat(tmp_cur, tmp_old, cv::FM_RANSAC, 3.0, 0.9, status);
// 	}
// }

Eigen::Matrix3d compute_E_21(vector<Eigen::Vector3d> &bearings_1, vector<Eigen::Vector3d> &bearings_2)
{
	const auto num_points = bearings_1.size();

	typedef Eigen::Matrix<Eigen::Matrix3d::Scalar, Eigen::Dynamic, 9> CoeffMatrix;
	CoeffMatrix A(num_points, 9);

	for (unsigned int i = 0; i < num_points; i++)
	{
		A.block<1, 3>(i, 0) = bearings_2.at(i)(0) * bearings_1.at(i);
		A.block<1, 3>(i, 3) = bearings_2.at(i)(1) * bearings_1.at(i);
		A.block<1, 3>(i, 6) = bearings_2.at(i)(2) * bearings_1.at(i);
	}

	const Eigen::JacobiSVD<CoeffMatrix> init_svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);

	const Eigen::Matrix<Eigen::Matrix3d::Scalar, 9, 1> v = init_svd.matrixV().col(8);

	const Eigen::Matrix3d init_E_21 = Eigen::Matrix3d(v.data()).transpose();

	const Eigen::JacobiSVD<Eigen::Matrix3d> svd(init_E_21, Eigen::ComputeFullU | Eigen::ComputeFullV);

	const Eigen::Matrix3d &U = svd.matrixU();
	Eigen::Vector3d lambda = svd.singularValues();
	const Eigen::Matrix3d &V = svd.matrixV();

	lambda(2) = 0.0;

	const Eigen::Matrix3d E_21 = U * lambda.asDiagonal() * V.transpose();

	return E_21;
	// }
	// double check_inliers(Eigen::Matrix3d &E_21, vector<uchar> &is_inlier_match, vector<Eigen::Vector3d> corr1, vector<Eigen::Vector3d> corr2)
	// {
	// 	const auto num_points = corr1.size();

	// 	const Eigen::Matrix3d E_12 = E_21.transpose();

	// 	float score = 0;

	// 	constexpr float residual_cos_thr = 0.00872653549837;

	// 	for (unsigned int i = 0; i < num_points; ++i)
	// 	{
	// 		const auto bearing_1 = corr1[i];
	// 		const auto bearing_2 = corr2[i];
	// 		const Eigen::Vector3d epiplane_in_2 = E_21 * bearing_1;

	// 		const double residual_in_2 = std::abs(epiplane_in_2.dot(bearing_2) / epiplane_in_2.norm());

	// 		if (residual_cos_thr < residual_in_2)
	// 		{

	// 			is_inlier_match.at(i) = 0;
	// 			continue;
	// 		}
	// 		else
	// 		{
	// 			is_inlier_match.at(i) = 1;
	// 			score += pow(residual_cos_thr - residual_in_2, 2.0);
	// 		}

	// 		const Eigen::Vector3d epiplane_in_1 = E_12 * bearing_2;

	// 		const double residual_in_1 = std::abs(epiplane_in_1.dot(bearing_1) / epiplane_in_1.norm());
	// 		if (residual_cos_thr < residual_in_1)
	// 		{
	// 			is_inlier_match.at(i) = 0;
	// 			continue;
	// 		}
	// 		else
	// 		{
	// 			is_inlier_match.at(i) = 1;
	// 			score += pow(residual_cos_thr - residual_in_1, 2.0);
	// 		}
	// 	}

	// 	return score;
	// }
}

void decomposeEssentialMat(Eigen::Matrix3d _E, Eigen::Matrix3d &_R1, Eigen::Matrix3d &_R2, Eigen::Vector3d &_t)
{

	Eigen::JacobiSVD<Eigen::Matrix3d> svd(_E, Eigen::ComputeFullU | Eigen::ComputeFullV);
	Eigen::Matrix3d U = svd.matrixU();
	Eigen::Vector3d lambda = svd.singularValues();
	Eigen::Matrix3d V = svd.matrixV();

	if (U.determinant() < 0)
		U = -U;
	if (V.determinant() < 0)
		V = -V;

	Eigen::Matrix3d W;
	W << 0, 1, 0, -1, 0, 0, 0, 0, 1;

	Eigen::Matrix3d R1, R2;
	Eigen::Vector3d t;
	R1 = U * W * V.transpose();
	R2 = U * W.transpose() * V.transpose();
	t = U.col(2) * 1.0;

	_R1 = R1;
	_R2 = R2;
	_t = t;
}

void triangulatePoint(Eigen::Matrix<double, 3, 4> &Pose0, Eigen::Matrix<double, 3, 4> &Pose1,
					  const Vector3d &point0, const Vector3d &point1, Vector3d &point_3d)
{
	Matrix4d design_matrix = Matrix4d::Zero();
	// changed by wz
	design_matrix.row(0) = point0[0] * Pose0.row(2) - point0[2] * Pose0.row(0);
	design_matrix.row(1) = point0[1] * Pose0.row(2) - point0[2] * Pose0.row(1);
	design_matrix.row(2) = point1[0] * Pose1.row(2) - point1[2] * Pose1.row(0);
	design_matrix.row(3) = point1[1] * Pose1.row(2) - point1[2] * Pose1.row(1);
	Vector4d triangulated_point;
	triangulated_point =
		design_matrix.jacobiSvd(Eigen::ComputeFullV).matrixV().rightCols<1>();
	point_3d(0) = triangulated_point(0) / triangulated_point(3);
	point_3d(1) = triangulated_point(1) / triangulated_point(3);
	point_3d(2) = triangulated_point(2) / triangulated_point(3);
}

void triangulatePoints(Eigen::Matrix<double, 3, 4> &Pose0, Eigen::Matrix<double, 3, 4> &Pose1,
					   const vector<Vector3d> &points0, const vector<Vector3d> &points1, vector<Vector3d> &points_3d)
{
	Eigen::Matrix<double, 3, 4> P0 = Pose0;
	Eigen::Matrix<double, 3, 4> P1 = Pose1;
	points_3d.clear();
	points_3d.resize(points0.size());
	for (auto i = 0; i < points0.size(); i++)
	{
		Vector3d tmp_point_3d;
		triangulatePoint(P0, P1, points0[i], points1[i], tmp_point_3d);
		points_3d[i] = tmp_point_3d;
	}
}
int recoverPose(Eigen::Matrix3d E, vector<Eigen::Vector3d> _points1, vector<Eigen::Vector3d> _points2,
				Matrix3d &_R, Vector3d &_t, vector<uchar> &_mask)
{
	Eigen::Matrix3d R1, R2;
	Eigen::Vector3d t;

	decomposeEssentialMat(E, R1, R2, t);
	Eigen::Matrix<double, 3, 4> P0, P1, P2, P3, P4;
	P0 << 1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 0;
	P1.block<3, 3>(0, 0) = R1;
	P1.block<3, 1>(0, 3) = t;

	P2.block<3, 3>(0, 0) = R2;
	P2.block<3, 1>(0, 3) = t;

	P3.block<3, 3>(0, 0) = R1;
	P3.block<3, 1>(0, 3) = -t;

	P4.block<3, 3>(0, 0) = R2;
	P4.block<3, 1>(0, 3) = -t;

	double dist = 20.0;
	double dist_min = 2.0;
	vector<Vector3d> Q;
	int good1 = 0, good2 = 0, good3 = 0, good4 = 0;
	vector<uchar> mask1, mask2, mask3, mask4;
	// std::cout << "E\n"
	//   << E << std::endl;
	// std::cout << "R1\n"
	//   << R1 << std::endl;
	// std::cout << "R2\n"
	//   << R2 << std::endl;
	// std::cout << "t\n"
	//   << t << std::endl;
	triangulatePoints(P0, P1, _points1, _points2, Q);
	// std::cout << "Q.size():" << Q.size() << std::endl;
	// std::cout << "_points1.size():" << _points1.size() << std::endl;
	// std::cout << "_points2.size():" << _points2.size() << std::endl;
	// double costh = 0.99862953475;
	double costh = 0.9;
	for (int i = 0; i < Q.size(); i++)
	{
		double p_3d_l_dot = _points1[i].dot(Q[i]) / Q[i].norm();
		Vector3d tmp_Q = P1.block<3, 3>(0, 0) * Q[i] + P1.block<3, 1>(0, 3);
		double p_3d_r_dot = _points2[i].dot(tmp_Q) / tmp_Q.norm();
		double l_r_dot = (P1.block<3, 3>(0, 0) * _points1[i]).dot(_points2[i]);

		if (p_3d_l_dot > costh && p_3d_r_dot > costh && l_r_dot > costh)
		// if (l_r_dot > costh)
		{
			good1++;
			mask1.push_back(1);
		}
		else
		{
			mask1.push_back(0);
		}
	}
	triangulatePoints(P0, P2, _points1, _points2, Q);
	for (int i = 0; i < Q.size(); i++)
	{
		double p_3d_l_dot = _points1[i].dot(Q[i]) / Q[i].norm();
		Vector3d tmp_Q = P2.block<3, 3>(0, 0) * Q[i] + P2.block<3, 1>(0, 3);
		double p_3d_r_dot = _points2[i].dot(tmp_Q) / tmp_Q.norm();
		double l_r_dot = (P2.block<3, 3>(0, 0) * _points1[i]).dot(_points2[i]);

		if (p_3d_l_dot > costh && p_3d_r_dot > costh && l_r_dot > costh)
		// if (l_r_dot > costh)
		{
			good2++;
			mask2.push_back(1);
		}
		else
		{
			mask2.push_back(0);
		}
	}
	triangulatePoints(P0, P3, _points1, _points2, Q);
	for (int i = 0; i < Q.size(); i++)
	{
		double p_3d_l_dot = _points1[i].dot(Q[i]) / Q[i].norm();
		Vector3d tmp_Q = P3.block<3, 3>(0, 0) * Q[i] + P3.block<3, 1>(0, 3);
		double p_3d_r_dot = _points2[i].dot(tmp_Q) / tmp_Q.norm();
		double l_r_dot = (P3.block<3, 3>(0, 0) * _points1[i]).dot(_points2[i]);

		if (p_3d_l_dot > costh && p_3d_r_dot > costh && l_r_dot > costh)
		// if (l_r_dot > costh)
		{
			good3++;
			mask3.push_back(1);
		}
		else
		{
			mask3.push_back(0);
		}
	}
	triangulatePoints(P0, P4, _points1, _points2, Q);
	for (int i = 0; i < Q.size(); i++)
	{
		double p_3d_l_dot = _points1[i].dot(Q[i]) / Q[i].norm();
		Vector3d tmp_Q = P4.block<3, 3>(0, 0) * Q[i] + P4.block<3, 1>(0, 3);
		double p_3d_r_dot = _points2[i].dot(tmp_Q) / tmp_Q.norm();
		double l_r_dot = (P4.block<3, 3>(0, 0) * _points1[i]).dot(_points2[i]);

		if (p_3d_l_dot > costh && p_3d_r_dot > costh && l_r_dot > costh)
		// if (l_r_dot > costh)
		{

			good4++;
			mask4.push_back(1);
		}
		else
		{
			mask4.push_back(0);
		}
	}

	// std::cout << "good1:" << good1 << std::endl;
	// std::cout << "good2:" << good2 << std::endl;
	// std::cout << "good3:" << good3 << std::endl;
	// std::cout << "good4:" << good4 << std::endl;
	if (good1 >= good2 && good1 >= good3 && good1 >= good4)
	{
		_R = R1;
		_t = t;
		_mask = mask1;
		return good1;
	}
	else if (good2 >= good1 && good2 >= good3 && good2 >= good4)
	{
		_R = R2;
		_t = t;
		_mask = mask2;
		return good2;
	}
	else if (good3 >= good1 && good3 >= good2 && good3 >= good4)
	{
		_R = R1;
		_t = -t;
		_mask = mask3;
		return good3;
	}
	else
	{
		_R = R2;
		_t = -t;
		_mask = mask4;
		return good4;
	}
}

double check_inliers(Eigen::Matrix3d &E_21, vector<uchar> &is_inlier_match, vector<Eigen::Vector3d> corr1, vector<Eigen::Vector3d> corr2)
{
	const auto num_points = corr1.size();

	const Eigen::Matrix3d E_12 = E_21.transpose();

	float score = 0;

	// constexpr float residual_cos_thr = 0.00872653549837;
	constexpr float residual_cos_thr = 0.05233595624; // 3degree
	// constexpr float residual_cos_thr = 0.004;

	for (unsigned int i = 0; i < num_points; ++i)
	{
		const auto bearing_1 = corr1[i];
		const auto bearing_2 = corr2[i];
		const Eigen::Vector3d epiplane_in_2 = E_21 * bearing_1;

		const double residual_in_2 = std::abs(epiplane_in_2.dot(bearing_2) / epiplane_in_2.norm());

		if (residual_cos_thr < residual_in_2)
		{

			is_inlier_match.at(i) = 0;
			continue;
		}
		else
		{
			is_inlier_match.at(i) = 1;
			score += pow(residual_cos_thr - residual_in_2, 2.0);
		}

		const Eigen::Vector3d epiplane_in_1 = E_12 * bearing_2;

		const double residual_in_1 = std::abs(epiplane_in_1.dot(bearing_1) / epiplane_in_1.norm());
		if (residual_cos_thr < residual_in_1)
		{
			is_inlier_match.at(i) = 0;
			continue;
		}
		else
		{
			is_inlier_match.at(i) = 1;
			score += pow(residual_cos_thr - residual_in_1, 2.0);
		}
	}

	return score;
}

Eigen::Matrix3d myfindFundamentalMat(vector<Eigen::Vector3d> corr1, vector<Eigen::Vector3d> corr2, int method, double theshold, double unknown, vector<uchar> &status)
{
	int size1 = corr1.size();
	int size2 = corr1.size();
	// ROS_WARN_STREAM("size1:" << size1 << "size2:" << size2);
	constexpr unsigned int min_set_size = 8;

	if (size1 != size2 | size1 < 8 | size2 < 8)
	{
		ROS_ERROR("findFundamentalMat fault");
	}
	const unsigned int num_matches = size1;
	Eigen::Matrix3d best_E_21_;
	double best_score_ = 0.0;
	int max_num_iter = 500;

	vector<Eigen::Vector3d> min_set_bearings_1(min_set_size);
	vector<Eigen::Vector3d> min_set_bearings_2(min_set_size);
	vector<uchar> is_inlier_match_in_sac;
	vector<uchar> is_inlier_match_;

	Eigen::Matrix3d E_21_in_sac;
	double score_in_sac = 0.0;
	TicToc t_ff;
	for (unsigned int iter = 0; iter < max_num_iter; iter++)
	{

		const auto indices = util::create_random_array(min_set_size, 0U, num_matches - 1);
		for (unsigned int i = 0; i < min_set_size; ++i)
		{
			const auto idx = indices.at(i);

			min_set_bearings_1.at(i) = corr1[idx];
			min_set_bearings_2.at(i) = corr2[idx];
		}
		E_21_in_sac = compute_E_21(min_set_bearings_1, min_set_bearings_2);
		is_inlier_match_in_sac.resize(num_matches, 0);
		score_in_sac = check_inliers(E_21_in_sac, is_inlier_match_in_sac, corr1, corr2);
		if (best_score_ < score_in_sac)
		{
			best_score_ = score_in_sac;
			best_E_21_ = E_21_in_sac;
			is_inlier_match_ = is_inlier_match_in_sac;
		}
	}
	vector<Eigen::Vector3d> inlier_bearing_1;
	vector<Eigen::Vector3d> inlier_bearing_2;
	inlier_bearing_1.reserve(num_matches);
	inlier_bearing_2.reserve(num_matches);
	for (unsigned int i = 0; i < num_matches; ++i)
	{
		if (is_inlier_match_.at(i))
		{
			inlier_bearing_1.push_back(corr1[i]);
			inlier_bearing_2.push_back(corr2[i]);
		}
	}
	best_E_21_ = compute_E_21(inlier_bearing_1, inlier_bearing_2);
	best_score_ = check_inliers(best_E_21_, is_inlier_match_, corr1, corr2);

	vector<int> index;
	inlier_bearing_1.clear();
	inlier_bearing_2.clear();
	for (auto i = 0; i < num_matches; i++)
	{
		if (is_inlier_match_.at(i))
		{
			inlier_bearing_1.push_back(corr1[i]);
			inlier_bearing_2.push_back(corr2[i]);
			index.push_back(i);
			// ROS_INFO_STREAM("i:" << i);
		}
	}
	vector<uchar> E_match_status;
	Matrix3d R_best;
	Vector3d t_best;
	// ROS_ERROR("E_ransac!!!");
	int E_num = recoverPose(best_E_21_, inlier_bearing_1, inlier_bearing_2, R_best, t_best, E_match_status);
	// ROS_ERROR_STREAM("E_ransac num: " << E_num);
	// ROS_ERROR_STREAM("E_match_status: " << E_match_status.size());
	// ROS_ERROR_STREAM("is_inlier_match_: " << is_inlier_match_.size());
	// for (auto i = 0; i < is_inlier_match_.size(); i++)
	// {
	// 	ROS_WARN_STREAM("is_inlier_match_:" << int(is_inlier_match_.at(i)));
	// }
	// for (auto i = 0; i < index.size(); i++)
	// {
	// 	ROS_INFO_STREAM("index:" << index.at(i));
	// }
	for (auto i = 0; i < E_match_status.size(); i++)
	{
		// ROS_INFO_STREAM("E_ransac status: " << E_match_status.at(i));
		if (E_match_status.at(i) == 0)
		{
			// ROS_INFO("E_ransac DELETE");
			// ROS_INFO_STREAM("INDEX: " << index.at(i));
			is_inlier_match_.at(index.at(i)) = 0;
		}
	}
	status = is_inlier_match_;

	return best_E_21_;
}

bool solveRelativeRT(const vector<pair<Vector3d, Vector3d>> &corres, Matrix3d &Rotation, Vector3d &Translation)
{
	if (corres.size() >= 15)
	{
		vector<cv::Point2f> ll, rr;
		for (int i = 0; i < int(corres.size()); i++)
		{
			ll.push_back(cv::Point2f(corres[i].first(0) / corres[i].first(2), corres[i].first(1) / corres[i].first(2)));
			rr.push_back(cv::Point2f(corres[i].second(0) / corres[i].second(2), corres[i].second(1) / corres[i].second(2)));
		}

		vector<Eigen::Vector3d> ll_3d, rr_3d;
		for (int i = 0; i < int(corres.size()); i++)
		{
			ll_3d.push_back(corres[i].first);
			rr_3d.push_back(corres[i].second);
		}
		vector<uchar> state;
		Eigen::Matrix3d E_my = myfindFundamentalMat(ll_3d, rr_3d, cv::FM_RANSAC, 0.3 / 460, 0.99, state);

		cv::Mat true_E = cv::findFundamentalMat(ll, rr);
		Eigen::Matrix3d cv_E;
		cv::cv2eigen(true_E, cv_E);

		Matrix3d rot;
		Vector3d trans;
		vector<uchar> mask;

		int inlier_cnt = recoverPose(E_my, ll_3d, rr_3d, rot, trans, mask);

		Eigen::Matrix3d R;
		Eigen::Vector3d T;
		Rotation = R.transpose();
		Translation = -R.transpose() * T;
		if (inlier_cnt > 12)
			return true;
		else
			return false;
	}
	return false;
}

void KeyFrame::FundmantalMatrixRANSAC(const std::vector<cv::Point3f> &matched_3d_cur_norm,
									  const std::vector<cv::Point3f> &matched_3d_old_norm,
									  vector<uchar> &status)
{
	int n = (int)matched_3d_cur_norm.size();
	for (int i = 0; i < n; i++)
		status.push_back(0);
	if (n >= 8)
	{
		// vector<cv::Point2f> tmp_cur(n), tmp_old(n);
		// for (int i = 0; i < (int)matched_3d_cur_norm.size(); i++)
		// {
		// 	double FOCAL_LENGTH = 160.0;
		// 	double tmp_x, tmp_y;

		// 	tmp_x = FOCAL_LENGTH * matched_3d_cur_norm[i].x / matched_3d_cur_norm[i].z + COL / 2.0;
		// 	tmp_y = FOCAL_LENGTH * matched_3d_cur_norm[i].y / matched_3d_cur_norm[i].z + ROW / 2.0;
		// 	tmp_cur[i] = cv::Point2f(tmp_x, tmp_y);

		// 	tmp_x = FOCAL_LENGTH * matched_3d_old_norm[i].x / matched_3d_old_norm[i].z + COL / 2.0;
		// 	tmp_y = FOCAL_LENGTH * matched_3d_old_norm[i].y / matched_3d_old_norm[i].z + ROW / 2.0;
		// 	tmp_old[i] = cv::Point2f(tmp_x, tmp_y);
		// }
		// cv::findFundamentalMat(tmp_cur, tmp_old, cv::FM_RANSAC, 3.0, 0.9, status);
		vector<Eigen::Vector3d> tmp_cur(n), tmp_old(n);
		for (int i = 0; i < (int)matched_3d_cur_norm.size(); i++)
		{

			tmp_cur[i] = Eigen::Vector3d(matched_3d_cur_norm[i].x, matched_3d_cur_norm[i].y, matched_3d_cur_norm[i].z);
			tmp_old[i] = Eigen::Vector3d(matched_3d_old_norm[i].x, matched_3d_old_norm[i].y, matched_3d_old_norm[i].z);
		}
		myfindFundamentalMat(tmp_cur, tmp_old, cv::FM_RANSAC, 1.0, 0.99, status);
		// for (int i = 0; i < status.size(); i++)
		// {
		// 	ROS_ERROR("status[%d] = %d", i, status[i]);
		// }
	}
}

// void KeyFrame::PnPRANSAC(const vector<cv::Point2f> &matched_2d_old_norm,
// 						 const std::vector<cv::Point3f> &matched_3d,
// 						 std::vector<uchar> &status,
// 						 Eigen::Vector3d &PnP_T_old, Eigen::Matrix3d &PnP_R_old)
// {
// 	// for (int i = 0; i < matched_3d.size(); i++)
// 	//	printf("3d x: %f, y: %f, z: %f\n",matched_3d[i].x, matched_3d[i].y, matched_3d[i].z );
// 	// printf("match size %d \n", matched_3d.size());
// 	cv::Mat r, rvec, t, D, tmp_r;
// 	cv::Mat K = (cv::Mat_<double>(3, 3) << 1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0);
// 	Matrix3d R_inital;
// 	Vector3d P_inital;
// 	Matrix3d R_w_c = origin_vio_R * qic;
// 	Vector3d T_w_c = origin_vio_T + origin_vio_R * tic;

// 	R_inital = R_w_c.inverse();
// 	P_inital = -(R_inital * T_w_c);

// 	cv::eigen2cv(R_inital, tmp_r);
// 	cv::Rodrigues(tmp_r, rvec);
// 	cv::eigen2cv(P_inital, t);

// 	cv::Mat inliers;
// 	TicToc t_pnp_ransac;

// 	if (CV_MAJOR_VERSION < 3)
// 		solvePnPRansac(matched_3d, matched_2d_old_norm, K, D, rvec, t, true, 100, 10.0 / 460.0, 100, inliers);
// 	else
// 	{
// 		if (CV_MINOR_VERSION < 2)
// 			solvePnPRansac(matched_3d, matched_2d_old_norm, K, D, rvec, t, true, 100, sqrt(10.0 / 460.0), 0.99, inliers);
// 		else
// 			solvePnPRansac(matched_3d, matched_2d_old_norm, K, D, rvec, t, true, 100, 10.0 / 460.0, 0.99, inliers);
// 	}

// 	for (int i = 0; i < (int)matched_2d_old_norm.size(); i++)
// 		status.push_back(0);

// 	for (int i = 0; i < inliers.rows; i++)
// 	{
// 		int n = inliers.at<int>(i);
// 		status[n] = 1;
// 	}

// 	cv::Rodrigues(rvec, r);
// 	Matrix3d R_pnp, R_w_c_old;
// 	cv::cv2eigen(r, R_pnp);
// 	R_w_c_old = R_pnp.transpose();
// 	Vector3d T_pnp, T_w_c_old;
// 	cv::cv2eigen(t, T_pnp);
// 	T_w_c_old = R_w_c_old * (-T_pnp);

// 	PnP_R_old = R_w_c_old * qic.transpose();
// 	PnP_T_old = T_w_c_old - PnP_R_old * tic;
// }

// changed by wz loop clourse
// void KeyFrame::PnPRANSAC(const vector<cv::Point3f> &matched_3d_old_norm,
// 						 const std::vector<cv::Point3f> &matched_3d,
// 						 std::vector<uchar> &status,
// 						 Eigen::Vector3d &PnP_T_old, Eigen::Matrix3d &PnP_R_old)
// {
// 	// for (int i = 0; i < matched_3d.size(); i++)
// 	//	printf("3d x: %f, y: %f, z: %f\n",matched_3d[i].x, matched_3d[i].y, matched_3d[i].z );
// 	// printf("match size %d \n", matched_3d.size());
// 	cv::Mat r, rvec, t, D, tmp_r;
// 	cv::Mat K = (cv::Mat_<double>(3, 3) << 1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0);
// 	Matrix3d R_inital;
// 	Vector3d P_inital;
// 	Matrix3d R_w_c = origin_vio_R * qic;
// 	Vector3d T_w_c = origin_vio_T + origin_vio_R * tic;

// 	R_inital = R_w_c.inverse();
// 	P_inital = -(R_inital * T_w_c);

// 	cv::eigen2cv(R_inital, tmp_r);
// 	cv::Rodrigues(tmp_r, rvec);
// 	cv::eigen2cv(P_inital, t);

// 	cv::Mat inliers;
// 	TicToc t_pnp_ransac;

// 	// changed by wz loop clourse
// 	std::vector<cv::Point2f> matched_2d_old_norm;
// 	std::vector<cv::Point3f> matched_3d_tmp;
// 	for (int i = 0; i < matched_3d_old_norm.size(); i++)
// 	{
// 		// if (matched_3d_old_norm[i].z > 0.0)
// 		{
// 			matched_2d_old_norm.push_back(cv::Point2f(matched_3d_old_norm[i].x / matched_3d_old_norm[i].z, matched_3d_old_norm[i].y / matched_3d_old_norm[i].z));
// 			matched_3d_tmp.push_back(matched_3d[i]);
// 		}
// 	}
// 	ROS_WARN("PnPRANSAC: %d", matched_2d_old_norm.size());

// 	if (CV_MAJOR_VERSION < 3)
// 	{
// 		solvePnPRansac(matched_3d_tmp, matched_2d_old_norm, K, D, rvec, t, true, 100, 10.0 / 460.0, 100, inliers);
// 	}
// 	else
// 	{
// 		if (CV_MINOR_VERSION < 2)
// 			solvePnPRansac(matched_3d_tmp, matched_2d_old_norm, K, D, rvec, t, true, 100, sqrt(10.0 / 460.0), 0.99, inliers);
// 		else
// 			solvePnPRansac(matched_3d_tmp, matched_2d_old_norm, K, D, rvec, t, true, 100, 10.0 / 460.0, 0.99, inliers);
// 	}
// 	ROS_WARN("PnPRANSAC_Finish");
// 	// for (int i = 0; i < (int)matched_2d_old_norm.size(); i++)
// 	// 	status.push_back(0);
// 	// changed by wz loop clourse
// 	vector<int> index;
// 	index.clear();
// 	for (int i = 0; i < (int)matched_3d_old_norm.size(); i++)
// 	{
// 		status.push_back(0);
// 		// if (matched_3d_old_norm[i].z > 0.0)
// 		{
// 			index.push_back(i);
// 		}
// 		ROS_WARN_STREAM("matched_3d_old_norm.size:" << matched_3d_old_norm.size() << " i" << i);
// 	}

// 	for (int i = 0; i < inliers.rows; i++)
// 	{
// 		int n = inliers.at<int>(i);
// 		// changed by wz loop clourse
// 		// status[n] = 1;
// 		status[index[n]] = 1;
// 	}

// 	cv::Rodrigues(rvec, r);
// 	Matrix3d R_pnp, R_w_c_old;
// 	cv::cv2eigen(r, R_pnp);
// 	R_w_c_old = R_pnp.transpose();
// 	Vector3d T_pnp, T_w_c_old;
// 	cv::cv2eigen(t, T_pnp);
// 	T_w_c_old = R_w_c_old * (-T_pnp);

// 	PnP_R_old = R_w_c_old * qic.transpose();
// 	PnP_T_old = T_w_c_old - PnP_R_old * tic;
// }
// void mysolvePnPRansac(std::vector<Eigen::Vector3d> &matched_3d_tmp, std::vector<Eigen::Vector3d> &matched_3d_old_norm_tmp, Matrix3d &R_pnp,
// 					  Vector3d &T_pnp, std::vector<uchar> &status)
// {
// 	PnpSolver pnp1;
// 	pnp1.set_internal_parameters(0, 0, 1, 1);

// 	// int featureNums = matched_3d_tmp.size();
// 	int min_set = matched_3d_old_norm_tmp.size();
// 	pnp1.set_maximum_number_of_correspondences(min_set);
// 	for (auto num = 0; num < matched_3d_tmp.size(); num++)
// 	{
// 		pnp1.add_correspondence(matched_3d_tmp[num], matched_3d_old_norm_tmp[num]);
// 	}
// 	pnp1.compute_pose(R_pnp, T_pnp);
// }
unsigned int pnp_check_inliers(Eigen::Matrix3d &R_pnp, Eigen::Vector3d &T_pnp, std::vector<Eigen::Vector3d> &matched_3d_tmp, std::vector<Eigen::Vector3d> &matched_3d_old_norm_tmp, std::vector<uchar> &is_inlier_match_in_sac)
{
	unsigned int num_inliers = 0;
	int matched_size = matched_3d_tmp.size();
	for (unsigned int i = 0; i < matched_size; i++)
	{
		Eigen::Vector3d p_matched = matched_3d_tmp[i];
		Eigen::Vector3d p_matched_old_norm = matched_3d_old_norm_tmp[i];
		Eigen::Vector3d p_pnp = R_pnp * p_matched + T_pnp;
		const double cos_theta = p_pnp.dot(p_matched_old_norm) / (p_pnp.norm() * p_matched_old_norm.norm());
		// ROS_INFO_STREAM("cos_theta: " << cos_theta);
		// double costh = 0.99862953475;
		double costh = 0.9;

		if (cos_theta > costh)
		// if (cos_theta >= 1.0)
		{
			is_inlier_match_in_sac[i] = 1;
			num_inliers++;
		}
		else
		{
			is_inlier_match_in_sac[i] = 0;
		}
	}
	return num_inliers;
}
void mysolvePnPRansac(std::vector<Eigen::Vector3d> &matched_3d_tmp, std::vector<Eigen::Vector3d> &matched_3d_old_norm_tmp, Matrix3d &R_pnp,
					  Vector3d &T_pnp, std::vector<uchar> &status)
{
	PnpSolver pnp1;
	pnp1.set_internal_parameters(0, 0, 1, 1);

	int matched_size = matched_3d_tmp.size();
	const unsigned int num_matches = matched_size;
	constexpr unsigned int min_set_size = 4;

	int max_num_iter = 1000;

	vector<uchar> is_inlier_match_in_sac;
	vector<uchar> is_inlier_match;
	int max_num_inliers = 0;
	Eigen::Matrix3d R_pnp_sac, R_pnp_sac_best;
	Eigen::Vector3d T_pnp_sac, T_pnp_sac_best;

	// if (matched_size < min_set_size)
	// {
	// 	ROS_WARN("Not enough matched points to solve PnP");
	// 	return;
	// }
	// else
	// {
	// 	ROS_INFO("MATRIX SIZE: %d", matched_size);
	// }
	// ROS_WARN("pnp_check_inliers");
	for (unsigned int iter = 0; iter < max_num_iter; ++iter)
	{
		// if (iter % 100 == 0)
		// {
		// 	ROS_WARN_STREAM("iter:" << iter);
		// }
		const auto random_indices = util::create_random_array(min_set_size, 0U, num_matches - 1);
		// ROS_INFO_STREAM("random_indices:" << random_indices.size());
		assert(random_indices.size() == min_set_size);
		pnp1.reset_number_of_correspondences();
		pnp1.set_maximum_number_of_correspondences(min_set_size);
		for (const auto i : random_indices)
		{
			pnp1.add_correspondence(matched_3d_tmp[i], matched_3d_old_norm_tmp[i]);
		}
		// ROS_INFO_STREAM("pnp1.compute_pose");
		pnp1.compute_pose(R_pnp_sac, T_pnp_sac);
		// ROS_INFO_STREAM("pnp1.compute_pose done,R_pnp_sac:" << R_pnp_sac << ",T_pnp_sac:" << T_pnp_sac);
		is_inlier_match_in_sac.resize(num_matches, 0);
		const auto num_inliers = pnp_check_inliers(R_pnp_sac, T_pnp_sac, matched_3d_tmp, matched_3d_old_norm_tmp, is_inlier_match_in_sac);
		// ROS_INFO_STREAM("pnp_check_inliers done, num_inliers:" << num_inliers);
		if (max_num_inliers < num_inliers)
		{
			max_num_inliers = num_inliers;
			is_inlier_match = is_inlier_match_in_sac;
			R_pnp_sac_best = R_pnp_sac;
			T_pnp_sac_best = T_pnp_sac;
			// ROS_INFO_STREAM("max_num_inliers:" << max_num_inliers);

			// for (const auto i : random_indices)
			// {
			// 	ROS_ERROR_STREAM("matched_3d_tmp[i]:" << matched_3d_tmp[i] << ",matched_3d_old_norm_tmp[i]:" << matched_3d_old_norm_tmp[i]);
			// }
			// ROS_INFO_STREAM("R_pnp_sac_best:" << R_pnp_sac_best << "T_pnp_sac_best:" << T_pnp_sac_best);
		}
	}
	// pnp1.reset_number_of_correspondences();
	// pnp1.set_maximum_number_of_correspondences(max_num_inliers);
	// // ROS_INFO_STREAM("max_num_inliers:" << max_num_inliers);
	// for (unsigned int i = 0; i < matched_size; i++)
	// {
	// 	if (is_inlier_match[i] == 1)
	// 	{
	// 		pnp1.add_correspondence(matched_3d_tmp[i], matched_3d_old_norm_tmp[i]);
	// 		// ROS_INFO_STREAM("matched_3d_tmp[i]:" << matched_3d_tmp[i] << "matched_3d_old_norm_tmp[i]:" << matched_3d_old_norm_tmp[i]);
	// 	}
	// }
	// pnp1.compute_pose(R_pnp, T_pnp);
	//! changed by wz
	R_pnp = R_pnp_sac_best;
	T_pnp = T_pnp_sac_best;
	status = is_inlier_match;
}
void KeyFrame::PnPRANSAC(const vector<cv::Point3f> &matched_3d_old_norm,
						 const std::vector<cv::Point3f> &matched_3d,
						 std::vector<uchar> &status,
						 Eigen::Vector3d &PnP_T_old, Eigen::Matrix3d &PnP_R_old)
{
	// for (int i = 0; i < matched_3d.size(); i++)
	//	printf("3d x: %f, y: %f, z: %f\n",matched_3d[i].x, matched_3d[i].y, matched_3d[i].z );
	// printf("match size %d \n", matched_3d.size());
	cv::Mat r, rvec, t, D, tmp_r;
	cv::Mat K = (cv::Mat_<double>(3, 3) << 1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0);
	Matrix3d R_inital;
	Vector3d P_inital;
	Matrix3d R_w_c = origin_vio_R * qic;
	Vector3d T_w_c = origin_vio_T + origin_vio_R * tic;

	R_inital = R_w_c.inverse();
	P_inital = -(R_inital * T_w_c);

	cv::eigen2cv(R_inital, tmp_r);
	cv::Rodrigues(tmp_r, rvec);
	cv::eigen2cv(P_inital, t);

	cv::Mat inliers;
	TicToc t_pnp_ransac;

	// changed by wz loop clourse
	std::vector<Eigen::Vector3d> matched_3d_old_norm_tmp;
	std::vector<Eigen::Vector3d> matched_3d_tmp;
	for (int i = 0; i < matched_3d_old_norm.size(); i++)
	{
		matched_3d_old_norm_tmp.push_back(Eigen::Vector3d(matched_3d_old_norm[i].x, matched_3d_old_norm[i].y, matched_3d_old_norm[i].z));
		matched_3d_tmp.push_back(Eigen::Vector3d(matched_3d[i].x, matched_3d[i].y, matched_3d[i].z));
	}

	// solvePnPRansac(matched_3d_tmp, matched_3d_old_norm, K, D, rvec, t, true, 100, 10.0 / 460.0, 0.99, inliers);
	for (int i = 0; i < (int)matched_3d_old_norm.size(); i++)
	{
		status.push_back(0);
	}

	Matrix3d R_pnp;
	Vector3d T_pnp;

	mysolvePnPRansac(matched_3d_tmp, matched_3d_old_norm_tmp, R_pnp, T_pnp, status);
	// for (auto i = 0; i < status.size(); i++)
	// {
	// 	ROS_INFO_STREAM("status[" << i << "]:" << int(status[i]));
	// }
	Matrix3d R_w_c_old;
	R_w_c_old = R_pnp.transpose();
	Vector3d T_w_c_old;
	T_w_c_old = R_w_c_old * (-T_pnp);

	PnP_R_old = R_w_c_old * qic.transpose();
	PnP_T_old = T_w_c_old - PnP_R_old * tic;
}

bool KeyFrame::findConnection(KeyFrame *old_kf)
{
	TicToc tmp_t;
	// printf("find Connection\n");
	vector<cv::Point2f> matched_2d_cur, matched_2d_old;
	// vector<cv::Point2f> matched_2d_cur_norm, matched_2d_old_norm;
	// changed by wz loop closure
	vector<cv::Point3f> matched_3d_cur_norm, matched_3d_old_norm;
	vector<cv::Point3f> matched_3d;
	vector<double> matched_id;
	vector<uchar> status;

	matched_3d = point_3d;
	matched_2d_cur = point_2d_uv;
	// matched_2d_cur_norm = point_2d_norm;
	// changed by wz loop closure
	matched_3d_cur_norm = point_3d_norm;
	matched_id = point_id;

	TicToc t_match;
#if 0
		if (DEBUG_IMAGE)    
	    {
	        cv::Mat gray_img, loop_match_img;
	        cv::Mat old_img = old_kf->image;
	        cv::hconcat(image, old_img, gray_img);
	        cvtColor(gray_img, loop_match_img, CV_GRAY2RGB);
	        for(int i = 0; i< (int)point_2d_uv.size(); i++)
	        {
	            cv::Point2f cur_pt = point_2d_uv[i];
	            cv::circle(loop_match_img, cur_pt, 5, cv::Scalar(0, 255, 0));
	        }
	        for(int i = 0; i< (int)old_kf->keypoints.size(); i++)
	        {
	            cv::Point2f old_pt = old_kf->keypoints[i].pt;
	            old_pt.x += COL;
	            cv::circle(loop_match_img, old_pt, 5, cv::Scalar(0, 255, 0));
	        }
	        ostringstream path;
	        path << "/home/tony-ws1/raw_data/loop_image/"
	                << index << "-"
	                << old_kf->index << "-" << "0raw_point.jpg";
	        cv::imwrite( path.str().c_str(), loop_match_img);
	    }
#endif
	// printf("search by des\n");
	// searchByBRIEFDes(matched_2d_old, matched_2d_old_norm, status, old_kf->brief_descriptors, old_kf->keypoints, old_kf->keypoints_norm);
	// changed by wz loop closure
	searchByBRIEFDes(matched_2d_old, matched_3d_old_norm, status, old_kf->brief_descriptors, old_kf->keypoints, old_kf->keypoints_norm);
	reduceVector(matched_2d_cur, status);
	reduceVector(matched_2d_old, status);
	// reduceVector(matched_2d_cur_norm, status);
	// reduceVector(matched_2d_old_norm, status);
	// changed by wz loop closure
	reduceVector(matched_3d_cur_norm, status);
	reduceVector(matched_3d_old_norm, status);
	reduceVector(matched_3d, status);
	reduceVector(matched_id, status);
	// printf("search by des finish\n");

#if 0 
		if (DEBUG_IMAGE)
	    {
			int gap = 10;
        	cv::Mat gap_image(ROW, gap, CV_8UC1, cv::Scalar(255, 255, 255));
            cv::Mat gray_img, loop_match_img;
            cv::Mat old_img = old_kf->image;
            cv::hconcat(image, gap_image, gap_image);
            cv::hconcat(gap_image, old_img, gray_img);
            cvtColor(gray_img, loop_match_img, CV_GRAY2RGB);
	        for(int i = 0; i< (int)matched_2d_cur.size(); i++)
	        {
	            cv::Point2f cur_pt = matched_2d_cur[i];
	            cv::circle(loop_match_img, cur_pt, 5, cv::Scalar(0, 255, 0));
	        }
	        for(int i = 0; i< (int)matched_2d_old.size(); i++)
	        {
	            cv::Point2f old_pt = matched_2d_old[i];
	            old_pt.x += (COL + gap);
	            cv::circle(loop_match_img, old_pt, 5, cv::Scalar(0, 255, 0));
	        }
	        for (int i = 0; i< (int)matched_2d_cur.size(); i++)
	        {
	            cv::Point2f old_pt = matched_2d_old[i];
	            old_pt.x +=  (COL + gap);
	            cv::line(loop_match_img, matched_2d_cur[i], old_pt, cv::Scalar(0, 255, 0), 1, 8, 0);
	        }

	        ostringstream path, path1, path2;
	        path <<  "/home/tony-ws1/raw_data/loop_image/"
	                << index << "-"
	                << old_kf->index << "-" << "1descriptor_match.jpg";
	        cv::imwrite( path.str().c_str(), loop_match_img);
	        /*
	        path1 <<  "/home/tony-ws1/raw_data/loop_image/"
	                << index << "-"
	                << old_kf->index << "-" << "1descriptor_match_1.jpg";
	        cv::imwrite( path1.str().c_str(), image);
	        path2 <<  "/home/tony-ws1/raw_data/loop_image/"
	                << index << "-"
	                << old_kf->index << "-" << "1descriptor_match_2.jpg";
	        cv::imwrite( path2.str().c_str(), old_img);	        
	        */
	        
	    }
#endif
	status.clear();
	/*
	FundmantalMatrixRANSAC(matched_2d_cur_norm, matched_2d_old_norm, status);
	reduceVector(matched_2d_cur, status);
	reduceVector(matched_2d_old, status);
	reduceVector(matched_2d_cur_norm, status);
	reduceVector(matched_2d_old_norm, status);
	reduceVector(matched_3d, status);
	reduceVector(matched_id, status);
	*/
	// changed by wz loop closure
	// Eigen::Vector3d PnP_T_test;
	// Eigen::Matrix3d PnP_R_test;
	// PnPRANSAC(matched_3d_cur_norm, matched_3d, status, PnP_T_test, PnP_R_test);
	// reduceVector(matched_2d_cur, status);
	// reduceVector(matched_2d_old, status);
	// reduceVector(matched_3d_cur_norm, status);
	// reduceVector(matched_3d_old_norm, status);
	// reduceVector(matched_3d, status);
	// reduceVector(matched_id, status);
	vector<cv::Point2f> all_matched_2d_cur = matched_2d_cur;
	vector<cv::Point2f> all_matched_2d_old = matched_2d_old;
	FundmantalMatrixRANSAC(matched_3d_cur_norm, matched_3d_old_norm, status);
	reduceVector(matched_2d_cur, status);
	reduceVector(matched_2d_old, status);
	reduceVector(matched_3d_cur_norm, status);
	reduceVector(matched_3d_old_norm, status);
	reduceVector(matched_3d, status);
	reduceVector(matched_id, status);
#if 0
		if (DEBUG_IMAGE)
	    {
			int gap = 10;
        	cv::Mat gap_image(ROW, gap, CV_8UC1, cv::Scalar(255, 255, 255));
            cv::Mat gray_img, loop_match_img;
            cv::Mat old_img = old_kf->image;
            cv::hconcat(image, gap_image, gap_image);
            cv::hconcat(gap_image, old_img, gray_img);
            cvtColor(gray_img, loop_match_img, CV_GRAY2RGB);
	        for(int i = 0; i< (int)matched_2d_cur.size(); i++)
	        {
	            cv::Point2f cur_pt = matched_2d_cur[i];
	            cv::circle(loop_match_img, cur_pt, 5, cv::Scalar(0, 255, 0));
	        }
	        for(int i = 0; i< (int)matched_2d_old.size(); i++)
	        {
	            cv::Point2f old_pt = matched_2d_old[i];
	            old_pt.x += (COL + gap);
	            cv::circle(loop_match_img, old_pt, 5, cv::Scalar(0, 255, 0));
	        }
	        for (int i = 0; i< (int)matched_2d_cur.size(); i++)
	        {
	            cv::Point2f old_pt = matched_2d_old[i];
	            old_pt.x +=  (COL + gap) ;
	            cv::line(loop_match_img, matched_2d_cur[i], old_pt, cv::Scalar(0, 255, 0), 1, 8, 0);
	        }

	        ostringstream path;
	        path <<  "/home/tony-ws1/raw_data/loop_image/"
	                << index << "-"
	                << old_kf->index << "-" << "2fundamental_match.jpg";
	        cv::imwrite( path.str().c_str(), loop_match_img);
	    }
#endif
	Eigen::Vector3d PnP_T_old;
	Eigen::Matrix3d PnP_R_old;
	Eigen::Vector3d relative_t;
	Quaterniond relative_q;
	double relative_yaw;
	if ((int)matched_2d_cur.size() > MIN_LOOP_NUM)
	{
		status.clear();
		// changed by wz loop clourse
		// PnPRANSAC(matched_2d_old_norm, matched_3d, status, PnP_T_old, PnP_R_old);
		PnPRANSAC(matched_3d_cur_norm, matched_3d, status, PnP_T_old, PnP_R_old);
		reduceVector(matched_2d_cur, status);
		reduceVector(matched_2d_old, status);
		// reduceVector(matched_2d_cur_norm, status);
		// reduceVector(matched_2d_old_norm, status);
		// changed by wz loop clourse
		reduceVector(matched_3d_cur_norm, status);
		reduceVector(matched_3d_old_norm, status);
		reduceVector(matched_3d, status);
		reduceVector(matched_id, status);

		// FundmantalMatrixRANSAC(matched_3d_cur_norm, matched_3d_old_norm, status);
		// reduceVector(matched_2d_cur, status);
		// reduceVector(matched_2d_old, status);
		// reduceVector(matched_3d_cur_norm, status);
		// reduceVector(matched_3d_old_norm, status);
		// reduceVector(matched_3d, status);
		// reduceVector(matched_id, status);

#if 1
		if (DEBUG_IMAGE)
		{
			// int gap = 10;
			int gap = 0;
			cv::Mat gap_image(ROW, gap, CV_8UC1, cv::Scalar(255, 255, 255));
			cv::Mat gray_img, loop_match_img;
			cv::Mat old_img = old_kf->image;
			double center_x = 640.991301;
			double center_y = 490.937512;
			int max_r = 509;
			int min_r = 160;
			cv::circle(image, cv::Point2f(center_x, center_y), min_r, cv::Scalar(255, 255, 255), -1);
			cv::circle(image, cv::Point2f(center_x, center_y), 1280, cv::Scalar(255, 255, 255), (1280 - max_r) * 2);
			cv::circle(old_img, cv::Point2f(center_x, center_y), min_r, cv::Scalar(255, 255, 255), -1);
			cv::circle(old_img, cv::Point2f(center_x, center_y), 1280, cv::Scalar(255, 255, 255), (1280 - max_r) * 2);

			cv::hconcat(image, gap_image, gap_image);

			cv::hconcat(gap_image, old_img, gray_img);
			cvtColor(gray_img, loop_match_img, CV_GRAY2RGB);

			for (int i = 0; i < (int)all_matched_2d_cur.size(); i++)
			{
				cv::Point2f cur_pt = all_matched_2d_cur[i];
				cv::circle(loop_match_img, cur_pt, 7, cv::Scalar(255, 0, 0), -1);
			}
			for (int i = 0; i < (int)all_matched_2d_old.size(); i++)
			{
				cv::Point2f old_pt = all_matched_2d_old[i];
				old_pt.x += (COL + gap);
				cv::circle(loop_match_img, old_pt, 7, cv::Scalar(255, 0, 0), -1);
			}
			for (int i = 0; i < (int)all_matched_2d_cur.size(); i++)
			{
				cv::Point2f old_pt = all_matched_2d_old[i];
				old_pt.x += (COL + gap);
				cv::line(loop_match_img, all_matched_2d_cur[i], old_pt, cv::Scalar(255, 0, 0), 2, 8, 0);
			}

			for (int i = 0; i < (int)matched_2d_cur.size(); i++)
			{
				cv::Point2f cur_pt = matched_2d_cur[i];
				cv::circle(loop_match_img, cur_pt, 7, cv::Scalar(0, 0, 255), -1);
			}
			for (int i = 0; i < (int)matched_2d_old.size(); i++)
			{
				cv::Point2f old_pt = matched_2d_old[i];
				old_pt.x += (COL + gap);
				cv::circle(loop_match_img, old_pt, 7, cv::Scalar(0, 0, 255), -1);
			}
			for (int i = 0; i < (int)matched_2d_cur.size(); i++)
			{
				cv::Point2f old_pt = matched_2d_old[i];
				old_pt.x += (COL + gap);
				cv::line(loop_match_img, matched_2d_cur[i], old_pt, cv::Scalar(0, 0, 255), 2, 8, 0);
			}
			// cv::Mat notation(50, COL + gap + COL, CV_8UC3, cv::Scalar(255, 255, 255));
			// putText(notation, "current frame: " + to_string(index) + "  sequence: " + to_string(sequence), cv::Point2f(20, 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255), 3);
			//
			// putText(notation, "previous frame: " + to_string(old_kf->index) + "  sequence: " + to_string(old_kf->sequence), cv::Point2f(20 + COL + gap, 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255), 3);
			// cv::vconcat(notation, loop_match_img, loop_match_img);

			// ostringstream path;
			// path << "/home/wt/LF-VIO/loop_rotation/loop_image/"
			// 	 << index << "-"
			// 	 << old_kf->index << "-"
			// 	 << "3pnp_match.jpg";
			// cv::imwrite(path.str().c_str(), loop_match_img);
			// ostringstream path1;
			// path1 << "/home/wt/LF-VIO/loop_rotation/loop_image/"
			// 	  << index << "-"
			// 	  << old_kf->index << "-"
			// 	  << "old.jpg";
			// cv::imwrite(path1.str().c_str(), old_img);
			// ostringstream path2;
			// path2 << "/home/wt/LF-VIO/loop_rotation/loop_image/"
			// 	  << index << "-"
			// 	  << old_kf->index << "-"
			// 	  << "loop.jpg";
			// cv::imwrite(path2.str().c_str(), image);

			// ostringstream path3;
			// path3 << "/home/wt/LF-VIO/loop_rotation/loop_image/"
			// 	  << index << "-"
			// 	  << old_kf->index << "-"
			// 	  << "rotation.jpg";
			// cv::imwrite(path3.str().c_str(), rotation_image);
			/*
			ostringstream path;
			path <<  "/home/tony-ws1/raw_data/loop_image/"
					<< index << "-"
					<< old_kf->index << "-" << "3pnp_match.jpg";
			cv::imwrite( path.str().c_str(), loop_match_img);
			*/
			// if ((int)matched_2d_cur.size() > MIN_LOOP_NUM)
			// changed by wz
			if (1)
			{

				cv::imshow("loop connection", loop_match_img);
				cv::waitKey(1);

				cv::Mat thumbimage;
				cv::resize(loop_match_img, thumbimage, cv::Size(loop_match_img.cols / 2, loop_match_img.rows / 2));
				sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", thumbimage).toImageMsg();
				msg->header.stamp = ros::Time(time_stamp);
				pub_match_img.publish(msg);
			}
		}

#endif
	}

	if ((int)matched_2d_cur.size() > MIN_LOOP_NUM)
	{
		relative_t = PnP_R_old.transpose() * (origin_vio_T - PnP_T_old);
		relative_q = PnP_R_old.transpose() * origin_vio_R;
		relative_yaw = Utility::normalizeAngle(Utility::R2ypr(origin_vio_R).x() - Utility::R2ypr(PnP_R_old).x());
		// printf("PNP relative\n");
		// cout << "pnp relative_t " << relative_t.transpose() << endl;
		// cout << "pnp relative_yaw " << relative_yaw << endl;
		// if (abs(relative_yaw) < 30.0 && relative_t.norm() < 20.0)
		// changed by wz loop closure
		if (abs(relative_yaw) < 360.0 && relative_t.norm() < 20.0)
		{

			has_loop = true;
			loop_index = old_kf->index;
			loop_info << relative_t.x(), relative_t.y(), relative_t.z(),
				relative_q.w(), relative_q.x(), relative_q.y(), relative_q.z(),
				relative_yaw;
			if (FAST_RELOCALIZATION)
			{
				sensor_msgs::PointCloud msg_match_points;
				msg_match_points.header.stamp = ros::Time(time_stamp);
				// for (int i = 0; i < (int)matched_2d_old_norm.size(); i++)
				// {
				// 	geometry_msgs::Point32 p;
				// 	p.x = matched_2d_old_norm[i].x;
				// 	p.y = matched_2d_old_norm[i].y;
				// 	p.z = matched_id[i];
				// 	msg_match_points.points.push_back(p);
				// }
				// changed by wz loop closure
				for (int i = 0; i < (int)matched_3d_old_norm.size(); i++)
				{
					geometry_msgs::Point32 p;
					p.x = matched_3d_old_norm[i].x;
					p.y = matched_3d_old_norm[i].y;
					if (matched_3d_old_norm[i].z >= 0)
						p.z = matched_id[i];
					else
						p.z = -matched_id[i];

					msg_match_points.points.push_back(p);
				}
				Eigen::Vector3d T = old_kf->T_w_i;
				Eigen::Matrix3d R = old_kf->R_w_i;
				Quaterniond Q(R);
				sensor_msgs::ChannelFloat32 t_q_index;
				t_q_index.values.push_back(T.x());
				t_q_index.values.push_back(T.y());
				t_q_index.values.push_back(T.z());
				t_q_index.values.push_back(Q.w());
				t_q_index.values.push_back(Q.x());
				t_q_index.values.push_back(Q.y());
				t_q_index.values.push_back(Q.z());
				t_q_index.values.push_back(index);
				msg_match_points.channels.push_back(t_q_index);
				pub_match_points.publish(msg_match_points);
			}
			return true;
		}
	}
	// printf("loop final use num %d %lf--------------- \n", (int)matched_2d_cur.size(), t_match.toc());
	return false;
}

int KeyFrame::HammingDis(const BRIEF::bitset &a, const BRIEF::bitset &b)
{
	BRIEF::bitset xor_of_bitset = a ^ b;
	int dis = xor_of_bitset.count();
	return dis;
}

void KeyFrame::getVioPose(Eigen::Vector3d &_T_w_i, Eigen::Matrix3d &_R_w_i)
{
	_T_w_i = vio_T_w_i;
	_R_w_i = vio_R_w_i;
}

void KeyFrame::getPose(Eigen::Vector3d &_T_w_i, Eigen::Matrix3d &_R_w_i)
{
	_T_w_i = T_w_i;
	_R_w_i = R_w_i;
}

void KeyFrame::updatePose(const Eigen::Vector3d &_T_w_i, const Eigen::Matrix3d &_R_w_i)
{
	T_w_i = _T_w_i;
	R_w_i = _R_w_i;
}

void KeyFrame::updateVioPose(const Eigen::Vector3d &_T_w_i, const Eigen::Matrix3d &_R_w_i)
{
	vio_T_w_i = _T_w_i;
	vio_R_w_i = _R_w_i;
	T_w_i = vio_T_w_i;
	R_w_i = vio_R_w_i;
}

Eigen::Vector3d KeyFrame::getLoopRelativeT()
{
	return Eigen::Vector3d(loop_info(0), loop_info(1), loop_info(2));
}

Eigen::Quaterniond KeyFrame::getLoopRelativeQ()
{
	return Eigen::Quaterniond(loop_info(3), loop_info(4), loop_info(5), loop_info(6));
}

double KeyFrame::getLoopRelativeYaw()
{
	return loop_info(7);
}

void KeyFrame::updateLoop(Eigen::Matrix<double, 8, 1> &_loop_info)
{
	if (abs(_loop_info(7)) < 30.0 && Vector3d(_loop_info(0), _loop_info(1), _loop_info(2)).norm() < 20.0)
	{
		// printf("update loop info\n");
		loop_info = _loop_info;
	}
}

BriefExtractor::BriefExtractor(const std::string &pattern_file)
{
	// The DVision::BRIEF extractor computes a random pattern by default when
	// the object is created.
	// We load the pattern that we used to build the vocabulary, to make
	// the descriptors compatible with the predefined vocabulary

	// loads the pattern
	cv::FileStorage fs(pattern_file.c_str(), cv::FileStorage::READ);
	if (!fs.isOpened())
		throw string("Could not open file ") + pattern_file;

	vector<int> x1, y1, x2, y2;
	fs["x1"] >> x1;
	fs["x2"] >> x2;
	fs["y1"] >> y1;
	fs["y2"] >> y2;

	m_brief.importPairs(x1, y1, x2, y2);
}
