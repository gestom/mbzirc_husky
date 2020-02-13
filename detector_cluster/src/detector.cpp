
#include "detector.h"

using namespace std;

// variables
ros::Publisher vis_pub;
ros::Publisher center_pub;
ros::Publisher origin_pcl_pub;

double normal_pdf(double x, double m, double s) {
    double a = (x - m) / s;
    return INV_SQRT_2PI / s * std::exp(-0.5f * a * a);
}

double diff_angle(double a1, double a2) {
    auto c1 = std::cos(a1);
    auto s1 = std::sin(a1);

    auto c2 = std::cos(a2);
    auto s2 = std::sin(a2);

    auto dot = c1 * c2 + s1 * s2;
    auto cross = c1 * s2 - s1 * c2;

    return std::atan2(cross, dot);
}

template<typename Iter, typename RandomGenerator>
Iter select_randomly(Iter start, Iter end, RandomGenerator &g) {
    std::uniform_int_distribution<> dis(0, std::distance(start, end) - 1);
    std::advance(start, dis(g));
    return start;
}

template<typename Iter>
Iter select_randomly(Iter start, Iter end) {
    static std::random_device rd;
    static std::mt19937 gen(rd());
    return select_randomly(start, end, gen);
}

array<MyPoint, 2> BrickDetector::fit_detection(array<vector<MyPoint>, 4> proposal, array<vector<MyPoint>, 4> wall,
                                               array<cv::Point2d, 2> tar_pt) {
    double min_distance = 10000;
    cv::Mat best_t;
    cv::Mat best_R;
    array<MyPoint, 2> ret_arr;

    // how many piles was detected?
    int proposal_size = 0;
    vector<int> avail_clrs;
    for (int clr_idx = 0; clr_idx < 4; clr_idx++) {
        if (not proposal[clr_idx].empty()) {
            avail_clrs.push_back(clr_idx);
            proposal_size += proposal[clr_idx].size();
        }
    }

    if ((avail_clrs.size() > 1 and proposal_size > 3) or proposal[0].size() > 4 or proposal[1].size() > 3) {
        for (int it = 0; it < 500; it++) {
            // run 1000 iterations of RANSAC
            vector<MyPoint> target_pts;
            vector<MyPoint> src_pts;

            // sample points
            for (int i = 0; i < 2; i++) {
                int clr = *select_randomly(avail_clrs.begin(), avail_clrs.end());
                MyPoint p1 = *select_randomly(proposal[clr].begin(), proposal[clr].end());
                target_pts.push_back(p1);
                int killme = 0;
                while (true) {
                    MyPoint p2 = *select_randomly(wall[clr].begin(), wall[clr].end());
                    if (abs(p1.z - p2.z) < 0.01 or killme > 50) {
                        src_pts.push_back(p2);
                        break;
                    }
                    killme++;
                }
            }

            // find rotation
            MyPoint target_vec = target_pts[0] - target_pts[1];
            MyPoint src_vec = src_pts[0] - src_pts[1];
            double phi = diff_angle(atan2(src_vec.y, src_vec.x), atan2(target_vec.y, target_vec.x));
            cv::Mat R;
            R = (cv::Mat_<double>(2, 2) << cos(phi), sin(phi), -sin(phi), cos(phi));

            // find translation
            cv::Mat t = cv::Mat(cv::Point2d(src_pts[0].x, src_pts[0].y)) -
                        R * cv::Mat(cv::Point2d(target_pts[0].x, target_pts[0].y));

            cv::Mat tar_pt2 = R * cv::Mat(cv::Point2d(target_pts[1].x, target_pts[1].y)) + t;
            cv::Mat src_pt2 = cv::Mat(cv::Point2d(src_pts[1].x, src_pts[1].y));
            double assert_size2 = norm(tar_pt2 - src_pt2);

            if (assert_size2 < 0.1) {
                // compute inliers
                double curr_distance = 0;
                for (int &clr : avail_clrs) {

                    for (auto &tar : proposal[clr]) {
                        double curr_min_dist = 100000;
                        cv::Mat tar_pt = R * cv::Mat(cv::Point2d(tar.x, tar.y)) + t;
                        for (auto &src : wall[clr]) {
                            MyPoint tar_mypt(tar_pt.at<double>(0), tar_pt.at<double>(1), tar.z);
                            MyPoint diff = src - tar_mypt;
                            diff.z *= 5;
                            double dist = cv::norm(diff);

                            if (dist < curr_min_dist) {
                                curr_min_dist = dist;
                            }
                        }
                        curr_distance += curr_min_dist;
                    }
                }
                if (curr_distance < min_distance) {
                    best_t = t;
                    best_R = R;
                    min_distance = curr_distance;
                }
            }
        }

        cv::Mat ret1 = best_R.t() * (cv::Mat(tar_pt[0]) - best_t);
        cv::Mat ret2 = best_R.t() * (cv::Mat(tar_pt[1]) - best_t);

        ret_arr[0] = MyPoint(ret1.at<double>(0), ret1.at<double>(1), min_distance/proposal_size);
        ret_arr[1] = MyPoint(ret2.at<double>(0), ret2.at<double>(1), min_distance/proposal_size);

        return ret_arr;
    }
    return ret_arr;
}

array<vector<MyPoint>, 4>
BrickDetector::get_centers_in_piles(array<vector<BrickLine>, 4> &lines, array<MyPoint, 4> &piles) {

    array<vector<MyPoint>, 4> ret = {};
    for (int clr_idx = 0; clr_idx < 4; clr_idx++) {
        vector<MyPoint> centers;
        if (piles[clr_idx].x != 0 and piles[clr_idx].y != 0) {      // check for valid pile
            for (auto &line : lines[clr_idx]) {
                MyPoint center = GET_CENTER(line[0], line[1]);
                if (norm(center - piles[clr_idx]) < 1.5) {          // 1.5 meters distance from pile center is ok
                    double new_z = int((center.z + LIDAR_HEIGHT) / BRICK_HEIGHT) * BRICK_HEIGHT +
                                   (BRICK_HEIGHT / 2);              // allign height to 0.1, 0.3 and so on
                    if (new_z < 2 * BRICK_HEIGHT) {
                        MyPoint candidate(center.x, center.y, new_z);
                        bool unique = true;
                        for (auto &inserted : centers) {
                            if (norm(cv::Point2d(inserted.x, inserted.y) - cv::Point2d(candidate.x, candidate.y)) <
                                0.15) {
                                if (abs(candidate.z - inserted.z) < 0.01) {
                                    unique = false;
                                }
                            }
                        }
                        if (unique) {
                            centers.push_back(candidate);
                        }
                    }
                }
            }
        }
        ret[clr_idx] = centers;
    }
    return ret;
}

vector<MyPoint> BrickDetector::get_wall_centers(vector<BrickLine> &lines){
    vector<MyPoint> ret;
    for (auto &line : lines){
        MyPoint line_center = GET_CENTER(line[0], line[1]);
        bool is_in = false;
        for (MyPoint &wall_center : ret){
            MyPoint diff = wall_center - line_center;
            double dist = cv::norm(cv::Point2d(diff.x, diff.y));
            if (dist < 1) {
                is_in = true;
            }
        }
        if (not is_in){
            ret.push_back(line_center);
        }
    }
    return ret;
}

geometry_msgs::Point BrickDetector::transform_point(MyPoint pt, geometry_msgs::TransformStamped tf){
    geometry_msgs::Point tf_pt;
    geometry_msgs::Point ret_pt;
    tf_pt.x = pt.x;
    tf_pt.y = pt.y;
    tf_pt.z = pt.z;
    tf2::doTransform(tf_pt, ret_pt, tf);
    return ret_pt;
}

array<MyPoint, 4> BrickDetector::get_piles(array<vector<BrickLine>, 4> &lines) {

    array<MyPoint, 4> ret{};

    double var = 1.4;
    for (int clr_idx = 0; clr_idx < 4; clr_idx++) {
        if ((clr_idx == 0 and lines[clr_idx].size() > 5) or         // need 6 <= of red detection to detect red pile
            (clr_idx == 1 and lines[clr_idx].size() > 3) or
            (clr_idx > 1 and lines[clr_idx].size() > 1)) {

            vector<array<double, 2>> probs;
            for (int i = 0; i < lines[clr_idx].size(); i++) {
                array<double, 2> prob = {1.0, 1.0};
                probs.push_back(prob);
            }

            double probs_sum_x = lines[clr_idx].size();
            double probs_sum_y = lines[clr_idx].size();
            double mean_x = 0;
            double mean_y = 0;
            double sum_x = 0;
            double sum_y = 0;

            for (int iter = 0; iter < 25; iter++) {

                sum_x = 0;
                sum_y = 0;
                for (int i = 0; i < lines[clr_idx].size(); i++) {
                    double center_x = GET_CENTER(lines[clr_idx][i][0].x, lines[clr_idx][i][1].x);
                    double center_y = GET_CENTER(lines[clr_idx][i][0].y, lines[clr_idx][i][1].y);
                    sum_x += center_x * probs[i][0];
                    sum_y += (center_y) * probs[i][1];
                }

                mean_x = sum_x / probs_sum_x;
                mean_y = sum_y / probs_sum_y;

                probs_sum_x = 0;
                probs_sum_y = 0;
                for (int i = 0; i < lines[clr_idx].size(); i++) {
                    double center_x = GET_CENTER(lines[clr_idx][i][0].x, lines[clr_idx][i][1].x);
                    double center_y = GET_CENTER(lines[clr_idx][i][0].y, lines[clr_idx][i][1].y);
                    double prob_x = normal_pdf(center_x, mean_x, var);
                    probs_sum_x += prob_x;
                    double prob_y = normal_pdf(center_y, mean_y, var);
                    probs_sum_y += prob_y;
                    probs[i] = {prob_x, prob_y};
                }

            }

            ret[clr_idx] = MyPoint(mean_x, mean_y, 0);

        }
    }

    return ret;

}

vector<BrickLine> BrickDetector::match_detections(vector<BrickLine> lines,
                                                  double obj_height, double obj_top, double req_height,
                                                  int min_detections, int max_detections) {

    vector<BrickLine> ret;

    for (int i = 0; i < lines.size(); i++) {
        MyPoint curr_center = GET_CENTER(lines[i][0], lines[i][1]);
        int hits = 0;
        double max_z = curr_center.z + LIDAR_HEIGHT;
        double curr_yaw = atan2(curr_center.y, curr_center.x);
        double curr_dist = norm(curr_center);
        MyPoint curr_line_vec = lines[i][1] - lines[i][0];
        double curr_line_yaw = atan2(curr_line_vec.y, curr_line_vec.x);
        double expected_z_dist = curr_dist * LEN_MULTIPLIER;
        int expected_hits = int(max(double(min_detections),
                                    min((obj_height / expected_z_dist), double(max_detections))));
        double expected_yaw = abs(
                acos(-(0.3 * 0.3) / (2 * curr_dist * curr_dist) + 1));     // 30 cm is ok yaw diff
        if (expected_hits > min_detections) {
            for (int j = 0; j < lines.size(); j++) {
                if (i != j) {
                    MyPoint new_center = GET_CENTER(lines[j][0], lines[j][1]);
                    double new_yaw = atan2(new_center.y, new_center.x);
                    double tmp = abs(diff_angle(curr_yaw, new_yaw));
                    double yaw_diff = min(tmp, abs(tmp - M_PI));
                    if (yaw_diff < expected_yaw) {      // filter using yaw
                        double new_dist = norm(new_center);
                        if (abs(new_dist - curr_dist) < 0.5) {      // filter using distance
                            MyPoint new_line_vec = lines[j][1] - lines[j][0];
                            double new_line_yaw = atan2(new_line_vec.y, new_line_vec.x);
                            if (abs(diff_angle(curr_line_yaw, new_line_yaw)) < M_PI/6){
                                hits++;
                                max_z = max(max_z, new_center.z + double(LIDAR_HEIGHT));
                                if (new_center.z + LIDAR_HEIGHT >
                                    obj_top) {     // 45cm is too high - filter using height
                                    hits = -1;
                                    break;
                                }
                            }
                        }
                    }
                }
            }
        }
        if (hits >= int(expected_hits) and max_z > req_height) {
            ret.push_back(lines[i]);
        }
    }

    return ret;
}

void BrickDetector::filter_on_line(MyPoint p1, MyPoint p2, array<vector<BrickLine>, 4> &ret) {

    // compute the line
    array<double, 3> line = {-(p1.y - p2.y), p1.x - p2.x, 0};    // line in general form
    line[2] = -(p1.x * line[0] + p1.y * line[1]);

    double line_size = sqrt(line[0] * line[0] + line[1] * line[1]);

    for (int color_idx = 0; color_idx < 4; color_idx++) {
        for (int i = ret[color_idx].size() - 1; i >= 0; i--) {
            MyPoint pt1 = ret[color_idx][i][0];
            MyPoint pt2 = ret[color_idx][i][1];
            double dist = abs(pt1.x * line[0] + pt1.y * line[1] + line[2]) / line_size;
            double angle1 = atan2(pt1.y - pt2.y, pt1.x - pt2.x);
            double angle2 = atan2(p1.y - p2.y, p1.x - p2.x);
            double diff = abs(diff_angle(angle1, angle2) * 180 / M_PI);
            double max_diff = max(diff, abs(diff - 180));
            if (dist > 1.0 and max_diff >
                               30.0) {   // delete everything further than one meter from line and with bigger than 30 degree deviation
                ret[color_idx].erase(ret[color_idx].begin() + i);
            }
        }
    }
}

void BrickDetector::filter_size(vector<BrickLine> *in_lines, double size, double tolerance,
                                vector<BrickLine> &ret) {
    for (int row = 0; row < LIDAR_ROWS; row++) {
        for (auto &line : in_lines[row]) {
            float dx = line[0].x - line[1].x;
            float dy = line[0].y - line[1].y;
            float dist = sqrt(dx * dx + dy * dy);
            float diff = abs(dist - size);
            if (diff < tolerance) {
                ret.push_back(line);
            }
        }
    }
}

void BrickDetector::split_and_merge(vector<MyPoint> *filtered,
                                    vector<array<int, 2>> *clusters,
                                    vector<BrickLine> *ret,
                                    double splitting_distance) {

    for (int row = 0; row < LIDAR_ROWS; row++) {
        while (not clusters[row].empty()) {

            array<int, 2> cluster_idxs = clusters[row].back();
            clusters[row].pop_back();

            MyPoint p1 = filtered[row][cluster_idxs[0]];
            MyPoint p2 = filtered[row][cluster_idxs[1]];

            // compute the line
            array<double, 3> line = {-(p1.y - p2.y), p1.x - p2.x, 0};    // line in general form
            line[2] = -(p1.x * line[0] + p1.y * line[1]);

            double line_size = sqrt(line[0] * line[0] + line[1] * line[1]);

            // find most distant point
            double max_dist = 0;
            int split_index;
            double dist;
            for (int i = cluster_idxs[0]; i <= cluster_idxs[1]; i += POINT_ITERATION_STEP) {
                dist = abs(filtered[row][i].x * line[0] +
                           filtered[row][i].y * line[1] + line[2]) / line_size;

                if (dist > max_dist) {
                    max_dist = dist;
                    split_index = i;
                }
            }

            // split using most distant point and append to clusters or results
            if (max_dist > splitting_distance) {
                if (split_index - cluster_idxs[0] > MINIMAL_LINE_LENGTH) {
                    array<int, 2> new_cluster = {cluster_idxs[0], split_index};
                    clusters[row].push_back(new_cluster);
                }
                if (cluster_idxs[1] - split_index > MINIMAL_LINE_LENGTH) {
                    array<int, 2> new_cluster = {split_index, cluster_idxs[1]};
                    clusters[row].push_back(new_cluster);
                }
            } else {
                BrickLine tmp = {p1, p2};
                ret[row].push_back(tmp);
            }
        }

        /*
        // connect all colinear lines - this doesnt really improve the result
        for (int row = 0; row < LIDAR_ROWS; row++) {
            for (int i = ret[row].size() - 1; i > 0; i--) {

                // compute distance
                vector<float> vec = {ret[row][i][0].x - ret[row][i - 1][1].x,
                                     ret[row][i][0].y - ret[row][i - 1][1].y,
                                     ret[row][i][0].z - ret[row][i - 1][1].z};
                float dist = sqrt(vec[0] * vec[0] + vec[1] * vec[1] + vec[2] * vec[2]);
                if (dist > MAXIMAL_JOIN_DISTANCE) {
                    continue;
                }

                // compute angle
                vector<float> line1 = {ret[row][i][0].x - ret[row][i][1].x,
                                       ret[row][i][0].y - ret[row][i][1].y,
                                       ret[row][i][0].z - ret[row][i][1].z};
                vector<float> line2 = {ret[row][i - 1][0].x - ret[row][i - 1][1].x,
                                       ret[row][i - 1][0].y - ret[row][i - 1][1].y,
                                       ret[row][i - 1][0].z - ret[row][i - 1][1].z};
                float size1 = sqrt(line1[0] * line1[0] + line1[1] * line1[1] + line1[2] * line1[2]);
                float size2 = sqrt(line2[0] * line2[0] + line2[1] * line2[1] + line2[2] * line2[2]);
                float nom = line1[0] * line2[0] + line1[1] * line2[1] + line1[2] * line2[2];
                float denum = (size1 * size2);
                float angle_deg = acos(nom / denum) * 180 / M_PI;

                if (angle_deg > MAXIMAL_JOIN_ANGLE) {
                    continue;
                }

                // compute distance from line
                array<float, 3> line = {-line1[1], line1[0], 0};
                line[2] = -(ret[row][i][0].x * line[0] +
                            ret[row][i][1].y * line[1]);
                float line_size = sqrt(line[0] * line[0] + line[1] * line[1]);
                dist = abs(ret[row][i][1].x * line[0] +
                           ret[row][i][1].y * line[1] + line[2]) / line_size;

                if (dist > MAXIMAL_JOIN_SHIFT) {
                    continue;
                }

                BrickLine tmp = {ret[row][i - 1][0], ret[row][i][1]};
                ret[row].erase(ret[row].begin() + i);
                ret[row].erase(ret[row].begin() + i - 1);
                ret[row].insert(ret[row].begin() + i - 1, tmp);

            }
        }
        */
    }
}


void BrickDetector::create_clusters(vector<MyPoint> *filtered,
                                    vector<array<int, 2>> *ret,
                                    double clustering_distance,
                                    double max_distance) {
    float dist;
    float d_x;
    float d_y;
    for (int row = 0; row < LIDAR_ROWS; row++) {
        int start = 0;
        int end;
        for (end = 1; end < filtered[row].size(); end++) {
            d_x = filtered[row][end - 1].x - filtered[row][end].x;
            d_y = filtered[row][end - 1].y - filtered[row][end].y;
            dist = sqrt(d_x * d_x + d_y * d_y);
            if (dist > clustering_distance) {
                if ((end - 1) - start > MIN_CLUSTER_SIZE and cv::norm(filtered[row][end - 1]) < max_distance and
                    cv::norm(filtered[row][start]) < max_distance) {
                    array<int, 2> tmp = {start, end - 1};
                    ret[row].push_back(tmp);
                }
                start = end;
            }
        }

        end--;
        if (dist < clustering_distance and end - start > MIN_CLUSTER_SIZE) {
            array<int, 2> tmp = {start, end};
            ret[row].push_back(tmp);
        }

    }
}

void
BrickDetector::filter_ground(vector<double> ground, vector<MyPoint> *point_rows, vector<MyPoint> *ret) {
    // remove ground points and points too high

    for (int row = 0; row < LIDAR_ROWS; row++) {
        for (int i = 0; point_rows[row].size() > i; i++) {
            /*
            double dist = (ground[0] * point_rows[row][i].x) +
                          (ground[1] * point_rows[row][i].y) +
                          (ground[2] * point_rows[row][i].z) + ground[3];
           */
            double dist = point_rows[row][i].z + ground[3];
            if (dist > GROUND_SAFE_DISTANCE and dist < MAX_HEIGHT) {   // 2 meters high is too much
                ret[row].push_back(point_rows[row][i]);
            }
        }
    }
}

double BrickDetector::fetch_pointcloud(sensor_msgs::PointCloud2 &ptcl, vector<MyPoint> *ret) {
    double min_z = numeric_limits<double>::infinity(); // some really high number

    sensor_msgs::PointCloud2ConstIterator<uint16_t> iter_ring((ptcl), "ring");
    sensor_msgs::PointCloud2ConstIterator<float> iter_x((ptcl), "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y((ptcl), "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z((ptcl), "z");

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_ring, ++iter_y, ++iter_z) {
        /*
        if (*iter_z < min_z and *iter_ring < 2) {
            if (sqrt((*iter_x)*(*iter_x) + (*iter_y)*(*iter_y)) < MAX_DIST) { // only close points
                min_z = *iter_z; // point with lowest height
            }
        }
        */
        if (sqrt((*iter_x) * (*iter_x) + (*iter_y) * (*iter_y)) < 100.0) {
            ret[*iter_ring].emplace_back(*iter_x, *iter_y, *iter_z);
        }


    }
    // when parsing velodyne_packets the points can be in incorrect order - sorting by yaw
    for (size_t row = 0; row < LIDAR_ROWS; row++) {
        std::sort(ret[row].begin(), ret[row].end(), [](const MyPoint &pt1, const MyPoint &pt2) {
            double angle1 = atan2(pt1.y, pt1.x);
            double angle2 = atan2(pt2.y, pt2.x);
            return diff_angle(angle1, angle2) < 0;
        });
    }

    return min_z;
}

void BrickDetector::fetch_parameters() {
    // get parameters
    n->param("/detector/lidar_rows", LIDAR_ROWS, 16);
    n->param("/detector/height_tolerance", HEIGHT_TOLERANCE, float(0.25));
    n->param("/detector/ground_safe_distance", GROUND_SAFE_DISTANCE, float(0.05));
    n->param("/detector/lidar_height", LIDAR_HEIGHT, float(0.4));
    n->param("/detector/clustering_dist", CLUSTERING_DIST, float(0.06));
    n->param("/detector/wall_clustering_dist", WALL_CLUSTERING_DIST, float(0.18));
    n->param("/detector/min_cluster_size", MIN_CLUSTER_SIZE, 10);
    n->param("/detector/point_iteration_step", POINT_ITERATION_STEP, 1);
    n->param("/detector/splitting_distance", SPLITTING_DISTANCE, float(0.1));
    n->param("/detector/wall_splitting_distance", WALL_SPLITTING_DISTANCE, float(0.18));
    n->param("/detector/minimal_line_length", MINIMAL_LINE_LENGTH, 5);
    n->param("/detector/maximal_join_angle", MAXIMAL_JOIN_ANGLE, float(3));
    n->param("/detector/maximal_join_distamce", MAXIMAL_JOIN_DISTANCE, float(0.1));
    n->param("/detector/maximal_join_shift", MAXIMAL_JOIN_SHIFT, float(0.02));
    n->param("/detector/brick_size", BRICK_SIZE, float(0.3));
    n->param("/detector/size_tolerance", SIZE_TOLERANCE, float(0.07));
    n->param("/detector/max_dist", MAX_DIST, float(20));
    n->param("/detector/max_brick_dist", MAX_BRICK_DIST, float(5));
    n->param("/detector/max_height", MAX_HEIGHT, float(2));
    n->param("/detector/ransac_tolerance", RANSAC_TOLERANCE, float(0.2));

}

void BrickDetector::build_walls() {
    /// WALL FROM FRONT
    // visible red bricks
    wall_setup[0].push_back(MyPoint(0.15, -0.3, 0.1));
    wall_setup[0].push_back(MyPoint(0.15, -0.3, 0.3));
    wall_setup[0].push_back(MyPoint(0.55, 0.0, 0.1));
    wall_setup[0].push_back(MyPoint(0.55, 0.0, 0.3));
    wall_setup[0].push_back(MyPoint(0.95, 0.0, 0.1));
    wall_setup[0].push_back(MyPoint(0.95, 0.0, 0.3));
    wall_setup[0].push_back(MyPoint(1.35, -0.3, 0.1));
    wall_setup[0].push_back(MyPoint(1.35, -0.3, 0.3));

    // visible green bricks
    wall_setup[1].push_back(MyPoint(2.3, 0.0, 0.1));
    wall_setup[1].push_back(MyPoint(2.3, -0.3, 0.3));
    wall_setup[1].push_back(MyPoint(3.0, 0.0, 0.1));
    wall_setup[1].push_back(MyPoint(3.0, -0.3, 0.3));

    // visible blue bricks
    wall_setup[2].push_back(MyPoint(4.4, 0.0, 0.1));
    wall_setup[2].push_back(MyPoint(4.4, -0.3, 0.3));

    // visible orange bricks
    wall_setup[3].push_back(MyPoint(6.4, 0.0, 0.1));
    wall_setup[3].push_back(MyPoint(6.4, 0.0, 0.3));
    wall_setup[3].push_back(MyPoint(6.4, -0.3, 0.5));

    /// WALL FROM BEHIND
    // visible orange bricks
    wall_setup_back[3].push_back(MyPoint(0.9, 0.0, 0.1));
    wall_setup_back[3].push_back(MyPoint(0.9, 0.0, 0.3));
    wall_setup_back[3].push_back(MyPoint(0.9, -0.3, 0.5));

    // visible blue bricks
    wall_setup_back[2].push_back(MyPoint(2.9, 0.0, 0.1));
    wall_setup_back[2].push_back(MyPoint(2.9, 0.0, 0.3));

    // visible green bricks
    wall_setup_back[1].push_back(MyPoint(4.3, 0.0, 0.1));
    wall_setup_back[1].push_back(MyPoint(4.3, 0.0, 0.3));
    wall_setup_back[1].push_back(MyPoint(5.0, 0.0, 0.1));
    wall_setup_back[1].push_back(MyPoint(5.0, 0.0, 0.3));

    // visible red bricks
    wall_setup_back[0].push_back(MyPoint(7.15, 0.0, 0.1));
    wall_setup_back[0].push_back(MyPoint(7.15, 0.0, 0.3));
    wall_setup_back[0].push_back(MyPoint(6.75, 0.0, 0.1));
    wall_setup_back[0].push_back(MyPoint(6.75, 0.0, 0.3));
    wall_setup_back[0].push_back(MyPoint(5.95, 0.0, 0.1));
    wall_setup_back[0].push_back(MyPoint(5.95, 0.0, 0.3));
    wall_setup_back[0].push_back(MyPoint(6.35, 0.0, 0.1));
    wall_setup_back[0].push_back(MyPoint(6.35, 0.0, 0.3));

}

array<vector<BrickLine>, 5> BrickDetector::find_brick_segments(vector<MyPoint> *filtered_ptcl) {

    vector<array<int, 2>> clusters[LIDAR_ROWS];
    create_clusters(filtered_ptcl, clusters, CLUSTERING_DIST, MAX_BRICK_DIST);    // group points into the clusters

    vector<BrickLine> lines[LIDAR_ROWS];
    split_and_merge(filtered_ptcl, clusters, lines,
                    SPLITTING_DISTANCE);       // create lines using split-and-merge algorithm

    vector<BrickLine> side_lines;         // get side of bricks
    filter_size(lines, 0.2, SIZE_TOLERANCE, side_lines);

    vector<BrickLine> red_lines;          // get red lines
    filter_size(lines, 0.3, SIZE_TOLERANCE, red_lines);

    vector<BrickLine> green_lines;        // get green lines
    filter_size(lines, 0.6, SIZE_TOLERANCE, green_lines);

    vector<BrickLine> blue_lines;         // get blue lines
    filter_size(lines, 1.2, SIZE_TOLERANCE, blue_lines);

    vector<BrickLine> orange_lines;       // get orange lines
    filter_size(lines, 1.8, SIZE_TOLERANCE, orange_lines);

    array<vector<BrickLine>, 5> ret = {red_lines, green_lines, blue_lines, orange_lines, side_lines};
    return ret;
}

vector<BrickLine> BrickDetector::find_wall_segments(vector<MyPoint> *filtered_ptcl) {

    vector<array<int, 2>> clusters[LIDAR_ROWS];
    create_clusters(filtered_ptcl, clusters, WALL_CLUSTERING_DIST, MAX_DIST);       // group points into the clusters

    vector<BrickLine> lines[LIDAR_ROWS];
    split_and_merge(filtered_ptcl, clusters, lines,
                    WALL_SPLITTING_DISTANCE);       // create lines using split-and-merge algorithm

    vector<BrickLine> wall_lines;                                              // get red lines
    filter_size(lines, 2.75, 1.5, wall_lines);

    return wall_lines;
}

void BrickDetector::subscribe_ptcl(sensor_msgs::PointCloud2 ptcl) // callback
{

    /// DETECTION  ------------------------------------------------------------------------------------------------

    // fill the row buffers and find point with the lowest height
    vector<MyPoint> point_rows[LIDAR_ROWS];
    double min_z = fetch_pointcloud(ptcl, point_rows);

    // filter ground points
    vector<MyPoint> filtered_dense[LIDAR_ROWS];
    vector<MyPoint> filtered_sparse[LIDAR_ROWS];
    std::vector<double> ground1 = {0, 0, 1, LIDAR_HEIGHT};
    std::vector<double> ground2 = {0, 0, 1, 0};
    filter_ground(ground1, point_rows, filtered_dense);
    filter_ground(ground2, point_rows, filtered_sparse);

    // find segments using split and merge algorithm
    array<vector<BrickLine>, 5> lines = find_brick_segments(filtered_dense);
    vector<BrickLine> wall_lines = find_wall_segments(filtered_sparse);

    // perform brick segment matching using hits with different heights
    array<vector<BrickLine>, 4> matched_bricks;
    array<vector<BrickLine>, 4> matched_single_bricks;
    vector<BrickLine> matched_walls;
    // vector<BrickLine> matched_wall_sides;
    // matched_bricks = {lines[0], lines[1], lines[2], lines[3]};

    for (int i = 0; i < 3; i++) {
        matched_bricks[i] = match_detections(lines[i], 0.4, 0.45, 0.225, 1, 2);
    }
    matched_bricks[3] = match_detections(lines[3], 0.6, 0.65, 0.425, 2, 3);

    matched_walls = match_detections(wall_lines, 1.1, 2.0, 0.8, 2, 3);

    /*
    matched_wall_sides = match_detections(lines[4], 1.1, 1.75, 0.8, 2, 3);
    for (int i = 0; i < 4; i++) {
        matched_single_bricks[i] = match_detections(lines[i], 0.2, 0.25, 0.025, 0, 2);
    }
    */

    // obtain piles
    array<MyPoint, 4> pile_centers = get_piles(matched_bricks);
    array<vector<MyPoint>, 4> brick_centers = get_centers_in_piles(matched_bricks, pile_centers);

    /*
    cout << "red detected: " << brick_centers[0].size() << endl << "green detected: " << brick_centers[1].size() << endl
         << "blue detected: " << brick_centers[2].size() << endl << "orange detected: " << brick_centers[3].size()
         << endl
         << endl;
    */

    // fit wall shape on my detection
    array<cv::Point2d, 2> tar_pt = {cv::Point2d(0.0, 0.0), cv::Point2d(5.0, 0.0)};
    array<MyPoint, 2> way_point = fit_detection(brick_centers, wall_setup, tar_pt);
    array<cv::Point2d, 2> tar_pt_back = {cv::Point2d(7.3, -0.8), cv::Point2d(2.3, -0.8)};
    array<MyPoint, 2> way_point_back = fit_detection(brick_centers, wall_setup_back, tar_pt_back);

    vector<MyPoint> wall_centers = get_wall_centers(matched_walls);

    // get transformations
    geometry_msgs::TransformStamped transformStamped;
    try {
        transformStamped = tf_buffer->lookupTransform(TARGET_FRAME, LIDAR_FRAME, ptcl.header.stamp, ros::Duration(0.2));
        for (int i = 0; i < 4; i++){
            if (pile_centers[i].x != 0 and pile_centers[i].y != 0){
                geometry_msgs::Point ret_pt = transform_point(pile_centers[i], transformStamped);
                mbzirc_husky::setPoi poi;
                poi.request.type = 4 + i;
                poi.request.x = pile_centers[i].x;
                poi.request.y = pile_centers[i].y;
                poi.request.covariance = 0;
                if (ros::service::call("set_map_poi", poi)){
                    cout << "Pile sent to symbolic map" << endl;
                } else {
                    cout << "Error calling service" << endl;
                }
            }
        }

        for (int i = 0; i < wall_centers.size(); i++){
            geometry_msgs::Point ret_pt = transform_point(pile_centers[i], transformStamped);
            mbzirc_husky::setPoi poi;
            poi.request.type = 2;
            poi.request.x = wall_centers[i].x;
            poi.request.y = wall_centers[i].y;
            poi.request.covariance = 0;
            if (ros::service::call("set_map_poi", poi)){
                cout << "Wall sent to symbolic map" << endl;
            } else {
                cout << "Error calling service" << endl;
            }
        }

    } catch (tf2::TransformException &ex) {
        ROS_WARN("%s",ex.what());
        ros::Duration(1.0).sleep();
    }

    /// visualise using marker publishing in rviz --------------------------------------------------------------

    visualization_msgs::Marker centers;
    centers.header.frame_id = LIDAR_FRAME;
    centers.header.stamp = ros::Time::now();
    centers.ns = "pile_centers";
    centers.action = visualization_msgs::Marker::ADD;
    centers.pose.orientation.w = 1.0;
    centers.id = 0;
    centers.type = visualization_msgs::Marker::SPHERE_LIST;
    centers.scale.x = 0.25;
    centers.scale.y = 0.25;
    centers.scale.z = 0.25;
    centers.color.a = 1.0;
    centers.color.g = 1.0;

    for (int i = 0; i < 4; i++) {
        if (pile_centers[i].x != 0 and pile_centers[i].y != 0) {
            geometry_msgs::Point pt;
            pt.x = pile_centers[i].x;
            pt.y = pile_centers[i].y;
            pt.z = 0.0;
            centers.points.push_back(pt);
        }
    }

    center_pub.publish(centers);

    if (way_point[0].x != 0 and way_point[0].y != 0 and way_point[0].z < RANSAC_TOLERANCE) {
        if (way_point[0].z > way_point_back[0].z) {
            way_point = way_point_back;
        }

        sensor_msgs::PointCloud origin_line1;
        origin_line1.header.frame_id = LIDAR_FRAME;
        origin_line1.header.stamp = ros::Time::now();
        // fill with points
        geometry_msgs::Point32 pub_pt1;
        pub_pt1.x = way_point[0].x;
        pub_pt1.y = way_point[0].y;
        pub_pt1.z = way_point[0].z;
        origin_line1.points.push_back(pub_pt1);
        geometry_msgs::Point32 pub_pt2;
        pub_pt2.x = way_point[1].x;
        pub_pt2.y = way_point[1].y;
        pub_pt2.z = 0;
        origin_line1.points.push_back(pub_pt2);
        origin_pcl_pub.publish(origin_line1);
    }

    /*
    visualization_msgs::MarkerArray lists;
    visualization_msgs::Marker line_list1;
    line_list1.header.frame_id = LIDAR_FRAME;
    line_list1.header.stamp = ros::Time::now();
    line_list1.ns = "points_and_lines";
    line_list1.action = visualization_msgs::Marker::ADD;
    line_list1.pose.orientation.w = 1.0;
    line_list1.id = 0;
    line_list1.type = visualization_msgs::Marker::LINE_LIST;
    line_list1.scale.x = 0.025;
    line_list1.color.a = 1.0;
    line_list1.color.r = 1.0;
    for (auto &red_line : matched_bricks[0]) {
        geometry_msgs::Point p1;
        geometry_msgs::Point p2;

        p1.x = red_line[0].x;
        p1.y = red_line[0].y;
        p1.z = red_line[0].z;
        line_list1.points.push_back(p1);

        p2.x = red_line[1].x;
        p2.y = red_line[1].y;
        p2.z = red_line[1].z;
        line_list1.points.push_back(p2);
    }

    // green
    visualization_msgs::Marker line_list2;
    line_list2.header.frame_id = LIDAR_FRAME;
    line_list2.header.stamp = ros::Time::now();
    line_list2.ns = "points_and_lines";
    line_list2.action = visualization_msgs::Marker::ADD;
    line_list2.pose.orientation.w = 1.0;
    line_list2.id = 1;
    line_list2.type = visualization_msgs::Marker::LINE_LIST;
    line_list2.scale.x = 0.025;
    line_list2.color.a = 1.0;
    line_list2.color.g = 1.0;
    for (auto &green_line : matched_bricks[1]) {
        geometry_msgs::Point p1;
        geometry_msgs::Point p2;

        p1.x = green_line[0].x;
        p1.y = green_line[0].y;
        p1.z = green_line[0].z;
        line_list2.points.push_back(p1);

        p2.x = green_line[1].x;
        p2.y = green_line[1].y;
        p2.z = green_line[1].z;
        line_list2.points.push_back(p2);
    }

    // blue
    visualization_msgs::Marker line_list3;
    line_list3.header.frame_id = LIDAR_FRAME;
    line_list3.header.stamp = ros::Time::now();
    line_list3.ns = "points_and_lines";
    line_list3.action = visualization_msgs::Marker::ADD;
    line_list3.pose.orientation.w = 1.0;
    line_list3.id = 2;
    line_list3.type = visualization_msgs::Marker::LINE_LIST;
    line_list3.scale.x = 0.025;
    line_list3.color.a = 1.0;
    line_list3.color.b = 1.0;
    for (auto &line : matched_bricks[2]) {
        geometry_msgs::Point p1;
        geometry_msgs::Point p2;

        p1.x = line[0].x;
        p1.y = line[0].y;
        p1.z = line[0].z;
        line_list3.points.push_back(p1);

        p2.x = line[1].x;
        p2.y = line[1].y;
        p2.z = line[1].z;
        line_list3.points.push_back(p2);
    }

    // orange
    visualization_msgs::Marker line_list4;
    line_list4.header.frame_id = LIDAR_FRAME;
    line_list4.header.stamp = ros::Time::now();
    line_list4.ns = "points_and_lines";
    line_list4.action = visualization_msgs::Marker::ADD;
    line_list4.pose.orientation.w = 1.0;
    line_list4.id = 3;
    line_list4.type = visualization_msgs::Marker::LINE_LIST;
    line_list4.scale.x = 0.025;
    line_list4.color.a = 1.0;
    line_list4.color.g = 1.0;
    line_list4.color.r = 1.0;
    for (auto &line : matched_bricks[3]) {
        geometry_msgs::Point p1;
        geometry_msgs::Point p2;

        p1.x = line[0].x;
        p1.y = line[0].y;
        p1.z = line[0].z;
        line_list4.points.push_back(p1);

        p2.x = line[1].x;
        p2.y = line[1].y;
        p2.z = line[1].z;
        line_list4.points.push_back(p2);
    }

    // drone walls
    visualization_msgs::Marker line_list5;
    line_list5.header.frame_id = LIDAR_FRAME;
    line_list5.header.stamp = ros::Time::now();
    line_list5.ns = "points_and_lines";
    line_list5.action = visualization_msgs::Marker::ADD;
    line_list5.pose.orientation.w = 1.0;
    line_list5.id = 4;
    line_list5.type = visualization_msgs::Marker::LINE_LIST;
    line_list5.scale.x = 0.025;
    line_list5.color.a = 1.0;
    line_list5.color.g = 0.0;
    line_list5.color.r = 0.0;
    line_list5.color.b = 0.0;
    for (auto &line : matched_walls) {
        geometry_msgs::Point p1;
        geometry_msgs::Point p2;

        p1.x = line[0].x;
        p1.y = line[0].y;
        p1.z = line[0].z;
        line_list5.points.push_back(p1);

        p2.x = line[1].x;
        p2.y = line[1].y;
        p2.z = line[1].z;
        line_list5.points.push_back(p2);
    }


    lists.markers.push_back(line_list1);
    lists.markers.push_back(line_list2);
    lists.markers.push_back(line_list3);
    lists.markers.push_back(line_list4);
    lists.markers.push_back(line_list5);

    vis_pub.publish(lists);
    */

    /// --------------------------------------------------------------------------------------
}

void init_velodyne_subscriber(int argc, char **argv) {
    ros::init(argc, argv, "detector");
    ros::NodeHandle n;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    BrickDetector detector(n, &tfBuffer);

    vis_pub = n.advertise<visualization_msgs::MarkerArray>("/visualization_marker", 5);
    center_pub = n.advertise<visualization_msgs::Marker>("/pile_centers", 5);
    origin_pcl_pub = n.advertise<sensor_msgs::PointCloud>("/origin_line", 5);

    ros::Subscriber sub = n.subscribe("/velodyne_points", 5, &BrickDetector::subscribe_ptcl, &detector);
    ros::spin();
}

int main(int argc, char **argv) {
    init_velodyne_subscriber(argc, argv);
    return 0;
}
