#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"
#include "objdet.hpp"

#include <cmath>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <unordered_map>
#include <vector>

using namespace cv;
using namespace std;

int count_frame(VideoCapture *cap, string input_video) {
    int frameCount = 0;
    int frameCountManual = 0;
    int frameCountOCV = cap->get(CAP_PROP_FRAME_COUNT);
    Mat frame;

    if (frameCountOCV != 0) {
        frameCount = frameCountOCV + 1; // frame starts from 1 in mv files
    } else {
        for (;;) {
            *cap >> frame;
            if (frame.empty())
                break;
            frameCountManual++;
        }
        frameCount = frameCountManual + 1;
        cap->open(input_video);
    }

    return frameCount;
}

bool
tracker_setup(CommandLineParser &parser, ifstream &mvs_file, ifstream &gt_file, ifstream &det_file, VideoCapture &cap,
              int &frameWidth, int &frameHeight, int &WIDTH_MB, int &HEIGHT_MB, unsigned int &MVSIZE_PER_FRAME,
              int &FRAME_COUNT, int &MB_SIZE, int &interpolate_winsize, bool &adaptive, double &threshold) {
    // It doesn't seem to make much of a difference whether we
    //  read from the video or image sequence.
    string input = parser.get<string>("@input");
    string mvs_s = parser.get<string>("@mvs");
    string gt_s = parser.get<string>("@gt");
    string det_s = parser.get<string>("@det");
    // Window size 2 is the case where every other frame is interpolated.
    // winsize 1 is unrealistic and meaningless in hw impl as it means
    //  interpolating and detecting for the same frame (redundant work).
    // In hw impl, if a frame is detected, then the frame right before
    //  doesn't have to do interpolation (i.e., next_bbox doesn't have
    //  to be produced).
    // These details can be handled in a separate hw simulator. Here we
    //  simply produce the raw sw-level data.
    interpolate_winsize = parser.get<int>("ip_size");
    int mb_size = parser.get<int>("mb_size");
    adaptive = parser.get<bool>("adaptive");
    threshold = parser.get<double>("threshold");

    mvs_file.open(mvs_s.c_str());
    if (!mvs_file.good()) {
        fprintf(stderr, "Could not read the motion vector file...\n");
        return false;
    }

    gt_file.open(gt_s.c_str());
    if (!gt_file.good()) {
        fprintf(stderr, "Could not read the ground truth file...\n");
        return false;
    }

    det_file.open(det_s.c_str());
    if (!det_file.good()) {
        fprintf(stderr, "Could not read the detected trajectory file...\n");
        return false;
    }

    cap.open(input);
    if (!cap.isOpened()) {
        fprintf(stderr, "Could not read the video file...\n");
        return false;
    } else {
        frameWidth = cap.get(CAP_PROP_FRAME_WIDTH);
        frameHeight = cap.get(CAP_PROP_FRAME_HEIGHT);
        int fps = cap.get(CAP_PROP_FPS);

        int fourcc = static_cast<int>(cap.get(CAP_PROP_FOURCC));
        // Transform from int to char via Bitwise operators
        char ext[] = {(char) (fourcc & 0XFF), (char) ((fourcc & 0XFF00) >> 8),
                      (char) ((fourcc & 0XFF0000) >> 16), (char) ((fourcc & 0XFF000000) >> 24), 0};

        if (parser.has("mb_size")) {
            MB_SIZE = mb_size;
        } else {
            if (!strcmp(ext, "XVID\0")) {
                MB_SIZE = 16;
            } else {
                // fall back to the conservative mode. Assume same width and height...
                MB_SIZE = 4;
            }
        }

        // Calculate number of macroblocks horizontally and vertically.
        // Always pad it. This might be wasteing a bit of space.
        WIDTH_MB = frameWidth / MB_SIZE + 1;
        HEIGHT_MB = frameHeight / MB_SIZE + 1;
        MVSIZE_PER_FRAME = WIDTH_MB * HEIGHT_MB;
        FRAME_COUNT = count_frame(&cap, input);

        fprintf(stderr, "Video info: %dx%d, %d frames, %d fps, %d mb_size, %d ip_size, %s\n", frameWidth, frameHeight,
                FRAME_COUNT - 1, fps, MB_SIZE, interpolate_winsize, ext);
    }

    return true;
}

bool isOOF(Point pixel, int frameWidth, int frameHeight) {
    if (pixel.x >= 0 && pixel.x < frameWidth && pixel.y >= 0 && pixel.y < frameHeight) return false;
    else return true;
}

void process_mv_file(ifstream &mvs_file, int frameWidth, int frameHeight, int WIDTH_MB, int MB_SIZE,
                     vector<vector<Point> > &mv_grid, vector<vector<double> > &conf_grid, vector<vector<Point> > &mvs,
                     int *last_init_frame) {
    char line[1024];
    int frame_id;
    char pict_type;
    int cur_x, cur_y, delta_x, delta_y;
    double conf = -1.0;

    while (mvs_file.getline(line, 1024)) {
        if (line[0] == '#') {
            // frame_id starts from 1 in motion vector file
            sscanf(line, "# %d %c\n", &frame_id, &pict_type);
            *last_init_frame = frame_id;
        } else {
            sscanf(line, "%d\t%d\t%d\t%d\t%lf\n", &cur_x, &cur_y, &delta_x, &delta_y, &conf);

            int prev_x = cur_x + delta_x;
            int prev_y = cur_y + delta_y;

            int prev_frame_id = frame_id - 1;
            // TODO: ignore out of frame pixels for now
            if (!isOOF(Point(prev_x, prev_y), frameWidth, frameHeight)) {
                int mb_pos = prev_y / MB_SIZE * WIDTH_MB + prev_x / MB_SIZE;
                // In mv_grid, we let each MV point from N - 1 to N.
                // Basically convert motion vectors from backward to forward.
                mv_grid[prev_frame_id][mb_pos] = Point(-delta_x, -delta_y);
                if (conf != -1.0) conf_grid[prev_frame_id][mb_pos] = conf;

                mvs[prev_frame_id].push_back(Point(prev_x, prev_y));
                mvs[prev_frame_id].push_back(Point(cur_x, cur_y));
            }
        }
    }

    if (conf == -1.0) {
        fprintf(stderr, "No confidence value\n");
    }
}

void process_gt_bbox(ifstream &gt_file, unordered_map<int, Rect *> &gt_bbox_map) {
    Rect *bbox;
    char line[1024];
    int frame_id = 0;

    while (gt_file.getline(line, 1024)) {
        frame_id++;

        int width, height;
        Point tl;
        string line_str(line);

        if (line_str.find_first_of('\t') != string::npos) {
            sscanf(line, "%d\t%d\t%d\t%d\n", &tl.x, &tl.y, &width, &height);
        } else {
            sscanf(line, "%d,%d,%d,%d\n", &tl.x, &tl.y, &width, &height);
        }
        bbox = new Rect(tl.x, tl.y, width, height);

        gt_bbox_map[frame_id] = bbox;
    }
}

void process_gt_bbox(ifstream &gt_file, unordered_map<int, vector<Point2f> *> &gt_bbox_map) {
    vector<Point2f> *bbox;
    char line[1024];
    int frame_id = 0;

    while (gt_file.getline(line, 1024)) {
        frame_id++;
        Point2f corners[4];

        sscanf(line, "%f,%f,%f,%f,%f,%f,%f,%f\n", &corners[0].x, &corners[0].y, &corners[1].x, &corners[1].y,
               &corners[2].x, &corners[2].y, &corners[3].x, &corners[3].y);
        bbox = new vector<Point2f>(corners, corners + 4);

        gt_bbox_map[frame_id] = bbox;
    }
}

void process_det_bbox_yolo(ifstream &det_file, unordered_map<int, vector<Pred_Rect *> > &det_bbox_map) {
    Rect *bbox;
    char line[1024];

    while (det_file.getline(line, 1024)) {
        int frame_id;
        int x, y, width, height, conf;
        sscanf(line, "%d %d %d,%d,%d,%d\n", &frame_id, &conf, &x, &y, &width, &height);
        Rect *bbox = new Rect(x, y, width, height);
        Pred_Rect *pred_bbox = new Pred_Rect(bbox, conf);
        det_bbox_map[frame_id].push_back(pred_bbox);
    }
}

void process_det_bbox_otb(ifstream &det_file, unordered_map<int, Rect *> &det_bbox_map) {
    Rect *bbox;
    char line[1024];
    int frame_id = 0;

    while (det_file.getline(line, 1024)) {
        frame_id++;

        int width, height;
        Point tl;
        string line_str(line);

        if (line_str.find_first_of('\t') != string::npos) {
            sscanf(line, "%d\t%d\t%d\t%d\n", &tl.x, &tl.y, &width, &height);
        } else {
            sscanf(line, "%d,%d,%d,%d\n", &tl.x, &tl.y, &width, &height);
        }
        bbox = new Rect(tl.x, tl.y, width, height);

        det_bbox_map[frame_id] = bbox;
    }
}

bool process_det_bbox_vot2014(ifstream &det_file, unordered_map<int, vector<Point2f> *> &det_bbox_map,
                              unordered_map<int, vector<Point2f> *> &gt_bbox_map) {
    vector<Point2f> *bbox;
    char line[1024];
    int frame_id = 0;
    vector<int> special_frames(3, 0);

    while (det_file.getline(line, 1024)) {
        frame_id++;
        bbox = NULL;

        string line_str(line);
        size_t num_of_fields = count(line_str.begin(), line_str.end(), ',');

        if (num_of_fields == 0) {
            special_frames[atoi(line)]++;
            // TODO: for VOT2014, we simply use gt if a frame is init/re-init/drop
            bbox = gt_bbox_map[frame_id];
        } else {
            Point2f corners[4];

            if (num_of_fields == 3) {
                float width, height;
                Point2f tl;

                sscanf(line, "%f,%f,%f,%f\n", &tl.x, &tl.y, &width, &height);

                corners[0] = Point2f(tl);
                corners[1] = Point2f(tl.x, tl.y + height);
                corners[2] = Point2f(tl.x + width, tl.y + height);
                corners[3] = Point2f(tl.x + width, tl.y);
            } else if (num_of_fields == 7) {
                sscanf(line, "%f,%f,%f,%f,%f,%f,%f,%f\n", &corners[0].x, &corners[0].y, &corners[1].x, &corners[1].y,
                       &corners[2].x, &corners[2].y, &corners[3].x, &corners[3].y);
            } else {
                fprintf(stderr, "Unsupported detector trajectory file\n");
                return false;
            }

            bbox = new vector<Point2f>(corners, corners + 4);
        }

        det_bbox_map[frame_id] = bbox;
    }
    return true;
}

bool getAndDraw_gt_bbox(Mat *frame, unordered_map<int, Rect *> *bbox_map, int frame_id, Rect *&gt_bbox) {
    unordered_map<int, Rect *>::const_iterator it;
    it = bbox_map->find(frame_id);

    if (it != bbox_map->end()) {
        // Read the ground-truth bbox.
        gt_bbox = it->second;

        // Always draw ground truth
        rectangle(*frame, Point(gt_bbox->x, gt_bbox->y),
                  Point(gt_bbox->x + gt_bbox->width, gt_bbox->y + gt_bbox->height),
                  cv::Scalar(255, 255, 255), 2);

        return true;
    }
    return false;
}

bool getAndDraw_gt_bbox(Mat *frame, unordered_map<int, vector<Point2f> *> *bbox_map, int frame_id,
                        vector<Point2f> *&gt_bbox) {
    unordered_map<int, vector<Point2f> *>::const_iterator it;
    it = bbox_map->find(frame_id);

    if (it != bbox_map->end()) {
        // Read the ground-truth bbox.
        gt_bbox = it->second;

        // Always draw ground truth
        line(*frame, (*gt_bbox)[0], (*gt_bbox)[1], cv::Scalar(255, 255, 255), 2);
        line(*frame, (*gt_bbox)[1], (*gt_bbox)[2], cv::Scalar(255, 255, 255), 2);
        line(*frame, (*gt_bbox)[2], (*gt_bbox)[3], cv::Scalar(255, 255, 255), 2);
        line(*frame, (*gt_bbox)[3], (*gt_bbox)[0], cv::Scalar(255, 255, 255), 2);

        return true;
    }
    return false;
}

bool
get_detected_bboxes(unordered_map<int, vector<Pred_Rect *>> *bbox_map, int frame_id, vector<Pred_Rect *> &det_bbox) {
    unordered_map<int, vector<Pred_Rect *>>::const_iterator it;
    it = bbox_map->find(frame_id);

    if (it != bbox_map->end()) {
        // Read the ground-truth bbox.
        det_bbox = it->second;
        return true;
    }
    return false;
}

bool getAndDraw_detected_bbox(Mat *frame, unordered_map<int, Rect *> *bbox_map, int frame_id, Rect *&det_bbox) {
    unordered_map<int, Rect *>::const_iterator it;
    it = bbox_map->find(frame_id);

    if (it != bbox_map->end()) {
        // Read the ground-truth bbox.
        det_bbox = it->second;

        // Always draw ground truth
        rectangle(*frame, Point(det_bbox->x, det_bbox->y),
                  Point(det_bbox->x + det_bbox->width, det_bbox->y + det_bbox->height),
                  cv::Scalar(0, 0, 255), 2);

        return true;
    }
    return false;
}

bool getAndDraw_detected_bbox(Mat *frame, unordered_map<int, vector<Point2f> *> *bbox_map, int frame_id,
                              vector<Point2f> *&det_bbox) {
    unordered_map<int, vector<Point2f> *>::const_iterator it;
    it = bbox_map->find(frame_id);

    if (it != bbox_map->end()) {
        det_bbox = it->second;
        if (!det_bbox) return false; // special frames

        line(*frame, (*det_bbox)[0], (*det_bbox)[1], cv::Scalar(0, 0, 255), 2);
        line(*frame, (*det_bbox)[1], (*det_bbox)[2], cv::Scalar(0, 0, 255), 2);
        line(*frame, (*det_bbox)[2], (*det_bbox)[3], cv::Scalar(0, 0, 255), 2);
        line(*frame, (*det_bbox)[3], (*det_bbox)[0], cv::Scalar(0, 0, 255), 2);

        return true;
    }
    return false;
}

void draw_next_bbox(Mat *frame, Rect *next_bbox) {
    if (next_bbox) {
        rectangle(*frame, Point(next_bbox->x, next_bbox->y),
                  Point(next_bbox->x + next_bbox->width, next_bbox->y + next_bbox->height),
                  cv::Scalar(255, 0, 0), 2);
    }
}

void draw_next_bbox(Mat *frame, vector<Point2f> *next_bbox) {
    if (next_bbox) {
        line(*frame, (*next_bbox)[0], (*next_bbox)[1], cv::Scalar(255, 0, 0), 2);
        line(*frame, (*next_bbox)[1], (*next_bbox)[2], cv::Scalar(255, 0, 0), 2);
        line(*frame, (*next_bbox)[2], (*next_bbox)[3], cv::Scalar(255, 0, 0), 2);
        line(*frame, (*next_bbox)[3], (*next_bbox)[0], cv::Scalar(255, 0, 0), 2);
    }
}

// http://paulbourke.net/geometry/polygonmesh/source1.c
void PolygonArea(vector<Point2f> &polygon, int N, float &area) {
    // OpenCV's intersectConvexConvex may generate polygon with one single point and the value is typically nonsense. simply return 0 in this case.
    if (N <= 1) return;

    for (int i = 0; i < N; i++) {
        int j = (i + 1) % N;
        area += polygon[i].x * polygon[j].y;
        area -= polygon[i].y * polygon[j].x;
        //fprintf(stdout, "%d/%d: %f, %f, %f, %f, %lf\n", i, N-1, polygon[i].x, polygon[j].y, polygon[i].y, polygon[j].x, area);
    }

    area /= 2.0;
    area = area < 0.0 ? -area : area;
}

float calc_iou(Rect *bbox1, Rect *bbox2) {
    if (!bbox1 || !bbox2) return -1.0;

    // Calculate IoU value
    Rect intersect = (*bbox1) & (*bbox2);
    unsigned int iValue = intersect.area();
    unsigned int uValue = bbox1->area() + bbox2->area() - iValue;
    float iou = (float) iValue / (float) uValue;

    return iou;
}

float calc_iou(vector<Point2f> *bbox1, vector<Point2f> *bbox2) {
    if (!bbox1 || !bbox2) return -1.0;

    vector<Point2f> intersect;
    intersectConvexConvex(*bbox1, *bbox2, intersect);
    float intersect_area = 0.0;
    PolygonArea(intersect, intersect.size(), intersect_area);

    float area1 = 0.0;
    PolygonArea(*bbox1, bbox1->size(), area1);

    float area2 = 0.0;
    PolygonArea(*bbox2, bbox2->size(), area2);

    float union_area = area1 + area2 - intersect_area;

    float iou = intersect_area / union_area;
    if (iou > 1.0) iou = 1.0; // precision issue in calculating intersect_area

    return iou;
}

float calc_prepare_iou(Rect *gt_bbox, Rect *base_bbox, vector<int> &iou_accept) {
    float iou = calc_iou(gt_bbox, base_bbox);

    // Prepare for success plot
    // Fill iou_accept[x] only if iou > 0.x;
    // e.g.: iou = 0.23, start = 2, fill 0~2;
    // e.g.: iou = 0.2, start = 1, fill 0~1;
    int start = (int) (iou * 10);
    if (iou == start / 10.0) start--;
    for (int i = 0; i <= start; i++) {
        iou_accept[i]++;
    }

    return iou;
}

float calc_prepare_iou(vector<Point2f> *gt_bbox, vector<Point2f> *base_bbox, vector<int> &iou_accept) {
    float iou = calc_iou(gt_bbox, base_bbox);

    int start = (int) (iou * 10);
    if (iou == start / 10.0) start--;
    for (int i = 0; i <= start; i++) {
        iou_accept[i]++;
    }

    return iou;
}

float calc_prepare_dist(Rect *gt_bbox, Rect *base_bbox, vector<int> &dist_accept) {
    // Calculate distance from ground truth
    Point2f gt_bbox_center(((float) gt_bbox->x + (float) gt_bbox->width / 2.0),
                           ((float) gt_bbox->y + (float) gt_bbox->height / 2.0));
    Point2f base_bbox_center(((float) base_bbox->x + (float) base_bbox->width / 2.0),
                             ((float) base_bbox->y + (float) base_bbox->height / 2.0));
    float distance = norm(gt_bbox_center - base_bbox_center);

    // Prepare for precision plot
    // Fill dist_accept[x] only if distance < x * 5;
    // e.g.: dist = 2.3, start = 1, fill 1~10;
    // e.g.: dist = 45, start = 10, fill 10;
    // e.g.: dist = 54.4, start = 11, no fill;
    int start = (int) distance / 5 + 1;
    for (int i = start; i <= 10; i++) {
        dist_accept[i]++;
    }

    return distance;
}

float calc_prepare_dist(vector<Point2f> *gt_bbox, vector<Point2f> *base_bbox, vector<int> &dist_accept) {
    // Calculate distance from ground truth
    Point2f corner1 = (*base_bbox)[0];
    Point2f corner2 = (*base_bbox)[2];
    Point2f base_bbox_center = (corner1 + corner2) * 0.5;

    corner1 = (*gt_bbox)[0];
    corner2 = (*gt_bbox)[2];
    Point2f gt_bbox_center = (corner1 + corner2) * 0.5;

    float distance = norm(gt_bbox_center - base_bbox_center);
    //fprintf(stdout, "%f, %f, %f\n", gt_bbox_center.x, base_bbox_center.x, distance);

    // Prepare for precision plot
    // Fill dist_accept[x] only if distance < x * 5;
    // e.g.: dist = 2.3, start = 1, fill 1~10;
    // e.g.: dist = 45, start = 10, fill 10;
    // e.g.: dist = 54.4, start = 11, no fill;
    int start = (int) distance / 5 + 1;
    for (int i = start; i <= 10; i++) {
        dist_accept[i]++;
    }

    return distance;
}

bool isEdge(int i, int j, Rect base_bbox) {
    int bound = 10;
    if (i > base_bbox.width * (bound - 1) / bound) {
        return true;
    } else {
        return i < base_bbox.width / bound ||
               j < base_bbox.height / bound ||
               j > base_bbox.height * (bound - 1) / bound;
    }

}

double find_avg_mv(Rect base_bbox, Point2f *avg_mv, vector<Point> &mv_vector, vector<double> &conf_vector, int WIDTH_MB,
                   int MB_SIZE, int frameWidth, int frameHeight) {
    float count = 0;
    double conf = 0.0;

    Point2f new_avg_mv(0.0, 0.0);

    for (int i = 0; i < base_bbox.width; i++) {
        for (int j = 0; j < base_bbox.height; j++) {
            int cur_x = base_bbox.x + i;
            int cur_y = base_bbox.y + j;
            // base_bbox could have negative coordinates. Make sure we don't access them.
            if (isOOF(Point(cur_x, cur_y), frameWidth, frameHeight)) continue;

            int mv_index = (cur_y - 1) / MB_SIZE * WIDTH_MB + (cur_x - 1) / MB_SIZE;
            Point mv = mv_vector[mv_index];

            //if ((mv.x != NON_MOTION) || (mv.y != NON_MOTION)) {
            bool edgePixel = isEdge(i, j, base_bbox);
            float edgeWeight = 1.0, centerWeight = 1.0;
            if (edgePixel) {
                new_avg_mv.x += (float) mv.x * edgeWeight;
                new_avg_mv.y += (float) mv.y * edgeWeight;
                conf += conf_vector[mv_index];
                count += edgeWeight;
            } else {
                new_avg_mv.x += (float) mv.x * centerWeight;
                new_avg_mv.y += (float) mv.y * centerWeight;
                conf += conf_vector[mv_index];
                count += centerWeight;
            }
            //fprintf(stdout, "%lf\n", conf_vector[mv_index]);
            //}
        }
    }
    // The corner case is: for the P frame right before an I, if it has a bbox,
    //  it can't pass it to the next (I) frame because there is no motion
    //  vector from this P to the next I. This can also happen for frames that
    //  are skipped in the mv file. In any case, count will be 0 and we will
    //  simply use the previous MV for now.
    if (count != 0) {
        avg_mv->x = new_avg_mv.x / count;
        avg_mv->y = new_avg_mv.y / count;
        conf /= count;
        //fprintf(stdout, "avg mv: %f, %f\n\n", avg_mv->x, avg_mv->y);
        //fprintf(stdout, "avg conf: %lf. %d out of %d\n", conf, count, base_bbox.width * base_bbox.height);
    }

    return conf;
}

// Idea: Calculate the average motion vector within the ROI.
double interpolate_next_bbox(Rect *&next_bbox, Rect *&base_bbox, Point2f *avg_mv, vector<Point> &mv_vector,
                             vector<double> &conf_vector, int WIDTH_MB, int MB_SIZE, int frameWidth, int frameHeight) {
    double confidence = find_avg_mv(*base_bbox, avg_mv, mv_vector, conf_vector, WIDTH_MB, MB_SIZE, frameWidth,
                                    frameHeight);

    next_bbox = new Rect(base_bbox->x + (int) avg_mv->x, base_bbox->y + (int) avg_mv->y, base_bbox->width,
                         base_bbox->height);

    return confidence;
}

double interpolate_next_bbox(vector<Point2f> *&next_bbox, vector<Point2f> *base_bbox, Point2f *avg_mv,
                             vector<Point> &mv_vector, vector<double> &conf_vector, int WIDTH_MB, int MB_SIZE,
                             int frameWidth, int frameHeight) {
    // Use the bounding rect for calculating the average mv
    Rect bbox = boundingRect(*base_bbox);

    double confidence = find_avg_mv(bbox, avg_mv, mv_vector, conf_vector, WIDTH_MB, MB_SIZE, frameWidth, frameHeight);

    next_bbox = new vector<Point2f>();
    for (int i = 0; i < base_bbox->size(); i++) {
        next_bbox->push_back((*base_bbox)[i] + *avg_mv);
    }

    return confidence;
}

void next_ip_winsize(float iou, int &ip_winsize, int &counter) {
    if (iou >= 0 && iou < 0.5) {
        counter = 0;
        ip_winsize = 2;
    } else if (iou > 0.8) {
        counter++;
        if (counter >= 5) ip_winsize += 1;
    }

    if (ip_winsize < 2) ip_winsize = 2;
        //else if (ip_winsize > 4) ip_winsize = 4; // good for yolo
    else if (ip_winsize > 8) ip_winsize = 8;
}

void draw_mv_arrows(Mat *frame, vector<vector<Point> > &mvs, int frame_id) {
    for (int i = 0; i < mvs[frame_id].size(); i += 2) {
        Point src = mvs[frame_id][i];
        Point dst = mvs[frame_id][i + 1];
        arrowedLine(*frame, src, dst, cv::Scalar(255, 255, 255), 2, 8, 0, 0.3);
    }
}

// TODO should assign name of out dir
void gen_images(int frame_id, int FRAME_COUNT, Mat *frame) {
    stringstream ss;
    int outImgFileWidth = (int) log10(FRAME_COUNT) + 1;
    ss << setw(outImgFileWidth) << setfill('0') << frame_id;
    string s = ss.str();
    imwrite("out/" + s + ".jpg", *frame);
}

void dump_success_stats(int total_annotated, vector<int> &iou_accept) {
    fprintf(stdout, "************************\n");
    fprintf(stdout, "***Success Statistics***\n");
    fprintf(stdout, "************************\n");
    for (int i = 0; i <= 10; i++) {
        fprintf(stdout, ">%.1f, %f %d\n", i / 10.0, (float) iou_accept[i] / (float) total_annotated, iou_accept[i]);
    }
}

void dump_precision_stats(int total_annotated, vector<int> &dist_accept) {
    fprintf(stdout, "************************\n");
    fprintf(stdout, "**Precision Statistics**\n");
    fprintf(stdout, "************************\n");
    for (int i = 0; i <= 10; i++) {
        fprintf(stdout, "<%d, %f %d\n", i * 5, (float) dist_accept[i] / (float) total_annotated, dist_accept[i]);
    }
}
