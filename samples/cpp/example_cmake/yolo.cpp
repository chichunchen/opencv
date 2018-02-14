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

//#define DRAW_GT
#define DRAW_DET
//#define DRAW_MV
//#define DRAW_EXPL

void find_gt_bbox(Rect *pred_bbox, vector<Pred_Rect *> *frame_gt_bbox, float &iou) {
    float max_iou = -1;

    for (int i = 0; i < frame_gt_bbox->size(); i++) {
        Rect *gt_bbox = (*frame_gt_bbox)[i]->rect;
        Rect intersect = (*gt_bbox) & (*pred_bbox);

        unsigned int iValue = intersect.area();
        unsigned int uValue = pred_bbox->area() + gt_bbox->area() - iValue;
        float cur_iou = (float) iValue / (float) uValue;

        if (cur_iou > max_iou) {
            max_iou = cur_iou;
        }
    }

    iou = max_iou;
    assert(iou != -1);
}

void draw_gt_bboxes(Mat &frame, unordered_map<int, vector<Rect *> > &gt_bbox_map, int frame_id) {
    for (int i = 0; i < gt_bbox_map[frame_id].size(); i++) {
        int x = gt_bbox_map[frame_id][i]->x + gt_bbox_map[frame_id][i]->width;
        int y = gt_bbox_map[frame_id][i]->y + gt_bbox_map[frame_id][i]->height;
        rectangle(frame, Point(gt_bbox_map[frame_id][i]->x, gt_bbox_map[frame_id][i]->y),
                  Point(x, y), cv::Scalar(0, 0, 255), 2);
    }
}

// TODO check why this one is not drawing
void draw_det_bboxes(Mat &frame, vector<Pred_Rect *> &temp_det_bboxes, int frame_id) {
    for (int i = 0; i < temp_det_bboxes.size(); i++) {
        int x = temp_det_bboxes[i]->rect->x + temp_det_bboxes[i]->rect->width;
        int y = temp_det_bboxes[i]->rect->y + temp_det_bboxes[i]->rect->height;
        rectangle(frame, Point(temp_det_bboxes[i]->rect->x, temp_det_bboxes[i]->rect->y),
                  Point(x, y), cv::Scalar(0, 0, 255), 2);
    }
}

int main(int argc, char **argv) {
    const String keys =
            "{@input          |       | path to video file                                 }"
                    "{@mvs            |       | path to motion vector file                         }"
                    "{@gt             |       | path to ground truth file                          }"
                    "{@det            |       | path to detected bbox file                         }"
                    "{mb_size         |  16   | macroblock size                                    }"
                    "{adaptive        | false | adaptive of constant mode?                         }"
                    "{ip_size         |  2    | consecutively interpolated frames in constant mode }"
                    "{threshold       | 0.92  | conf threshold for mv                              }";

    // Read various files, get basic video info
    ifstream mvs_file;
    ifstream gt_file;
    ifstream det_file;
    VideoCapture cap;
    int FRAME_COUNT;
    int frameWidth, frameHeight;
    int WIDTH_MB, HEIGHT_MB;
    int MB_SIZE;
    unsigned int MVSIZE_PER_FRAME;
    int ip_winsize;
    bool adaptive;
    double threshold;

    CommandLineParser parser(argc, argv, keys);

    bool success = tracker_setup(parser, mvs_file, gt_file, det_file, cap,
                                 frameWidth, frameHeight, WIDTH_MB, HEIGHT_MB, MVSIZE_PER_FRAME,
                                 FRAME_COUNT, MB_SIZE, ip_winsize, adaptive, threshold);
    if (!success) return 0;

    // Process motion vectors
    vector<vector<Point> > mv_grid(FRAME_COUNT, vector<Point>(MVSIZE_PER_FRAME, Point(NON_MOTION, NON_MOTION)));
    vector<vector<double> > conf_grid(FRAME_COUNT, vector<double>(MVSIZE_PER_FRAME, 1.0));
    vector<vector<Point> > mvs(FRAME_COUNT, vector<Point>());
    int last_init_frame = 0; // also serves the total frames in mv
    process_mv_file(mvs_file, frameWidth, frameHeight, WIDTH_MB, MB_SIZE,
                    mv_grid, conf_grid, mvs, &last_init_frame);

    // Process GT
    Rect *bbox;
    char line[1024];
    unordered_map<int, vector<Rect *> > gt_bbox_map;

    while (gt_file.getline(line, 1024)) {
        int frame_id;
        int x, y, width, height;
        sscanf(line, "%d %d %d %d %d\n", &frame_id, &x, &y, &width, &height);
        Rect *bbox = new Rect(x, y, width, height);
        gt_bbox_map[frame_id].push_back(bbox);
    }

    // Process detected bboxes
    unordered_map<int, vector<Pred_Rect *> > det_bbox_map;
    process_det_bbox_yolo(det_file, det_bbox_map);

    // Process video
    vector<Pred_Rect *> *next_bboxes = NULL; // interpolated by previous frame
    vector<Pred_Rect *> *det_bboxes = NULL; // detected by CNN
    vector<Pred_Rect *> temp_det_bboxes;
    Mat frame;
    Point2f avg_mv(0.0, 0.0);
    int frame_id = 0;
    bool toInterpolate = false; // is this frame interpolated or detected?
    int ip_count = 0, det_count = 0;
    int adaptive_counter = 0;

    for (;;) {
        cap >> frame;
        // It's possible that the MV file contains fewer frames than in the video.
        if (frame.empty() || frame_id >= last_init_frame)
            break;

        frame_id++;
        toInterpolate = (ip_winsize != 0) && ((frame_id - 1) % ip_winsize != 0);
        if (toInterpolate) ip_count++;
        else det_count++;
        //fprintf(stdout, "mode: %d\n", toInterpolate);

#ifdef DRAW_GT
        draw_gt_bboxes(frame, gt_bbox_map, frame_id);
#endif

        // Get CNN and draw predicted bbox if a frame is to be detected
        if (!toInterpolate) {
            temp_det_bboxes.clear();
            get_detected_bboxes(&det_bbox_map, frame_id, temp_det_bboxes);

#ifdef DRAW_DET
            draw_det_bboxes(frame, temp_det_bboxes, frame_id);
#endif

            det_bboxes = &temp_det_bboxes;

            if (adaptive) {
                // In reality we should calculate the AP here..
                if (next_bboxes) {
                    float iou = 0.0;
                    int max_conf = 0;
                    int max_id = -1;
                    for (int i = 0; i < next_bboxes->size(); i++) {
                        if ((*next_bboxes)[i]->conf > max_conf) {
                            max_conf = (*next_bboxes)[i]->conf;
                            max_id = i;
                        }
                    }
                    if (max_id != -1) find_gt_bbox((*next_bboxes)[max_id]->rect, det_bboxes, iou);
                    next_ip_winsize(iou, ip_winsize, adaptive_counter);
                    //fprintf(stdout, "frame_%d: %f, %d\n", frame_id, iou, ip_winsize);
                }
            }
        }

        // dump predicted bboxes
        vector<Pred_Rect *> *base_bboxes;
        base_bboxes = toInterpolate ? next_bboxes : det_bboxes;

        if (base_bboxes) {
            for (int i = 0; i < base_bboxes->size(); i++) {
                Pred_Rect *base_bbox = (*base_bboxes)[i];
                fprintf(stdout, "%d %d %d,%d,%d,%d\n", frame_id, base_bbox->conf, base_bbox->rect->x,
                        base_bbox->rect->y, base_bbox->rect->width, base_bbox->rect->height);
            }
        }

        // TODO trace this
        // Make a prediction for the next frame.
        if (base_bboxes) {
            vector<Pred_Rect *> *temp_next_bboxes = new vector<Pred_Rect *>();
            for (int i = 0; i < base_bboxes->size(); i++) {
                Pred_Rect *base_bbox = (*base_bboxes)[i];
                Rect *next_bbox = NULL;

                double confidence = interpolate_next_bbox(next_bbox, base_bbox->rect, &avg_mv, mv_grid[frame_id],
                                                          conf_grid[frame_id], WIDTH_MB, MB_SIZE, frameWidth,
                                                          frameHeight);

                Pred_Rect *pred_bbox = new Pred_Rect(next_bbox, base_bbox->conf);
                temp_next_bboxes->push_back(pred_bbox);

            }
            next_bboxes = temp_next_bboxes;
        }

#ifdef DRAW_MV
        draw_mv_arrows(&frame, mvs, frame_id);
#endif

#ifdef DRAW_DET
        draw_det_bboxes(frame, temp_det_bboxes, frame_id);
#endif

#if defined(DRAW_MV) || defined(DRAW_EXPL) || defined(DRAW_GT) || defined(DRAW_DET)
        gen_images(frame_id, FRAME_COUNT, &frame);
#endif
    }
    fprintf(stdout, "\nTotal Annotated: %d (%d, %d)\n", det_count + ip_count, det_count, ip_count);

    cap.release();

    return 0;
}
