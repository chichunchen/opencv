#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"
#include <vector>
#include <unordered_map>

#ifndef MV_OBJDET
#define MV_OBJDET

using namespace std;
using namespace cv;

struct Pred_Rect {
  Rect *rect;
  int conf;
  Pred_Rect(Rect *r, int c) {
    rect = r;
    conf = c;
  }
};

// In reality, we probably should set this to an invalid motion, but evaluation
//  results show that setting to 0 is slightly better because it allows us to
//  ignore motion of pixels that are predicted to be still.
#define NON_MOTION 0

int count_frame(VideoCapture *, string);

bool tracker_setup(CommandLineParser &, ifstream &, ifstream &, ifstream &, VideoCapture &, int &, int &, int &, int &, unsigned int &, int &, int &, int &, bool &, double &);

bool isOOF(Point);

void process_mv_file(ifstream &, int, int, int, int, vector< vector<Point> > &, vector< vector<double> > &, vector< vector<Point> > &, int *);

void process_gt_bbox(ifstream &, unordered_map<int, Rect *> &);

void process_gt_bbox(ifstream &, unordered_map<int, vector<Point2f> *> &);

bool process_det_bbox_vot2014(ifstream &, unordered_map<int, vector<Point2f> *> &, unordered_map<int, vector<Point2f> *> &);

void process_det_bbox_otb(ifstream &, unordered_map<int, Rect *> &);

void process_det_bbox_yolo(ifstream &, unordered_map< int, vector<Pred_Rect *> > &);

bool getAndDraw_gt_bbox(Mat *, unordered_map<int, Rect *> *, int, Rect* &);

bool getAndDraw_gt_bbox(Mat *, unordered_map<int, vector<Point2f>*> *, int, vector<Point2f>* &);

bool getAndDraw_detected_bbox(Mat *, unordered_map<int, Rect *> *, int, Rect* &);

bool getAndDraw_detected_bbox(Mat *, unordered_map<int, vector<Point2f> *> *, int, vector<Point2f>* &);

bool get_detected_bboxes(unordered_map<int, vector<Pred_Rect *>> *, int, vector<Pred_Rect *> &);

void draw_next_bbox(Mat *, Rect *);

void draw_next_bbox(Mat *, vector<Point2f>*);

void PolygonArea(vector<Point2f> &, int, float &);

float calc_iou(Rect *, Rect *);

float calc_iou(vector<Point2f> *, vector<Point2f> *);

float calc_prepare_iou(Rect *, Rect *, vector<int> &);

float calc_prepare_iou(vector<Point2f> *, vector<Point2f> *, vector<int> &);

float calc_prepare_dist(Rect *, Rect *, vector<int> &);

float calc_prepare_dist(vector<Point2f> *, vector<Point2f>*, vector<int> &);

double find_avg_mv(Rect, Point2f *, vector<Point> &, vector<double> &, int, int, int, int);

double interpolate_next_bbox(Rect* &, Rect* &, Point2f*, vector<Point> &, vector<double> &, int, int, int, int);

double interpolate_next_bbox(vector<Point2f>* &, vector<Point2f>*, Point2f*, vector<Point> &, vector<double> &, int, int, int, int);

void next_ip_winsize(float, int &, int &);

void draw_mv_arrows(Mat *, vector< vector<Point> > &, int);

void gen_images(int, int, Mat *);

void dump_success_stats(int, vector<int> &);

void dump_precision_stats(int, vector<int> &);

#endif
