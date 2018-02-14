#include "opencv2/imgproc.hpp"
#include <opencv2/videoio.hpp>
#include <zconf.h>
#include "objdet.hpp"

using namespace cv;
using namespace std;

static double MAX_SAD;
unsigned long long total_ops = 0;

unsigned long calc_sad(Mat &dst_mb, Mat &src_mb) {
    total_ops += dst_mb.cols * dst_mb.rows;
    return (unsigned long) norm(dst_mb, src_mb, NORM_L1);
}

bool searchable(Point p, int left, int right, int up, int down) {
    if (p.x >= left && p.x <= right && p.y >= up && p.y <= down) return true;
    return false;
}

// Three Step Search (TSS).
unsigned long tss(int search_para, int &dst_x, int &dst_y, int src_x, int src_y, int bounds_x_left, int bounds_x_right,
                  int bounds_y_up, int bounds_y_down, int mb_size, Mat &dst_img, Mat &src_mb,
                  unsigned int &per_block_ops) {
    int dst1_x, dst1_y, dst2_x, dst2_y;

    unsigned long min_sad = ULONG_MAX;
    int stride = (search_para + 1) / 2;

    // First step
    for (int m = src_x - stride; m <= src_x + stride; m += stride) {
        for (int n = src_y - stride; n <= src_y + stride; n += stride) {
            if (searchable(Point(m, n), bounds_x_left, bounds_x_right, bounds_y_up, bounds_y_down)) {
                per_block_ops++;
                Mat dst_mb = dst_img(Rect(m, n, mb_size, mb_size));

                unsigned long cur_sad = calc_sad(dst_mb, src_mb);
                if (cur_sad < min_sad) {
                    min_sad = cur_sad;
                    dst1_x = m;
                    dst1_y = n;
                }
            }
        }
    }
    if (min_sad == ULONG_MAX) {
        fprintf(stdout, "step1: %d\t%d\t%d\n", src_x, src_y, stride);
    }

    // Second step
    min_sad = ULONG_MAX;
    stride /= 2;
    for (int m = dst1_x - stride; m <= dst1_x + stride; m += stride) {
        for (int n = dst1_y - stride; n <= dst1_y + stride; n += stride) {
            if (searchable(Point(m, n), bounds_x_left, bounds_x_right, bounds_y_up, bounds_y_down)) {
                per_block_ops++;
                Mat dst_mb = dst_img(Rect(m, n, mb_size, mb_size));

                unsigned long cur_sad = calc_sad(dst_mb, src_mb);
                if (cur_sad < min_sad) {
                    min_sad = cur_sad;
                    dst2_x = m;
                    dst2_y = n;
                }
            }
        }
    }
    if (min_sad == ULONG_MAX) {
        fprintf(stdout, "step2: %d\t%d\t%d\n", dst1_x, dst2_y, stride);
    }

    // Third step
    min_sad = ULONG_MAX;
    stride /= 2;
    for (int m = dst2_x - stride; m <= dst2_x + stride; m += stride) {
        for (int n = dst2_y - stride; n <= dst2_y + stride; n += stride) {
            if (searchable(Point(m, n), bounds_x_left, bounds_x_right, bounds_y_up, bounds_y_down)) {
                per_block_ops++;
                Mat dst_mb = dst_img(Rect(m, n, mb_size, mb_size));

                unsigned long cur_sad = calc_sad(dst_mb, src_mb);
                if (cur_sad < min_sad) {
                    min_sad = cur_sad;
                    dst_x = m;
                    dst_y = n;
                }
            }
        }
    }
    if (min_sad == ULONG_MAX) {
        fprintf(stdout, "step3: %d\t%d\t%d\n", dst2_x, dst2_y, stride);
    }

    return min_sad;
}

unsigned long es(int &dst_x, int &dst_y, int src_x, int src_y, int bounds_x_left, int bounds_x_right, int bounds_y_up,
                 int bounds_y_down, int mb_size, Mat &dst_img, Mat &src_mb, unsigned int &per_block_ops) {
    unsigned long min_sad = ULONG_MAX;

    for (int m = bounds_x_left; m <= bounds_x_right; m++) {
        for (int n = bounds_y_up; n <= bounds_y_down; n++) {
            per_block_ops++;
            Mat dst_mb = dst_img(Rect(m, n, mb_size, mb_size));

            unsigned long cur_sad = calc_sad(dst_mb, src_mb);
            if (cur_sad < min_sad) {
                min_sad = cur_sad;
                dst_x = m;
                dst_y = n;
            }
        }
    }
    if (min_sad == ULONG_MAX) {
        fprintf(stdout, "%d\t%d\n", src_x, src_y);
    }

    return min_sad;
}

// We are trying to find a match in the reference frame (dst) for each MB
//  in a predicted frame (src). That is, src_img is where we are iterating
//  through and dst_img is where matches are found.
// In backward mode, src is frame N  and dst is frame N - 1.
// In forward mode, src is frame N - 1 and dst is frame N.
void blockmatching(Mat &src_img, Mat &dst_img, int mb_size, vector<Rect> &motion_vector, vector<float> &confidence,
                   string mode, bool isBackward) {
    int width_mb = src_img.cols / mb_size;
    int height_mb = src_img.rows / mb_size;
    int dst_x, dst_y, src_x, src_y;
    int search_para = 7;

    // TODO: ignore "corner" cases for now. In reality, we could pad it.
    for (int i = 0; i < width_mb; i++) {
        for (int j = 0; j < height_mb; j++) {
            src_x = i * mb_size;
            src_y = j * mb_size;

            Mat src_mb = src_img(Rect(src_x, src_y, mb_size, mb_size));

            int bounds_x_left = max(src_x - search_para, 0);
            int bounds_x_right = min(src_x + search_para, dst_img.cols - mb_size);
            int bounds_y_up = max(src_y - search_para, 0);
            int bounds_y_down = min(src_y + search_para, dst_img.rows - mb_size);

            unsigned int per_block_ops = 0;
            unsigned long sad;

            if (mode == "tss") {
                sad = tss(search_para, dst_x, dst_y, src_x, src_y, bounds_x_left, bounds_x_right, bounds_y_up,
                          bounds_y_down, mb_size, dst_img, src_mb, per_block_ops);
            } else {
                sad = es(dst_x, dst_y, src_x, src_y, bounds_x_left, bounds_x_right, bounds_y_up, bounds_y_down, mb_size,
                         dst_img, src_mb, per_block_ops);
            }

            // MV always points from frame N to frame N - 1, independent of the search
            //  direction (to simply the implementation in the interpolation code).
            //  That is, <x, y> in frame N adds mv equal <x, y> in frame N - 1.
            //  In video codec though, MV always is dst - src (same as BW case here).
            if (isBackward) {
                int mv_x = dst_x - src_x;
                int mv_y = dst_y - src_y;
                motion_vector.push_back(Rect(src_x + mb_size / 2, src_y + mb_size / 2, mv_x, mv_y));
            } else {
                int mv_x = src_x - dst_x;
                int mv_y = src_y - dst_y;
                motion_vector.push_back(Rect(dst_x + mb_size / 2, dst_y + mb_size / 2, mv_x, mv_y));
            }
            confidence.push_back(1 - (double) sad / MAX_SAD);

            assert(confidence.size() == motion_vector.size());
            //fprintf(stdout, "block (%d, %d): %u searches\n", i, j, per_block_ops);
        }
    }
}

int main(int argc, char **argv) {
    const String keys =
    "{@input     |       | path to input image files          }"
    "{@output    |       | path to output mv file             }"
    "{@start     |       | start frame number                 }"
    "{@end       |       | end frame number                   }"
    "{if_gap     |  20   | window size for an I frame         }"
    "{bits       |  4    | number of bits in image file names }"
    "{mb_size    |  16   | macroblock size                    }"
    "{format     | jpg   | image format                       }"
    "{mode       | tss   | search mode: tss or es             }"
    "{direction  | bw    | search direction: fw or bw         }"
    "{conf       | false | dump confidence value also?        }";

//    char buf[1000];
//    fprintf(stderr, "%s\n", getcwd(buf, 1000));

    cv::CommandLineParser parser(argc, argv, keys);
    string out_file = parser.get<string>("@output");
    string in_dir = parser.get<string>("@input");
    string format = parser.get<string>("format");
    int if_winsize = parser.get<int>("if_gap");
    int bits = parser.get<int>("bits");
    int mb_size = parser.get<int>("mb_size");
    MAX_SAD = (double) (255 * mb_size * mb_size);
    bool conf = parser.get<bool>("conf");

    string mode = parser.get<string>("mode");
    if (mode != "tss" && mode != "es") {
        fprintf(stderr, "Wrong BM Mode!\n");
        return 0;
    }

    // Forward search iterates through each MB in frame N - 1 and finds
    //  its best match in frame N while Backward search iterates through
    //  each MB in frame N and finds its best match in frame N - 1.
    // BW is the default approach. It is also how BM is typically done
    //  such as in video codecs. But it requires us to convert the backward
    //  MV (point from N to N - 1) to forward MV (from N - 1 to N) before
    //  interpolation. FW makes interpolation a bit more intuitive, but its
    //  acuracy is a bit worse from what I have seen so far.
    string direction = parser.get<string>("direction");
    if (direction != "fw" && direction != "bw") {
        fprintf(stderr, "Wrong Search Direction!\n");
        return 0;
    }
    bool isBackward = (direction == "bw");

    FILE *outfile = fopen(out_file.c_str(), "w");
    if (!outfile) {
        fprintf(stderr, "Can't open output mv file\n");
        return 0;
    }

    // TODO default name of input_stream
    string input_stream = in_dir + "%0" + to_string(bits) + "d." + format;
    //printf("test input_stream: %s\n", input_stream.c_str());
    VideoCapture in_capture(input_stream);
    int frameCounts = (int) in_capture.get(CAP_PROP_FRAME_COUNT);
    int start = parser.has("@start") ? parser.get<int>("@start") : 1;
    int end = parser.has("@end") ? end = parser.get<int>("@end") : frameCounts;

    int frameWidth = in_capture.get(CAP_PROP_FRAME_WIDTH);
    int frameHeight = in_capture.get(CAP_PROP_FRAME_HEIGHT);

    fprintf(stdout, "generate MVs between [%d, %d] for images %d x %d with mb_size %d\n", start, end, frameWidth,
            frameHeight, mb_size);

    Mat img_rgb, img_yuv, prev_img_rgb, cur_y, prev_y;
    vector<Rect> motion_vector;
    vector<float> confidence;
    vector<Mat> channels(3);

    for (int i = 1; i < start; i++) {
        in_capture >> img_rgb;
    }

    int frame_id = 0;
    bool isIframe = false;
    for (int i = start; i <= end; i++) {
        in_capture >> img_rgb;
        if (img_rgb.empty()) break;

        frame_id++;

        isIframe = ((frame_id - 1) % if_winsize) == 0;

        // In reality people would use YUV420, but since we only do BM
        // on the Y channel 444 is fine here.
        cvtColor(img_rgb, img_yuv, COLOR_BGR2YUV);
        split(img_yuv, channels);
        cur_y = channels[0];

        if (isIframe) {
            fprintf(outfile, "# %d I\n", frame_id);
            fprintf(stdout, "# %d I\n", frame_id);
        } else {
            // block matching in the Y channel only
            if (isBackward)
                blockmatching(cur_y, prev_y, mb_size, motion_vector, confidence, mode, isBackward);
            else
                blockmatching(prev_y, cur_y, mb_size, motion_vector, confidence, mode, isBackward);

            fprintf(outfile, "# %d P\n", frame_id);

            // TODO might need to draw or print something?
            for (int i = 0; i < motion_vector.size(); i++) {
                Rect rect = motion_vector[i];
                //fprintf(outfile, "%d\t%d\t%d\t%d\n", rect.x + rect.width, rect.y + rect.height, rect.width, rect.height);
                if (!conf)
                    fprintf(outfile, "%d\t%d\t%d\t%d\n", rect.x, rect.y, rect.width, rect.height);
                else
                    fprintf(outfile, "%d\t%d\t%d\t%d\t%.04lf\n", rect.x, rect.y, rect.width, rect.height,
                            confidence[i]);

                // Draw motion vectors
                //arrowedLine(prev_img_rgb, Point(rect.x, rect.y), Point(rect.x + rect.width, rect.y + rect.height), cv::Scalar(255,255,255), 1);
            }
            //gen_images(frame_id - 1, frameCounts, &prev_img_rgb);
        }

        // TODO: optimize this using double buffer or pointers.
        prev_y = cur_y.clone();
        //prev_image_rgb = img_rgb.close();
        motion_vector.clear();
        confidence.clear();

        //fprintf(stdout, "Total OPs: %llu\n", total_ops);
        //total_ops = 0;
        //if (i == start + 1) break;
    }

    fclose(outfile);

    return 0;
}
