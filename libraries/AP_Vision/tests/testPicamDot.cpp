

#include <stdio.h>
#include <unistd.h> // usleep
#include "Vision.h"
#include "TimeStatistics.h"

using namespace std;

int main ( int argc,char **argv ) {
    //static cv::Mat mat;
    bool ans;
    int sucsses = 0;
    //ip_dot dot;
    const char* prefix = "out-";

    Vision vision;

    /* statistics */
    of_stat_t retrive_stat, total_stat, save_stat, optic_stat;
    STAT_INIT(retrive_stat, "retrive_stat");
    STAT_INIT(total_stat, "total_stat");
    STAT_INIT(save_stat, "save_stat");
    STAT_INIT(optic_stat, "optic_stat");

    int iteration = 0;
    int nCount = 100;
    char filename[50];

    if (argc > 1){
        prefix = argv[1];
    }
    if (argc > 2){
        nCount = atoi(argv[2]);
    }

    // initialization
    vision.init(960, 1280);
    for (int i=0 ; i<4 ; i++){
        printf("x%d , y%d,", i, i);
    }
    printf("\n");

    while(iteration < nCount)
    {
        STATS_START(total_stat);
        STATS_START(optic_stat);
        while (!(ans = vision.update())) {
            usleep(1);
        STATS_START(optic_stat);
        }
        STATS_END(optic_stat);

        // have new frame
        if (iteration % 10 == 0 || 1){
            printf("iterations: %d\n", iteration);
        }

        static ip_point points[4];

        if (vision.healthy()){
            sucsses++;
#define SAVE 1
#if SAVE
            //Writing the image
            STATS_START(save_stat);
            cv::Mat * mat = vision.getImage();

            int npoints = vision.getBestPoints(points, 4, 0);
            for (int i=0 ; i<npoints ; i++){
                cv::circle( *mat,
                        cv::Point( points[i].x, points[i].y),
                        points[i].radius,
                        255,
                        1);
                mat->at<uchar>(points[i].x, points[i].y) = 255;
            }

            sprintf(filename, "outMarked-%d.jpg", iteration);
            cv::imwrite(filename, *mat);
            STATS_END(save_stat);
#endif
        }

        for (int i=0 ; i<4 ; i++){
            printf("%d , %d,", points[i].x, points[i].y);
        }
        printf("\n");

        iteration++;
        STATS_END(total_stat);

    }

    STATS_SHOW(optic_stat);
    STATS_SHOW(save_stat);
    printf("%3.2f  ", (float)sucsses/(float)nCount );
    STATS_SHOW(total_stat);
    //printf("nDots=%d, dots[0]: x=%d, y=%d, radius=%d\n", ipData.nDots, dot.x, dot.y, dot.radius);
    //printf("nCount=%d, fps:%f\n", nCount, (nCount*1000.0)/total_stat.total_us);




    return 0;

}
