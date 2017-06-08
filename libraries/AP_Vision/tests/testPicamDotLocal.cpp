

#include <stdio.h>
#include <unistd.h> // usleep
#include "Vision.h"

using namespace std;


bool retrive(cv::Mat& mat, const char* prefix){
    static int id=0;

    char filename[50];
    sprintf(filename, "%s%d.jpg", prefix, id++);

    mat = cv::imread(filename, cv::IMREAD_GRAYSCALE);
    return true;
}

int main ( int argc,char **argv ) {
    static cv::Mat mat;
    bool ans;
    ip_data ipData[5];
    int sucsses[5];
    ip_dot dot;
    const char* prefix = "out-";

    /* statistics */
    of_stat_t retrive_stat, total_stat, save_stat, optic_stat[5];
    STAT_INIT(retrive_stat, "retrive_stat");
    STAT_INIT(total_stat, "total_stat");
    STAT_INIT(save_stat, "save_stat");
    for (int i=0; i<5; i++) {
        STAT_INIT(optic_stat[i], "optic_stat");
    }

    int iteration = 0;
    int nCount = 100;
    char filename[50];

    if (argc > 1){
        prefix = argv[1];
    }
    if (argc > 2){
        nCount = atoi(argv[2]);
    }



    while(iteration < nCount)
    {
    STATS_START(total_stat);
        STATS_START(retrive_stat);
        while (!(ans = retrive(mat, prefix))) {
            usleep(1);
            STATS_START(retrive_stat);
        }
        // have new frame
        printf("res:%d x %d\n", mat.cols, mat.rows);
        STATS_END(retrive_stat);
        if (iteration % 10 == 0 || 1){
            printf("retrive .. %d\n", iteration);
        }

#if PRINT_MAT
        for(int i =0 ; i<mat.rows ; i+=10){
            for (int j = 0; j<mat.cols ; j+=10){
                printf(" %3hhu", mat.at<uchar>(i,j));
            }
            printf("\n");
        }
        printf("\n");
#endif
#if WRITE
        //Writing the image before image processing
        STATS_START(save_stat);
        sprintf(filename, "out-%d-B.jpg", iteration);
        //cv::imwrite(filename, mat);
        STATS_END(save_stat);
#endif

        for (int i=0; i<5; i++) {
            if (iteration == 0){
                sucsses[i] = 0;
            }
            cv::Mat mat2;
            mat2.create ( mat.rows, mat.cols, CV_8UC1);
            memcpy(mat2.data, mat.data, mat2.cols*mat2.rows);

            STATS_START(optic_stat[i]);
            ipData[i].dots.clear();
            ipData[i].nDots=0;
            findDots(mat2, ipData[i],i+1);
            STATS_END(optic_stat[i]);

            if (ipData[i].nDots != 1){
                ipData[i].nDots=0;
            }
            printf("%d,", ipData[i].nDots);
            sucsses[i] += ipData[i].nDots;
        }
        printf("\n");

#if SAVE
        //Writing the image
        STATS_START(save_stat);
        while (!ipData.dots.empty()){
            dot = ipData.dots.back();
            ipData.dots.pop_back();

            cv::circle( mat,
                    cv::Point( dot.x, dot.y),
                    dot.radius,
                    255,
                    1);
        }

        sprintf(filename, "out-%d:[n%d].jpg", iteration, ipData.nDots);
        //cv::imwrite(filename, mat);
        STATS_END(save_stat);
#endif
        iteration++;
        STATS_END(total_stat);

    }



    STATS_SHOW(retrive_stat);
    //STATS_SHOW(save_stat);
    for (int i=0; i<5; i++) {
        printf("%3.2f  ", (float)sucsses[i]/(float)nCount );
        STATS_SHOW(optic_stat[i]);
    }
    //STATS_SHOW(total_stat);
    //printf("nDots=%d, dots[0]: x=%d, y=%d, radius=%d\n", ipData.nDots, dot.x, dot.y, dot.radius);
    //printf("nCount=%d, fps:%f\n", nCount, (nCount*1000.0)/total_stat.total_us);




    return 0;

}
