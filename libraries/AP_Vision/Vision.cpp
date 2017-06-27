
#include "Vision.h"


using namespace std;

static uint32_t iterationStart_us;
static int computationStatus; // 0 -> done, 1 -> only retrieve done, 2-> middle of findDots

#define MICROS() AP_HAL::micros()

struct vision_stat_t{
    bool initialized = false;

    const char* name;
    uint32_t iterations;
    uint32_t total_us;
    uint32_t max_us;
    float avg;

    /* temp */
    volatile uint32_t begin;
};

void STATS_SHOW(struct vision_stat_t& stat){
    if (stat.initialized){
        printf("*** stat - %s: avg=%10.5f(us), max=%d(us), count=%d\n", stat.name, stat.avg, stat.max_us, stat.iterations);
    } else {
        printf("*** stat - not Initialized\n");
    }
}

void STAT_INIT(struct vision_stat_t& stat, char* name){
        stat.initialized = true;
        stat.iterations = 0;
        stat.max_us = 0;
        stat.total_us = 0;
        stat.avg = 0;
        stat.name = name;
}

void STAT_ZERO(struct vision_stat_t& stat){
        stat.initialized = true;
        stat.iterations = 0;
        stat.max_us = 0;
        stat.total_us = 0;
        stat.avg = 0;
}

void STATS_START(struct vision_stat_t& stat){
    if (!stat.initialized){
        STAT_INIT(stat, "noName");
    }
    stat.begin = MICROS();
}
void STATS_END(struct vision_stat_t& stat){
    int dif = MICROS() - stat.begin;
    if(dif < 0){
        return;
    }
    if (stat.begin != 0){
        stat.iterations++;
        stat.total_us += (uint32_t)dif;
        stat.avg = (float)stat.total_us / stat.iterations;
        if (stat.max_us < (uint32_t)dif){
            stat.max_us = (uint32_t)dif;
        }
    }
    stat.begin = 0;
}


/* statistics */
static vision_stat_t retrive_stat, stat_update_iter, optic_stat;

// constructor
Vision::Vision() : _last_update_ms(0), _height(0), _width(0){
    computationStatus = 0;
}

void Vision::init(int height, int width) {
    _height = height;
    _width = width;
    if (_camDriv.startCapture(90, height, width)){
        printf("start capturing at %d x %d \n", height, width);
        _flags.cameraError = false;
    } else {
        printf("Error - fail start capturing \n");
        _flags.cameraError = true;
    }
    _flags.healthy = false; // no data

    /* statistics */
    STAT_INIT(retrive_stat, "retrive_stat");
    STAT_INIT(stat_update_iter, "update iter stat");
    STAT_INIT(optic_stat, "optic_stat");
}

#if 0
void Vision::setResolution(int height, int width){
    if (_height != height || _width != width){
        _height = height;
        _width = width;
        //_camDriv.setResolution(height, width);
    }
}
#endif

/*
 * factor of 240X320
 */
bool Vision::setResolution(int height, int width){
    if ((height % 240 != 0) || (width % 320 != 0) ){
        return false;
    }
    if (height == _height || width == _width){
        return true;
    }

    _height = height;
    _width = width;

    if (_camDriv.setResolution(height, width)){
        _flags.cameraError = false;
        return true;
    } else {
        _flags.cameraError = true;
        return false;
    }
}

bool Vision::update() {
    bool ans;
    //static bool working = false;

    STATS_START(stat_update_iter);
    iterationStart_us = MICROS();

    //if(working){
    //    return false;
    //}
    //working = true;
    if (0 == computationStatus){
        STATS_START(retrive_stat);
        if (!(ans = _camDriv.retrive(_mat, true))) {
            //working = false;
            return false;
        }
        STATS_END(retrive_stat);
        computationStatus = 1;
        if ( (MICROS() - iterationStart_us) > 2000){
            // too much time, lets wait to next iteration
            STATS_END(stat_update_iter);
            return false;
        }
    }

    // have new frame
#if 0
    for(int i =0 ; i<mat.rows ; i+=10){
        for (int j = 0; j<mat.cols ; j+=10){
            printf(" %3hhu", mat.at<uchar>(i,j));
        }
        printf("\n");
    }
    printf("\n");
#endif

    if (1 == computationStatus){
        // prepare
        _data.points.clear();
        _data.nPoints=0;
        _data.height= _mat.rows;
    }

    STATS_START(optic_stat);
    findDots(_mat, _data);
    STATS_END(optic_stat);

    if (0 != computationStatus){
        // findDots didn't finish
        STATS_END(stat_update_iter);
        return false;
    }

    _last_update_ms = AP_HAL::millis();
    //if (_data.nPoints <= 0 || _data.nPoints > 6) {
    if (_data.nPoints >= 2) {
        _flags.healthy = true;
    } else {
        _flags.healthy = false;
    }

    STATS_END(stat_update_iter);

    //working = false;
    return true;
}

int Vision::getBestPoints(struct ip_point* points, int count, bool regulate){
    int foundCount = 0;
    int minMass = 10000;

    if (!healthy()) {
        return -1;
    }

    for (int i=0 ; i<_data.nPoints ; i++) {
        if (minMass > _data.points[i].mass){
            minMass = _data.points[i].mass;
        }
    }

    if (_data.height < 240){
        _data.height = 240;
    }

    for (int i=0 ; i<_data.nPoints && foundCount < count ; i++) {
        if (_data.points[i].mass <= (minMass*2) ){
            points[foundCount] = _data.points[i];
            if(regulate){
                points[foundCount].x = ( points[foundCount].x / (float)(_data.height/2)) - 1.0f;
                points[foundCount].y = ( points[foundCount].y / (float)(_data.height/2)) - 1.0f;
            }
            foundCount++;
        }
    }


    return foundCount;

}

void Vision::showStatistics(){
    STATS_SHOW(retrive_stat);
    STATS_SHOW(optic_stat);
    STATS_SHOW(stat_update_iter);
}

void Vision::initStatistics(){
    STAT_ZERO(retrive_stat);
    STAT_ZERO(optic_stat);
    STAT_ZERO(stat_update_iter);
}


#define MAX_DEPTH 100 // for preventing stack overflow
#define MAX_MASS 2000 // for preventing stack overflow

static uint64_t dfs_x_sum;
static uint64_t dfs_y_sum;

int Vision::dfs(cv::Mat& mat, int x, int y, uchar trashold, /*int depth,*/ int prevCount) {
    int count = 0;

    count++; // for self pixel
    dfs_x_sum += x;
    dfs_y_sum += y;
    mat.at<uchar>(y,x) = 0; // mark self pixel
/*
    if (depth > MAX_DEPTH){
        return 1;
    }
*/
    if (prevCount > MAX_MASS){
        return 1;
    }

    if (x < 1 || x >= (mat.cols-1) || y < 1 || y >= (mat.rows-1)){
        return 1;
    }
    //if (mat.at<uchar>(y,x) > trashold){

    // vertical & horizontal
    if (mat.at<uchar>(y-1,x) >= trashold) count += dfs(mat, x, y-1, trashold, /*depth,*/ count + prevCount);
    if (mat.at<uchar>(y+1,x) >= trashold) count += dfs(mat, x, y+1, trashold, /*depth,*/ count + prevCount);
    if (mat.at<uchar>(y,x-1) >= trashold) count += dfs(mat, x-1, y, trashold, /*depth,*/ count + prevCount);
    if (mat.at<uchar>(y,x+1) >= trashold) count += dfs(mat, x+1, y, trashold, /*depth,*/ count + prevCount);

#if 0
    if (mat.at<uchar>(y-1,x-1) >= trashold) count += dfs(mat, x-1, y-1, trashold, depth);
    if (mat.at<uchar>(y+1,x+1) >= trashold) count += dfs(mat, x+1, y+1, trashold, depth);
    if (mat.at<uchar>(y+1,x-1) >= trashold) count += dfs(mat, x-1, y+1, trashold, depth);
    if (mat.at<uchar>(y-1,x+1) >= trashold) count += dfs(mat, x+1, y-1, trashold, depth);
#endif
    return count;
}

void Vision::findDots(cv::Mat& mat, struct ip_data& ipData){

    uchar avgPixel;
    uchar maxPixel=150;
    uint64_t totPixel=0;
    uchar trashold;

    uchar* data = mat.data;
#if 0
    int msize = mat.rows*mat.cols;
    // 320*250 2.5ms
    for(int i =0 ; i<msize ; i+= 1){
        uchar pixel = data[i];
        if ( pixel > maxPixel ){
            maxPixel = pixel;
        }
        totPixel += pixel;
    }
    avgPixel = totPixel/msize;
    trashold = (avgPixel + maxPixel*3) / 4;
    //printf("optic pixel avg=%d, max=%d, trash=%d\n", avgPixel, maxPixel, trashold);
#else
    trashold = 0xF0;
#endif

    int minDotMass = pow(mat.rows , 2) / 7680; // 120; // TODO in 960p minimum is 150
    int maxDotMass = pow(mat.rows , 2) / 800;  // 1000-1300; // TODO need to check
    //int minDotMass = 30; // TODO in 960p minimum is 150
    //int maxDotMass = 2000;  // 1000; // TODO need to check

#if 0
    for(int y =0 ; y<mat.rows ; y+= 1){
        for (int x = 0; x<mat.cols ; x+= 1){
            uchar pixel = mat.at<uchar>(y,x);
            if ( pixel > maxPixel ){
                maxPixel = pixel;
            }
            totPixel += pixel;
        }
    }
#endif

    static int y;

    if (1 == computationStatus){
        // starting with new image
        y = 0;
    }
    computationStatus = 2;

    int cols = mat.cols;
    for( ; y<mat.rows ; y+=1){
        if ((0 == y%1) && (MICROS() - iterationStart_us) > 2000 ){
            // too much time, wait for next iteration
            return;
        }
        for (int x=0; x<mat.cols ; x+=1){
            //if (mat.at<uchar>(y,x) > trashold){
            if (data[y*cols+x] >= trashold){
                dfs_x_sum = 0;
                dfs_y_sum = 0;
                int sol = dfs(mat, x, y, trashold*0.5);
                //printf("sol before = %d\n", sol);
                if (minDotMass < sol && sol < maxDotMass){
                    //printf("sol after = %d\n", sol);
                    //printf("dfs_x_sum/sol = %lld/%d = %lld\n", dfs_x_sum, sol,dfs_x_sum/sol);
                    //printf("sol afterafter = %d\n", sol);
                    // found dot
                    ip_point dot;
                    dot.mass = sol;
                    dot.radius = sqrt(dot.mass/3.14f);
                    //dot.x = x;
                    //dot.y = y + dot.radius;
                    dot.x = dfs_x_sum/sol;
                    dot.y = dfs_y_sum/sol;
                    ipData.points.push_back(dot);
                    ipData.nPoints++;
                }
            }
        }
    }

    computationStatus = 0; // computation done

}
