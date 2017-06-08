/*
 * MyLogger.h
 *
 *  Created on: Apr 6, 2017
 *      Author: hodai
 */

#ifndef ARDUCOPTER_MYLOGGER_H_
#define ARDUCOPTER_MYLOGGER_H_


#include <fcntl.h> // open
#include <unistd.h>  // open
#define MAX_VALUES 100

#define PRINT_VAL(loger, vid, vname)\
    static int vname = vid;\
    loger.printVal(vid, #vname);

class MyLogger{
    int fd;
    bool ready;
    const char* names[MAX_VALUES];
    bool printValus[MAX_VALUES];
    double values[MAX_VALUES];
public:
    MyLogger(const char* filename): fd(0), ready(false){
        fd = open(filename, O_WRONLY | O_CREAT | O_TRUNC, 0644);
        if(fd == -1){
            perror("open");
            fd = 0;
        } else {
            ready = true;
        }
        for (int i=0 ; i<MAX_VALUES ; i++ ){
            printValus[i] = false;
            names[i] = 0;
        }
    }

    ~MyLogger(){
        closeFile();
    }
    void closeFile(){
        if (fd != 0){
            close (fd);
            fd = 0;
            ready = false;
        }
    }

    void putVal(int id, double value){
        if (id < MAX_VALUES){
            values[id] = value;
        }
    }

    void printVal(int id, const char* name="no-name", bool val=true){
        if (id < MAX_VALUES){
            printValus[id] = val;
            values[id] = 0;
            names[id] = name;
        }
    }

    void flashLine(){
        if (!ready) return;

        static bool firstTime = true;
        if (firstTime){
            firstTime = false;
            flashTitle();
        }

        for (int i=0 ; i<MAX_VALUES ; i++ ){
            if (printValus[i]){
                dprintf(fd, "%.5f,", values[i]);
            }
        }
        dprintf(fd, "\n");
    }

    void flashTitle(){
        for (int i=0 ; i<MAX_VALUES ; i++ ){
            if (printValus[i] && names[i]){
                dprintf(fd, "%s,", names[i]);
            }
        }
        dprintf(fd, "\n");
    }

};





#endif /* ARDUCOPTER_MYLOGGER_H_ */
