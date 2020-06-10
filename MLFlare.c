//
//  MLFlare.c
//  MLFlare
//
//  Created by Jackie Wang on 2019/7/2.
//  Copyright © 2019 Jackie Wang. All rights reserved.
//
//

//#define NDEBUG

#include "MLFlare.h"
#include "LTSMC.h"
#include "MLConfiger.h"
#include "serial.h"
#include "byte_fifo.h"
#include "DLRS1A.h"
#include "MLExpection.h"
#include "JKCL200A.h"
#include "XJC608T.h"
#include "SCA126T.h"

#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <unistd.h>
#include <stdarg.h>
#include <time.h>
#include <limits.h>
#include <pthread.h>
#include <math.h>
#include <unistd.h>
#include <sys/timeb.h>
#include <stdio.h>
#include <time.h>
#include <dirent.h>
// Motion API Handle
typedef int MLMotionHandle;

typedef struct {
    void *arg1;
    double arg2;
    bool arg3;
    void *arg4;
} TArgs;

#define FIFO_DEBUG(...)

#define MLAbsolute  1       // 绝对运动模式
#define MLRelative  0       // 相对运动模式

//#undef  _ML_ASSERT_TEST_      //禁用
#define _ML_ASSERT_TEST_        //启用

#ifdef _ML_ASSERT_TEST_         //启用断言测试
void assert_report( const char * file_name, const char * function_name, unsigned int line_no ) {
    printf( "\n[MLFlare]Error Report file_name: %s, function_name: %s, line %u\n", file_name, function_name, line_no );
}
#define ASSERT_REPORT( condition )       \
do {       \
    if ( condition )       \
        NULL;        \
    else         \
        assert_report( __FILE__, __func__, __LINE__ ); \
} while(0)
#else // 禁用断言测试
#define ASSERT_REPORT( condition )  NULL
#endif /* end of ASSERT */

typedef struct {
    float lv;      // lux
    float x;       // color.x
    float y;       // color.y
    float u;
    float v;
    float X;       // RGB Color X
    float Y;
    float Z;
} MLLuxMeterVal;

pthread_mutex_t _mutex;

// global Variables
static MLMotionHandle gHandle;
static AxisParam gAxisPrm[MLMaxAxisCount];
static MLDUT *duts;
static int dutCount;

static bool gStopped;
static bool gIsTesting;
static bool gIsRaster;
static bool gLuxConnected;

static bool doorOpened;
static bool doorClosing;
static bool lightTestReadyed;

static bool gIsPowerOn;

static bool gIsOpenDoor;
static bool gIsCloseDoor;
static bool gIsStartTest;
static bool gIsEmgStopTest;
static bool gIsStopTest;

string leftCellPortName;    // loadcell left
string rightCellPortName;   // loadcell right
string frontCellPortName;   // loadcell front
string backCellPortName;    // loadcell backre

string anglePortName; // Illuminometer
string illuminometerPortName; // Illuminometer
string laserPortName;       // laser
MLDUT dutPrm;

string errmsg;

static bool bCalibrated;
static MLLuxMeterVal *calLuxVals;

static bool isConnectedLoadCell = false;
static bool isNeedStop = false;
static bool isNeedEmgStop = false;
static bool isActiveFinished = false;

void MoveToInitPos(int *axises, int count);
void ResetAllAxis(bool pthread);

// ========================= private method ============================
char* CreateLogPath(char *path)  {
    char *flareLogPath;
    char cmd[128];
    char *home = getenv("HOME");
    
    flareLogPath = (char*)malloc(strlen(home) + strlen(path) + 1);
    strcat(flareLogPath, home);
    strcat(flareLogPath, path);
    
    int error = access(flareLogPath, F_OK);
    
    if (error != 0) {
    // create log path, create intermediate directories as required.
        strcpy(cmd, "mkdir -p ");
        strcat(cmd, flareLogPath);
        system(cmd);
    }
    
    return flareLogPath;
}

void Logger(MLLogLevel type, char* message, ...) {
    //create log path
    char* path = CreateLogPath("/Documents/Flare/Logs/");
    
    char logString[1024];
    memset(logString, 0, 1024);
    va_list args;
    va_start(args, message);
    vsnprintf(logString, 1024, message, args);
    va_end(args);
    
    // current time.
    char tips[64];
    char folder[64];
    memset(tips, 0, 64);
    memset(folder, 0, 64);
    
    struct tm *tmPtr = NULL;
    struct timeb stTimeb;
    
    ftime(&stTimeb);
    tmPtr  = localtime(&stTimeb.time);
    sprintf(tips, "[%04d-%02d-%02d %02d:%02d:%02d:%03d] ", (1900+tmPtr->tm_year), (1+tmPtr->tm_mon), tmPtr->tm_mday, tmPtr->tm_hour, tmPtr->tm_min, tmPtr->tm_sec, stTimeb.millitm);
    sprintf(folder, "%s/%04d_%02d_%02d/", path, (1900+tmPtr->tm_year), (1+tmPtr->tm_mon), tmPtr->tm_mday);
    
    char result[1024+32];
    memset(result, 0, 1024+32);
    // write logString to file.
    switch (type) {
        case MLLogInfo: {
            strcat(tips, "[Info]: ");
            strcat(result, tips);
            strcat(result, logString);
            break;
        }
        case MLLogWarning: {
            strcat(tips, "[Warning]: ");
            strcat(result, tips);
            strcat(result, logString);
            break;
        }
        case MLLogError: {
            strcat(tips, "[Error]: ");
            strcat(result, tips);
            strcat(result, logString);
            break;
        }
        default:
            break;
    }
    
    FILE *logFile = NULL;
    FILE *errorLogFile = NULL;
    
    char *errFile = malloc(64);
    strcpy(errFile, folder);
    strcat(errFile, "error.log");
    
    // create folder.
    if (access(folder, 0)) {
        char *cmd = (char*)malloc(256);
        strcpy(cmd, "mkdir -p ");
        strcat(cmd, folder);
        system(cmd);
        free(cmd);
        cmd = NULL;
    }
    
    strcat(folder, "process.log");
    
    logFile = fopen(folder, "a+");
    
    if (logFile) {
        fputs(result, logFile);
        fclose(logFile);
    }
    
    // append error message.
    if (type == MLLogError) {
        errorLogFile = fopen(errFile, "a+");
        if (errorLogFile) {
            fputs(result, errorLogFile);
            fclose(errorLogFile);
        }
    }
    
    // destory malloc buffer
    free(errFile);
    free(path);
    errFile = NULL;
    path = NULL;
}

void LoadDefaultProfile() {
    SetIniFileName(MLConfigFileName);
//    double startSpeed = GetDoubleValue("axis_0", "start_speed");
}

int GetCountOfPtr(MLDUT *buffer) {
    int currentDUTTypeCnt = 0;
    MLDUT *p;
    p = buffer;
    
    while (p->name != NULL) {
        currentDUTTypeCnt++;
        p++;
    }
    
    return currentDUTTypeCnt;
}

void ReleaseSysResource() {
//    int count = GetCountOfPtr(duts);
    for (int axis = 0; axis < MLMaxAxisCount; axis++) {
        AxisParam param = gAxisPrm[axis];
        
        char *title = (char *)malloc(32);
        sprintf(title, "axis_%d", axis);
        PutDoubleValue(title, "start_speed", param.startSpeed);
        PutDoubleValue(title, "run_speed",   param.runSpeed);
        PutDoubleValue(title, "stop_speed",  param.stopSpeed);
        PutDoubleValue(title, "home_speed",  param.homeSpeed);
        PutDoubleValue(title, "acc_time",    param.accTime);
        PutIntValue(title,    "home_dir",    param.homeDirect);
        PutIntValue(title,    "home_level",  param.homeLevel);
        PutDoubleValue(title, "pp_ratio",    param.ppratio);
        PutDoubleValue(title, "initialize_positon", param.posToInit);
        PutDoubleValue(title, "test_position", param.posToTest);
        
        free(title);
        title = NULL;
    }
    ConfigLoadCellPortName(frontCellPortName, backCellPortName, leftCellPortName, rightCellPortName);
    ConfigLaserPortName(laserPortName);
    ConfigIlluminometerPortName(illuminometerPortName);
    ConfigAnglePortName(anglePortName);

}

void SetEncoderUnit() {
    short ret = 0;
    short res = 0;
    
    WORD MyCardNo = 0;
    WORD Mymode = 3;
    
    for (int i = 1; i <= 12; i++) {
        ret=smc_set_counter_inmode(MyCardNo,i,Mymode);//设置编码器计数方式
        res=smc_set_encoder_unit(MyCardNo,i,100); //设置编码值为 100
    }
}

void Encoder() {
    /*********************变量定义****************************/
    short ret = 0;
    short res = 0;
    //错误返回
    WORD MyCardNo = 0;
    WORD Myaxis = 9;
    WORD Mymode = 3;
    double Myencoder_value=0;
    /*********************函数调用执行**************************/ //第一步、设置 0 号轴的编码器计数方式
    ret=smc_set_counter_inmode(MyCardNo,Myaxis,Mymode); //第二步、设置 0 轴的编码值为 100
    res=smc_set_encoder_unit(MyCardNo,Myaxis,100); //第三步、读轴 0 轴的编码值
    res=smc_get_encoder_unit(MyCardNo,Myaxis,&Myencoder_value);
}

double retain3decimals(double value) {
    return (long)((value+0.0005) * 1000) / 1000.0;
}

char* GetProjectName(){
    
    char dirname[128];
    char *filename[1024];
    char *homepath = getenv("HOME");
    sprintf(dirname, "%s/Documents/MATLAB/project", homepath);
    
    int i = 0;
    DIR *dirp;
    struct dirent *dp;
    dirp = opendir(dirname); //打开目录指针
    while ((dp = readdir(dirp)) != NULL) { //通过目录指针读目录
        printf("%s\n", dp->d_name );
        i++;
        filename[i] = dp->d_name;
    }
    (void) closedir(dirp);
    
    return filename;
}

void InitializeSystem() {
    // free malloc objects.
    if (duts != NULL) { free(duts); duts = NULL; }
    if (rightCellPortName != NULL) { free(rightCellPortName); rightCellPortName = NULL; }
    if (frontCellPortName != NULL) { free(frontCellPortName); frontCellPortName = NULL; }
    if (laserPortName != NULL) { free(laserPortName); laserPortName = NULL; }
    if (illuminometerPortName != NULL) { free(illuminometerPortName); illuminometerPortName = NULL; }
    if (anglePortName != NULL) { free(anglePortName); anglePortName = NULL; }

    errmsg = (string)malloc(256);
    
    char filename[128];
    char *homepath = getenv("HOME");
    sprintf(filename, "%s/Documents/Flare/profile.ini", homepath);
    CreateLogPath("/Documents/Flare/");
    SetIniFileName(filename);
    
    int dutTypeCnt = GetIntValue("syscfg", "dut_type_count");
    if (dutTypeCnt <= 0) {
        PutIntValue("syscfg", "dut_type_count", 0);
        dutTypeCnt = 0;
    }
    
    dutCount = dutTypeCnt;
    duts = (MLDUT *)malloc(MLMaxDUTTypeCount * sizeof(MLDUT) + 1);
    
    for (int cnt = 0; cnt < dutTypeCnt; cnt++) {
        char *title = (char *)malloc(32);
//        sprintf(title, "%s", "DUTNAME");
        sprintf(title, "duttype_%d", cnt);
        char *name = (char *)malloc(64);
        memset(name, 0, 64);
        dutPrm.type = GetIntValue(title, "type");   if (dutPrm.type == INT_MIN) { PutIntValue(title, "type", 0); }
        GetStringValue(title, "name", name);
        dutPrm.name = name;                         if (0 == strcmp(dutPrm.name, "")) { PutStringValue(title, "name", ""); }
        dutPrm.posStandbyFront = GetDoubleValue(title, "front_standby_position");     if (dutPrm.posStandbyFront == LONG_MIN) { PutLongValue(title, "front_standby_position", 0); }
        dutPrm.posStandbyBack = GetDoubleValue(title, "back_standby_position");       if (dutPrm.posStandbyBack == LONG_MIN) { PutLongValue(title, "back_standby_position", 0); }
        dutPrm.posStandbyLeft = GetDoubleValue(title, "left_standby_position");       if (dutPrm.posStandbyLeft == LONG_MIN) { PutLongValue(title, "left_standby_position", 0); }
        dutPrm.posStandbyRight = GetDoubleValue(title, "right_standby_position");     if (dutPrm.posStandbyRight == LONG_MIN) { PutLongValue(title, "right_standby_position", 0); }
        dutPrm.posHoldFront = GetDoubleValue(title, "front_hold_position");   if (dutPrm.posHoldFront == LONG_MIN) { PutLongValue(title, "front_hold_position", 0); }
        dutPrm.posHoldBack = GetDoubleValue(title, "back_hold_position");     if (dutPrm.posHoldBack == LONG_MIN) { PutLongValue(title, "back_hold_position", 0); }
        dutPrm.posHoldLeft = GetDoubleValue(title, "left_hold_position");     if (dutPrm.posHoldLeft == LONG_MIN) { PutLongValue(title, "left_hold_position", 0); }
        dutPrm.posHoldRight = GetDoubleValue(title, "right_hold_position");   if (dutPrm.posHoldRight == LONG_MIN) { PutLongValue(title, "right_hold_position", 0); }
        dutPrm.posLifter = GetDoubleValue(title, "lifter_position");          if (dutPrm.posLifter == LONG_MIN) { PutLongValue(title, "lifter_position", 0); }
        dutPrm.posAxisX = GetDoubleValue(title, "axisX_position");          if (dutPrm.posAxisX == LONG_MIN) { PutLongValue(title, "axisX_position", 0); }
        dutPrm.posAxisY = GetDoubleValue(title, "axisY_position");          if (dutPrm.posAxisY == LONG_MIN) { PutLongValue(title, "axisY_position", 0); }
        dutPrm.posLifter = GetDoubleValue(title, "lifter_position");          if (dutPrm.posLifter == LONG_MIN) { PutLongValue(title, "lifter_position", 0); }
        duts[cnt] = dutPrm;
    }
    
    for (int axis = 0; axis < MLMaxAxisCount; axis++) {
        char *title = (char *)malloc(32);
        sprintf(title, "axis_%d", axis);
        double startSpeed = GetDoubleValue(title, "start_speed");
        double runSpeed =   GetDoubleValue(title, "run_speed");
        double stopSpeed =  GetDoubleValue(title, "stop_speed");
        double homeSpeed =  GetDoubleValue(title, "home_speed");
        double accTime =    GetDoubleValue(title, "acc_time");
        int homeDirect =    GetIntValue(title, "home_dir");
        int homeLevel =     GetIntValue(title, "home_level");
        double ppratio =    GetDoubleValue(title, "pp_ratio");
        double posInit =      GetDoubleValue(title, "initialize_positon");
        double posTest =      GetDoubleValue(title, "test_position");
        
        AxisParam param;
        param.axis = axis;
        param.isRuning = false;
        param.startSpeed =  (LONG_MIN != startSpeed) ? startSpeed : 1;        if (LONG_MIN == startSpeed) { PutDoubleValue(title, "start_speed", 1); }
        param.runSpeed =    (LONG_MIN != runSpeed) ? runSpeed : 1;            if (LONG_MIN == runSpeed) { PutDoubleValue(title, "run_speed", 1); }
        param.stopSpeed =   (LONG_MIN != stopSpeed) ? stopSpeed : 1;          if (LONG_MIN == stopSpeed) { PutDoubleValue(title, "stop_speed", 1); }
        param.homeSpeed =   (LONG_MIN != homeSpeed) ? homeSpeed : 1;          if (LONG_MIN == homeSpeed) { PutDoubleValue(title, "home_speed", 1); }
        param.accTime =     (LONG_MIN != accTime) ? accTime : 0.2;              if (LONG_MIN == accTime) { PutDoubleValue(title, "acc_time", 0.2); }
        param.homeDirect =  (INT_MIN != homeDirect) ? homeDirect : 0;           if (INT_MIN == homeDirect) { PutDoubleValue(title, "home_dir", 0); }
        param.homeLevel =   (MLLevel)((INT_MIN != homeLevel) ? homeLevel : 0);  if (INT_MIN == homeLevel) { PutDoubleValue(title, "home_level", 0); }
        param.equiv = 1;
        param.backlash = 1;
        param.ppratio =     ppratio < 0 ? 0 : ppratio;                          if (ppratio < 0) { PutDoubleValue(title, "pp_ratio", 0); }
        param.posToInit =   (LONG_MIN != posInit) ? posInit : 0;                if (LONG_MIN == posInit) { PutDoubleValue(title, "initialize_positon", 0); }
        param.posToTest =   (LONG_MIN != posTest) ? posTest : 0;                if (LONG_MIN == posTest) { PutDoubleValue(title, "test_position", 0); }
        gAxisPrm[axis] = param;
        
        free(title);
        title = NULL;
    }
    
    // loadcell port name.
    leftCellPortName = (string)malloc(128);
    rightCellPortName = (string)malloc(128);
    frontCellPortName = (string)malloc(128);
    backCellPortName = (string)malloc(128);
    
    if (!GetStringValue("loadcell", "left_side_portname", leftCellPortName)) {
        Logger(MLLogError, "{left side}, No loadcell port has configed.\n");
        InsertStringValue("loadcell", NULL, "left_side_portname", "/dev");
    }
    
    if (!GetStringValue("loadcell", "right_side_portname", rightCellPortName)) {
        Logger(MLLogError, "{right side}, No loadcell port has configed.\n");
        InsertStringValue("loadcell", NULL, "right_side_portname", "/dev");
    }
    
    if (!GetStringValue("loadcell", "front_side_portname", frontCellPortName)) {
        Logger(MLLogError, "{front side}, No loadcell port has configed.\n");
        InsertStringValue("loadcell", NULL, "front_side_portname", "/dev");
    }
    
    if (!GetStringValue("loadcell", "back_side_portname", backCellPortName)) {
        Logger(MLLogError, "{back side}, No loadcell port has configed.\n");
        InsertStringValue("loadcell", NULL, "back_side_portname", "/dev");
    }
    
    laserPortName = (string)malloc(128);
    
    if (!GetStringValue("laser", "portname", laserPortName)) {
        Logger(MLLogError, "No laser port has configed.\n");
        InsertStringValue("laser", NULL, "portname", "/dev");
    }
    
    illuminometerPortName = (string)malloc(128);
    
    if (!GetStringValue("illumionmeter", "portname", illuminometerPortName)) {
        Logger(MLLogError, "No illumionmeter port has configed.\n");
        InsertStringValue("illumionmeter", NULL, "portname", "/dev");
    }
    
    anglePortName = (string)malloc(128);
    
    if (!GetStringValue("angle", "portname", anglePortName)) {
        Logger(MLLogError, "No angle port has configed.\n");
        InsertStringValue("angle", NULL, "portname", "/dev");
    }
}

// 判断输入的字符串是否是正确的IP，但不能过滤掉192.168.1..33这种情况
bool IsValidIP(char *ip) {
    bool flag = false;
    
    char ipStr[20];
    memset(ipStr, 0, 20);
    strcpy(ipStr, ip);
    
    int count = 0;
    char *ptr = strtok(ipStr, ".");
    
    do {
        if (ptr) {
            int num = atoi(ptr);
            
            if (num <= 255 && num >= 0) { count++; }
            else { break; }     // 超出ip的数值的最大值
            
            do {
                ptr = strtok(NULL, ".");
                if (ptr) {
                    int num = atoi(ptr);
                    
                    if (num <= 255 && num >= 0) { count++; }
                    else { break; }     // 超出ip的数值的最大值
                }
            } while (ptr != NULL);
        }
    } while (0);
    
    if (count == 4) {
        flag = true;
    }
    
    return flag;
}

/*
 *check the axis's specified IO
 *@return 0 -> Servo drive alarm, 1 -> positive limit, 2 -> negative limit, 3 -> EMG, 4 -> ORG
 */
int CheckAxisIOState(int axis) {
    ASSERT_REPORT(axis >= 0);
    
    if (gHandle != -1) {
        DWORD state = smc_axis_io_status(gHandle, axis);
//        DWORD state;
//        smc_axis_io_status_ex(gHandle, axis, &state);
        
        int index = 0;
        state <<= 1;
        
        do {
            state = state >> 1;
            int bit = state & 0x01;
            
            switch (index) {
                case 0:
                    if (bit==1) {
                        Logger(MLLogError, "<CheckAxisIOState>: Axis %d Servo drive alarm.\n", axis);
                        memset(errmsg, 0, 256);
                        sprintf(errmsg, "Axis %d Servo drive alarm.\n", axis);
                        return 0;
                    } break;
                case 1:
                    if (bit==1) {
                        Logger(MLLogInfo, "<CheckAxisIOState>: Axis %d Positive limit on.\n", axis);
                        return 1;
                    } break;
                case 2:
                    if (bit==1) {
                        Logger(MLLogInfo, "<CheckAxisIOState>: Axis %d Negative limit on.\n", axis);
                        return 2;
                    } break;
                case 3:
                    if (bit==1) {
                        Logger(MLLogInfo, "<CheckAxisIOState>: Axis %d EMG stop sensor is on.\n", axis);
                        return 3;
                    } break;
                case 4:
                    if (bit==1) {
                        Logger(MLLogInfo, "<CheckAxisIOState>: Axis %d ORG sensor is on.\n", axis);
                        return 4;
                    } break;
                default: break;
            }
            index++;
        } while (state);
    }
    
    return 10;
}

AxisParam GetAxisParam(int axis) {
    assert(gAxisPrm != NULL);
    assert(axis >= 0);
    AxisParam param;
    pthread_mutex_lock(&_mutex);
    param = gAxisPrm[axis];
    pthread_mutex_unlock(&_mutex);
    
    return param;
}

bool SetAxisParam(int axis, AxisParam param) {
    bool flag = true;
    
    assert(gAxisPrm != NULL);
    assert(axis >= 0);
    gAxisPrm[axis] = param;
    
    return flag;
}

void PowerOn() {
    Logger(MLLogInfo, "<%s>: Power ON system.\n", __func__);
    isNeedStop = false;
    isNeedEmgStop = false;
    isActiveFinished = false;
    SetBitState(MLOutLightStop, MLLow);
    SetBitState(MLOutLightStart, MLLow);
    SetBitState(MLOutLightReset, MLLow);
//    SetBitState(MLOutLeftDoorOpen, MLLow);
    Logger(MLLogInfo, "<%s>: Open auto-door.\n", __func__);
    smc_write_outbit(gHandle, MLOutLeftDoorOpen,MLLow);
    smc_write_outbit(gHandle, MLOutLeftDoorClose,MLHigh);
    gIsPowerOn = true;
    AllAxisGoHome(true);
    Logger(MLLogInfo, "<%s>: Power ON..., reset all axises.\n", __func__);
}

void PowerOff() {
    Logger(MLLogInfo, "<%s>: Power OFF system.\n", __func__);
    isNeedStop = false;
    isNeedEmgStop = false;
    isActiveFinished = false;
    SetBitState(MLOutLightPower, MLHigh);
    SetBitState(MLOutSpotPower, MLHigh);
    SetBitState(MLOutLeftDoorOpen, MLHigh);
    SetBitState(MLOutLight, MLHigh);
    SetBitState(MLOutLightStop, MLHigh);
    SetBitState(MLOutLightStart, MLHigh);
    SetBitState(MLOutLightReset, MLHigh);
    SetBitState(MLOutLaserPower, MLHigh);
    SetBitState(MLOutIlluminanceLight, MLHigh);
    ResetAllAxis(true);
    gIsPowerOn = false;
}

bool Connect(char *ip) {
    Logger(MLLogInfo, "\n\n\n----------------- start ----------------------\n");
    
    bool flag = IsValidIP(ip);
    gHandle = -1;
    gIsPowerOn = true;
    if (!flag) {
        Logger(MLLogError, "<Connect>: Invaid IP {%s}.\n", ip);
    }
    
    isNeedStop = false;
    isNeedEmgStop = false;
    
    if (flag) {
        if (0 == smc_board_init(0, Ethernet, ip, 0)) {
            WORD cardNum;
            DWORD cardTypeList;
            WORD cardIdList;
            smc_get_CardInfList(&cardNum, &cardTypeList, &cardIdList);
            Logger(MLLogInfo, "<CardNum> = %d, <CardTypeList> = %ld, <CardIDList> = %d\n", cardNum, cardTypeList, cardIdList);
            gHandle = 0;
            nmcs_clear_card_errcode(gHandle);   // clear card error
            nmcs_clear_errcode(gHandle,0);      // clear bus error
            nmcs_set_alarm_clear(gHandle,2,0);
            
            for (int axis = 1; axis <= 12; axis++) {
                nmcs_clear_axis_errcode(gHandle, axis);
            }
            
            pthread_mutex_init(&_mutex, NULL);
            
            sleep(5);   // wait board init finish and stable.
            
            int rtn = 0;
            for (int i = 1; i < 13 ; i++) {
                rtn |= smc_write_sevon_pin(gHandle, i, 0);
                usleep(500000);
            }
            
            smc_write_outbit(gHandle, MLOutIlluminanceLight, MLLow);
            CheckInputSignal();
            Logger(MLLogInfo, "<Connect>: Success to connect controller.\n");
            CheckStart();
            CheckStop();
            CheckEmgStop();
            CheckStopActivedSignal();
            DWORD carVersion, firmID, subFirmID, libVersion;
            smc_get_card_version(gHandle, &carVersion);
            Logger(MLLogInfo, "<Card Version> = %ld\n", carVersion);
            smc_get_card_soft_version(gHandle, &firmID, &subFirmID);
            Logger(MLLogInfo, "<Firmware Version> = 0x%x, <Subfirmware Version> = 0x%x\n", firmID, subFirmID);
            smc_get_card_lib_version(&libVersion);
            Logger(MLLogInfo, "<Library Version> = %ld\n", libVersion);
            flag = true;
            SetEncoderUnit();
        } else {
            Logger(MLLogError, "<Connect>: Fail to connect controller.\n");
            memset(errmsg, 0, 256);
            sprintf(errmsg, "Fail to connect controller.\n");
            flag = false;
        }
    }

    return flag;
}

double ReadDDAngle() {
    short res = 0;
    WORD MyCardNo = 0;
    double Myencoder_value=0;
    res=smc_get_encoder_unit(MyCardNo,1,&Myencoder_value);//读编码值

    double angle = 0;
    angle = 1.0 * Myencoder_value/100;
   
    return angle;
}

void ResetSystem(void) {
    nmcs_reset_etc(gHandle);
}

void Disconnect(void) {
    if (gHandle != -1) {
        gStopped = true;
        AllAxisStop();                      // stop all axis before disconnect controller.
        usleep(250000);
        nmcs_clear_card_errcode(gHandle);   // clear card error
        nmcs_clear_errcode(gHandle,0);      // clear bus error
        nmcs_set_alarm_clear(gHandle,2,0);
        
        for (int axis = 1; axis <= 12; axis++) {
            nmcs_clear_axis_errcode(gHandle, axis);
        }
        usleep(200000);
        
        int rtn;
        
        if ((rtn = smc_board_close(gHandle))) {
            Logger(MLLogError, "<Disconnect>: Fail to close controller, rtn:{%d}.\n", rtn);
        } else {
            Logger(MLLogInfo, "<Disconnect>: Success to disconnect controller.\n");
        }
        pthread_mutex_destroy(&_mutex);
    }
    Logger(MLLogInfo, "----------------- end ----------------------\n\n\n");
}

void ChangeAxisParam(int axis, double startSpeed, double runSpeed, double acc) {
    AxisParam param = gAxisPrm[axis];
    param.startSpeed = (startSpeed > 0) ? startSpeed : param.startSpeed;
    param.runSpeed = (runSpeed >= param.startSpeed) ? runSpeed : param.startSpeed;
    param.accTime = (acc > 0) ? acc : param.accTime;
    gAxisPrm[axis] = param;
    Logger(MLLogInfo, "<ChangeAxisParam>: Config axis %d start speed to %lf, run speed to %lf, acc time to %lf.\n", axis, param.startSpeed, param.runSpeed, param.accTime);
}

void TMoveAxisToPosition(void *args) {
    TArgs *arguments = (TArgs *)args;
    int axis = *((int *)arguments->arg1);
    double position = arguments->arg2;
    bool blocked = arguments->arg3;
    if (isNeedStop) {return;}
    if (isNeedEmgStop) {return;}
    long currentPos = -1;
    int rtn = 0;
    gIsTesting = true;
    short res = 0;
    WORD MyCardNo = 0;
    double Myencoder_value=0;
    
    assert(axis >= 0);
    
    if (gHandle != -1) {
        rtn |= smc_write_sevon_pin(gHandle, axis, 0);
        AxisParam param = gAxisPrm[axis];
        rtn |= smc_set_pulse_outmode(gHandle, axis, MLPluseOutMode);
        rtn |= smc_set_equiv(gHandle, axis, param.equiv);
        rtn |= smc_set_backlash_unit(gHandle, axis, param.backlash);
        rtn |= smc_set_profile_unit(gHandle, axis, param.startSpeed*param.ppratio, param.runSpeed*param.ppratio, param.accTime, param.accTime, param.stopSpeed);
        
        if (rtn == 0) {
            res=smc_get_encoder_unit(MyCardNo,axis,&Myencoder_value);//读编码值
            currentPos = Myencoder_value;
            Logger(MLLogInfo, "<%s>: Axis %d, [Before_Position]: %ld\n", __func__, axis, currentPos);
            
            rtn |= smc_pmove_unit(gHandle, axis, (long)(position*param.ppratio), MLAbsolute);
            
            if (rtn == 0) {
                if (blocked) {
                    while (!smc_check_done(gHandle, axis)) {
                        usleep(200000);     // check axis's state every 200ms
                        if (isNeedStop) { break; }  // stop to check state
                        if (isNeedEmgStop) {break;}
                    };
                    CheckAxisIOState(axis);
                    res=smc_get_encoder_unit(MyCardNo,axis,&Myencoder_value);//读编码值
                    currentPos = Myencoder_value;
                    Logger(MLLogInfo, "<%s>: Axis %d, [After_Position]: %ld\n", __func__, axis, currentPos);
                    param.isRuning = false;
                } else {
                    Logger(MLLogInfo, "<%s>: Axis %d running unblock mode\n", __func__, axis);
                }
            } else {
                Logger(MLLogError, "<%s>: Fail to move axis{%d} using MoveAxisToPosition, rtn: {%d}.\n", __func__, axis, rtn);
                memset(errmsg, 0, 256);
                sprintf(errmsg, "Fail to move axis{%d} using MoveAxisToPosition, rtn: {%d}.\n", axis, rtn);
            }
        }
    }
    gIsTesting = false;
}

long MoveAxisToPosition(int axis, double position, bool blocked, bool pthread) {
    AxisParam axisPrm = GetAxisParam(axis);
    if (axisPrm.isRuning) { return 0; }
    if (isNeedStop) {return 0;}
    if (isNeedEmgStop) {return 0;}
    if (pthread) {
        TArgs *args = (TArgs *)malloc(sizeof(TArgs));
        args->arg1 = &axis;
        args->arg2 = position;
        args->arg3 = blocked;
        pthread_t thrd;
        pthread_create(&thrd, NULL, (void *)TMoveAxisToPosition, args);
        return 0;
    } else {
        long currentPos = -1;
        int rtn = 0;
        gIsTesting = true;
        short res = 0;
        WORD MyCardNo = 0;
        double Myencoder_value=0;

        assert(axis >= 0);
        
        if (gHandle != -1) {
            smc_write_sevon_pin(gHandle, axis, 0);
            AxisParam param = gAxisPrm[axis];
            rtn |= smc_set_pulse_outmode(gHandle, axis, MLPluseOutMode);
            rtn |= smc_set_equiv(gHandle, axis, param.equiv);
            rtn |= smc_set_backlash_unit(gHandle, axis, param.backlash);
            rtn |= smc_set_profile_unit(gHandle, axis, param.startSpeed*param.ppratio, param.runSpeed*param.ppratio, param.accTime, param.accTime, param.stopSpeed);

            if (rtn == 0) {
                res=smc_get_encoder_unit(MyCardNo,axis,&Myencoder_value);//读编码值
                currentPos = Myencoder_value;
                Logger(MLLogInfo, "<%s>: Axis %d, [Before_Position]: %ld\n", __func__, axis, currentPos);
            
                rtn |= smc_pmove_unit(gHandle, axis, (long)(position*param.ppratio), MLAbsolute);

                if (rtn == 0) {
                    if (blocked) {
                        while (!smc_check_done(gHandle, axis)) {
                            usleep(200000);     // check axis's state every 200ms
                            if (isNeedStop) {
                                break;
                            }
                            if (isNeedEmgStop) {
                                break;
                            }
                        };
                        CheckAxisIOState(axis);
                        res=smc_get_encoder_unit(MyCardNo,axis,&Myencoder_value);//读编码值
                        currentPos = Myencoder_value;
                        Logger(MLLogInfo, "<%s>: Axis %d, [After_Position]: %ld\n", __func__, axis, currentPos);
                    } else {
                        Logger(MLLogInfo, "<%s>: Axis %d running unblock mode\n", __func__, axis);
                    }
                } else {
                    Logger(MLLogError, "<%s>: Fail to move axis{%d} using MoveAxisToPosition, rtn: {%d}.\n", __func__, axis, rtn);
                    memset(errmsg, 0, 256);
                    sprintf(errmsg, "Fail to move axis{%d} using MoveAxisToPosition, rtn: {%d}.\n", axis, rtn);
                }
            }
        }
        gIsTesting = false;

        return blocked ? currentPos : -997;
    }
}

void AdjustHoldPosition(int *axises, double distance, bool blocked) {
    if (isNeedStop) {return;}
    if (isNeedEmgStop) {return;}
    MLAxis axis = axises[0];
    MoveAxisDistance(axis,distance, false, false);

    MLAxis axi1 = axises[1];
    MoveAxisDistance(axi1,-distance, false, false);
}

void TMoveAxisDistance(void *args) {
    TArgs *arguments = (TArgs *)args;
    int axis = *((int *)arguments->arg1);
    double distance = arguments->arg2;
    bool blocked = arguments->arg3;
    if (isNeedStop) {return;}
    if (isNeedEmgStop) {return;}
    long currentPos = -1;
    long validPos = distance;
    int rtn = 0;
    bool flag = false;
    short res = 0;
    WORD MyCardNo = 0;
    double Myencoder_value=0;
    
    assert(axis >= 0);
    
    gIsTesting = true;
    
    if (gHandle != -1) {
        smc_write_sevon_pin(gHandle, axis, 0);
        AxisParam param = gAxisPrm[axis];
        rtn |= smc_set_pulse_outmode(gHandle, axis, MLPluseOutMode);
        rtn |= smc_set_equiv(gHandle, axis, param.equiv);
        rtn |= smc_set_backlash_unit(gHandle, axis, param.backlash);
        rtn |= smc_set_profile_unit(gHandle, axis, param.startSpeed*param.ppratio, param.runSpeed*param.ppratio, param.accTime, param.accTime, param.stopSpeed);
        
        if (rtn==0) {
            res=smc_get_encoder_unit(MyCardNo,axis,&Myencoder_value);//读编码值
            
            currentPos = Myencoder_value;
            validPos += currentPos;
            Logger(MLLogInfo, "<%s>: Axis %d, [Before_Position]: %ld\n", __func__, axis, currentPos);
            
            rtn |= smc_pmove_unit(gHandle, axis, distance*param.ppratio, MLRelative);
            
            if (rtn==0) {
                flag = true;
                if (blocked) {
                    while (!smc_check_done(gHandle, axis)) {
                        usleep(200000);     // check axis's state every 200ms
                        if (isNeedStop) { break; } // stop to check state.
                        if (isNeedEmgStop) {
                            break;
                        }
                     }
                    res=smc_get_encoder_unit(MyCardNo,axis,&Myencoder_value);//读编码值
                    currentPos = Myencoder_value;
                    Logger(MLLogInfo, "<%s>: Axis %d, [After_Position>]: %ld\n", __func__, axis, currentPos);
                    param.isRuning = false;
                } else {
                    Logger(MLLogInfo, "<%s>: Axis %d running unblock mode\n", __func__, axis);
                }
            } else {
                flag = false;
                Logger(MLLogError, "<%s>: Fail to move axis{%d} using MoveAxisToPosition, rtn: {%d}.\n", __func__, axis, rtn);
                memset(errmsg, 0, 256);
                sprintf(errmsg, "Fail to move axis{%d} using MoveAxisToPosition, rtn: {%d}.\n", axis, rtn);
            }
        }
    }
    gIsTesting = false;
}

bool MoveAxisDistance(int axis, double distance, bool blocked, bool pthread) {
    if (pthread) {
        AxisParam axisPrm = GetAxisParam(axis);
        if (axisPrm.isRuning) { return false; }
        if (isNeedStop) {return 0;}
        if (isNeedEmgStop) {return 0;}
        TArgs *args = (TArgs *)malloc(sizeof(TArgs));
        args->arg1 = &axis;
        args->arg2 = distance;
        args->arg3 = blocked;
        
        pthread_t thrd;
        pthread_create(&thrd, NULL, (void *)TMoveAxisDistance, args);
        
        return false;
    } else {
        long currentPos = -1;
        long validPos = distance;
        int rtn = 0;
        gIsTesting = true;
        bool flag = false;
        short res = 0;
        WORD MyCardNo = 0;
        double Myencoder_value=0;

        assert(axis >= 0);
        
        if (gHandle != -1) {
            smc_write_sevon_pin(gHandle, axis, 0);
            AxisParam param = gAxisPrm[axis];
            rtn |= smc_set_pulse_outmode(gHandle, axis, MLPluseOutMode);
            rtn |= smc_set_equiv(gHandle, axis, param.equiv);
            rtn |= smc_set_backlash_unit(gHandle, axis, param.backlash);
            rtn |= smc_set_profile_unit(gHandle, axis, param.startSpeed*param.ppratio, param.runSpeed*param.ppratio, param.accTime, param.accTime, param.stopSpeed);
            
            if (rtn==0) {
                res=smc_get_encoder_unit(MyCardNo,axis,&Myencoder_value);//读编码值

                currentPos = Myencoder_value;
                validPos += currentPos;
                Logger(MLLogInfo, "<%s>: Axis %d, [Before_Position]: %ld\n", __func__, axis, currentPos);
                
                rtn |= smc_pmove_unit(gHandle, axis, distance*param.ppratio, MLRelative);
                
                if (rtn==0) {
                    flag = true;
                    if (blocked) {
                        while (!smc_check_done(gHandle, axis)) {
                            usleep(200000);     // check axis's state every 200ms
                            if (isNeedStop) {
                                break;
                            }
                            if (isNeedEmgStop) {
                                break;
                            }
                        }
                        res=smc_get_encoder_unit(MyCardNo,axis,&Myencoder_value);//读编码值
                        currentPos = Myencoder_value;
                        param.isRuning = false;
                        Logger(MLLogInfo, "<%s>: Axis %d, [After_Position]: %ld\n", __func__,  axis, currentPos);
                    } else {
                        Logger(MLLogInfo, "<%s>: Axis %d running unblock mode\n", __func__, axis);
                    }
                } else {
                    flag = false;
                    Logger(MLLogError, "<%s>: Fail to move axis{%d} using MoveAxisToDistance, rtn: {%d}.\n", __func__, axis, rtn);
                    memset(errmsg, 0, 256);
                    sprintf(errmsg, "Fail to move axis{%d} using MoveAxisToDistance, rtn: {%d}.\n", axis, rtn);
                }
            }
        }
        gIsTesting = false;

        return flag;
    }
}

long MoveAxisDistanceQuick(int axis, double distance, bool blocked) {
    long currentPos = -1;
    long validPos = distance;
    int rtn = 0;
    gIsTesting = true;
    short res = 0;
    WORD MyCardNo = 0;
    double Myencoder_value=0;
    assert(axis >= 0);
    if (isNeedStop) {return 0;}
    if (isNeedEmgStop) {return 0;}
    if (gHandle != -1) {
        smc_write_sevon_pin(gHandle, axis, 0);
        AxisParam param = gAxisPrm[axis];
        rtn |= smc_set_pulse_outmode(gHandle, axis, MLPluseOutMode);
        rtn |= smc_set_equiv(gHandle, axis, param.equiv);
        rtn |= smc_set_backlash_unit(gHandle, axis, param.backlash);
        rtn |= smc_set_profile_unit(gHandle, axis, 8000, 15000, param.accTime, param.accTime, 5000);
        
        if (rtn==0) {
            res=smc_get_encoder_unit(MyCardNo,axis,&Myencoder_value);//读编码值
            currentPos = Myencoder_value;
            validPos += currentPos;
            Logger(MLLogInfo, "<%s>: Axis %d, [Before_Position]: %ld\n", __func__, axis, currentPos);
            
            rtn |= smc_pmove_unit(gHandle, axis, distance*param.ppratio, MLRelative);
            
            if (rtn==0) {
                if (blocked) {
                    while (!smc_check_done(gHandle, axis)) {
                        usleep(200000);     // check axis's state every 200ms
                    }
                    CheckAxisIOState(axis);
                    res=smc_get_encoder_unit(MyCardNo,axis,&Myencoder_value);//读编码值
                    currentPos = Myencoder_value;
                    Logger(MLLogInfo, "<%s>: Axis %d, [After_Position]: %ld\n", __func__,  axis, currentPos);
                } else {
                    Logger(MLLogInfo, "<%s>: Axis %d running unblock mode\n", __func__, axis);
                }
            } else {
                Logger(MLLogError, "<%s>: Fail to move axis{%d} using MoveAxisToDistance, rtn: {%d}.\n", __func__, axis, rtn);
                memset(errmsg, 0, 256);
                sprintf(errmsg, "Fail to move axis{%d} using MoveAxisToDistance, rtn: {%d}.\n", axis, rtn);
            }
        }
    }
    gIsTesting = false;
    
    return blocked ? currentPos : -997;
}

void TJMoveAxis(void *args) {
    TArgs *arguments = (TArgs *)args;
    int axis = *((int *)arguments->arg1);
    int direction = arguments->arg2;
    AxisParam param = gAxisPrm[axis];
    if (isNeedStop) {return;}
    if (isNeedEmgStop) {return;}
    int rtn = 0;
    gIsTesting = true;
    if (gHandle != -1) {
        param.isRuning = true;
        gAxisPrm[axis] = param;
        rtn |= smc_write_sevon_pin(gHandle, axis, 0);
        
        if (direction == 0) {
            if (2 == CheckAxisIOState(axis)) {
                Logger(MLLogInfo, "<%s>: Axis %d at negative limit, not need move, rtn: {%d}.\n", __func__, axis, rtn);
                gIsTesting = false;
                param.isRuning = false;
                gAxisPrm[axis] = param;
                return;
            }
        } else if (direction == 1) {
            if (1 == CheckAxisIOState(axis)) {
                Logger(MLLogInfo, "<%s>: Axis %d at positive limit, not need move, rtn: {%d}.\n", __func__, axis, rtn);
                gIsTesting = false;
                param.isRuning = false;
                gAxisPrm[axis] = param;
                return;
            }
        }
        
        sleep(1);
        rtn |= smc_set_profile_unit(gHandle, axis, param.startSpeed*param.ppratio, param.runSpeed*param.ppratio, param.accTime, param.accTime, param.stopSpeed);
        rtn |= smc_set_s_profile(gHandle, axis,0,0);
        rtn |= smc_vmove(gHandle, axis, direction);
        
        if (rtn != 0) {
            Logger(MLLogError, "<%s>: Axis %d fail to move axis using JMove, rtn: {%d}.\n", __func__, axis, rtn);
            memset(errmsg, 0, 256);
            sprintf(errmsg, "Axis %d fail to move axis using JMove, rtn: {%d}.\n", axis, rtn);
        }
    }
    gIsTesting = false;
//    param.isRuning = false;
    gAxisPrm[axis] = param;
}

void JMoveAxis(int axis, int direction, bool pthread) {
    AxisParam param = gAxisPrm[axis];
    
    if (gIsTesting) { return; }
    if (param.isRuning) {return;}
    if (isNeedStop) {return;}
    if (isNeedEmgStop) {return;}
    assert(axis >= 0);
    
    if (pthread) {
        pthread_t thrd;
        pthread_create(&thrd, NULL, (void *)TJMoveAxis, NULL);
    } else {
        int rtn = 0;
        gIsTesting = true;
        if (gHandle != -1) {
            param.isRuning = true;
            gAxisPrm[axis] = param;
            rtn |= smc_write_sevon_pin(gHandle, axis, 0);
            
            if (direction == 0) {
                if (2 == CheckAxisIOState(axis)) {
                    Logger(MLLogInfo, "<%s>: Axis %d at negative limit, not need move, rtn: {%d}.\n", __func__, axis, rtn);
                    gIsTesting = false;
                    param.isRuning = false;
                    gAxisPrm[axis] = param;
                    return;
                }
            } else if (direction == 1) {
                if (1 == CheckAxisIOState(axis)) {
                    Logger(MLLogInfo, "<%s>: Axis %d at positive limit, not need move, rtn: {%d}.\n", __func__, axis, rtn);
                    gIsTesting = false;
                    param.isRuning = false;
                    gAxisPrm[axis] = param;
                    return;
                }
            }
            
            sleep(1);
            rtn |= smc_set_profile_unit(gHandle, axis, param.startSpeed*param.ppratio, param.runSpeed*param.ppratio, param.accTime, param.accTime, param.stopSpeed);
            rtn |= smc_set_s_profile(gHandle, axis,0,0);
            rtn |= smc_vmove(gHandle, axis, direction);
            
            if (rtn != 0) {
                Logger(MLLogError, "<%s>: Axis %d fail to move axis using JMove, rtn: {%d}.\n", __func__, axis, rtn);
                memset(errmsg, 0, 256);
                sprintf(errmsg, "Axis %d fail to move axis using JMove, rtn: {%d}.\n", axis, rtn);
            }
        }
        gIsTesting = false;
        param.isRuning = false;
        gAxisPrm[axis] = param;
    }
}

void JHoldMoveAxis(int axis, int direction) {
    if (gIsTesting) { return; }
    if (isNeedStop) {return;}
    if (isNeedEmgStop) {return;}
    assert(axis >= 0);
    int rtn = 0;
    gIsTesting = true;

    if (gHandle != -1) {
        AxisParam param = gAxisPrm[axis];
        
        rtn |= smc_write_sevon_pin(gHandle, axis, 0);
        
        if (direction == 0) {
            if (2 == CheckAxisIOState(axis)) {
                Logger(MLLogInfo, "<%s>: Axis %d at negative limit, not need move, rtn: {%d}.\n", __func__, axis, rtn);
                gIsTesting = false;
                param.isRuning = false;
                gAxisPrm[axis] = param;
                return;
            }
        } else if (direction == 1) {
            if (1 == CheckAxisIOState(axis)) {
                Logger(MLLogInfo, "<%s>: Axis %d at positive limit, not need move, rtn: {%d}.\n", __func__, axis, rtn);
                gIsTesting = false;
                param.isRuning = false;
                gAxisPrm[axis] = param;
                return;
            }
        }
        
        sleep(1);
        rtn |= smc_set_profile_unit(gHandle, axis,5000, 5000, param.accTime, param.accTime, param.stopSpeed);
        rtn |= smc_set_s_profile(gHandle, axis,0,0);
        rtn |= smc_vmove(gHandle, axis, direction);
        
        if (rtn != 0) {
            Logger(MLLogError, "Axis{%d}, Fail to move axis using JMove, rtn: {%d}.\n", axis, rtn);
            memset(errmsg, 0, 256);
            sprintf(errmsg, "Axis %d fail to move axis using JMove, rtn: {%d}.\n", axis, rtn);
        }
    }
    gIsTesting = false;
}

void TJMoveAxisWithBlock(void *args) {
    TArgs *arguments = (TArgs *)args;
    int axis = *((int *)arguments->arg1);
    int direction = arguments->arg2;
    if (isNeedStop) {return;}
    if (isNeedEmgStop) {return;}
    JMoveAxis(axis, direction, false);
    gIsTesting = true;
    
    if (gHandle != -1) {
        while (!smc_check_done(gHandle, axis)) {
            usleep(200000);     // check axis's state every 200ms
            if (0 == CheckAxisIOState(axis)) {
                Logger(MLLogError, "<%s>: Servo clarm at axis %d.\n", __func__, axis);
                break;
            }
            if (isNeedStop) { break; }  // stop to check state.
            if (isNeedEmgStop) {
                break;
            }
        }
    }
    AxisParam axisPrm = GetAxisParam(axis);
    axisPrm.isRuning = false;
    SetAxisParam(axis, axisPrm);
    
    if (isNeedStop) {
        Logger(MLLogInfo, "<%s>: Axis %d stop jmove.\n", __func__, axis);
    } else {
        Logger(MLLogInfo, "<%s>: Axis %d finished jmove.\n", __func__, axis);
    }
    gIsTesting = false;
}

void JMoveAxisWithBlock(int axis, int direction, bool pthread) {
    AxisParam axisPrm = GetAxisParam(axis);
    if (axisPrm.isRuning) { return; }
    if (isNeedStop) {return;}
    if (isNeedEmgStop) {return;}
    Logger(MLLogInfo, "<%s>: axis %d call `JMoveAxisWithBlock`\n", __func__, axis);
    
    if (pthread) {
        TArgs *args = (TArgs *)malloc(sizeof(TArgs));
        args->arg1 = &axis;
        args->arg2 = direction;
        pthread_t thrd;
        pthread_create(&thrd, NULL, (void *)TJMoveAxisWithBlock, args);
    } else {
        JMoveAxis(axis, direction, false);
        gIsTesting = true;
        
        if (gHandle != -1) {
            while (!smc_check_done(gHandle, axis)) {
                usleep(200000);     // check axis's state every 200ms
                if (0 == CheckAxisIOState(axis)) {
                    Logger(MLLogError, "<%s>: Servo clarm at axis %d.\n", __func__, axis);
                    break;
                }
                if (isNeedStop) { break; }  // stop to check axis state.
                if (isNeedEmgStop) {
                    break;
                }
            }
        }
        if (isNeedStop) {
            Logger(MLLogInfo, "<%s>: Axis %d stop jmove.\n", __func__, axis);
        } else {
            Logger(MLLogInfo, "<%s> Axis %d finished jmove.\n", __func__, axis);
        }
        gIsTesting = false;
    }
}

bool CheckAxisState(int *axises, int axisCount, bool pthread) {
    bool flag = false;
    int bitflags = 0;
    
    // 设定标志，默认为1， 如果变为0则表示轴运动到位.
    for (int i = 0; i < axisCount; i++) {
        bitflags = ((bitflags << 1) | 0x1);
    }
    
    if (gHandle != -1) {
        do {
            for (int index = 0; index < axisCount; index++) {
                int axis = axises[index];
                
                if (((bitflags >> index) & 0x01) != 0) {
                    if (1 == smc_check_done(gHandle, axis)) {
                        bitflags = (~bitflags) ^ (1 << index);
                        bitflags = ~bitflags;
                        
                        AxisParam axisPrm = GetAxisParam(axis);
                        axisPrm.isRuning = false;
                        SetAxisParam(axis, axisPrm);
                    }
                }
            }
            
            if (isNeedStop) {   // stop to check axis state.
//                Logger(MLLogInfo, "<%s>: stop to check state.\n", __func__);
                break;
            }
            if (isNeedEmgStop) {
                break;
            }
            if (!bitflags) {    // 轴全部到位
                flag = true;
            }
        } while (!flag && !pthread);
    } else {
        Logger(MLLogInfo, "<%s>: Controller is disconnected.\n", __func__);
    }
    
    return flag;
}

bool CheckMutliAxisState(int *axises, int axisCount, func callback, double value1, double value2) {
    bool flag = false;
    int bitflags = 0;
    
    // 设定标志，默认为1， 如果变为0则表示轴运动到位.
    for (int i = 0; i < axisCount; i++) {
        bitflags = ((bitflags << 1) | 0x1);
    }
    
    while (!flag) {
        for (int index = 0; index < axisCount; index++) {
            int axis = axises[index];
            
            if (((bitflags >> index) & 0x01) != 0) {
                if (1 == smc_check_done(gHandle, axis)) {
                    bitflags = (~bitflags) ^ (1 << index);
                    bitflags = ~bitflags;
                }
            }
        }
        
        if (!bitflags) {    // 轴全部到位
            flag = true;
        }
        
        if (callback != NULL) {
            callback(value1, value2);
        }
    }
    
    return flag;
}

bool CheckAxisHomeState(int *axises, int axisCount) {
    bool flag = false;
    int bitflags = 0;
    
    // 设定标志，默认为1， 如果变为0则表示轴运动到位.
    for (int i = 0; i < axisCount; i++) {
        bitflags = ((bitflags << 1) | 0x1);
    }
    
    while (!flag) {
        for (int index = 0; index < axisCount; index++) {
            int axis = axises[index];
            
            if (((bitflags >> index) & 0x01) != 0) {
                WORD state;
                if ((0==smc_get_home_result(gHandle, axis, &state)) && state) {
                    bitflags = (~bitflags) ^ (1 << index);
                    bitflags = ~bitflags;
                    Logger(MLLogInfo, "<%s>: Axis %d success to go home\n", __func__, axis);
                }
            }
        }
        
        if (!bitflags) {    // 轴全部到位
            flag = true;
        }
        
        if (isNeedStop) {
            Logger(MLLogInfo, "<%s>: Stop button pressed\n", __func__);
            break;
        }
        if (isNeedEmgStop) {
            break;
        }
    }
    
    return flag;
}

void AxisGoHome(int axis, bool blocked) {
    AxisParam param = gAxisPrm[axis];
//    if (gIsTesting) { return; }
    if (param.isRuning) {return;}
    if (isNeedStop) {return;}
    if (isNeedEmgStop) {return;}
    assert(axis >= 0);
    gIsTesting = true;
    int rtn = 0;
    
    if (gHandle != -1) {
//        AxisParam param = gAxisPrm[axis];
//        DWORD state = smc_axis_io_status(gHandle, axis);
        param.isRuning = true;
        gAxisPrm[axis] = param;
        Logger(MLLogInfo, "<%s>: Axis %d call `AxisGoHome`\n", __func__, axis);

        rtn |= smc_set_home_profile_unit(gHandle, axis, param.startSpeed*param.ppratio, param.homeSpeed*param.ppratio, param.accTime, 0);
        rtn |= smc_set_home_pin_logic(gHandle, axis, param.homeLevel, 0);
        
        if (rtn==0) {
            rtn |= smc_write_sevon_pin(gHandle, axis, 0);
            rtn |= smc_set_pulse_outmode(gHandle, axis, MLPluseOutMode);
            rtn |= smc_set_equiv(gHandle, axis, param.equiv);
            rtn |= smc_set_backlash_unit(gHandle, axis, param.backlash);
            
            int iostate = CheckAxisIOState(axis);
            
            if (axis>=1&&axis<=8) {
                rtn |= smc_set_homemode(gHandle, axis, param.homeDirect, 1, 27, 0);
            } else {
                rtn |= smc_set_homemode(gHandle, axis, 1, 1, 30, 0);
                if (iostate == 1){
                    rtn |= smc_pmove_unit(gHandle, axis, -5000, MLRelative);
               }
            }
            
            if (rtn != 0) {
                Logger(MLLogError, "<%s>: Axis %d problem setting parameters, rtn: {%d}\n", __func__, axis, rtn);
            }
            
            while (!smc_check_done(gHandle, axis)) {
                usleep(100000);
            }
            if (axis==MLAxisLight) {
                SetBitState(MLOutCylinderHome, 0);
                SetBitState(MLOutCylinderShop, 1);
            }
            rtn = smc_home_move(gHandle, axis);
            
            if (rtn==0) {
                WORD state;
                
                if (blocked) {
                    while (!smc_get_home_result(gHandle, axis, &state)) {
                        if (state) {
                            double orgPosition;
                            smc_get_position_unit(gHandle, axis, &orgPosition);
                            Logger(MLLogInfo, "<%s>: Axis %d success to go home, Position: %lf\n", __func__, axis, orgPosition);
                            break;  // 回原点成功
                        }
                        if (isNeedStop) { return; }  // stop to check axis state.
                        if (isNeedEmgStop) {return;}
                        usleep(100000);
                    }
                   
                    if (axis == MLAxisRotation) {
                        rtn |= smc_pmove_unit(gHandle, axis, -110, MLAbsolute);
                        Logger(MLLogInfo, "<%s>: Axis %d go to init position %d pluse, rtn: (%d}\n", __func__, axis, -110, rtn);
                    }
                    if (axis == MLAxisRotationX) {
                        rtn |= smc_pmove_unit(gHandle, axis, -2500, MLAbsolute);
                        Logger(MLLogInfo, "<%s>: Axis %d go to init position %d pluse, rtn: {%d}\n", __func__, axis, -2500, rtn);
                    }
                    if (axis == MLAxisX) {
                        rtn |= smc_pmove_unit(gHandle, axis, 16000, MLAbsolute);
                        Logger(MLLogInfo, "<%s>: Axis %d go to init position %d pluse, rtn: {%d}\n", __func__, axis, 16000, rtn);
                    }
                    if (axis == MLAxisY) {
                        rtn |= smc_pmove_unit(gHandle, axis, -10000, MLAbsolute);
                        Logger(MLLogInfo, "<%s>: Axis %d go to init position %d pluse, rtn: {%d}\n", __func__, axis, -10000, rtn);
                    }
                } else {
                    Logger(MLLogInfo, "<%s>: Axis %d go home using block mode\n", __func__, axis);
                }
            } else {
                Logger(MLLogError, "<%s>: Axis %d fail to moving at home mode, rtn: {%d}.\n", __func__, axis, rtn);
                memset(errmsg, 0, 256);
                sprintf(errmsg, "Axis %d fail to moving at home mode, rtn: {%d}.\n", axis, rtn);
            }
        } else  {
            Logger(MLLogError, "<%s>: Axis %d fail to setup home mode, rtn: {%d}.\n", __func__, axis, rtn);
            memset(errmsg, 0, 256);
            sprintf(errmsg, "Axis %d fail to setup home mode, rtn: {%d}.\n", axis, rtn);
        }
    }
    gIsTesting = false;
    param.isRuning = false;
    gAxisPrm[axis] = param;
}

bool IsStopActived(void) {
    return isNeedStop;
}

bool IsMoveFinished(void) {
    return isActiveFinished;
}

void stopSingleChange(void){
    isNeedStop = false;
}

void InactiveStop(void) {
//    pthread_mutex_lock(&_mutex);
    isNeedStop = false;
    isNeedEmgStop = false;
    isActiveFinished = false;
    gIsTesting = false;
    Logger(MLLogInfo, "<%s>: Reset stop state.\n", __func__);
//    pthread_mutex_unlock(&_mutex);
}

char *GetErrorMessage() {
    char *errorMsg = (char *)malloc(512 * sizeof(char));
    memset(errorMsg, 0, 512);
    
    if (gHandle != -1) {
        do {
            DWORD errorcode = 0;
            nmcs_get_errcode(gHandle, 2, &errorcode);

            if (errorcode != 0) {
                sprintf(errorMsg, "The bus error, errorcode: 0x%lx\n", errorcode);
                break;
            }
            
            nmcs_get_card_errcode(gHandle, &errorcode);

            if (errorcode != 0) {
                sprintf(errorMsg, "The bus error, errorcode: 0x%lx\n", errorcode);
                break;
            }
            
            for (int axis = 1; axis <= 12; axis++) {
                int state = CheckAxisIOState(axis);
                
                if (state == 0) {
                    sprintf(errorMsg, " Axis %d servo drive alarm, errorcode: %ld\n", axis, errorcode);
                    break;
                }
            }
            
        } while (0);
        sprintf(errorMsg, "%s%s", errorMsg, errmsg);
    }
    return errorMsg;
}

void TResetAllAxis() {
    isActiveFinished = false;
    SetBitState(MLOutCylinderHome, 0);
    SetBitState(MLOutCylinderShop, 1);
    if (isNeedStop) { Logger(MLLogInfo, "<%s>: Stop reset system.\n", __func__); return; }
    if (isNeedEmgStop) { Logger(MLLogInfo, "<%s>: Emg stop.\n", __func__); return; }
    JMoveAxisWithBlock(MLAxisLight, 0, false);
    int axises0[] = {MLAxisLight};
    CheckAxisState(axises0, 1, false);
    
    if (isNeedStop) {
        isActiveFinished = true;
        Logger(MLLogInfo, "<%s>: Stop button pressed when light axis go home, at call `JMoveAxisWithBlock`\n", __func__);
        return;
    }
    if (isNeedEmgStop) { Logger(MLLogInfo, "<%s>: Emg stop.\n", __func__); return;}
    JMoveAxisWithBlock(MLAxisLaser, 0, false);
    int axises1[] = {MLAxisLaser};
    CheckAxisState(axises1, 1, false);
    
    if (isNeedStop) {
        isActiveFinished = true;
        Logger(MLLogInfo, "<%s>: Stop button pressed when light axis go home, at call `JMoveAxisWithBlock`\n", __func__);
        return;
    }
    if (isNeedEmgStop) { Logger(MLLogInfo, "<%s>: Emg stop.\n", __func__); return;}
    MLSwingAngleAbsolute(0, 0, 0, 1, 1, 1, false);
    int axises[] = { MLAxisRotationX, MLAxisRotation };
    MoveToInitPos(axises, 2);
    isActiveFinished = true;
    Logger(MLLogInfo, "<%s>: Power OFF completed\n", __func__);
}

void ResetAllAxis(bool pthread) {
    bool moving = false;
    for (int axis = 0; axis <= 12; axis++) {
        AxisParam axisPrm = GetAxisParam(axis);
        moving |= axisPrm.isRuning;
    }
    
    if (moving) {
        Logger(MLLogInfo, "<%s>: Some axis is moving, please wait moving finish then call `AllAxisGoHome`.\n", __func__);
        return;
    }
    
    if (pthread) {
        pthread_t thrd;
        pthread_create(&thrd, NULL, (void *)TResetAllAxis, NULL);
    } else {
        TResetAllAxis();
    }
}

void TAllAxisGoHome() {
    Logger(MLLogInfo, "<%s>: Reset system...\n", __func__);
    isActiveFinished = false;
    SetBitState(MLOutCylinderHome, 0);
    SetBitState(MLOutCylinderShop, 1);
    if (isNeedStop) {return;}
    if (isNeedEmgStop) {return;}
    JMoveAxisWithBlock(MLAxisLight, 0, false);
    int axises0[] = {MLAxisLight};
    CheckAxisState(axises0, 1, false);

    if (isNeedStop) {
        isActiveFinished = true;
        Logger(MLLogInfo, "<%s>: Stop button pressed when light axis go home, at call `JMoveAxisWithBlock`\n", __func__);
        return;
    }
    if (isNeedEmgStop) { Logger(MLLogInfo, "<%s>: Emg stop.\n", __func__); return; }
    AxisGoHome(MLAxisLifter, true); // wait success to go home.

    if (isNeedStop) {
        isActiveFinished = true;
        Logger(MLLogInfo, "<%s>: Stop button pressed when lifter axis go home, at call `AxisGoHome`.\n", __func__);
        return;
    }
    if (isNeedEmgStop) { Logger(MLLogInfo, "<%s>: Emg stop.\n", __func__); return; }
    int axises1[] = {MLAxisRotation, MLAxisClampUp, MLAxisClampRight, MLAxisClampDown, MLAxisClampLeft, MLAxisLaser, MLAxisX, MLAxisY};
    ManyAxisGoHome(axises1, 8, false);

    if (isNeedStop) {
        isActiveFinished = true;
        Logger(MLLogInfo, "<%s>: Stop button pressed when some axises go home, at call `ManyAxisGoHome`.\n", __func__);
        return;
    }
    if (isNeedEmgStop) { Logger(MLLogInfo, "<%s>: Emg stop.\n", __func__); return; }
    JMoveAxisWithBlock(MLAxisLaser, 0, false);
    int axises2[] = {MLAxisLaser};
    CheckAxisState(axises2, 1, false);

    if (isNeedStop) {
        isActiveFinished = true;
        Logger(MLLogInfo, "<%s>: Stop button pressed when laser axis go home, at call `JMoveAxisWithBlock`.\n", __func__);
        return;
    }
    if (isNeedEmgStop) { Logger(MLLogInfo, "<%s>: Emg stop.\n", __func__); return; }
    int axises3[] = {MLAxisRotationX, MLAxisRotationY};
    ManyAxisGoHome(axises3, 2, false);

    if (isNeedStop) {
        Logger(MLLogInfo, "<%s>: Stop button pressed when rotate axis go home, at call `ManyAxisGoHome`.\n", __func__);
    }
    if (isNeedEmgStop) { Logger(MLLogInfo, "<%s>: Emg stop.\n", __func__); return; }
    isActiveFinished = true;
    Logger(MLLogInfo, "<%s>: Power ON completed and all axises finish go home.\n", __func__);
}

void AllAxisGoHome(bool pthread) {
    bool moving = false;
    for (int axis = 0; axis <= 12; axis++) {
        AxisParam axisPrm = GetAxisParam(axis);
        moving |= axisPrm.isRuning;
    }

    if (moving) {
        Logger(MLLogInfo, "<%s>: Some axis is moving, please wait moving finish then call `AllAxisGoHome`.\n", __func__);
        return;
    }
    
    if (pthread) {
        Logger(MLLogInfo, "<%s>:start new thread...\n", __func__);
        pthread_t thrd;
        pthread_create(&thrd, NULL, (void *)TAllAxisGoHome, NULL);
         Logger(MLLogInfo, "<%s>: Run function in new thread...\n", __func__);
    } else {
        TAllAxisGoHome();
        /*
        //    if (gIsTesting) { return; }
        //    gIsTesting = true;
        SetBitState(MLOutCylinderHome, 0);
        SetBitState(MLOutCylinderShop, 1);
        
        JMoveAxisWithBlock(MLAxisLight, 0, false);
        int axises0[] = {MLAxisLight};
        CheckAxisState(axises0, 1, false);
        
        AxisGoHome(MLAxisLifter, false);

        int axises1[] = {1,5,6,7,8,10,11,12};
        ManyAxisGoHome(axises1, 8, false);

        JMoveAxisWithBlock(MLAxisLaser, 0, false);
        int axises2[] = {MLAxisLaser};
        CheckAxisState(axises2, 1, false);
        
        int axises3[] = {2,3};
        ManyAxisGoHome(axises3, 2, false);
        
    //    gIsTesting = false;
         */
    }
}

void DebugHold1KMove(int *axises, int axisCount,int testCount) {
    gIsTesting = true;
    
    const char *pFileName = "/vault/FlareLog.txt";
    FILE *pFile;
    pFile = fopen(pFileName,"a");
    if (NULL == pFile) {
        printf("error");
    }
    
    for (int i = 0; i < testCount; i++) {
        if (isNeedStop) {return;}
        if (isNeedEmgStop) {return;}
        JMoveAxis(axises[0],  1, false);
        JMoveAxis(axises[1],  1, false);
        JMoveAxis(axises[2],  1, false);
        JMoveAxis(axises[3],  1, false);
        
        CheckAxisState(axises, axisCount, false);
        
        JMoveAxis(axises[0],  0, false);
        JMoveAxis(axises[1],  0, false);
        JMoveAxis(axises[2],  0, false);
        JMoveAxis(axises[3],  0, false);
        
        CheckAxisState(axises, axisCount, false);
        
        fprintf(pFile,"<%s>: Dut Hold Move %d times\n", __func__, i+61);
    }
    
    fclose(pFile);
    gIsTesting = false;
    
}

void DebugManyAxisMovePAndN(int *axises, int axisCount,int testCount) {
    gIsTesting = true;
    
    const char *pFileName = "/vault/FlareLog.txt";
    FILE *pFile;
    pFile = fopen(pFileName,"a");
    if (NULL == pFile) {
        printf("error");
    }

    for (int i = 0; i < testCount; i++) {
        
        for (int index = 0; index < axisCount; index++) {
            MLAxis axis = axises[index];
            Logger(MLLogInfo, "<%s>: Axis %d start go to limit ...\n", __func__, axis);
            AxisParam axisPrm = GetAxisParam(axis);
            JMoveAxis(axis, axisPrm.homeDirect ? 0 : 1, false);
            if (isNeedStop) {return;}
            if (isNeedEmgStop) {return;}
        }
        
        CheckAxisState(axises, axisCount, false);
        
        for (int index = 0; index < axisCount; index++) {
            MLAxis axis = axises[index];
            Logger(MLLogInfo, "<%s>: Axis %d start go to limit ...\n", __func__, axis);
            AxisParam axisPrm = GetAxisParam(axis);
            JMoveAxis(axis, axisPrm.homeDirect ? 1 : 0, false);
        }

        
        CheckAxisState(axises, axisCount, false);
        
        MLAxis axis = axises[0];
        fprintf(pFile,"<%s>: Axis %d Move %d times\n", __func__, axis, i+1);

    }
    
    fclose(pFile);
    gIsTesting = false;
    
}

void TManyAxisGoBack(void *args) {
    TArgs *arguments = (TArgs *)args;
    int *axises = (int *)arguments->arg1;
    int axisCount = (int)arguments->arg2;
    
    for (int index = 0; index < axisCount; index++) {
        MLAxis axis = axises[index];
        Logger(MLLogInfo, "<%s>: Axis %d start go to limit ...\n", __func__, axis);
        AxisParam axisPrm = GetAxisParam(axis);
        JMoveAxis(axis, axisPrm.homeDirect ? 0 : 1, false);
        if (isNeedStop) {
            Logger(MLLogInfo, "<%s>: Stop button pressed when some axises go back to negative limit\n", __func__);
            break;
        }
        if (isNeedEmgStop) {
            Logger(MLLogInfo, "<%s>: Emg stop.\n", __func__);
            return;
        }
    }
    
    CheckAxisState(axises, axisCount, false);
    Logger(MLLogInfo, "<%s>: Many axises finish go negative limit.\n", __func__);
}

void ManyAxisGoBack(int *axises, int axisCount, bool pthread) {
    bool isRuning = false;
    
    for (int ax = 0; ax < axisCount; ax++) {
        int axis = axises[ax];
        AxisParam axisPrm = GetAxisParam(axis);
        isRuning |= axisPrm.isRuning;
    }
    
    if (isRuning) {
        Logger(MLLogInfo, "<%s>: Some axis is running, please wait it.\n", __func__);
        return;
    }
    
    if (pthread) {
        TArgs *args = (TArgs *)malloc(sizeof(TArgs));
        args->arg1 = axises;
        args->arg2 = axisCount;
        
        pthread_t thrd;
        pthread_create(&thrd, NULL, (void *)TManyAxisGoBack, args);
        
    } else {
        for (int index = 0; index < axisCount; index++) {
            MLAxis axis = axises[index];
            Logger(MLLogInfo, "<%s>: Axis %d start go to limit ...\n", __func__, axis);
            AxisParam axisPrm = GetAxisParam(axis);
            JMoveAxis(axis, axisPrm.homeDirect ? 0 : 1, false);
            
            if (isNeedStop) {
                Logger(MLLogInfo, "<%s>: Stop button pressed when some axises go back to negative limit\n", __func__);
                break;
            }
            if (isNeedEmgStop) {
                Logger(MLLogInfo, "<%s>: Emg stop.\n", __func__);
                return;
            }
        }
        
        // need no-block
        CheckAxisState(axises, axisCount, true);
    }
}

void MoveToInitPos(int *axises, int count) {
    int rtn;
    
    for (int i = 0; i < count; i++) {
        int axis = axises[i];
        switch (axis) {
            case MLAxisRotation: {
                rtn = smc_pmove_unit(gHandle, MLAxisRotation, -110, MLAbsolute);
                Logger(MLLogInfo, "<%s>: Axis %d go to init position %d pluse, rtn: (%d}\n", __func__,  MLAxisRotation, -110, rtn);
            }
                break;
            case MLAxisRotationX: {
                rtn = smc_pmove_unit(gHandle, MLAxisRotationX, -2500, MLAbsolute);
                Logger(MLLogInfo, "<%s>: Axis %d go to init position %d pluse, rtn: {%d}\n", __func__, MLAxisRotationX, -2500, rtn);
            }
                break;
            case MLAxisX: {
                rtn = smc_pmove_unit(gHandle, MLAxisX, 16000, MLAbsolute );
                Logger(MLLogInfo, "<%s>: Axis %d go to init position %d pluse, rtn: {%d}\n", __func__, MLAxisX, 16000, rtn);
            }
                break;
            case MLAxisY: {
                rtn = smc_pmove_unit(gHandle, MLAxisY, -10000, MLAbsolute);
                Logger(MLLogInfo, "<%s>: Axis %d go to init position %d pluse, rtn: {%d}\n", __func__, MLAxisY, -10000, rtn);
            }
                break;
            default:
                break;
        }
    }
}

void TManyAxisGoHome(void *args) {
    TArgs *arguments;
    arguments = (TArgs *)args;
    int *axises = (int *)arguments->arg1;
    int axisCount = arguments->arg2;
    
    bool block = false;
    
    for (int index = 0; index < axisCount; index++) {
        MLAxis axis = axises[index];
        Logger(MLLogInfo, "<%s>: Axis %d start go to limit ...\n", __func__, axis);
        AxisParam axisPrm = GetAxisParam(axis);
        JMoveAxis(axis, axisPrm.homeDirect ? 0 : 1, false);
        
        if (isNeedStop) {
            Logger(MLLogInfo, "<%s>: Stop button pressed when some axises go back to negative limit.\n", __func__);
            isActiveFinished = true;
            return;
        }
        if (isNeedEmgStop) {
            Logger(MLLogInfo, "<%s>: Emg stop.\n", __func__);
            return;
        }
    }
    
    if (CheckAxisState(axises, axisCount, false)) {
        Logger(MLLogInfo, "<%s>: Many axises have moved finish.\n", __func__);
    }
    
    if (isNeedStop) {
        Logger(MLLogInfo, "<%s>: Stop button pressed when some axises go back to negative limit.\n", __func__);
        isActiveFinished = true;
        return;
    }
    if (isNeedEmgStop) { Logger(MLLogInfo, "<%s>: Emg stop.\n", __func__); return; }
    for (int index = 0; index < axisCount; index++) {
        MLAxis axis = axises[index];
        Logger(MLLogInfo, "<%s>: Axis %d start go home ...\n", __func__, axis);
        AxisGoHome(axis, block);
        
        if (isNeedStop) {
            Logger(MLLogInfo, "<%s>: Stop button pressed when some axises go home.\n", __func__);
            isActiveFinished = true;
            return;
        }
        if (isNeedEmgStop) { Logger(MLLogInfo, "<%s>: Emg stop.\n", __func__); return;}
    }
    
    if (CheckAxisHomeState(axises, axisCount)) {
        Logger(MLLogInfo, "<%s>: Many axis success to go home\n", __func__);
    }
    
    if (isNeedStop) {
        Logger(MLLogInfo, "<%s>: Stop button pressed when some axises go home.\n", __func__);
        isActiveFinished = true;
        return;
    }
    if (isNeedEmgStop) { Logger(MLLogInfo, "<%s>: Emg stop.\n", __func__); return; }
    if (!block) {
        MoveToInitPos(axises, axisCount);
        
        if (isNeedStop) {
            Logger(MLLogInfo, "<%s>: Stop button pressed when some axises move the init position.\n", __func__);
        }
        if (isNeedEmgStop) { Logger(MLLogInfo, "<%s>: Emg stop.\n", __func__); return; }
    }
    
    isActiveFinished = true;
    Logger(MLLogInfo, "<%s>: Many axises finish go home\n", __func__);
}

void ManyAxisGoHome(int *axises, int axisCount, bool pthread) {
    bool isRuning = false;
    
    for (int ax = 0; ax < axisCount; ax++) {
        int axis = axises[ax];
        AxisParam axisPrm = GetAxisParam(axis);
        isRuning |= axisPrm.isRuning;
    }
    
    if (isRuning) {
        Logger(MLLogInfo, "<%s>: Some axis is running, please wait it.\n", __func__);
        return;
    }
    
    if (pthread) {
        TArgs *args = (TArgs *)malloc(sizeof(TArgs));
        args->arg1 = axises;
        args->arg2 = axisCount;
        
        pthread_t thrd;
        pthread_create(&thrd, NULL, (void *)TManyAxisGoHome, args);
    } else {
        Logger(MLLogInfo, "<%s>: Call `ManyAxisGoHome`\n", __func__);
        bool block = false;
        
        for (int index = 0; index < axisCount; index++) {
            MLAxis axis = axises[index];
            Logger(MLLogInfo, "<%s>: Axis %d start go to limit ...\n", __func__, axis);
            AxisParam axisPrm = GetAxisParam(axis);
            JMoveAxis(axis, axisPrm.homeDirect ? 0 : 1, false);
            
            if (isNeedStop) {
                Logger(MLLogInfo, "<%s>: Stop button pressed when some axises go home.\n", __func__);
                isActiveFinished = true;
                return;
            }
        }
        
        if (CheckAxisState(axises, axisCount, false)) {
            Logger(MLLogInfo, "<%s>: Many axises have moved finish.\n", __func__);
        }
        
        if (isNeedStop) {
            Logger(MLLogInfo, "<%s>: Stop button pressed when some axises go home.\n", __func__);
            isActiveFinished = true;
            return;
        }
        if (isNeedEmgStop) { Logger(MLLogInfo, "<%s>: Emg stop.\n", __func__); return; }
        
        for (int index = 0; index < axisCount; index++) {
            MLAxis axis = axises[index];
            Logger(MLLogInfo, "<%s>: Axis %d start go home ...\n", __func__, axis);
            AxisGoHome(axis, block);
            
            if (isNeedStop) {
                Logger(MLLogInfo, "<%s>: Stop button pressed when some axises go home.\n", __func__);
                isActiveFinished = true;
                return;
            }
            if (isNeedEmgStop) { Logger(MLLogInfo, "<%s>: Emg stop.\n", __func__); return; }
        }
        
        if (CheckAxisHomeState(axises, axisCount)) {
            Logger(MLLogInfo, "<%s>: Many axis success to go home\n", __func__);
        }
        
        if (!block) {
            MoveToInitPos(axises, axisCount);
            
            if (isNeedStop) {
                Logger(MLLogInfo, "<%s>: Stop button pressed when go to init position.\n", __func__);
                isActiveFinished = true;
                return;
            }
            if (isNeedEmgStop) { Logger(MLLogInfo, "<%s>: Emg stop.\n", __func__); return; }
        }
        Logger(MLLogInfo, "<%s>: Many axises finish go home\n", __func__);
    }
}

void ResetAxisSystem(void) {
    AllAxisStop();
    AllAxisGoHome(false);
}

void ConfigSpeed(int axis, int speed, int mode) {}

void AxisStop(int axis) {
    assert(axis >= 0);
    
    if (gHandle != -1) {
        smc_stop(gHandle, axis, 1);
    }
}

void TAllAxisStop() {
    isNeedStop = true;
    isNeedEmgStop = true;
    for (int axis = 0; axis < MLMaxAxisCount; axis++) {
        AxisStop(axis);
    }
    gIsTesting = false;
    Logger(MLLogInfo, "<%s>: All axis stop.\n", __func__);
}

void AllAxisStop(void) {
//    pthread_t thrd;
//    pthread_create(&thrd, NULL, (void *)TAllAxisStop, NULL);
    
    isNeedStop = true;
    isNeedEmgStop = true;
    for (int axis = 0; axis < MLMaxAxisCount; axis++) {
        AxisStop(axis);
    }
    Logger(MLLogInfo, "<%s>: All axis stop.\n", __func__);
}

void SetAxisRatio(int axis, double ratio) {
    assert(axis >= 0);
    
    AxisParam prm = gAxisPrm[axis];
    prm.ppratio = ratio;
    gAxisPrm[axis] = prm;
}

void SetBitState(int bit, MLLevel level) {
    if (gHandle != -1) {
        smc_write_outbit(gHandle, bit, level);
        
        if (bit == MLOutLeftDoorOpen && level == MLLow) {
            smc_write_outbit(gHandle, MLOutLeftDoorClose, MLHigh);
            while (!doorOpened) { usleep(500000); }
            smc_write_outbit(gHandle, MLOutLeftDoorOpen, MLHigh);
        }
        if (bit == MLOutLeftDoorClose && level == MLLow) {
//            doorOpened = false;
            doorClosing = true;
            smc_write_outbit(gHandle, MLOutLeftDoorOpen, MLHigh);
            while (doorOpened) { usleep(500000); }
            smc_write_outbit(gHandle, MLOutLeftDoorClose, MLHigh);
        }
        if (bit == MLOutCylinderHome && level == MLLow) {
            smc_write_outbit(gHandle, MLOutCylinderShop, MLHigh);
        }
        if (bit == MLOutCylinderShop && level == MLLow) {
            smc_write_outbit(gHandle, MLOutCylinderHome, MLHigh);
        }
    }
}

MLLevel GetBitState(int bit) {
    MLLevel level = MLLow;
    
    if (gHandle != -1) {
        level = smc_read_inbit(gHandle, bit);
    }
    
    return level;
}

void Cylinder(int bit, MLLevel level) {
    if (gHandle != -1) {
        smc_write_outbit(gHandle, bit, level);
    }
}

// Illuminometer API
bool ConnectIlluminometer(string portName) {
    assert(portName != NULL);   // 断言串口名不为空
    gLuxConnected = false;
    
    if (!JKConnectCL200A(portName, "00")) {
        Logger(MLLogError, "<%s>: Can't connect the illuminometer.\n", __func__);
        gLuxConnected = false;
        memset(errmsg, 0, 256);
        sprintf(errmsg, "Can't connect the luxmeter.\n");
    } else {
        Logger(MLLogInfo, "<%s>: Connect the illuminometer.\n", __func__);
        gLuxConnected = true;
    }

    return gLuxConnected;
}

void DisconnectIlluminometer() {
    int rtn;
    if (-1 != (rtn = JKDisconnectCL200A())) {
        gLuxConnected = false;
        Logger(MLLogInfo, "<%s>: Disconnect the illuminometer.\n", __func__);
    } else {
        Logger(MLLogError, "<%s>: Fail disconnecct illuminometer. Errorcode: %d\n", __func__, rtn);
        memset(errmsg, 0, 256);
        sprintf(errmsg, "Fail disconnect luxmeter,errorcode: %d.\n", rtn);
    }
}

double GetIlluminanceValue(int timeout) {
    float lv = 0;
    float x = 0;
    float y = 0;
    
    JKSetCL200ATimeout(timeout);
    JKGetCL200AEvXY(&lv, &x, &y);
    
    if (lv == 0) {
        Logger(MLLogInfo, "<%s>: Fail to get illuminance measure data.\n", __func__);
    }
    
    return lv;
}

double MLGetIlluminanceValueNT() {
    float lv = 0;
    float x = 0;
    float y = 0;
    
    JKGetCL200AEvXY(&lv, &x, &y);
    
    if (lv == 0) {
        Logger(MLLogInfo, "<%s>: Fail to get illuminance measure data.\n", __func__);
    }
    
    return lv;
}


double GetColorX(int timeout) {
    float lv = 0;
    float x = 0;
    float y = 0;
    
    JKSetCL200ATimeout(timeout);
    JKGetCL200AEvXY(&lv, &x, &y);
    
    if (lv == 0) {
        Logger(MLLogInfo, "<%s>: Fail to get illuminance measure data.\n", __func__);
    }
    
    return x;
}

double GetColorY(int timeout) {
    float lv = 0;
    float x = 0;
    float y = 0;
    
    JKSetCL200ATimeout(timeout);
    JKGetCL200AEvXY(&lv, &x, &y);
    
    if (lv == 0) {
        Logger(MLLogInfo, "<%s>: Fail to get illuminance measure data.\n", __func__);
    }
    
    return y;
}

void GetColorLvxy(float *lv, float *x, float *y, int timeout) {
    JKSetCL200ATimeout(timeout);
    JKGetCL200AEvXY(lv, x, y);
    
    if (*lv == 0 || *x == 0 || *y == 0) {
        Logger(MLLogInfo, "<%s>: Fail to get illuminance measure data.\n", __func__);
    }
}

void ConfigAnglePortName(string portName) {
    assert(portName != NULL);
    sprintf(anglePortName, "%s", portName);
    PutStringValue("angle", "portname", portName);
}

void ConfigIlluminometerPortName(string portName) {
    assert(portName != NULL);
    sprintf(illuminometerPortName, "%s", portName);
    PutStringValue("illumionmeter", "portname", portName);
}

void CheckInputSignal() {
    gStopped = false;
    pthread_t thrd;
    pthread_create(&thrd, NULL, (void *)CheckSensor, NULL);
}

void DoorOpen() {
    if (gHandle != -1) {
        doorOpened = false;
        smc_write_outbit(gHandle, MLOutLeftDoorOpen, MLLow);
        smc_write_outbit(gHandle, MLOutLeftDoorClose, MLHigh);
        Logger(MLLogInfo, "<%s>: Auto door opening\n", __func__);
        
        int time = 0;
        while (!doorOpened && time < 7000) { time += 500; usleep(500000); }
        smc_write_outbit(gHandle, MLOutLeftDoorOpen, MLHigh);
        smc_write_outbit(gHandle, MLOutLight, MLLow);      // 打开日光灯
        
        if (time >= 7000) {
            Logger(MLLogError, "<%s>: Undetected auto-door status signal after OPEN the door\n", __func__);
        }
        
        Logger(MLLogInfo, "<%s>: Auto door has opened\n", __func__);
    }
}

void DoorClose() {
    if (gHandle != -1) {
//        doorOpened = false;
        doorClosing = true;
        doorOpened = true;
        smc_write_outbit(gHandle, MLOutLight, MLHigh);       // 关闭日光灯
        smc_write_outbit(gHandle, MLOutLeftDoorOpen, MLHigh);
        smc_write_outbit(gHandle, MLOutLeftDoorClose, MLLow);
        //        smc_write_outbit(gHandle, MLOutRightDoorOpen, MLHigh);
        //        smc_write_outbit(gHandle, MLOutRightDoorClose, MLLow);
        Logger(MLLogInfo, "<%s>: Auto door closing\n", __func__);
        
        int time = 0;
        while (doorOpened && time < 7000) { time += 500; usleep(500000); }
        smc_write_outbit(gHandle, MLOutLeftDoorClose, MLHigh);
        
        if (time >= 7000) {
            Logger(MLLogError, "<%s>: Undetected auto-door status signal after CLOSE the door\n", __func__);
        }
        
        Logger(MLLogInfo, "<%s>: Auto door has closed\n", __func__);
    }
}

// union API
int CheckSensor() {
    pthread_mutex_t mutex;
    pthread_mutex_init(&mutex, NULL);
    
    if (gHandle != -1) {
        Logger(MLLogInfo, "<%s>: Start listen input signal...\n", __func__);
        while (!gStopped) {
            DWORD state = 0;
            
            pthread_mutex_lock(&mutex);
            state = smc_read_inport(gHandle, 0);
            pthread_mutex_unlock(&mutex);
            
            for (int i = 0; i < 8; i++) {
                if (((state >> i) & 0x01) == MLLow) {
                    Logger(MLLogError, "<%s>: Servo {%d} alarm.\n", __func__, i+1);
                    memset(errmsg, 0, 256);
                    sprintf(errmsg, "Servo {%d} alarm.\n", i+1);
                    smc_emg_stop(gHandle);
                    gIsTesting = false;
//                    return 1;
                }
            }
            
            if (((state >> MLInDoorLeftOpened) & 0x01) == MLLow && ((state >> MLInDoorLeftClosed) & 0x01) == MLHigh) {
                doorOpened = false;
                doorClosing = false;
            }
            
            if (((state >> MLInDoorLeftOpened) & 0x01) == MLHigh && ((state >> MLInDoorLeftClosed) & 0x01) == MLLow) {
                doorOpened = true;
                doorClosing = true;
            }
            
            if (((state >> MLInCylinderShop) & 0x01) == MLLow && ((state >> MLInCylinderHome) & 0x01) == MLHigh) {
                lightTestReadyed = true;
            }
            
            if (((state >> MLInCylinderShop) & 0x01) == MLHigh && ((state >> MLInCylinderHome) & 0x01) == MLLow) {
                lightTestReadyed = false;
            }
            
            if (((state >> MLInRaster) & 0x01) == MLHigh) {
//                gIsRaster = true;
//                if (doorOpened||doorClosing) {
                if (doorClosing) {
                    // open door
                    smc_write_outbit(gHandle, MLOutLeftDoorOpen, MLLow);
                    smc_write_outbit(gHandle, MLOutLeftDoorClose, MLHigh);
                }
                if (gIsTesting) {
                    Logger(MLLogWarning, "<%s>: Grating alarm.\n", __func__);
                }
            }
            if (((state >> MLInEmgStop) & 0x01) == MLHigh) {
                if (gIsPowerOn) {
                    Logger(MLLogInfo, "<%s>: EMG stop button pressed.\n", __func__);
                    smc_emg_stop(gHandle);
                    isNeedEmgStop = true;
                    AllAxisStop();
                    gIsTesting = false;
                }
            }
            if (((state >> MLInStop) & 0x01) == MLLow) {
                if (gIsPowerOn) {
                    Logger(MLLogInfo, "<%s>: Stop button pressed.\n", __func__);
                    AllAxisStop();
                    gIsStopTest = true;
                    gIsTesting = false;
                    isNeedStop = true;
                }
            }else{
                gIsStopTest = false;
            }
            if (((state >> MLInReset) & 0x01) == MLLow) {
                if (gIsPowerOn) {
                    Logger(MLLogInfo, "<%s>: Reset button pressed.\n", __func__);
                    usleep(200000);
                    gIsOpenDoor = true;
                    gIsTesting = false;
                }
                
                if (doorOpened) {
                    smc_write_outbit(gHandle, MLOutLeftDoorOpen, MLHigh);
                    smc_write_outbit(gHandle, MLOutLeftDoorClose, MLLow);
                } else {
                    smc_write_outbit(gHandle, MLOutLeftDoorOpen, MLLow);
                    smc_write_outbit(gHandle, MLOutLeftDoorClose, MLHigh);
                }
            }
            if (((state >> MLInStart) & 0x01) == MLLow) {
                if (gIsTesting && gIsPowerOn) {
                    Logger(MLLogInfo, "<%s>: start button pressed.\n", __func__);
                    gIsTesting = false;
                }
                if (gIsPowerOn) {
                    gIsStartTest = true;
                }
            } else {
                gIsStartTest = false;
            }
            usleep(100000);
        }
    }
    Logger(MLLogInfo, "<%s>: Stop listen input signal...\n", __func__);
    pthread_mutex_destroy(&mutex);
    
    return 0;
}

void CheckEmgStop(){
    pthread_t thrd;
    pthread_create(&thrd, NULL, (void*)CheckEmgStopSignal, NULL);
}

void CheckStopActivedSignal(){
    pthread_t thrd;
    pthread_create(&thrd, NULL, (void*)IsStopActived, NULL);
}

bool CheckEmgStopSignal() {
    
    return gIsEmgStopTest;
}

void CheckStart(){
    pthread_t thrd;
    pthread_create(&thrd, NULL, (void*)CheckStartSignal, NULL);
}

bool CheckStartSignal() {

//    pthread_mutex_t mutex;
//    pthread_mutex_init(&mutex, NULL);
//    while (!gIsStartTest) {
//        usleep(10000);
//    }
//    pthread_mutex_destroy(&mutex);
   return gIsStartTest;
}

void CheckStop(){
    pthread_t thrd;
    pthread_create(&thrd, NULL, (void*)CheckStopSignal, NULL);
}

bool CheckStopSignal() {
//    pthread_mutex_t mutex;
//    pthread_mutex_init(&mutex, NULL);
//    while (!gIsStopTest) {
//        usleep(10000);
//    }
//    pthread_mutex_destroy(&mutex);
    return gIsStopTest;
}

void CheckDoor(){
    if (gIsOpenDoor) {
        smc_write_outbit(gHandle, MLOutLeftDoorOpen, MLLow);
        smc_write_outbit(gHandle, MLOutLeftDoorClose, MLHigh);
    }
}

double GetLoadCellValue(string portName) {
    assert(portName != NULL);
    
    double value = 0;
   int fd = ConnectXJCPort(portName);
//    usleep(50000);
    OneByOneSend(fd);
//    usleep(50000);
    value = ReadXJCExisting(fd,"@");
//    usleep(50000);
    DisconnectXJCPort(fd);
//    usleep(50000);

    return value;
}

void HoldOneDUTDemo(int dutType,int force1,int force2) {
    assert(frontCellPortName != NULL);
    assert(backCellPortName != NULL);
    assert(leftCellPortName != NULL);
    assert(rightCellPortName != NULL);
//    if (gIsTesting) { return; }

//    gIsTesting = true;

    if (gHandle != -1) {
        HoldDut(dutType);
    }
    
//    gIsTesting = false;

}

void AdjustAxisPosistion(int direction, int step) {
    int direct = 1;
    switch (direction) {
        case 0: {   // go left
            int axises[] = { MLAxisClampLeft, MLAxisClampRight };
            direct = 1;
            MoveAxisDistance(axises[0], direct * abs(step), false, false);
            MoveAxisDistance(axises[1], -direct * abs(step), false, false);
            CheckAxisState(axises, 2, false);
            break;
        }
        case 1: {   // go right
            int axises[] = { MLAxisClampLeft, MLAxisClampRight };
            direct = -1;
            MoveAxisDistance(axises[0], direct * abs(step), false, false);
            MoveAxisDistance(axises[1], -direct * abs(step), false, false);
            CheckAxisState(axises, 2, false);
            break;
        }
        case 2: {   // go front
            int axises[] = { MLAxisClampUp, MLAxisClampDown };
            direct = -1;
            MoveAxisDistance(axises[0], direct * abs(step), false, false);
            MoveAxisDistance(axises[1], -direct * abs(step), false, false);
            CheckAxisState(axises, 2, false);
            break;
        }
        case 3: {   // go back
            int axises[] = { MLAxisClampUp, MLAxisClampDown };
            direct = 1;
            MoveAxisDistance(axises[0], direct * abs(step), false, false);
            MoveAxisDistance(axises[1], -direct * abs(step), false, false);
            CheckAxisState(axises, 2, false);
            break;
        }
        default: break;
    }
    
    gIsTesting = false;
    
}

int MLConnectXJCPort(const char* portName) {
    
    return ConnectXJCPort(portName);
}

double MLReadXJCExisting(int fd,char *str){
    int count = 0;
    double value = 0;
    
    do {
        value = ReadXJCExisting(fd,"@");
        
        if (value != -997 && value != -998) {
            break;
        }
    } while (count < 1);
    
    return value;
}

void MLDisconnectXJCPort(int fd){
    DisconnectXJCPort(fd);
}

void MLZeroXJCValue(int fd) {
    ZeroXJC(fd);
}

void ManualHoldDut(int dutType, double force1, double force2) {
    if (gIsTesting) {
        return;
    }
    double beforMoveForce1 = 0;
    double afterMoveForce1 = 0;
    double beforMoveForce2 = 0;
    double afterMoveForce2 = 0;
    bool isStop1 = false;
    bool isStop2 = false;
    bool isStop3 = false;
    bool isStop4 = false;
    
    assert(frontCellPortName != NULL);
    assert(backCellPortName != NULL);
    assert(leftCellPortName != NULL);
    assert(rightCellPortName != NULL);
    
    gIsTesting = true;
    
    int fd = MLConnectXJCPort(leftCellPortName);
    int fd1 = MLConnectXJCPort(frontCellPortName);
    XJCSend(fd,2);
    XJCSend(fd1,2);
    const char *pFileName = "/vault/FlareLog.txt";
    FILE *pFile;
    pFile = fopen(pFileName,"a");
    if (NULL == pFile) {
        printf("error");
    }
    
    usleep(200000);
    
    if (gHandle != -1) {
        for (int i = 0; i < 3; i++) {
            beforMoveForce1 = MLReadXJCExisting(fd,"@");
            
            if (beforMoveForce1 == -998) { continue; }
            if (beforMoveForce1 != -999) {
                break;
            }
        }
        
        for (int i = 0; i < 3; i++) {
            beforMoveForce2 = MLReadXJCExisting(fd1,"@");
            
            if (beforMoveForce2 == -998) {continue;}
            if (beforMoveForce2 != -999) {
                break;
            }
        }
        
        int axises[] = { MLAxisClampLeft, MLAxisClampUp };
        JMoveAxis(axises[0], 1, false);
        JMoveAxis(axises[1], 1, false);
        if (isNeedStop) {return;}
        if (isNeedEmgStop) {return;}
        do{
            if (!isStop1) {
                afterMoveForce1 = MLReadXJCExisting(fd,"@");
                if (afterMoveForce1 == -998) { continue; }
                
                double offset =   afterMoveForce1 - beforMoveForce1 ;
                printf("left = %lf\n", afterMoveForce1);
                
                if (offset >= 0.2 || beforMoveForce1 == -999 ||afterMoveForce1 == -999 ) {
                    AxisStop(axises[0]);
                    isStop1 = true;
                    
                    if ( beforMoveForce1 == -999 ||afterMoveForce1 == -999) {
                        printf("left error\n");
                    }
                    else{
                        printf("[OK] left:%lf\n",offset);
                        fprintf(pFile,"<%s>: left value %lf\n", __func__, offset);

                    }
                }
            }
            
            if (!isStop2) {
                afterMoveForce2 = MLReadXJCExisting(fd1,"@");
                if (afterMoveForce2 == -998) {continue;}
                
                double offset1 =   afterMoveForce2 - beforMoveForce2 ;
                printf("front = %lf\n", afterMoveForce2);
                
                if (offset1 >= 0.2 || beforMoveForce2 == -999 ||afterMoveForce2 == -999 ) {
                    AxisStop(axises[1]);
                    isStop2 = true;
                    
                    if (beforMoveForce2 == -999 ||afterMoveForce2 == -999) {
                        printf("front error\n");
                    }
                    else{
                        printf("[OK] front:%lf\n",offset1);
                        fprintf(pFile,"<%s>: front value %lf\n", __func__, offset1);

                    }
                }
            }
        } while (!isStop1||!isStop2);
        
        MLDisconnectXJCPort(fd);
        MLDisconnectXJCPort(fd1);
        
        beforMoveForce1 = beforMoveForce2 = 0;
        afterMoveForce1 = afterMoveForce2 = 0;
        
        int fd2 = MLConnectXJCPort(rightCellPortName);
        int fd3 = MLConnectXJCPort(backCellPortName);
        XJCSend(fd2,2);
        XJCSend(fd3,2);
        for (int i = 0; i < 3; i++) {
            beforMoveForce1 = MLReadXJCExisting(fd2,"@");
            
            if (beforMoveForce1 == -998) {continue;}
            if (beforMoveForce1 != -999) {
                break;
            }
        }
        
        for (int i = 0; i < 3; i++) {
            beforMoveForce2 = MLReadXJCExisting(fd3,"@");
            
            if (beforMoveForce2 == -998) {continue;}
            if (beforMoveForce2 != -999) {
                break;
            }
        }
        
        int axises1[] = { MLAxisClampRight, MLAxisClampDown };
        JHoldMoveAxis(axises1[0], 1);
        JHoldMoveAxis(axises1[1], 1);
        if (isNeedStop) {return;}
        if (isNeedEmgStop) {return;}
        do{
            if (!isStop3) {
                afterMoveForce1 = MLReadXJCExisting(fd2,"@");
                if (afterMoveForce1 == -998) { continue; }
                
                double offset =  afterMoveForce1 - beforMoveForce1 ;
                printf("right = %lf\n", afterMoveForce1);
                
                if (offset >= 0.2 || beforMoveForce1 == -999 ||afterMoveForce1 == -999 ) {
                    AxisStop(axises1[0]);
                    isStop3 = true;
                    if (beforMoveForce1 == -999 ||afterMoveForce1 == -999) {
                        printf("right error\n");
                    }else{
                        printf("[OK] right:%lf\n",offset);
                        fprintf(pFile,"<%s>: right value %lf\n", __func__, offset);

                    }
                    
                }
            }
            
            if (!isStop4) {
                afterMoveForce2 = MLReadXJCExisting(fd3,"@");
                if (afterMoveForce2 == -998) { continue; }
                
                double offset1 =  afterMoveForce2 - beforMoveForce2 ;
                printf("back = %lf\n", afterMoveForce2);
                
                if (offset1 >= 0.2 || beforMoveForce2 == -999 ||afterMoveForce2 == -999 ) {
                    AxisStop(axises1[1]);
                    isStop4 = true;
                    
                    if (beforMoveForce2 == -999 ||afterMoveForce2 == -999) {
                        printf("back error\n");
                    }else{
                        printf("[OK] back:%lf\n",offset1);
                        fprintf(pFile,"<%s>: back value %lf\n", __func__, offset1);

                    }
                }
            }
        } while (!isStop3||!isStop4);
        
        beforMoveForce1 = beforMoveForce2 = 0;
        afterMoveForce1 = afterMoveForce2 = 0;
        
        fclose(pFile);
        
        MLDisconnectXJCPort(fd2);
        MLDisconnectXJCPort(fd3);
        
        printf("finished.\n");
    
        } else {
            Logger(MLLogError, "<%s>: Connect controller firstly.\n", __func__);
        }
    gIsTesting = false;
    
}

void MLManualHoldDut(double force1, double force2) {
    if (gIsTesting) {
        return;
    }
    double beforMoveForce1 = 0;
    double afterMoveForce1 = 0;
    double beforMoveForce2 = 0;
    double afterMoveForce2 = 0;
    bool isStop1 = false;
    bool isStop2 = false;
    bool isStop3 = false;
    bool isStop4 = false;
    
    assert(frontCellPortName != NULL);
    assert(backCellPortName != NULL);
    assert(leftCellPortName != NULL);
    assert(rightCellPortName != NULL);
    
    gIsTesting = true;
    
    int fd = MLConnectXJCPort(leftCellPortName);
    int fd1 = MLConnectXJCPort(frontCellPortName);
    
    const char *pFileName = "/vault/FlareLog.txt";
    FILE *pFile;
    pFile = fopen(pFileName,"a");
    if (NULL == pFile) {
        printf("error");
    }
    
    usleep(200000);
    
    if (gHandle != -1) {
        for (int i = 0; i < 3; i++) {
            beforMoveForce1 = MLReadXJCExisting(fd,"@");
            
            if (beforMoveForce1 == -998) { continue; }
            if (beforMoveForce1 != -999) {
                break;
            }
        }
        
        for (int i = 0; i < 3; i++) {
            beforMoveForce2 = MLReadXJCExisting(fd1,"@");
            
            if (beforMoveForce2 == -998) {continue;}
            if (beforMoveForce2 != -999) {
                break;
            }
        }
        
        int axises[] = { MLAxisClampLeft, MLAxisClampUp };
        JMoveAxis(axises[0], 1, false);
        JMoveAxis(axises[1], 1, false);
        if (isNeedStop) {return;}
        if (isNeedEmgStop) {return;}
        do{
            if (!isStop1) {
                afterMoveForce1 = MLReadXJCExisting(fd,"@");
                if (afterMoveForce1 == -998) { continue; }
                
                double offset =   afterMoveForce1 - beforMoveForce1 ;
                printf("left = %lf\n", afterMoveForce1);
                
                if (offset >= force1 || beforMoveForce1 == -999 ||afterMoveForce1 == -999 ) {
                    AxisStop(axises[0]);
                    isStop1 = true;
                    
                    if ( beforMoveForce1 == -999 ||afterMoveForce1 == -999) {
                        printf("left error\n");
                    }
                    else{
                        printf("[OK] left:%lf\n",offset);
                        fprintf(pFile,"<%s>: left value %lf\n", __func__, offset);
                        
                    }
                }
            }
            
            if (!isStop2) {
                afterMoveForce2 = MLReadXJCExisting(fd1,"@");
                if (afterMoveForce2 == -998) {continue;}
                
                double offset1 =   afterMoveForce2 - beforMoveForce2 ;
                printf("front = %lf\n", afterMoveForce2);
                
                if (offset1 >= force1 || beforMoveForce2 == -999 ||afterMoveForce2 == -999 ) {
                    AxisStop(axises[1]);
                    isStop2 = true;
                    
                    if (beforMoveForce2 == -999 ||afterMoveForce2 == -999) {
                        printf("front error\n");
                    }
                    else{
                        printf("[OK] front:%lf\n",offset1);
                        fprintf(pFile,"<%s>: front value %lf\n", __func__, offset1);
                        
                    }
                }
            }
        } while (!isStop1||!isStop2);
        
        MLDisconnectXJCPort(fd);
        MLDisconnectXJCPort(fd1);
        
        beforMoveForce1 = beforMoveForce2 = 0;
        afterMoveForce1 = afterMoveForce2 = 0;
        
        int fd2 = MLConnectXJCPort(rightCellPortName);
        int fd3 = MLConnectXJCPort(backCellPortName);
        
        for (int i = 0; i < 3; i++) {
            beforMoveForce1 = MLReadXJCExisting(fd2,"@");
            
            if (beforMoveForce1 == -998) {continue;}
            if (beforMoveForce1 != -999) {
                break;
            }
        }
        
        for (int i = 0; i < 3; i++) {
            beforMoveForce2 = MLReadXJCExisting(fd3,"@");
            
            if (beforMoveForce2 == -998) {continue;}
            if (beforMoveForce2 != -999) {
                break;
            }
        }
        
        int axises1[] = { MLAxisClampRight, MLAxisClampDown };
        JHoldMoveAxis(axises1[0], 1);
        JHoldMoveAxis(axises1[1], 1);
        if (isNeedStop) {return;}
        if (isNeedEmgStop) {return;}
        do{
            if (!isStop3) {
                afterMoveForce1 = MLReadXJCExisting(fd2,"@");
                if (afterMoveForce1 == -998) { continue; }
                
                double offset =  afterMoveForce1 - beforMoveForce1 ;
                printf("right = %lf\n", afterMoveForce1);
                
                if (offset >= force2 || beforMoveForce1 == -999 ||afterMoveForce1 == -999 ) {
                    AxisStop(axises1[0]);
                    isStop3 = true;
                    if (beforMoveForce1 == -999 ||afterMoveForce1 == -999) {
                        printf("right error\n");
                    }else{
                        printf("[OK] right:%lf\n",offset);
                        fprintf(pFile,"<%s>: right value %lf\n", __func__, offset);
                        
                    }
                    
                }
            }
            
            if (!isStop4) {
                afterMoveForce2 = MLReadXJCExisting(fd3,"@");
                if (afterMoveForce2 == -998) { continue; }
                
                double offset1 =  afterMoveForce2 - beforMoveForce2 ;
                printf("back = %lf\n", afterMoveForce2);
                
                if (offset1 >= force2 || beforMoveForce2 == -999 ||afterMoveForce2 == -999 ) {
                    AxisStop(axises1[1]);
                    isStop4 = true;
                    
                    if (beforMoveForce2 == -999 ||afterMoveForce2 == -999) {
                        printf("back error\n");
                    }else{
                        printf("[OK] back:%lf\n",offset1);
                        fprintf(pFile,"<%s>: back value %lf\n", __func__, offset1);
                        
                    }
                }
            }
        } while (!isStop3||!isStop4);
        
        beforMoveForce1 = beforMoveForce2 = 0;
        afterMoveForce1 = afterMoveForce2 = 0;
        
        fclose(pFile);
        
        MLDisconnectXJCPort(fd2);
        MLDisconnectXJCPort(fd3);
        
        printf("finished.\n");
        
    } else {
        Logger(MLLogError, "<%s>: Connect controller firstly.\n", __func__);
    }
    gIsTesting = false;
    
}

bool HoldDut(int dutType) {
    if (gIsTesting) { return false; }
    
    int flag = 0;
    gIsTesting = true;
    
    assert(frontCellPortName != NULL);
    assert(rightCellPortName != NULL);
    
    if (gHandle != -1) {
        MLDUT dut = duts[dutType];
        
        int axises[] = {MLAxisClampLeft, MLAxisClampUp};
        MoveAxisToPosition(MLAxisClampLeft, dut.posHoldLeft, false, false);
        MoveAxisToPosition(MLAxisClampUp, dut.posHoldFront, false, false);
        if (isNeedStop) {return 0;}
        if (isNeedEmgStop) {return 0;}
        while (!CheckAxisState(axises, 2, false)) { usleep(100000); }
        
        int axises1[] = {MLAxisClampDown, MLAxisClampRight};
        MoveAxisToPosition(MLAxisClampRight, dut.posStandbyRight, false, false);
        MoveAxisToPosition(MLAxisClampDown, dut.posStandbyBack, false, false);
        if (isNeedStop) {return 0;}
        if (isNeedEmgStop) {return 0;}
        while (!CheckAxisState(axises1, 2, false)) { usleep(100000); }
        
        AxisParam prmRight = gAxisPrm[MLAxisClampRight];
        AxisParam prmBack = gAxisPrm[MLAxisClampDown];
        
        smc_change_speed_unit(gHandle, MLAxisClampDown, prmBack.runSpeed / 2, prmBack.accTime);
        smc_change_speed_unit(gHandle, MLAxisClampRight, prmRight.runSpeed / 2, prmRight.accTime);
        
        MoveAxisToPosition(MLAxisClampRight, dut.posHoldRight, false, false);
        MoveAxisToPosition(MLAxisClampDown, dut.posHoldBack, false, false);
        if (isNeedStop) {return 0;}
        if (isNeedEmgStop) {return 0;}
        while (!CheckAxisState(axises1, 2, false)) { usleep(100000); }
      
        
        int axises3[] = {MLAxisX, MLAxisY,MLAxisLifter};
        MoveAxisToPosition(MLAxisX, dut.posAxisX, false, false);
        MoveAxisToPosition(MLAxisY, dut.posAxisY, false, false);
        MoveAxisToPosition(MLAxisLifter, dut.posLifter, false, false);
        if (isNeedStop) {return 0;}
        if (isNeedEmgStop) {return 0;}
        while (!CheckAxisState(axises3, 3, false)) { usleep(100000); }
        
        /*
        while (flag != 2) {
            if (currentForce1 < force1) {
                step1 = abs(step1); // push direct
                MoveAxisDistance(MLAxisClampDown, step1, true);
                currentForce1 = GetLoadCellValue(frontCellPortName);
            } else {
                if (((currentForce1 - force1) / force1) > 0.2) {
                    step1 /= 2;
                    step1 = (abs(step1) > 0) ? -step1 : -1;
                } else {
                    flag++;
                }
            }
            
            if (currentForce2 < force2) {
                step2 = abs(step2);
                MoveAxisDistance(MLAxisClampRight, step2, true);
                currentForce2 = GetLoadCellValue(rightCellPortName);
            } else {
                if (((currentForce2 - force2) / force2) > 0.2) {
                    step2 /= 2;
                    step2 = (abs(step2) > 0) ? -step2 : -1;
                } else {
                    flag++;
                }
            }
            
            times++;
            
            if (times > 20000) {    // 2w time to check loadcell value.
                break;
            }
        }
        */
        smc_change_speed_unit(gHandle, MLAxisClampDown, prmBack.runSpeed, prmBack.accTime);
        smc_change_speed_unit(gHandle, MLAxisClampRight, prmRight.runSpeed, prmRight.accTime);
        flag = 2;
    }
    
    gIsTesting = false;
    return (flag == 2) ? true : false;
}

void UnholdDut(int dutType) {
    if (gHandle != -1) {
        gIsTesting = true;

        MLDUT dut = duts[dutType];
        int axises[] = {MLAxisClampLeft, MLAxisClampUp,  MLAxisClampDown, MLAxisClampRight};
        int axises1[] = {MLAxisClampLeft, MLAxisClampUp};
        ManyAxisGoBack(axises, 4, false);
        MoveAxisToPosition(axises[0], dut.posStandbyLeft, false, false);
        MoveAxisToPosition(axises[1], dut.posStandbyFront, false, false);
        if (isNeedStop) {return;}
        if (isNeedEmgStop) {return;}
//        MoveAxisToPosition(axises[2], dut.posStandbyBack, false);
//        MoveAxisToPosition(axises[3], dut.posStandbyRight, false);
        while (!CheckAxisState(axises1, 2, false)) { usleep(100000); }
    }
}

void ConfigLaserPortName(string portName) {
    assert(portName != NULL);
    sprintf(laserPortName, "%s", portName);
    PutStringValue("laser", "portname", portName);
}

double MLLaserCalibrator(double offset) {
    assert(laserPortName != NULL);
    
    if (gIsTesting) { return -1; }
    
    int axises1[] = {5,6,7,8,9};
    ManyAxisGoBack(axises1, 5, false);
    
    int axises[] = {10};
    ManyAxisGoHome(axises, 1, false);
    
    int times = 0;
    double height1, height2;
    double diff = -1;
    int direct = 1;
    double off = fabs(offset);
    
    SetBitState(25,0);
    SetBitState(14,0);
    
    gIsTesting = true;
    
    short res = 0;
    WORD MyCardNo = 0;
    double Myencoder_value=0;
    
    if (gHandle != -1) {
        AxisParam param = gAxisPrm[MLAxisLaser];
        
        if (!ConnectDLRS1A(laserPortName)) {
            DisconnectDLRS1A();
            Logger(MLLogInfo, "<%s>: Start laser calibrate.\n", __func__);
            MoveAxisToPosition(MLAxisLaser, param.posToInit, true, false);
            usleep(500000);
            height1 = GetLaserDiffOfHeight(laserPortName);
            usleep(500000);
            height1 = GetLaserDiffOfHeight(laserPortName);
            usleep(500000);
            height1 = GetLaserDiffOfHeight(laserPortName);
            usleep(500000);
            height1 = GetLaserDiffOfHeight(laserPortName);
            MoveAxisToPosition(MLAxisLaser, param.posToTest-40, true, false);
            
            do {
                height2 = GetLaserDiffOfHeight(laserPortName);
                diff = fabs(height1 - height2);
                direct = (height1 - height2) > 0 ? 1 : -1;
                
                MoveAxisDistance(MLAxisLifter,diff*direct, true, false);
                if (isNeedStop) {return 0;}
                if (isNeedEmgStop) {return 0;}
                res=smc_get_encoder_unit(MyCardNo,4,&Myencoder_value);//读编码值
                times++;
            } while ((diff > off) && (times < 100));
            DisconnectDLRS1A();
            Logger(MLLogInfo, "<%s>: Laser calibration finished.\n", __func__);
        } else {
            Logger(MLLogError, "<%s>: Not connect the laser device.\n", __func__);
            memset(errmsg, 0, 256);
            sprintf(errmsg, "Not connect the laser device.\n");
        }
        
        MoveAxisToPosition(MLAxisLaser, param.posToTest, true, false);
//        smc_write_outbit(gHandle, MLOutLightPower, MLLow);
        
        //        MoveAxisToPosition(MLAxisLaser, 0, true);   // back to init position
        Logger(MLLogInfo, "<%s>: Laser calibrate done.\n", __func__);
    }
    
    gIsTesting = false;
    
    return diff;
}

double LaserCalibrator(double offset) {
    assert(laserPortName != NULL);
    
    if (gIsTesting) { return -1; }
    
    int axises1[] = {5,6,7,8,9};
    ManyAxisGoBack(axises1, 5, false);
    
    int axises[] = {10};
    ManyAxisGoHome(axises, 1, false);
    
    int times = 0;
    double height1, height2;
    double diff = -1;
    int direct = 1;
    double off = fabs(offset);
    
    SetBitState(25,0);
    SetBitState(14,0);
    
    gIsTesting = true;
    
    short res = 0;
    WORD MyCardNo = 0;
    double Myencoder_value=0;
    
    if (gHandle != -1) {
        AxisParam param = gAxisPrm[MLAxisLaser];
        
        if (!ConnectDLRS1A(laserPortName)) {
            Logger(MLLogInfo, "<%s>: Start laser calibrate.\n", __func__);
            MoveAxisToPosition(MLAxisLaser, param.posToInit, true, false);
            usleep(500000);
            height1 = GetLaserDiffOfHeight(laserPortName);
            usleep(500000);
            height1 = GetLaserDiffOfHeight(laserPortName);
            usleep(500000);
            height1 = GetLaserDiffOfHeight(laserPortName);
            usleep(500000);
            height1 = GetLaserDiffOfHeight(laserPortName);
            MoveAxisToPosition(MLAxisLaser, param.posToTest-40, true, false);
            if (isNeedStop) {return 0;}
            if (isNeedEmgStop) {return 0;}
            do {
                height2 = GetLaserDiffOfHeight(laserPortName);
                diff = fabs(height1 - height2);
                direct = (height1 - height2) > 0 ? 1 : -1;
                
                MoveAxisDistance(MLAxisLifter,2*direct, true, false);
                res=smc_get_encoder_unit(MyCardNo,4,&Myencoder_value);//读编码值
                times++;
            } while ((diff > off) && (times < 100));
            DisconnectDLRS1A();
            Logger(MLLogInfo, "<%s>: Laser calibration finished.\n", __func__);
        } else {
            Logger(MLLogError, "<%s>: Not connect the laser device.\n", __func__);
            memset(errmsg, 0, 256);
            sprintf(errmsg, "Not connect the laser device.\n");
        }
        
        MoveAxisToPosition(MLAxisLaser, param.posToTest, true, false);
        smc_write_outbit(gHandle, MLOutLightPower, MLLow);

        //        MoveAxisToPosition(MLAxisLaser, 0, true);   // back to init position
        Logger(MLLogInfo, "<%s>: Laser calibrate done.\n", __func__);
    }
    
    gIsTesting = false;
    
    return diff;
}



void DebugLaserCalibrator(int testCount) {
    assert(laserPortName != NULL);
    
    int rtn = 0;
    int iostate = CheckAxisIOState(9);
    
    AxisParam param = gAxisPrm[9];
    
    rtn |= smc_write_sevon_pin(gHandle, 9, 0);
    sleep(1);
    rtn |= smc_set_profile_unit(gHandle, 9, param.startSpeed*param.ppratio, param.runSpeed*param.ppratio, param.accTime, param.accTime, param.stopSpeed);
    rtn |= smc_set_s_profile(gHandle, 9,0,0);
    
    if (iostate != 2) {
        
        rtn |= smc_vmove(gHandle, 9, 0);
    }
    while (smc_check_done(gHandle, 9)==0) {
        usleep(200000);
    };
    
    
    for (int i = 0; i < testCount; i++) {
        
        const char *pFileName = "/vault/FlareLog.txt";
        FILE *pFile;
        pFile = fopen(pFileName,"a");
        if (NULL == pFile) {
            printf("error");
        }
        
        int times = 0;
        double height1, height2;
        double diff = -1;
        int direct = 1;
        double off = 1;
        smc_write_outbit(gHandle, MLOutLaserPower, MLLow);
        smc_write_outbit(gHandle, MLOutSpotPower, MLLow);
        
        gIsTesting = true;
        
        short res = 0;
        WORD MyCardNo = 0;
        double Myencoder_value=0;
        
        if (gHandle != -1) {
            AxisParam param = gAxisPrm[MLAxisLaser];
            
            if (!ConnectDLRS1A(laserPortName)) {
                Logger(MLLogInfo, "<%s>: Start laser calibrate.\n", __func__);
                MoveAxisToPosition(MLAxisLaser, param.posToInit, true, false);
                usleep(500000);
                height1 = GetDLRS1AM0MeasureValue();
                usleep(500000);
                height1 = GetDLRS1AM0MeasureValue();
                usleep(500000);
                height1 = GetDLRS1AM0MeasureValue();
                usleep(500000);
                height1 = GetDLRS1AM0MeasureValue();
                MoveAxisToPosition(MLAxisLaser, param.posToTest-40, true, false);
                
                do {
                    height2 = GetDLRS1AM0MeasureValue();
                    diff = fabs(height1 - height2);
                    direct = (height1 - height2) > 0 ? 1 : -1;
                    
                    MoveAxisDistance(MLAxisLifter,2*direct, true, false);
                    res=smc_get_encoder_unit(MyCardNo,9,&Myencoder_value);//读编码值
                    times++;
                } while ((diff > off) && (times < 100));
                
                fprintf(pFile,"<%s>: Laser calibration value %f times %d\n", __func__, diff,i);
                
                fclose(pFile);
                DisconnectDLRS1A();
                Logger(MLLogInfo, "<%s>: Laser calibration finished.\n", __func__);
            } else {
                Logger(MLLogError, "<%s>: Not connect the laser device.\n", __func__);
            }
            
            MoveAxisToPosition(MLAxisLaser, param.posToTest, true, false);
            if (isNeedStop) {return;}
            if (isNeedEmgStop) {return;}
            MoveAxisToPosition(MLAxisLaser, 0, true, false);   // back to init position
            if (isNeedStop) {return;}
            if (isNeedEmgStop) {return;}
            Logger(MLLogInfo, "<%s>: Laser calibrate done.\n", __func__);
        }
        
    }
    
    gIsTesting = false;
    
}

double GetAngleValue(string portName,char*bufx,char*bufy,char*buft) {
    assert(portName != NULL);
    uint8_t* buff = (uint8_t*)malloc(1024);
    char* buffXoY = (char*)malloc(1024);

    if (!ConnectSCAPort(portName)) {

        //    SendXAxisCom();
        SendXYAxisCom();
        sleep(1);
        
        XYReadExistingHex(buff,(uint8_t*)buffXoY,"@");
        DisconnectSCAPort();
        printf("value:%s\n",buffXoY);
        
        //    XorYBytetoHex(buffXoY,buf);
        XYBytetoHex(buffXoY,bufx,bufy,buft);
        
        Logger(MLLogInfo, "<%s>: Angle calibration finished. angle-x: {%s}, angle-y: {%s}, angle-t: {%s}.\n", __func__, bufx, bufy, buft);
    } else {
        Logger(MLLogError, "<%s>: Not connect the angle device.\n", __func__);
        memset(errmsg, 0, 256);
        sprintf(errmsg, "Not connect the angle device.\n");
    }
    
    return 0;
}

double GetAngleDoubleValue(string portName, double *anglex, double *angley, double *tem) {
    char* buffX = (char*)malloc(1024);
    char* buffY = (char*)malloc(1024);
    char* buffTem = (char*)malloc(1024);
    double v = GetAngleValue(portName, buffX, buffY, buffTem);
    
    *anglex = atof(buffX);
    *angley = atof(buffY);
    *tem = atof(buffTem);
    
    if (buffX != NULL) {
        free(buffX);
    }
    
    if (buffY != NULL) {
        free(buffY);
    }
    
    if (buffTem != NULL) {
        free(buffTem);
    }
    
    return v;
}


double GetLaserDiffOfHeight(string portName) {
    assert(portName != NULL);
    
    double height = 0;
    
    if (!ConnectDLRS1A(portName)) {
        height = GetDLRS1AM0MeasureValue();
        DisconnectDLRS1A();
        Logger(MLLogInfo, "<%s>: Laser calibration finished. height: {%lf}\n", __func__, height);
    } else {
        Logger(MLLogError, "<%s>: Not connect the laser device.\n", __func__);
        memset(errmsg, 0, 256);
        sprintf(errmsg, "Not connect the laser device.\n");
    }
    
    return height;
}

void ConfigPPRatio(int axis, double ppratio) {
    assert(axis >= 0);
    if (ppratio <= 0) {
        Logger(MLLogWarning, "<%s>: PP ratio is invalid.\n", __func__);
        return;
    }
    AxisParam axisPrm = gAxisPrm[axis];
    axisPrm.ppratio = ppratio;
}

void MLSwingAngleQuick(double angleX, double angleY, double rotate, int directionX, int directionY, int directionR) {
    int dirX = (directionX >= 0) ? 1 : -1;
    int dirY = (directionY >= 0) ? 1 : -1;
    int dirR = (directionR >= 0) ? 1 : -1;
    
    if (gIsTesting) { return; }
    if (isNeedStop) { Logger(MLLogInfo, "<%s>: Stop swing angle.\n", __func__); return; }
    if (isNeedEmgStop) { Logger(MLLogInfo, "<%s>: Emg stop.\n", __func__); return; }
    gIsTesting = true;
    
    if (gHandle != -1) {
        int swingAxis[3] = { MLAxisRotationX, MLAxisRotationY, MLAxisRotation };
        MoveAxisDistanceQuick(MLAxisRotationX, dirX * angleX, false);
        MoveAxisDistanceQuick(MLAxisRotationY, dirY * angleY , false);
        MoveAxisDistanceQuick(MLAxisRotation, dirR * rotate, false);
        CheckAxisState(swingAxis, 3, false);
        Logger(MLLogInfo, "<%s>: Success to rotate an angle.\n", __func__);
    }
    
    gIsTesting = false;
}

void SwingAngleQuick(double angleX, double angleY, double rotate, int directionX, int directionY, int directionR) {
    int dirX = (directionX >= 0) ? 1 : -1;
    int dirY = (directionY >= 0) ? 1 : -1;
    int dirR = (directionR >= 0) ? 1 : -1;
    
    if (gIsTesting) { return; }
    if (isNeedStop) { Logger(MLLogInfo, "<%s>: Stop swing angle.\n", __func__); return; }
    if (isNeedEmgStop) { Logger(MLLogInfo, "<%s>: Emg stop.\n", __func__); return; }
    gIsTesting = true;
    
    SetBitState(MLOutCylinderHome, MLLow);
    SetBitState(MLOutCylinderShop, MLHigh);
    
    int lightState =  CheckAxisIOState(MLAxisLight);
    if (lightState != 2) {
        JMoveAxisWithBlock(MLAxisLight, 0, false);
        int swingLightAxis[1] = { MLAxisLight};
        CheckAxisState(swingLightAxis, 1, false);
    }
    
    int laserState = CheckAxisIOState(MLAxisLaser);
    
    if (laserState != 2) {
        JMoveAxisWithBlock(MLAxisLaser, 0, false);
        int swingLaserAxis[1] = { MLAxisLaser};
        CheckAxisState(swingLaserAxis, 1, false);
    }
    
    
    if (gHandle != -1) {
        int swingAxis[3] = { MLAxisRotationX, MLAxisRotationY, MLAxisRotation };
        MoveAxisDistanceQuick(MLAxisRotationX, dirX * angleX, false);
        MoveAxisDistanceQuick(MLAxisRotationY, dirY * angleY , false);
        MoveAxisDistanceQuick(MLAxisRotation, dirR * rotate, false);
        CheckAxisState(swingAxis, 3, false);
        Logger(MLLogInfo, "<%s>: Success to rotate an angle.\n", __func__);
    }
    
    gIsTesting = false;
}

bool MoveAxisDistanceAbsolute(int axis, double distance, bool blocked) {
    long currentPos = -1;
    long validPos = distance;
    int rtn = 0;
    gIsTesting = true;
    bool flag = false;
    short res = 0;
    WORD MyCardNo = 0;
    double Myencoder_value=0;
    if (isNeedStop) { Logger(MLLogInfo, "<%s>: Stop move absolute distance.\n", __func__); return 0;}
    if (isNeedEmgStop) { Logger(MLLogInfo, "<%s>: Emg stop.\n", __func__); return 0; }
    assert(axis >= 0);
    
    if (gHandle != -1) {
        smc_write_sevon_pin(gHandle, axis, 0);
        AxisParam param = gAxisPrm[axis];
        rtn |= smc_set_pulse_outmode(gHandle, axis, MLPluseOutMode);
        rtn |= smc_set_equiv(gHandle, axis, param.equiv);
        rtn |= smc_set_backlash_unit(gHandle, axis, param.backlash);
        rtn |= smc_set_profile_unit(gHandle, axis, param.startSpeed*param.ppratio, param.runSpeed*param.ppratio, param.accTime, param.accTime, param.stopSpeed);
        
        if (rtn==0) {
            res=smc_get_encoder_unit(MyCardNo,axis,&Myencoder_value);//读编码值
            currentPos = Myencoder_value;
            validPos += currentPos;
            Logger(MLLogInfo, "<%s>: Axis %d, [Before_Position]: %ld\n", __func__, axis, currentPos);
            
            rtn |= smc_pmove_unit(gHandle, axis, distance*param.ppratio, MLAbsolute);
            
            if (rtn==0) {
                flag = true;
                if (blocked) {
                    while (!smc_check_done(gHandle, axis)) {
                        usleep(200000);     // check axis's state every 200ms
                        if (isNeedStop) { Logger(MLLogInfo, "<%s>: Stop swing angle.\n", __func__); break; }
                        if (isNeedEmgStop) { Logger(MLLogInfo, "<%s>: Emg stop.\n", __func__); break;}
                    }
                    res=smc_get_encoder_unit(MyCardNo,axis,&Myencoder_value);//读编码值
                    currentPos = Myencoder_value;
                    Logger(MLLogInfo, "<%s>: Axis %d, [After_Position]: %ld\n", __func__, axis, currentPos);
                } else {
                    Logger(MLLogInfo, "<%s>: Axis %d running unblock mode\n", __func__, axis);
                }
            } else {
                flag = false;
                Logger(MLLogError, "<%s>: Fail to move axis{%d}, rtn: {%d}.\n", __func__, axis, rtn);
                memset(errmsg, 0, 256);
                sprintf(errmsg, "Fail to move axis{%d}, rtn: {%d}.\n", axis, rtn);
            }
        } else {
            Logger(MLLogError, "<%s>: Fail to set axis{%d}'s parameters, rtn: {%d}.\n", __func__, axis, rtn);
            memset(errmsg, 0, 256);
            sprintf(errmsg, "Fail to set axis{%d}'s parameters, rtn: {%d}.\n", axis, rtn);
        }
    }
    gIsTesting = false;
    
    return flag;
}

void MLSwingDegreesOption(double angleStep,double fromAngle, double toAngle, int delay, bool pthread) {
    int swingValue = fabs(fromAngle)+fabs(toAngle);
    MLSwingAngleQuick(0, fromAngle, 0, 1, 1, 1);
    
    for (int i = 0; i < swingValue; i++) {
        MLSwingAngle(0, angleStep, 0, 1, 1, 1, pthread);
        sleep(delay);
        if (isNeedStop) {
            Logger(MLLogError, "<%s>: Stop button pressed when swing angle.\n", __func__);
            break;
        }
        if (isNeedEmgStop) { Logger(MLLogInfo, "<%s>: Emg stop.\n", __func__); return; }
    }
    isNeedStop = false;
    isNeedEmgStop = false;
}

void SwingDegreesOption(double angleStep, double fromAngle,double toAngle) {
    int swingValue = fabs(fromAngle)+fabs(toAngle);
    if (isNeedStop) {return;}
    if (isNeedEmgStop) {return;}
    SwingAngleQuick(0, fromAngle, 0, 1, 1, 1);
    
    for (int i = 0; i < swingValue; i++) {
        SwingAngle(0, angleStep, 0, 1, 1, 1);
        sleep(1);
        if (isNeedStop) {
            Logger(MLLogError, "<%s>: Stop button pressed when swing angle.\n", __func__);
            break;
        }
        if (isNeedEmgStop) { Logger(MLLogInfo, "<%s>: Emg stop.\n", __func__); return;}
    }
    isNeedStop = false;
    isNeedEmgStop = false;
}

void MLSwingAngleAbsolute(double angleX, double angleY, double rotate, int directionX, int directionY, int directionR, bool pthread) {
    int dirX = (directionX >= 0) ? 1 : -1;
    int dirY = (directionY >= 0) ? 1 : -1;
    int dirR = (directionR >= 0) ? 1 : -1;
    
    if (isNeedStop) { Logger(MLLogError, "<%s>: Stop button pressed when swing angle.\n", __func__); return;}
    if (isNeedEmgStop) { Logger(MLLogInfo, "<%s>: Emg stop.\n", __func__); return; }
    if (gIsTesting) { return; }
    
    gIsTesting = true;
  
    if (gHandle != -1) {
        int swingAxis[3] = { MLAxisRotationX, MLAxisRotationY, MLAxisRotation };
        if (directionX != 0) {
            MoveAxisDistanceAbsolute(MLAxisRotationX, dirX * angleX, false);
        }
        if (directionY != 0) {
            MoveAxisDistanceAbsolute(MLAxisRotationY, dirY * angleY , false);
        }
        if (directionR != 0) {
            MoveAxisDistanceAbsolute(MLAxisRotation, dirR * rotate, false);
        }
        
        CheckAxisState(swingAxis, 3, pthread);
        if (!pthread) {
            Logger(MLLogInfo, "<%s>: Success to rotate an angle.\n", __func__);
        } else {
            Logger(MLLogInfo, "<%s>: Need query state of rotate axis.\n", __func__);
        }
    }
    
    gIsTesting = false;
}

void SwingAngleAbsolute(double angleX, double angleY, double rotate, int directionX, int directionY, int directionR) {
    int dirX = (directionX >= 0) ? 1 : -1;
    int dirY = (directionY >= 0) ? 1 : -1;
    int dirR = (directionR >= 0) ? 1 : -1;
    
    if (isNeedStop) {return;}
    if (isNeedEmgStop) {return;}
    if (gIsTesting) { return; }
    
    gIsTesting = true;
    
    SetBitState(MLOutCylinderHome, MLLow);
    SetBitState(MLOutCylinderShop, MLHigh);
    
    int lightState =  CheckAxisIOState(MLAxisLight);
    if (lightState != 2) {
        JMoveAxisWithBlock(MLAxisLight, 0, false);
        int swingLightAxis[1] = { MLAxisLight};
        CheckAxisState(swingLightAxis, 1, false);
    }
    
    int laserState = CheckAxisIOState(MLAxisLaser);
    
    if (laserState != 2) {
        JMoveAxisWithBlock(MLAxisLaser, 0, false);
        int swingLaserAxis[1] = { MLAxisLaser};
        CheckAxisState(swingLaserAxis, 1, false);
    }
    
    
    if (gHandle != -1) {
        int swingAxis[3] = { MLAxisRotationX, MLAxisRotationY, MLAxisRotation };
        MoveAxisDistanceAbsolute(MLAxisRotationX, dirX * angleX, false);
        MoveAxisDistanceAbsolute(MLAxisRotationY, dirY * angleY , false);
        MoveAxisDistanceAbsolute(MLAxisRotation, dirR * rotate, false);
        CheckAxisState(swingAxis, 3, false);
        Logger(MLLogInfo, "<%s>: Success to rotate an angle.\n", __func__);
    }
    
    gIsTesting = false;
}

void MLSwingAngle(double angleX, double angleY, double rotate, int directionX, int directionY, int directionR, bool pthread) {
    int dirX = (directionX >= 0) ? 1 : -1;
    int dirY = (directionY >= 0) ? 1 : -1;
    int dirR = (directionR >= 0) ? 1 : -1;
    
    if (isNeedStop) { Logger(MLLogError, "<%s>: Stop button pressed when swing angle.\n", __func__); return; }
    if (isNeedEmgStop) { Logger(MLLogInfo, "<%s>: Emg stop.\n", __func__); return; }
    if (gIsTesting) { return; }
    
    gIsTesting = true;
    
    
    if (gHandle != -1) {
        int swingAxis[3] = { MLAxisRotationX, MLAxisRotationY, MLAxisRotation };
        if (directionX != 0) {
            MoveAxisDistance(MLAxisRotationX, dirX * angleX, false, false);
        }
        if (directionY != 0) {
            MoveAxisDistance(MLAxisRotationY, dirY * angleY , false, false);
        }
        if (directionR != 0) {
             MoveAxisDistance(MLAxisRotation, dirR * rotate, false, false);
        }
       
        CheckAxisState(swingAxis, 3, pthread);
        
        if (!pthread) {
            Logger(MLLogInfo, "<%s>: Success to rotate an angle.\n", __func__);
        } else {
            Logger(MLLogInfo, "<%s>: Need qurey state of rotate axis.\n", __func__);
        }
    }
    
    gIsTesting = false;
}

void SwingAngle(double angleX, double angleY, double rotate, int directionX, int directionY, int directionR) {
    int dirX = (directionX >= 0) ? 1 : -1;
    int dirY = (directionY >= 0) ? 1 : -1;
    int dirR = (directionR >= 0) ? 1 : -1;
    
    if (isNeedStop) { Logger(MLLogError, "<%s>: Stop button pressed when swing angle.\n", __func__); return; }
    if (isNeedEmgStop) { Logger(MLLogInfo, "<%s>: Emg stop.\n", __func__); return; }
    if (gIsTesting) { return; }
    
    gIsTesting = true;

    SetBitState(MLOutCylinderHome, MLLow);
    SetBitState(MLOutCylinderShop, MLHigh);
    
    int lightState =  CheckAxisIOState(MLAxisLight);
    if (lightState != 2) {
        JMoveAxisWithBlock(MLAxisLight, 0, false);
        int swingLightAxis[1] = { MLAxisLight};
        CheckAxisState(swingLightAxis, 1, false);
    }
    
   int laserState = CheckAxisIOState(MLAxisLaser);

    if (laserState != 2) {
        JMoveAxisWithBlock(MLAxisLaser, 0, false);
        int swingLaserAxis[1] = { MLAxisLaser};
        CheckAxisState(swingLaserAxis, 1, false);
    }

    if (gHandle != -1) {
        int swingAxis[3] = { MLAxisRotationX, MLAxisRotationY, MLAxisRotation };
        MoveAxisDistance(MLAxisRotationX, dirX * angleX, false, false);
        MoveAxisDistance(MLAxisRotationY, dirY * angleY , false, false);
        MoveAxisDistance(MLAxisRotation, dirR * rotate, false, false);
        CheckAxisState(swingAxis, 3, false);
        Logger(MLLogInfo, "<%s>: Success to rotate an angle.\n", __func__);
    }
    
    gIsTesting = false;
}

void Swing90Degrees(double angleValue) {
    for (int i = 0; i < 90; i++) {
        SwingAngle(0, angleValue, 0, 1, 1, 1);
        sleep(1);
        if (isNeedStop) {return;}
        if (isNeedEmgStop) {return;}
    }
    
    SwingAngleQuick(0, 90, 0, 1, -1, 1);
    sleep(1);
    if (isNeedStop) {return;}
    if (isNeedEmgStop) {return;}
    for (int i = 0; i < 90; i++) {
        SwingAngle(0, angleValue, 0, 1, -1, 1);
        sleep(1);
        if (isNeedStop) {return;}
        if (isNeedEmgStop) {return;}
    }
    
    SwingAngleQuick(0, 90, 0, 1, 1, 1);
}

void SwingAngleTest(int dutType, double force1, double force2,double angleV,int testCount) {
    
    for (int i = 0; i < testCount; i ++) {
        Swing90Degrees(angleV);
        if (isNeedStop) {return;}
        if (isNeedEmgStop) {return;}
        SwingAngle(0, 0, 45, 1, 1, -1);
        sleep(1);
        if (isNeedStop) {return;}
        if (isNeedEmgStop) {return;}
        Swing90Degrees(angleV);
        if (isNeedStop) {return;}
        if (isNeedEmgStop) {return;}
        SwingAngle(0, 0, 45, 1, 1, -1);
        sleep(1);
        if (isNeedStop) {return;}
        if (isNeedEmgStop) {return;}
        Swing90Degrees(angleV);
        if (isNeedStop) {return;}
        if (isNeedEmgStop) {return;}
        SwingAngle(0, 0, 45, 1, 1, -1);
        sleep(1);
        if (isNeedStop) {return;}
        if (isNeedEmgStop) {return;}
        Swing90Degrees(angleV);
        if (isNeedStop) {return;}
        if (isNeedEmgStop) {return;}
        SwingAngle(0, 0, 135, 1, 1, 1);
        sleep(1);
        if (isNeedStop) {return;}
        if (isNeedEmgStop) {return;}
        // release hold.
        int axises[] = {MLAxisClampLeft, MLAxisClampUp,MLAxisClampDown,MLAxisClampRight};
        ManyAxisGoBack(axises,4, false);
        smc_write_outbit(gHandle, MLOutLightPower, MLHigh);
    }
}

void DebugSwing45Degrees(double angleValue,int testCount) {
    
    for (int i = 0; i < testCount; i++) {
        
        double axisPosition = 0;
        const char *pFileName = "/vault/FlareLog.txt";
        FILE *pFile;
        pFile = fopen(pFileName,"a");
        if (NULL == pFile) {
            printf("error");
        }
        axisPosition = ReadEncoderValue(MLAxisRotation);
        fprintf(pFile,"Current Rotation Position:%f\n",axisPosition);
        
        for (int i = 0; i < 2; i++) {
           SwingAngle(0, 0, 45, 1, 1, 1);
           if (isNeedStop) {return;}
            if (isNeedEmgStop) {return;}
           sleep(1);
           axisPosition = ReadEncoderValue(MLAxisRotation);
           fprintf(pFile,"Current Rotation Position:%f\n",axisPosition);
        }
        
        for (int i = 0; i < 6; i++) {
            SwingAngle(0, 0, 45, 1, 1, -1);
            if (isNeedStop) {return;}
            if (isNeedEmgStop) {return;}
            sleep(1);
            axisPosition = ReadEncoderValue(MLAxisRotation);
            fprintf(pFile,"Current Rotation Position:%f\n",axisPosition);
        }
        
//        SwingAngle(0, 0, 90, 1, 1, -1);
//        sleep(1);
        
        for (int i = 0; i < 4; i++) {
            SwingAngle(0, 0, 45, 1, 1, 1);
            if (isNeedStop) {return;}
            if (isNeedEmgStop) {return;}
            sleep(1);
            axisPosition = ReadEncoderValue(MLAxisRotation);
            fprintf(pFile,"Current Rotation Position:%f\n",axisPosition);
        }
        
//        SwingAngle(0, 0, 180, 1, 1, 1);
        fclose(pFile);
    }
}

void DebugSwing90Degrees(double angleValue,int testCount) {
    
    for (int i = 0; i < testCount; i++) {
        
        char* buffX = (char*)malloc(1024);
        char* buffY = (char*)malloc(1024);
        char* buffTem = (char*)malloc(1024);
        
        const char *pFileName = "/vault/FlareLog.txt";
        FILE *pFile;
        pFile = fopen(pFileName,"a");
        if (NULL == pFile) {
            printf("error");
        }
        
        for (int i = 0; i < 90; i++) {
            SwingAngle(0, angleValue, 0, 1, 1, 1);
            if (isNeedStop) {return;}
            if (isNeedEmgStop) {return;}
            sleep(2);
            GetAngleValue(anglePortName,buffX,buffY,buffTem);
            fprintf(pFile,"buffX:%s buffY:%s\n",buffX,buffY);
        }
        
        SwingAngleQuick(0, 90, 0, 1, -1, 1);
        sleep(2);
        
        for (int i = 0; i < 90; i++) {
            SwingAngle(0, angleValue, 0, 1, -1, 1);
            if (isNeedStop) {return;}
            if (isNeedEmgStop) {return;}
            sleep(2);
            GetAngleValue(anglePortName,buffX,buffY,buffTem);
            fprintf(pFile,"buffX:%s buffY:%s\n",buffX,buffY);
        }
        
        SwingAngleQuick(0, 90, 0, 1, 1, 1);
        fclose(pFile);
    }
}

void DebugRotationAngleTest(double angleV,int testCount) {
    
    DebugSwing45Degrees(angleV,testCount);
}

void DebugSwingAngleTest(int dutType, double force1, double force2,double angleV,int testCount) {
    
    DebugSwing90Degrees(angleV,testCount);
}

void RotateAngle(double angle) {
    
}

void SwingOneAngle(int direction) {
    
}


void DebugIlluminanceMeasure(int testCount) {
    double measValue = -1;
    int tmCount = 0;
    
    for (int i = 0; i < testCount; i++) {
        smc_write_outbit(gHandle, MLOutIlluminanceLight, MLLow);
        sleep(3);
        
        const char *pFileName = "/vault/FlareLog.txt";
        FILE *pFile;
        pFile = fopen(pFileName,"a");
        if (NULL == pFile) {
            printf("error");
        }
        
        if (ConnectIlluminometer(illuminometerPortName)) {
            if (gHandle != -1) {
                int rtn = 0;
                int iostate = CheckAxisIOState(9);
                gIsTesting = true;
                
                AxisParam param = gAxisPrm[9];
                
                rtn |= smc_write_sevon_pin(gHandle, 9, 0);
                sleep(1);
                rtn |= smc_set_profile_unit(gHandle, 9, param.startSpeed*param.ppratio, param.runSpeed*param.ppratio, param.accTime, param.accTime, param.stopSpeed);
                rtn |= smc_set_s_profile(gHandle, 9,0,0);
                rtn |= smc_vmove(gHandle, 9, 0);
                
                while (smc_check_done(gHandle, 9)==0) {
                    usleep(200000);     // check axis's state every 200ms
                };
                
                smc_write_outbit(gHandle, MLOutLight, MLHigh);
                smc_write_outbit(gHandle, MLOutLaserPower, MLHigh);
                smc_write_outbit(gHandle, MLOutSpotPower, MLHigh);
                DoorClose();
                while (doorOpened) { usleep(500000); }
                
                // back to orignal position.
                if (!bCalibrated) {
                    calLuxVals = (MLLuxMeterVal *)malloc(sizeof(MLLuxMeterVal));
                    if (iostate == 2) {
                        smc_write_outbit(gHandle, MLOutLightPower, MLLow);
                        smc_write_outbit(gHandle, MLOutCylinderShop, MLLow);
                        smc_write_outbit(gHandle, MLOutCylinderHome, MLHigh);
                    }
                    while (!lightTestReadyed) { usleep(500000); }       // check state every 500ms.
                    calLuxVals->lv = GetIlluminanceValue(5000);
                    Logger(MLLogInfo, "Illuminometer Calibrate value = %lf.\n", calLuxVals->lv);
                }
                
                Logger(MLLogInfo, "Cylinder has ready, waiting to measure intensity of illumination.\n");
                bool flag = false;
                measValue = GetIlluminanceValue(5000);
                double diff = 0;
                do {
                    diff = fabs(GetIlluminanceValue(5000) - measValue);
                    
                    if (diff <= 1) {
                        flag = true;
                        break;
                    }
                } while (tmCount++ <= 3);
                
                if (flag) {
                    measValue = GetIlluminanceValue(5000);
                } else {
                    bCalibrated = flag;
                    measValue = -1;
                }
                
                fprintf(pFile,"Illuminometer calibration value %f times %d\n",diff,i);
                fclose(pFile);
                
                smc_write_outbit(gHandle, MLOutCylinderShop, MLHigh);
                smc_write_outbit(gHandle, MLOutCylinderHome, MLLow);
                while (lightTestReadyed) { usleep(500000); }   // check state every 500ms.
                //            DisconnectIlluminometer();
            } else {
                Logger(MLLogWarning, "You need connect controller firstly.\n");
            }
        }
        
        DoorOpen();
        //    DisconnectCL200A();
        gIsTesting = false;
        smc_write_outbit(gHandle, MLOutIlluminanceLight, MLHigh);
    }
}

void LightSourceON(){
    SetBitState(5,0);
}

void LightSourceOFF(){
    SetBitState(5,1);
}

double GetLuxmeter(string portName) {
    double measValue = -1;
    
    if (gHandle != -1) {
        gIsTesting = true;
        
        SetBitState(MLOutIlluminanceLight, MLLow);
        usleep(500 * 1000);
        
        if (!gLuxConnected) {
            sleep(2);
            if (!ConnectIlluminometer(portName)) {
                Logger(MLLogError, "<%s>: Fail to connect the luxmeter, port name: %s.\n", __func__, portName);
                return measValue;
            }
        }
        
        smc_write_outbit(gHandle, MLOutLight, MLHigh);
        smc_write_outbit(gHandle, MLOutLaserPower, MLHigh);
        smc_write_outbit(gHandle, MLOutSpotPower, MLHigh);
        
        int iostate = CheckAxisIOState(MLAxisLight);
        Logger(MLLogInfo, "<%s>: Axis %d state is %d.\n", __func__, MLAxisLight, iostate);
        
        if (iostate != 2 && iostate != 0) {
            JMoveAxisWithBlock(MLAxisLight, 0, false);
        }
//        JMoveAxisWithBlock(MLAxisLaser, 0);
        
        DoorClose();
        if (doorOpened) {
            Logger(MLLogError, "<%s>: Fail to close auto-door.\n", __func__);
            return measValue;
        }
        
        if (!bCalibrated) {
            calLuxVals = (MLLuxMeterVal *)malloc(sizeof(MLLuxMeterVal));
            SetBitState(MLOutCylinderHome, MLLow);
            while (lightTestReadyed) { usleep(500000); }       // check state every 500ms.
            calLuxVals->lv = GetIlluminanceValue(5000);
            measValue = calLuxVals->lv;
            Logger(MLLogInfo, "<%s>: Get Illuminometer Calibrate value = %lf.\n", __func__, calLuxVals->lv);
            bCalibrated = true;
            Logger(MLLogInfo, "<%s>: Is calibrated: %s.\n", __func__, bCalibrated ? "Yes" : "No");
            sleep(3);
        } else {
            measValue = calLuxVals->lv;
            Logger(MLLogInfo, "<%s>: Use calibration value: %lf.\n", __func__, calLuxVals->lv);
        }
        
        if (iostate == 2) {     // negative limit
            SetBitState(MLOutCylinderShop, MLLow);
        }
        
        while (!lightTestReadyed) { usleep(500000); }       // check state every 500ms.
        
        gIsTesting = false;
    } else {
        Logger(MLLogInfo, "<%s>: Please connect controller firstly.\n", __func__);
    }
    
    return measValue;
}

void LuxmeterMeasure(double measureValues[], int size, int timeout) {
    float lv = 0;
    float x = 0;
    float y = 0;
    
//    double *measureValues = malloc(3 * sizeof(double));
    size = size < 3 ? 3 : size;
    memset(measureValues, 0, 3);
    
    JKSetCL200ATimeout(timeout);
    JKGetCL200AEvXY(&lv, &x, &y);
    
    if (lv == 0) {
        measureValues[0] = 0;
        measureValues[1] = 0;
        measureValues[2] = 0;
        Logger(MLLogInfo, "<%s>: Fail to get illuminance measure data.\n", __func__);
    } else {
        measureValues[0] = lv;
        measureValues[1] = x;
        measureValues[2] = y;
        Logger(MLLogInfo, "<%s>: Luxmeter measure value, Ev: %lf, x: %lf, y: %lf.\n", __func__, lv, x, y);
    }
    
//    return measureValues;
}

bool MoveBackLuxmeter(void) {
    bool flag = false;
    
    gIsTesting = true;
    SetBitState(MLOutIlluminanceLight,MLHigh);
    SetBitState(MLOutCylinderHome, MLLow);
    while (lightTestReadyed) { usleep(500000); }   // check state every 500ms.
    DoorOpen();
//    JKDisconnectCL200A();
    DisconnectIlluminometer();
    Logger(MLLogInfo, "<%s>: Open auto-door and disconnect luxmeter.\n", __func__);
    gIsTesting = false;
    
    return flag;
}

bool LuxMeterON(string portName) {
    double measValue = -1;
    SetBitState(26,0);
    sleep(2);
    if (MLConnectIlluminometer(portName)) {
        if (gHandle != -1) {
            
            int iostate = CheckAxisIOState(9);
            gIsTesting = true;
            
            JMoveAxis(9, 0, false);
            
            int axises[] = {9};
            CheckAxisState(axises, 1, false);
            
            smc_write_outbit(gHandle, MLOutLight, MLHigh);
            smc_write_outbit(gHandle, MLOutLaserPower, MLHigh);
            smc_write_outbit(gHandle, MLOutSpotPower, MLHigh);
            
            DoorClose();
            if (doorOpened) {
                Logger(MLLogError, "<%s>: Fail to open auto-door.\n", __func__);
                memset(errmsg, 0, 256);
                sprintf(errmsg, "Fail to open auto-door when turn on luxmeter.\n");
                return -1;
            }
            
            // back to orignal position.
            if (!bCalibrated) {
                calLuxVals = (MLLuxMeterVal *)malloc(sizeof(MLLuxMeterVal));
                if (iostate == 2) {
                    SetBitState(16,0);
                }
                while (!lightTestReadyed) { usleep(500000); }       // check state every 500ms.
                calLuxVals->lv = MLGetIlluminanceValue();
                Logger(MLLogInfo, "<%s>: Illuminometer Calibrate value = %lf.\n", __func__, calLuxVals->lv);
            }
            
            measValue = MLGetIlluminanceValue();
            Logger(MLLogInfo, "<%s>: Cylinder has ready, waiting to measure intensity of illumination.\n", __func__);
        }
    } else {
        Logger(MLLogError, "<%s>: Fail to connect the luxmeter, port name: %s.\n", __func__, portName);
        memset(errmsg, 0, 256);
        sprintf(errmsg, "Fail to connect the luxmeter.\n");
        return -1;
    }
    gIsTesting = false;
    return (measValue != -1) ? (measValue - calLuxVals->lv) : measValue;
}


double IlluminanceValue() {
    double measValue = -1;
    measValue = MLGetIlluminanceValue();
    Logger(MLLogInfo, "<%s>: Read luxmeter value: {%lf}.\n", __func__, (measValue != -1) ? (measValue - calLuxVals->lv) : measValue);
    return (measValue != -1) ? (measValue - calLuxVals->lv) : measValue;
}

void LuxMeterOFF() {
    gIsTesting = true;
    SetBitState(26,1);
    SetBitState(15,0);
    while (lightTestReadyed) { usleep(500000); }   // check state every 500ms.
    DoorOpen();
    JKDisconnectCL200A();
    Logger(MLLogInfo, "<%s>: Open auto-door and disconnect luxmeter.\n", __func__);
    gIsTesting = false;
}

bool MLConnectIlluminometer(string portName) {
    return ConnectIlluminometer(portName);
}

double MLGetIlluminanceValue() {
    return MLGetIlluminanceValueNT();
}

double IlluminanceMeasure(string portName, double offset, int timeout) {
    double measValue = -1;
    int tmCount = 0;
    smc_write_outbit(gHandle, MLOutIlluminanceLight, MLLow);
    sleep(3);
    if (MLConnectIlluminometer(illuminometerPortName)) {
        if (gHandle != -1) {
            int rtn = 0;
            int iostate = CheckAxisIOState(9);
            gIsTesting = true;

            AxisParam param = gAxisPrm[9];

            rtn |= smc_write_sevon_pin(gHandle, 9, 0);
            sleep(1);
            rtn |= smc_set_profile_unit(gHandle, 9, param.startSpeed*param.ppratio, param.runSpeed*param.ppratio, param.accTime, param.accTime, param.stopSpeed);
            rtn |= smc_set_s_profile(gHandle, 9,0,0);
            rtn |= smc_vmove(gHandle, 9, 0);

            while (CheckAxisIOState(9) != 2) {
                usleep(200000);     // check axis's state every 200ms
            };
           
            smc_write_outbit(gHandle, MLOutLight, MLHigh);
            smc_write_outbit(gHandle, MLOutLaserPower, MLHigh);
            smc_write_outbit(gHandle, MLOutSpotPower, MLHigh);
            DoorClose();
            if (doorOpened) { return -1; }
            
            // back to orignal position.
            if (!bCalibrated) {
                calLuxVals = (MLLuxMeterVal *)malloc(sizeof(MLLuxMeterVal));
                if (iostate == 2) {
                    smc_write_outbit(gHandle, MLOutLightPower, MLLow);
                    smc_write_outbit(gHandle, MLOutCylinderShop, MLLow);
                    smc_write_outbit(gHandle, MLOutCylinderHome, MLHigh);
                }
                while (!lightTestReadyed) { usleep(500000); }       // check state every 500ms.
                calLuxVals->lv = MLGetIlluminanceValue();
                Logger(MLLogInfo, "<%s>: Illuminometer Calibrate value = %lf.\n", __func__, calLuxVals->lv);
            }
            
            Logger(MLLogInfo, "<%s>: Cylinder has ready, waiting to measure intensity of illumination.\n", __func__);
            bool flag = false;
            measValue = MLGetIlluminanceValue();
            
            do {
                double diff = fabs(MLGetIlluminanceValue() - measValue);
                
                if (diff <= offset) {
                    flag = true;
                    break;
                }
            } while (tmCount++ <= 3);

            if (flag) {
                measValue = MLGetIlluminanceValue();
            } else {
                bCalibrated = flag;
                measValue = -1;
            }
            smc_write_outbit(gHandle, MLOutCylinderShop, MLHigh);
            smc_write_outbit(gHandle, MLOutCylinderHome, MLLow);
            while (lightTestReadyed) { usleep(500000); }   // check state every 500ms.
//            DisconnectIlluminometer();
        } else {
            Logger(MLLogWarning, "<%s>: You need connect controller firstly.\n", __func__);
        }
    }
    DoorOpen();
//    DisconnectCL200A();
    gIsTesting = false;
    smc_write_outbit(gHandle, MLOutIlluminanceLight, MLHigh);
    return (measValue != -1) ? (measValue - calLuxVals->lv) : measValue;
}

void ConfigLoadCellPortName(string frontPortName, string backPortName, string leftPortName, string rightPortName) {
    if (frontPortName) {
        sprintf(frontCellPortName, "%s", frontPortName);
        PutStringValue("loadcell", "front_side_portname", frontCellPortName);
    } else {
        Logger(MLLogWarning, "<%s>: Fail to config front side portname for loadcell.\n", __func__);
    }
    
    if (backPortName) {
        sprintf(backCellPortName, "%s", backPortName);
        PutStringValue("loadcell", "back_side_portname", backCellPortName);
    } else {
        Logger(MLLogWarning, "<%s>: Fail to config back side portname for loadcell.\n", __func__);
    }
    
    if (leftPortName) {
        sprintf(leftCellPortName, "%s", leftPortName);
        PutStringValue("loadcell", "left_side_portname", leftCellPortName);
    } else {
        Logger(MLLogWarning, "<%s>: Fail to config left side portname for loadcell.\n", __func__);
    }
    
    if (rightPortName) {
        sprintf(rightCellPortName, "%s", rightPortName);
        PutStringValue("loadcell", "right_side_portname", rightCellPortName);
    } else {
        Logger(MLLogWarning, "<%s>: Fail to config right side portname for loadcell.\n", __func__);
    }
}

void AddDUTType(int type, string name, double posFrontHold, double posRightHold, double posBackHold, double posLeftHold,double posAxisX,double posAxisY,double posLifter) {
//    int currentDUTTypeCnt = GetCountOfPtr(duts);
    const char *searchname = GetDUTName(type);
    
    if (searchname == NULL) {
        return;
    }
    
    MLDUT duttype;
    duttype.type = type;
    duttype.name = (char *)malloc(128*sizeof(char));
    sprintf(duttype.name, "%s", name);
//    duttype.name = name;
//    duttype.posStandby = posStandby;
    
    duttype.posHoldFront = posFrontHold;
    duttype.posHoldBack = posBackHold;
    duttype.posHoldLeft = posLeftHold;
    duttype.posHoldRight = posRightHold;
    
    duttype.posStandbyFront = duttype.posHoldFront - 3;
    duttype.posStandbyBack = duttype.posHoldBack - 3;
    duttype.posStandbyLeft = duttype.posHoldLeft - 3;
    duttype.posStandbyRight = duttype.posHoldRight - 3;
    
    duttype.posAxisX = posAxisX;
    duttype.posAxisY = posAxisY;
    duttype.posLifter = posLifter;
    if (dutCount < MLMaxDUTTypeCount) {
        duts[dutCount-1] = duttype;
    }
    MLDUT dut = duts[dutCount];
    char *title = (char *)malloc(32);
    char *dutType = (char *)malloc(32);

    sprintf(title, "duttype_%d", dutCount-1);
    sprintf(dutType, "duttype_%d",dut.type);

    int flag = checkTitle(dutType);
    if (flag) {
        sprintf(title, "duttype_%d",dut.type);
        dutCount--;
    }
    
    PutIntValue(title, "type", dut.type);
    PutStringValue(title, "name", dut.name);
    PutDoubleValue(title, "front_standby_position",   dut.posStandbyFront);
    PutDoubleValue(title, "back_standby_position",    dut.posStandbyBack);
    PutDoubleValue(title, "left_standby_position",    dut.posStandbyLeft);
    PutDoubleValue(title, "right_standby_position",   dut.posStandbyRight);
    PutDoubleValue(title, "front_hold_position",      dut.posHoldFront);
    PutDoubleValue(title, "back_hold_position",       dut.posHoldBack);
    PutDoubleValue(title, "left_hold_position",       dut.posHoldLeft);
    PutDoubleValue(title, "right_hold_position",      dut.posHoldRight);
    PutDoubleValue(title, "lifter_position",          dut.posLifter);
    PutDoubleValue(title, "axisX_position",           dut.posAxisX);
    PutDoubleValue(title, "axisY_position",           dut.posAxisY);
    PutDoubleValue(title, "lifter_position",           dut.posLifter);

//    ModifyKeyString(title,dut.name);
    free(title);
    title = NULL;
    dutCount++;
    PutIntValue("syscfg", "dut_type_count", dutCount);
}

double GetAxisPosition(int axis){

    short res = 0;
    WORD MyCardNo = 0;
    double Myencoder_value=0;
    AxisParam param = gAxisPrm[axis];
    res=smc_get_encoder_unit(MyCardNo,axis,&Myencoder_value);//读编码值
    double position = Myencoder_value/param.ppratio;
    return position;
}

void AddDUTTypeAndName(int type, string name) {
    if (gHandle != -1) {

        AxisParam param4 = gAxisPrm[4];
        AxisParam param5 = gAxisPrm[5];
        AxisParam param6 = gAxisPrm[6];
        AxisParam param7 = gAxisPrm[7];
        AxisParam param8 = gAxisPrm[8];
        AxisParam param11 = gAxisPrm[11];
        AxisParam param12 = gAxisPrm[12];

        
        short res = 0;
        WORD MyCardNo = 0;
        double Myencoder_value=0;

        res=smc_get_encoder_unit(MyCardNo,5,&Myencoder_value);//读编码值
        double posFrontHold = Myencoder_value/param5.ppratio;
        res=smc_get_encoder_unit(MyCardNo,6,&Myencoder_value);//读编码值
        double posRightHold = Myencoder_value/param6.ppratio;
        res=smc_get_encoder_unit(MyCardNo,7,&Myencoder_value);//读编码值
        double posBackHold = Myencoder_value/param7.ppratio;
        res=smc_get_encoder_unit(MyCardNo,8,&Myencoder_value);//读编码值
        double posLeftHold = Myencoder_value/param8.ppratio;

        res=smc_get_encoder_unit(MyCardNo,11,&Myencoder_value);//读编码值
        double posAxisX = Myencoder_value/param11.ppratio;
        res=smc_get_encoder_unit(MyCardNo,12,&Myencoder_value);//读编码值
        double posAxisY = Myencoder_value/param12.ppratio;
        
        res=smc_get_encoder_unit(MyCardNo,4,&Myencoder_value);//读编码值
        double posLifter = Myencoder_value/param4.ppratio;
        
        AddDUTType(type, name, posFrontHold,posRightHold,posBackHold,posLeftHold,posAxisX,posAxisY,posLifter);
    }
}

void LookDUTTypeAndName(int type, string name) {
    if (gHandle != -1) {

        char *title = (char *)malloc(32);
        sprintf(title, "%s",name);

        int axises[] = {MLAxisClampLeft, MLAxisClampUp};
        long posFrontHold = GetLongValue(title, "front_hold_position");
        MoveAxisToPosition(MLAxisClampUp,posFrontHold,false, false);
        if (isNeedStop) {return;}
        if (isNeedEmgStop) {return;}
        long posLeftHold = GetLongValue(title, "left_hold_position");
        MoveAxisToPosition(MLAxisClampLeft,posLeftHold,false, false);
        if (isNeedStop) {return;}
        if (isNeedEmgStop) {return;}
        while (!CheckAxisState(axises, 2, false)) { usleep(100000); }
        
        int axises1[] = {MLAxisClampDown, MLAxisClampRight};
        long posRightHold = GetLongValue(title, "right_hold_position");
        MoveAxisToPosition(MLAxisClampRight,posRightHold,false, false);
        if (isNeedStop) {return;}
        if (isNeedEmgStop) {return;}
        long posBackHold = GetLongValue(title, "back_hold_position");
        MoveAxisToPosition(MLAxisClampDown,posBackHold,false, false);
        if (isNeedStop) {return;}
        if (isNeedEmgStop) {return;}
        while (!CheckAxisState(axises1, 2, false)) { usleep(100000); }
        
        
        int axises2[] = {MLAxisX, MLAxisY};
        long posAxisX = GetLongValue(title, "axisX_position");
        MoveAxisToPosition(MLAxisX,posAxisX,false, false);
        if (isNeedStop) {return;}
        if (isNeedEmgStop) {return;}
        long posAxisY = GetLongValue(title, "axisY_position");
        MoveAxisToPosition(MLAxisY,posAxisY,false, false);
        if (isNeedStop) {return;}
        if (isNeedEmgStop) {return;}
        while (!CheckAxisState(axises2, 2, false)) { usleep(100000); }
        
        
        int axises3[] = {MLAxisLifter};
        long posLifter = GetLongValue(title, "lifter_position");
        MoveAxisToPosition(MLAxisLifter,posLifter,false, false);
        if (isNeedStop) {return;}
        if (isNeedEmgStop) {return;}
        while (!CheckAxisState(axises3, 1, false)) { usleep(100000); }
    }
    
}

double ReadEncoderValue(int axis){
    
    AxisParam param = gAxisPrm[axis];
    
    short res = 0;
    WORD MyCardNo = 0;
    double Myencoder_value=0;
    
    res=smc_get_encoder_unit(MyCardNo,axis,&Myencoder_value);//读编码值
    double posValue = Myencoder_value/param.ppratio;
    Logger(MLLogInfo, "<%s>: Read encoder value: {%d}.\n", __func__, posValue);
    
    return posValue;
    
}

bool DebugHoldDut(int dutType, double force1, double force2,int testCount) {
    if (gIsTesting) { return false; }
    
    int flag = 0;
    gIsTesting = true;
    
    assert(frontCellPortName != NULL);
    assert(rightCellPortName != NULL);
    
    double MoveForce = 0;
    double MoveForce1 = 0;
    double MoveForce2 = 0;
    double MoveForce3 = 0;
    double axisPosition = 0;
    double axisPosition1 = 0;
    double axisPosition2 = 0;
    double axisPosition3 = 0;

    time_t timep;
    struct tm *p;
    
    if (gHandle != -1) {
        
        const char *pFileName = "/vault/FlareLog.csv";
        FILE *pFile;
        
        pFile = fopen(pFileName,"a+");
        fprintf(pFile,"time,count,leftcell,frontCell,rightCell,backCell,leftPos,frontPos,rightPos,backPos\n");
//        fclose(pFile);

        for (int i = 0; i < testCount; i++) {
            
//            pFile = fopen(pFileName,"a+");
//            if (NULL == pFile) {
//                printf("error");
//            }
            int fd = ConnectXJCPort(leftCellPortName);
            int fd1 = ConnectXJCPort(frontCellPortName);
            int fd2 = ConnectXJCPort(rightCellPortName);
            int fd3 = ConnectXJCPort(backCellPortName);
            
            MLDUT dut = duts[dutType];
            
            int axises[] = {MLAxisClampLeft, MLAxisClampUp};
            MoveAxisToPosition(MLAxisClampLeft, dut.posHoldLeft, false, false);
            MoveAxisToPosition(MLAxisClampUp, dut.posHoldFront, false, false);
            if (isNeedStop) {return 0;}
            if (isNeedEmgStop) {return 0;}
            while (!CheckAxisState(axises, 2, false)) { usleep(100000); }
            axisPosition = ReadEncoderValue(MLAxisClampLeft);
            axisPosition1 = ReadEncoderValue(MLAxisClampUp);
            
            int axises1[] = {MLAxisClampDown, MLAxisClampRight};
            MoveAxisToPosition(MLAxisClampRight, dut.posStandbyRight, false, false);
            MoveAxisToPosition(MLAxisClampDown, dut.posStandbyBack, false, false);
            if (isNeedStop) {return 0;}
            if (isNeedEmgStop) {return 0;}
            while (!CheckAxisState(axises1, 2, false)) { usleep(100000); }
            
            AxisParam prmRight = gAxisPrm[MLAxisClampRight];
            AxisParam prmBack = gAxisPrm[MLAxisClampDown];
            
            smc_change_speed_unit(gHandle, MLAxisClampDown, prmBack.runSpeed / 2, prmBack.accTime);
            smc_change_speed_unit(gHandle, MLAxisClampRight, prmRight.runSpeed / 2, prmRight.accTime);
            
            MoveAxisToPosition(MLAxisClampRight, dut.posHoldRight, false, false);
            MoveAxisToPosition(MLAxisClampDown, dut.posHoldBack, false, false);
            if (isNeedStop) {return 0;}
            if (isNeedEmgStop) {return 0;}
            while (!CheckAxisState(axises1, 2, false)) { usleep(100000); }
            axisPosition2 = ReadEncoderValue(MLAxisClampRight);
            axisPosition3 = ReadEncoderValue(MLAxisClampDown);

            
            MoveForce = ReadXJCExisting(fd,"@");
            MoveForce1 = ReadXJCExisting(fd1,"@");
            MoveForce2 = ReadXJCExisting(fd2,"@");
            MoveForce3 = ReadXJCExisting(fd3,"@");
            
            smc_change_speed_unit(gHandle, MLAxisClampDown, prmBack.runSpeed, prmBack.accTime);
            smc_change_speed_unit(gHandle, MLAxisClampRight, prmRight.runSpeed, prmRight.accTime);
            
            
            DisconnectXJCPort(fd);
            DisconnectXJCPort(fd1);
            DisconnectXJCPort(fd2);
            DisconnectXJCPort(fd3);
            
            time (&timep);
            p=gmtime(&timep);
            fprintf(pFile,"%d/%d/%d:%d:%d,%d,%f,%f,%f,%f,%f,%f,%f,%f\n",1+p->tm_mon,p->tm_mday,8+p->tm_hour,p->tm_min,p->tm_sec,i,MoveForce,MoveForce1,MoveForce2,MoveForce3,axisPosition,axisPosition1,axisPosition2,axisPosition3);
            
            printf("%d/%d/%d:%d:%d,%d,%f,%f,%f,%f,%f,%f,%f,%f\n",1+p->tm_mon,p->tm_mday,8+p->tm_hour,p->tm_min,p->tm_sec,i,MoveForce,MoveForce1,MoveForce2,MoveForce3,axisPosition,axisPosition1,axisPosition2,axisPosition3);
            
            int axisesBack[] = {MLAxisClampLeft, MLAxisClampUp,MLAxisClampDown,MLAxisClampRight};
            ManyAxisGoBack(axisesBack,4, false);
        }
            fclose(pFile);
    }
    
    gIsTesting = false;
    return (flag == 2) ? true : false;
}

void DebugLookDUTTypeAndName(int type, string name) {
    
    if (gHandle != -1) {
        
        const char *pFileName = "/vault/FlareLog.txt";
        FILE *pFile;
        pFile = fopen(pFileName,"a");
        if (NULL == pFile) {
            printf("error");
        }
        
        for (int i = 0; i < 10; i++) {
            
            char *title = (char *)malloc(32);
            sprintf(title, "%s",name);
            
            int axises[] = {MLAxisClampLeft, MLAxisClampUp};
            long posFrontHold = GetLongValue(title, "front_hold_position");
            MoveAxisToPosition(MLAxisClampUp,posFrontHold,false, false);
            fprintf(pFile,"posFrontHold value %ld times %d\n",posFrontHold,i);
            long posLeftHold = GetLongValue(title, "left_hold_position");
            MoveAxisToPosition(MLAxisClampLeft,posLeftHold,false, false);
            fprintf(pFile,"posLeftHold value %ld times %d\n",posLeftHold,i);
            while (!CheckAxisState(axises, 2, false)) { usleep(100000); }
            
            int axises1[] = {MLAxisClampDown, MLAxisClampRight};
            long posRightHold = GetLongValue(title, "right_hold_position");
            fprintf(pFile,"posRightHold value %ld times %d\n",posRightHold,i);
            MoveAxisToPosition(MLAxisClampRight,posRightHold,false, false);
            long posBackHold = GetLongValue(title, "back_hold_position");
            fprintf(pFile,"posBackHold value %ld times %d\n",posBackHold,i);
            MoveAxisToPosition(MLAxisClampDown,posBackHold,false, false);
            while (!CheckAxisState(axises1, 2, false)) { usleep(100000); }
            
            
            int axises2[] = {MLAxisX, MLAxisY};
            long posAxisX = GetLongValue(title, "axisX_position");
            MoveAxisToPosition(MLAxisX,posAxisX,false, false);
            long posAxisY = GetLongValue(title, "axisY_position");
            MoveAxisToPosition(MLAxisY,posAxisY,false, false);
            while (!CheckAxisState(axises2, 2, false)) { usleep(100000); }
            
            
            int axises3[] = {MLAxisLifter};
            long posLifter = GetLongValue(title, "lifter_position");
            MoveAxisToPosition(MLAxisLifter,posLifter,false, false);
            while (!CheckAxisState(axises3, 1, false)) { usleep(100000); }
            
            fclose(pFile);
            
            int axisesBack[] = {MLAxisClampLeft, MLAxisClampUp,MLAxisClampDown,MLAxisClampRight};
            ManyAxisGoBack(axisesBack,4, false);
        }
    }
}

int GetCountOfDUTType() {
    return dutCount; //GetCountOfPtr(duts);
}

const char* GetDUTName(int type) {
    bool found = false;
    MLDUT *ptr;
    ptr = duts;
    usleep(50000);
    while (ptr->name != NULL) {
         usleep(50000);
        if (ptr->type == type) {
            found = true;
            break;
        }
        ptr++;
    }
    
    if (!found) {
        Logger(MLLogWarning, "<%s>: No DUT which type is %d, You can add it using AddDUTType function.\n", __func__, type);
    }
    
    return found ? ptr->name : "";
}

const char* GetTypeName(int type) {

    char sTitle[64];
    sprintf(sTitle, "duttype_%d", type);
    char *name = (char *)malloc(64);
    GetStringValue(sTitle, "name", name);
    
    return name;
}

bool SetActivedDUT(int type) {
    bool flag = false;
    int count = GetCountOfDUTType();
    
    if (type >= count || type < 0) {
        Logger(MLLogError, "<%s>: No dut which type is %d\n", __func__, type);
        return flag;
    }
    
    if (gHandle != -1) {
        MLDUT dut = duts[type];
        
        int axises[] = {MLAxisClampLeft, MLAxisClampUp};
        MoveAxisToPosition(MLAxisLifter, dut.posLifter, true, false);
        MoveAxisToPosition(MLAxisClampLeft, dut.posHoldLeft, false, false);
        MoveAxisToPosition(MLAxisClampUp, dut.posHoldFront, false, false);
        while (!CheckAxisState(axises, 2, false)) { usleep(100000); }
        Logger(MLLogInfo, "<%s>: Success to set active DUT\n.", __func__);
        flag = true;
    } else {
        Logger(MLLogError, "<%s>: Connect controller firstly.\n", __func__);
    }
    
    return flag;
}

MLDUT GetDUT(int dutType) {
    MLDUT dut;
    MLDUT *ptr;
    ptr = duts;
    
    while (1) {
        if (ptr->type == dutType) {
            dut = *ptr;
            break;
        }
        ptr++;
    }
    
    return dut;
}
