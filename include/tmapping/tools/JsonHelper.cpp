//
// Created by stumbo on 2019/12/18.
//

#include "JsonHelper.h"
#include <iostream>
#include <fstream>
#include <unistd.h>
#include <pwd.h>
#include <sys/stat.h>

using namespace std;

namespace {

void chDir2TopoFileFloder() {
    uid_t uid;
    struct passwd* pwd;
    uid = getuid();
    pwd = getpwuid(uid);
    chdir(pwd->pw_dir);
    mkdir(tmap::TMAP_STD_FILE_SAVE_FLODER_NAME, 0b111111111);
    chdir(tmap::TMAP_STD_FILE_SAVE_FLODER_NAME);
}

}

std::string tmap::JsonHelper::JS2Str(const Jsobj& js, bool shortVersion, uint8_t precision)
{
    Json::StreamWriterBuilder builder;
    builder["precision"] = precision > 8 ? 8 : precision;
    builder["indentation"] = shortVersion ? "" : "   ";
    auto writerP(builder.newStreamWriter());
    stringstream ss;
    writerP->write(js, &ss);
    delete writerP;
    return std::move(ss.str());
}

tmap::Jsobj tmap::JsonHelper::Str2JS(const std::string& str)
{
    Jsobj res;
    Json::CharReaderBuilder b;
    auto readerP(b.newCharReader());
    string errs;
    if(!readerP->parse(str.data(), str.data() + str.size(), &res, &errs)) {
        cerr << FILE_AND_LINE << " read Json Failure! msg:\n" << errs << endl;
    }
    delete readerP;
    return res;
}

int tmap::JsonHelper::saveJson(const tmap::Jsobj& js, string fileName, bool addTime)
{
    chDir2TopoFileFloder();
    if (fileName.empty() || addTime) {
        struct tm * timeStructP;
        time_t timeLong;
        timeLong = time(nullptr);
        timeStructP = localtime(&timeLong);
        char tmp[64];
        strftime(tmp, sizeof(tmp), "%Y%m%d_%H%M", timeStructP);
        if (!fileName.empty()) {
            fileName += '_';
        }
        fileName += tmp;
    }
    fstream fs(fileName, std::ios::out | std::ios::trunc);
    fs << JS2Str(js, false);
    return 0;
}

tmap::Jsobj tmap::JsonHelper::loadJson(const std::string& fileName)
{
    Jsobj res;
    chDir2TopoFileFloder();
    fstream fs(fileName, std::ios::in);
    Json::CharReaderBuilder b;
    string errs;
    if (!parseFromStream(b, fs, &res, &errs)) {
        res = Json::nullValue;
        cerr << FILE_AND_LINE << errs << endl;
    }
    return res;
}
