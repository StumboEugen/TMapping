//
// Created by stumbo on 2019/12/18.
//

#include "JsonHelper.h"
#include <iostream>
#include <fstream>
#include <unistd.h>
#include <pwd.h>
#include <sys/stat.h>

#include "../StructedMap.h"

using namespace std;

namespace {

void chDir2TopoFileFloder() {
    uid_t uid;
    struct passwd* pwd;
    uid = getuid();
    pwd = getpwuid(uid);
    chdir(pwd->pw_dir);
    mkdir(tmap::TOPO_STD_FILE_SAVE_FLODER_NAME, 0b111111111);
    chdir(tmap::TOPO_STD_FILE_SAVE_FLODER_NAME);
}

}

std::string tmap::JsonHelper::JS2Str(const Jsobj& js, bool shortVersion, uint8_t precision)
{
    Json::StreamWriterBuilder builder;
    builder["precision"] = precision > 8 ? 8 : precision;
    builder["indentation"] = shortVersion ? "" : "\t";
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
    return tmap::Jsobj();
}

int tmap::JsonHelper::saveStructedMap(const tmap::StructedMap& map2save, const std::string& fileName)
{
    chDir2TopoFileFloder();
    fstream fs(fileName, std::ios::out | std::ios::trunc);
    fs << JS2Str(map2save->toJS());
    return 0;
}

tmap::StructedMap loadStructedMapFromFile(const std::string& fileName) {
    chDir2TopoFileFloder();
    fstream fs(fileName, std::ios::in);
    tmap::Jsobj jsobj;
    fs >> jsobj;
    return make_shared<tmap::StructedMapImpl>(jsobj);
}
