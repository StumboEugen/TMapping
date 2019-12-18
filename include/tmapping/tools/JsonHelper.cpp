//
// Created by stumbo on 2019/12/18.
//

#include "JsonHelper.h"
#include <iostream>

using namespace std;

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
