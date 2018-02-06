#include "Request.h"

Request::Request()
{
}

void Request::SetValue(std::string key, std::string value)
{
    std::map<std::string, std::string>::iterator it;

    it = _request.find(key);

    if (it != _request.end())
    {
        it->second = value;
    }
    else
    {
        _request.insert(std::make_pair(key,value));
    }
}

std::string Request::GetValue(std::string key)
{
    std::map<std::string, std::string>::iterator it;

    it = _request.find(key);
    if (it != _request.end())
        return it->second;
    return "";
}

std::map<std::string, std::string>& Request::GetRequest()
{
    return _request;
}
