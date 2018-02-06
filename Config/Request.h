#ifndef REQUEST_H
#define REQUEST_H
#include <map>
#include <string>

class Request
{
public:
    Request();
    void SetValue( std::string key, std::string value);
    std::string GetValue(std::string key);
    std::map<std::string, std::string>& GetRequest();

private:
    std::map<std::string, std::string> _request;
};

#endif // REQUEST_H
