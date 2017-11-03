#ifndef CONFIGURATION_H
#define CONFIGURATION_H
#include <map>
#include <string>

class Configuration
{
public:
    Configuration();
    void SetValue( std::string key, std::string value);
    std::string GetValue(std::string key);
    std::map<std::string, std::string>& GetConfig();

private:
    std::map<std::string, std::string> _config;
};

#endif // CONFIGURATION_H
