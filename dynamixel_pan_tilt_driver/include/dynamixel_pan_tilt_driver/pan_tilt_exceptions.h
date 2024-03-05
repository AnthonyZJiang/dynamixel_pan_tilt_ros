#pragma once
#include <exception>
#include <string>

class IndirectSyncIOException: public std::exception
{
public:
    IndirectSyncIOException(const std::string &msg) : msg_(msg) {}
    const char *what() const noexcept override { return msg_.c_str(); }

private:
    std::string msg_;
};

class PanTiltException: public std::exception
{
public:
    PanTiltException(const std::string &msg) : msg_(msg) {}
    const char *what() const noexcept override { return msg_.c_str(); }

private:
    std::string msg_;
};